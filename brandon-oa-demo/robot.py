import pygame
import math
from collections import Counter

from config.robot_config import RobotConfig
from core.physics_engine import PhysicsEngine
from core.navigation_controller import NavigationController
from strategies.navigation_strategy import SimpleNavigationStrategy
from states.robot_state import MovingState, AvoidingState, StuckState, IdleState


class Robot:
    def __init__(self, x, y, obstacles, course, dt=0.1, m=1.0, f_max=55.0, diam=15.0, config=None, navigation_strategy=None):
        if config is None:
            config = RobotConfig.from_parameters(dt=dt, m=m, f_max=f_max, diam=diam)

        self.physics_config = config.physics
        self.navigation_config = config.navigation
        self.environment_config = config.environment
        self.avoidance_config = config.avoidance
        self.stuck_config = config.stuck_detection
        self.viz_config = config.visualization

        self.obstacles = obstacles
        self.diam = self.physics_config.diameter
        self.virtualforce = True

        self.trail = []
        self.wall_mode_direction = 0
        self.last_force = pygame.Vector2(0, 0)
        self.steps_since_obstacle = 0
        self.current_env_type = 'SPARSE'
        self.env_history = []
        self.env_history_length = getattr(self.environment_config, 'env_history_length', 5)
        self.stuck_counter = 0

        self.physics = PhysicsEngine(pygame.Vector2(x, y), pygame.Vector2(0, 0), self.physics_config)
        strategy = navigation_strategy if navigation_strategy else SimpleNavigationStrategy()
        self.navigation = NavigationController(course, self.navigation_config, strategy)
        self.smoothed_vel = pygame.Vector2(0, 0)

        self.state = MovingState()

    # ------------------------------------------------------------------
    # Properties delegating to subsystems
    # ------------------------------------------------------------------

    @property
    def pos(self):
        return self.physics.position

    @property
    def vel(self):
        return self.physics.velocity

    @property
    def m(self):
        return self.physics_config.mass

    @property
    def f_max(self):
        return self.physics_config.max_force

    @property
    def max_speed(self):
        return self.physics_config.max_speed

    @property
    def dt(self):
        return self.physics_config.dt

    @property
    def course(self):
        return self.navigation.course

    @property
    def course_completed(self):
        return self.navigation.course_completed

    @property
    def current_waypoint_index(self):
        return self.navigation.current_waypoint_index

    @property
    def target(self):
        return self.navigation.current_target

    @target.setter
    def target(self, value):
        self.navigation.current_target = value

    # ------------------------------------------------------------------
    # State management
    # ------------------------------------------------------------------

    def set_state(self, state):
        self.state = state

    # ------------------------------------------------------------------
    # Sensor / environment methods (called by states)
    # ------------------------------------------------------------------

    def _detect_corridor(self, near_obstacles):
        """Detects parallel wall configuration. Returns confidence 0.0–1.0."""
        if len(near_obstacles) < 2:
            return 0.0

        left_count = right_count = 0
        vel_angle = math.atan2(self.vel.y, self.vel.x) if self.vel.length() > 0 else 0

        for item in near_obstacles:
            relative_angle = item['angle'] - vel_angle
            relative_angle = (relative_angle + math.pi) % (2 * math.pi) - math.pi

            if -math.pi / 2 - 0.3 < relative_angle < -math.pi / 2 + 0.3:
                left_count += 1
            elif math.pi / 2 - 0.3 < relative_angle < math.pi / 2 + 0.3:
                right_count += 1

        if left_count >= 1 and right_count >= 1:
            balance = 1.0 - abs(left_count - right_count) / max(left_count, right_count)
            return min(1.0, (left_count + right_count) / 4.0) * balance

        return 0.0

    def _classify_environment(self):
        """Analyzes nearby obstacles to determine environment type."""
        scan_area_radius = getattr(self.environment_config, 'scan_area_radius', 160.0)
        near_obstacles = []

        for obs in self.obstacles:
            dist = (obs.p - self.pos).length()
            if dist < scan_area_radius:
                angle = math.atan2(obs.p.y - self.pos.y, obs.p.x - self.pos.x)
                near_obstacles.append({'obs': obs, 'dist': dist, 'angle': angle})

        area = math.pi * scan_area_radius ** 2
        density = len(near_obstacles) / area * 10000.0

        corridor_confidence = self._detect_corridor(near_obstacles)

        corridor_thresh = getattr(self.environment_config, 'corridor_confidence_threshold', 0.6)
        dense_thresh = getattr(self.environment_config, 'density_threshold_dense', 0.08)
        moderate_thresh = getattr(self.environment_config, 'density_threshold_moderate', 0.03)

        if corridor_confidence > corridor_thresh:
            return 'CORRIDOR', density, corridor_confidence
        elif density > dense_thresh:
            return 'DENSE', density, corridor_confidence
        elif density > moderate_thresh:
            return 'MODERATE', density, corridor_confidence
        else:
            return 'SPARSE', density, corridor_confidence

    def _get_stable_env_type(self, new_env_type):
        """Prevent rapid mode switching using history."""
        self.env_history.append(new_env_type)
        if len(self.env_history) > self.env_history_length:
            self.env_history.pop(0)

        if len(self.env_history) >= 3:
            return Counter(self.env_history).most_common(1)[0][0]
        return new_env_type

    def _scan_for_obstacles(self, env_type=None):
        """Dual-check obstacle scan: tests both current and predicted future position."""
        scan_radii = getattr(self.avoidance_config, 'scan_radius', {
            'SPARSE': (38.0, 68.0),
            'MODERATE': (45.0, 82.0),
            'DENSE': (53.0, 98.0),
            'CORRIDOR': (53.0, 98.0),
        })
        entry_exit = scan_radii.get(env_type, (60.0, 110.0))
        if isinstance(entry_exit, tuple):
            entry_radius, exit_radius = entry_exit
        else:
            entry_radius, exit_radius = entry_exit.entry_radius, entry_exit.exit_radius
        scan_radius = exit_radius if self.wall_mode_direction != 0 else entry_radius

        look_ahead_base = getattr(self.environment_config, 'look_ahead_distance_base', 40.0)
        look_ahead_mult = getattr(self.environment_config, 'look_ahead_velocity_mult', 1.5)
        look_ahead_dist = max(look_ahead_base, self.vel.length() * look_ahead_mult)
        future_pos = (
            self.pos + (self.vel.normalize() * look_ahead_dist) if self.vel.length() > 0 else self.pos
        )

        wall_normal_sum = pygame.Vector2(0, 0)
        obs_count = 0

        for obs in self.obstacles:
            dist_future = (obs.p - future_pos).length()
            dist_present = (obs.p - self.pos).length()

            if dist_future < scan_radius or dist_present < scan_radius:
                if dist_future < scan_radius:
                    weight = (scan_radius - dist_future) / scan_radius
                    push_vec = obs.p - future_pos
                else:
                    weight = (scan_radius - dist_present) / scan_radius
                    push_vec = obs.p - self.pos

                if push_vec.length() > 0:
                    wall_normal_sum += (-push_vec.normalize() * weight)

                obs_count += 1

        return wall_normal_sum, obs_count

    def _calculate_adaptive_avoidance_force(self, wall_normal_sum, env_type, density):
        """Calculate avoidance force based on environment type."""
        force = pygame.Vector2(0, 0)
        urgency = min(1.0, wall_normal_sum.length())

        if urgency < 0.01:
            return force

        wall_normal = wall_normal_sum.normalize()

        if self.wall_mode_direction == 0:
            tangent_left = pygame.Vector2(-wall_normal.y, wall_normal.x)
            tangent_right = pygame.Vector2(wall_normal.y, -wall_normal.x)
            to_target = (self.target - self.pos).normalize() if self.target else pygame.Vector2(0, 0)
            self.wall_mode_direction = -1 if tangent_left.dot(to_target) > tangent_right.dot(to_target) else 1

        tangent = (
            pygame.Vector2(-wall_normal.y, wall_normal.x)
            if self.wall_mode_direction == -1
            else pygame.Vector2(wall_normal.y, -wall_normal.x)
        )

        profile = getattr(self.avoidance_config, env_type.lower(), None)
        if profile is not None:
            force += wall_normal * (profile.normal_base + profile.normal_urgency_mult * urgency)
            tangent_mult = profile.tangent_base + getattr(profile, 'tangent_urgency_mult', 0.0) * urgency
            if self.vel.length() < getattr(profile, 'low_speed_threshold', 10.0):
                force += tangent * tangent_mult * getattr(profile, 'low_speed_mult', 1.0)
            else:
                steer = tangent * self.max_speed - self.vel
                force += steer * getattr(profile, 'steering_gain', 1.0)
            if env_type == 'CORRIDOR' and hasattr(profile, 'goal_force_strength'):
                to_goal = (self.target - self.pos).normalize() if self.target else pygame.Vector2(0, 0)
                force += to_goal * profile.goal_force_strength

        return force

    # ------------------------------------------------------------------
    # Main update
    # ------------------------------------------------------------------

    def update_position(self):
        # 1. Advance waypoint tracking
        self.navigation.update_target(self.physics.position)

        target = self.navigation.current_target
        velocity = self.physics.velocity
        new_force = pygame.Vector2(0, 0)

        # 2. Transition to idle when course is done
        if self.navigation.course_completed and not isinstance(self.state, IdleState):
            self.set_state(IdleState())

        # 3. Delegate force calculation to current state
        if self.virtualforce and (target or isinstance(self.state, IdleState)):
            new_force = self.state.handle(self)
        elif not target and velocity.length() > 0.5:
            # No waypoints placed yet — gentle braking
            new_force = -velocity.normalize() * self.physics_config.max_force * 0.8

        # 4. Adaptive force smoothing
        smoothing_factor = (
            self.avoidance_config.smoothing_factors.get(self.current_env_type, 0.4)
            if isinstance(self.state, AvoidingState)
            else self.avoidance_config.free_flight_smoothing
        )
        smoothed_force = self.last_force.lerp(new_force, smoothing_factor)
        self.last_force = smoothed_force

        # 5. Apply to physics
        self.physics.apply_force(smoothed_force)
        self.physics.update()

        # 6. Velocity smoothing
        vel_smooth = getattr(self.physics_config, 'velocity_smoothing', 0.25)
        self.smoothed_vel = self.smoothed_vel.lerp(self.physics.velocity, vel_smooth)
        self.physics.position -= self.physics.velocity * self.physics_config.dt
        self.physics.position += self.smoothed_vel * self.physics_config.dt

        # 7. Stuck detection & recovery
        sc = self.stuck_config
        if self.physics.velocity.length() < sc.velocity_threshold and target:
            self.stuck_counter += 1
            if not isinstance(self.state, (StuckState, IdleState)) and self.stuck_counter > sc.stuck_counter_threshold:
                self.set_state(StuckState())
            elif isinstance(self.state, StuckState) and self.stuck_counter > sc.stuck_reset_threshold:
                self.stuck_counter = 0
                self.wall_mode_direction = 0
                self.set_state(MovingState())
        else:
            if isinstance(self.state, StuckState):
                self.set_state(MovingState())
            self.stuck_counter = 0

        # 8. Full stop when idle and slow
        if not target and self.physics.velocity.length() < 0.5:
            self.physics.velocity = pygame.Vector2(0, 0)
            self.smoothed_vel = pygame.Vector2(0, 0)

        self._update_trail()

    # ------------------------------------------------------------------
    # Rendering
    # ------------------------------------------------------------------

    def _update_trail(self):
        self.trail.append(self.physics.position.copy())
        if len(self.trail) > self.viz_config.trail_length:
            self.trail.pop(0)

    def draw(self, surface):
        t = pygame.time.get_ticks() / 1000.0
        vc = self.viz_config
        pulse_radius = vc.pulse_radius_base * (1.0 + vc.pulse_amplitude * math.sin(t * vc.pulse_frequency))
        radar_surf = pygame.Surface((pulse_radius * 2, pulse_radius * 2), pygame.SRCALPHA)
        pygame.draw.circle(radar_surf, (0, 200, 255, 40), (int(pulse_radius), int(pulse_radius)), int(pulse_radius))
        pygame.draw.circle(radar_surf, (0, 200, 255, 100), (int(pulse_radius), int(pulse_radius)), int(pulse_radius), 2)
        surface.blit(radar_surf, (self.physics.position.x - pulse_radius, self.physics.position.y - pulse_radius))

        if len(self.trail) > 1:
            pygame.draw.lines(surface, (100, 100, 255), False, self.trail, 2)

        look_ahead_dist = self.physics.velocity.length() * 1.5
        if look_ahead_dist < 40:
            look_ahead_dist = 40
        future_pos = (
            self.physics.position + (self.physics.velocity.normalize() * look_ahead_dist)
            if self.physics.velocity.length() > 0
            else self.physics.position
        )

        target = self.navigation.current_target
        if target:
            for obs in self.obstacles:
                to_future = obs.p - future_pos
                to_present = obs.p - self.physics.position
                dist_future = to_future.length()
                dist_present = to_present.length()

                if dist_future < vc.avoid_dist or dist_present < vc.avoid_dist:
                    if dist_future < dist_present:
                        outward_dir = -to_future.normalize()
                        repulsion_mag = (vc.avoid_dist - dist_future) / vc.avoid_dist
                    else:
                        outward_dir = -to_present.normalize()
                        repulsion_mag = (vc.avoid_dist - dist_present) / vc.avoid_dist

                    pygame.draw.line(
                        surface, (255, 50, 50), self.physics.position,
                        self.physics.position + outward_dir * (repulsion_mag * vc.vector_scale), 2
                    )
                    vortex_dir = pygame.Vector2(-outward_dir.y, outward_dir.x)
                    pygame.draw.line(
                        surface, (50, 255, 50), self.physics.position,
                        self.physics.position + vortex_dir * (repulsion_mag * vc.vector_scale), 2
                    )

        color = vc.env_colors.get(self.current_env_type, (128, 128, 128))
        pygame.draw.circle(
            surface, color,
            (int(self.physics.position.x), int(self.physics.position.y)),
            int(self.diam / 2) + 3, 2
        )

        pygame.draw.circle(
            surface, (255, 255, 255),
            (int(self.physics.position.x), int(self.physics.position.y)),
            int(self.diam / 2)
        )
        for angle in [45, 135, 225, 315]:
            rad = math.radians(angle)
            prop_pos = self.physics.position + pygame.Vector2(math.cos(rad), math.sin(rad)) * (self.diam * 0.8)
            pygame.draw.circle(surface, (50, 150, 250), (int(prop_pos.x), int(prop_pos.y)), 4)
