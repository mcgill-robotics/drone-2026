"""
oa_core/robot3d.py
3-D port of Robot.  No pygame.  No display.
Inputs  : list of Obstacle3D  +  optional 3-D target waypoint
Outputs : self.vel  (Vector3D)   – desired velocity in world frame
          self.yaw_rate (float)  – desired yaw rate (rad/s)

Key differences from the 2-D version
--------------------------------------
* pygame.Vector2  → Vector3D
* Wall-normal computation now sums XYZ repulsion vectors
* A separate vertical (Z) repulsion keeps the drone away from
  floor/ceiling obstacles
* yaw_rate is derived from the XY component of the goal force
  so the drone naturally turns to face its target
"""

import math
from collections import Counter

from oa_core.vector3d   import Vector3D
from oa_core.physics3d  import PhysicsEngine3D
from oa_core.obstacle3d import Obstacle3D
from config.robot_config import RobotConfig


class Robot3D:
    def __init__(self, x, y, z, config: RobotConfig = None):
        if config is None:
            config = RobotConfig()

        self.phys_cfg = config.physics
        self.nav_cfg  = config.navigation
        self.env_cfg  = config.environment
        self.avoid_cfg = config.avoidance
        self.stuck_cfg = config.stuck_detection

        self.obstacles: list[Obstacle3D] = []
        self._target: Vector3D | None    = None

        self.physics = PhysicsEngine3D(
            Vector3D(x, y, z),
            Vector3D(0, 0, 0),
            self.phys_cfg,
        )

        self.smoothed_vel        = Vector3D(0, 0, 0)
        self.last_force          = Vector3D(0, 0, 0)
        self.wall_mode_direction = 0          # -1 / 0 / +1
        self.steps_since_obstacle = 0
        self.stuck_counter        = 0
        self.current_env_type     = "SPARSE"
        self.env_history: list[str] = []
        self.yaw_rate             = 0.0       # rad/s, sent to Webots
        self.course_completed     = False
        self.wall_stall_counter   = 0
        self.last_target_distance = None

    # ── public API ──────────────────────────────────────────────────────

    @property
    def pos(self) -> Vector3D:
        return self.physics.position

    @property
    def vel(self) -> Vector3D:
        return self.physics.velocity

    def set_target(self, x, y, z):
        self._target = Vector3D(x, y, z)
        self.course_completed = False

    # ── main step ───────────────────────────────────────────────────────

    def update(self):
        """Call once per Webots timestep."""
        target = self._target

        # ── environment classification ───────────────────────────────
        env_type, density, _ = self._classify_environment()
        self.current_env_type = self._get_stable_env_type(env_type)

        # ── scan obstacles ───────────────────────────────────────────
        wall_normal, obs_count = self._scan_for_obstacles(self.current_env_type)

        # ── choose force ─────────────────────────────────────────────
        if obs_count > 0:
            self.steps_since_obstacle = 0

            # If we stop making progress while near obstacles, switch wall side.
            if target:
                dist_to_target = self.pos.distance_to(target)
                no_progress = (
                    self.last_target_distance is not None
                    and dist_to_target >= self.last_target_distance - 0.01
                )
                if no_progress and self.vel.length() < 0.2:
                    self.wall_stall_counter += 1
                else:
                    self.wall_stall_counter = max(0, self.wall_stall_counter - 2)

                if self.wall_stall_counter > 20:
                    if self.wall_mode_direction == 0:
                        self.wall_mode_direction = 1
                    else:
                        self.wall_mode_direction *= -1
                    self.wall_stall_counter = 0

                self.last_target_distance = dist_to_target

            new_force = self._calculate_adaptive_avoidance_force(
                wall_normal, self.current_env_type, density
            )
        else:
            self.steps_since_obstacle += 1
            self.wall_stall_counter = 0
            mem = self.avoid_cfg.memory_thresholds.get(self.current_env_type, 20)
            if self.steps_since_obstacle > mem:
                self.wall_mode_direction = 0
            new_force = self._goal_force() if target else Vector3D(0, 0, 0)

            if target:
                self.last_target_distance = self.pos.distance_to(target)
            else:
                self.last_target_distance = None

        # ── stuck detection ──────────────────────────────────────────
        sc = self.stuck_cfg
        if self.physics.velocity.length() < sc.velocity_threshold and target:
            self.stuck_counter += 1
            if self.stuck_counter > sc.stuck_counter_threshold:
                new_force = self._escape_force()
        else:
            self.stuck_counter = 0

        # ── smoothing ────────────────────────────────────────────────
        avoid_smoothing = self.avoid_cfg.smoothing_factors.get(self.current_env_type, 0.4)
        sf = avoid_smoothing if obs_count > 0 else self.avoid_cfg.free_flight_smoothing
        smoothed = self.last_force.lerp(new_force, sf)
        self.last_force = smoothed

        # ── physics ──────────────────────────────────────────────────
        self.physics.apply_force(smoothed)
        self.physics.update()

        # ── velocity smoothing (same trick as 2-D version) ───────────
        vs = getattr(self.phys_cfg, "velocity_smoothing", 0.25)
        self.smoothed_vel = self.smoothed_vel.lerp(self.physics.velocity, vs)
        self.physics.position -= self.physics.velocity * self.phys_cfg.dt
        self.physics.position += self.smoothed_vel   * self.phys_cfg.dt

        # ── yaw rate: face the XY direction of travel ────────────────
        hvel = Vector3D(self.smoothed_vel.x, self.smoothed_vel.y, 0)
        if hvel.length() > 0.5:
            desired_yaw = math.atan2(hvel.y, hvel.x)
            # Proportional yaw controller (Webots will clamp it)
            self.yaw_rate = desired_yaw * 0.5
        else:
            self.yaw_rate = 0.0

        # ── arrival check ────────────────────────────────────────────
        if target and self.pos.distance_to(target) < self.nav_cfg.arrival_radius:
            self.physics.velocity = Vector3D(0, 0, 0)
            self.smoothed_vel     = Vector3D(0, 0, 0)
            self.course_completed = True
            self._target          = None

    # ── goal force ──────────────────────────────────────────────────────

    def _goal_force(self) -> Vector3D:
        """Steering force toward the 3-D target waypoint."""
        target = self._target
        if target is None:
            return Vector3D(0, 0, 0)

        desired = target - self.pos
        dist    = desired.length()
        if dist < 1e-6:
            return Vector3D(0, 0, 0)

        desired.normalize_ip()
        slow_r = getattr(self.nav_cfg, "slow_radius", 3.0)   # metres in 3-D
        max_sp = self.phys_cfg.max_speed

        if dist < slow_r:
            desired = desired * (max_sp * dist / slow_r)
        else:
            desired = desired * max_sp

        return desired - self.physics.velocity

    # ── environment classification (3-D version) ─────────────────────────

    def _classify_environment(self):
        scan_r = getattr(self.env_cfg, "scan_area_radius", 3.0)   # metres
        near   = []

        for obs in self.obstacles:
            dist = obs.distance_to(self.pos)
            if dist < scan_r:
                dx = obs.p.x - self.pos.x
                dy = obs.p.y - self.pos.y
                dz = obs.p.z - self.pos.z
                az  = math.atan2(dy, dx)
                el  = math.atan2(dz, math.sqrt(dx**2 + dy**2))
                near.append({"obs": obs, "dist": dist, "az": az, "el": el})

        volume  = (4/3) * math.pi * scan_r**3
        density = len(near) / volume * 1000.0 if volume > 0 else 0

        corridor_conf = self._detect_corridor_3d(near)

        dense_t    = getattr(self.env_cfg, "density_threshold_dense",    0.08)
        moderate_t = getattr(self.env_cfg, "density_threshold_moderate", 0.03)
        corridor_t = getattr(self.env_cfg, "corridor_confidence_threshold", 0.6)

        if corridor_conf > corridor_t:
            return "CORRIDOR", density, corridor_conf
        elif density > dense_t:
            return "DENSE", density, corridor_conf
        elif density > moderate_t:
            return "MODERATE", density, corridor_conf
        else:
            return "SPARSE", density, corridor_conf

    def _detect_corridor_3d(self, near_obstacles):
        """Simplified 3-D corridor detector: checks for left/right symmetry in XY."""
        if len(near_obstacles) < 2:
            return 0.0
        vel_az = math.atan2(self.vel.y, self.vel.x) if self.vel.length() > 0.1 else 0
        left = right = 0
        tol = 0.3
        for item in near_obstacles:
            rel = item["az"] - vel_az
            rel = (rel + math.pi) % (2*math.pi) - math.pi
            if -math.pi/2 - tol < rel < -math.pi/2 + tol:
                left += 1
            elif math.pi/2 - tol < rel < math.pi/2 + tol:
                right += 1
        if left >= 1 and right >= 1:
            balance = 1.0 - abs(left-right) / max(left, right)
            return min(1.0, (left+right)/4.0) * balance
        return 0.0

    def _get_stable_env_type(self, new_type):
        hist_len = getattr(self.env_cfg, "env_history_length", 5)
        self.env_history.append(new_type)
        if len(self.env_history) > hist_len:
            self.env_history.pop(0)
        if len(self.env_history) >= 3:
            return Counter(self.env_history).most_common(1)[0][0]
        return new_type

    # ── obstacle scan (3-D) ─────────────────────────────────────────────

    def _scan_for_obstacles(self, env_type):
        """Returns (wall_normal_3d: Vector3D, obs_count: int)."""
        radii       = self.avoid_cfg.scan_radius
        entry_exit  = radii.get(env_type, None)
        if entry_exit is None:
            entry_r, exit_r = 1.5, 3.0
        elif isinstance(entry_exit, tuple):
            entry_r, exit_r = entry_exit
        else:
            entry_r = entry_exit.entry_radius
            exit_r  = entry_exit.exit_radius

        # Convert pixel-space radii (orig) to metres if they look too large
        if entry_r > 10:
            entry_r /= 20.0
            exit_r  /= 20.0

        scan_r = exit_r if self.wall_mode_direction != 0 else entry_r

        la_base = getattr(self.env_cfg, "look_ahead_distance_base", 1.0)
        la_mult = getattr(self.env_cfg, "look_ahead_velocity_mult", 1.5)
        la_dist = max(la_base, self.vel.length() * la_mult)
        future_pos = (
            self.pos + self.vel.normalize() * la_dist
            if self.vel.length() > 0.01
            else self.pos
        )

        wall_normal_sum = Vector3D(0, 0, 0)
        obs_count = 0

        for obs in self.obstacles:
            d_future  = obs.distance_to(future_pos)
            d_present = obs.distance_to(self.pos)

            if d_future < scan_r or d_present < scan_r:
                if d_future < scan_r:
                    weight   = (scan_r - d_future) / scan_r
                    push_vec = obs.p - future_pos
                else:
                    weight   = (scan_r - d_present) / scan_r
                    push_vec = obs.p - self.pos

                if push_vec.length() > 1e-6:
                    wall_normal_sum += (-push_vec.normalize() * weight)

                obs_count += 1

        return wall_normal_sum, obs_count

    # ── adaptive avoidance force (3-D) ──────────────────────────────────

    def _calculate_adaptive_avoidance_force(self, wall_normal_sum, env_type, density):
        force   = Vector3D(0, 0, 0)
        urgency = min(1.0, wall_normal_sum.length())

        if urgency < 0.01:
            return force

        wall_normal = wall_normal_sum.normalize()

        # Choose tangent in XY plane (same as 2-D logic, Z stays separate)
        wn_xy = Vector3D(wall_normal.x, wall_normal.y, 0)
        if wn_xy.length() > 1e-6:
            wn_xy.normalize_ip()
            if self.wall_mode_direction == 0:
                t_left  = Vector3D(-wn_xy.y,  wn_xy.x, 0)
                t_right = Vector3D( wn_xy.y, -wn_xy.x, 0)
                to_tgt_xy = Vector3D(0, 0, 0)
                if self._target:
                    diff = self._target - self.pos
                    to_tgt_xy = Vector3D(diff.x, diff.y, 0)
                    if to_tgt_xy.length() > 1e-6:
                        to_tgt_xy.normalize_ip()
                self.wall_mode_direction = (
                    -1 if t_left.dot(to_tgt_xy) > t_right.dot(to_tgt_xy) else 1
                )
            elif self._target:
                # Allow direction changes in dead-ends instead of committing forever.
                t_left  = Vector3D(-wn_xy.y,  wn_xy.x, 0)
                t_right = Vector3D( wn_xy.y, -wn_xy.x, 0)
                diff = self._target - self.pos
                to_tgt_xy = Vector3D(diff.x, diff.y, 0)
                if to_tgt_xy.length() > 1e-6:
                    to_tgt_xy.normalize_ip()
                    left_score = t_left.dot(to_tgt_xy)
                    right_score = t_right.dot(to_tgt_xy)
                    switch_margin = 0.35
                    if self.wall_mode_direction == -1 and right_score > left_score + switch_margin:
                        self.wall_mode_direction = 1
                    elif self.wall_mode_direction == 1 and left_score > right_score + switch_margin:
                        self.wall_mode_direction = -1

            tangent = (
                Vector3D(-wn_xy.y,  wn_xy.x, 0)
                if self.wall_mode_direction == -1
                else Vector3D( wn_xy.y, -wn_xy.x, 0)
            )
        else:
            tangent = Vector3D(0, 0, 0)

        profile = getattr(self.avoid_cfg, env_type.lower(), None)
        if profile is not None:
            # XY repulsion + tangent steering (same as 2-D)
            force += wall_normal * (profile.normal_base + profile.normal_urgency_mult * urgency)
            t_mult = profile.tangent_base + getattr(profile, "tangent_urgency_mult", 0.0) * urgency
            if self.vel.length() < getattr(profile, "low_speed_threshold", 0.5):
                force += tangent * t_mult * getattr(profile, "low_speed_mult", 1.0)
            else:
                steer  = tangent * self.phys_cfg.max_speed - self.vel
                force += steer * getattr(profile, "steering_gain", 1.0)

            if self._target and tangent.length() > 1e-6:
                # Keep making progress in obstacle mode without pushing into the wall.
                goal_force = self._goal_force()
                toward_wall_component = min(0.0, goal_force.dot(wall_normal))
                goal_force = goal_force - wall_normal * toward_wall_component

                tangent_goal_mag = max(0.0, goal_force.dot(tangent))
                force += tangent * tangent_goal_mag * 0.8

                # If nearly stopped near an obstacle, bias tangential flow to break deadlocks.
                if self.vel.length() < 0.15:
                    force += tangent * (t_mult * 0.4)

            # Z-axis: vertical repulsion to avoid floor/ceiling
            z_repulsion = wall_normal.z * (profile.normal_base * 0.5 + profile.normal_urgency_mult * urgency * 0.5)
            force.z += z_repulsion

            if env_type == "CORRIDOR" and hasattr(profile, "goal_force_strength") and self._target:
                to_goal = (self._target - self.pos).normalize()
                force  += to_goal * profile.goal_force_strength

        # Guarantee some tangential flow so we move around walls instead of stalling.
        if tangent.length() > 1e-6:
            tangent_component = force.dot(tangent)
            min_tangent_component = self.phys_cfg.max_force * (0.18 + 0.22 * urgency)
            if tangent_component < min_tangent_component:
                force += tangent * (min_tangent_component - tangent_component)

        return force

    # ── stuck escape ────────────────────────────────────────────────────

    def _escape_force(self) -> Vector3D:
        import random
        angle = random.uniform(-math.pi, math.pi)
        elev  = random.uniform(-math.pi/4, math.pi/4)
        mag   = self.phys_cfg.max_force * self.stuck_cfg.escape_force_mult
        return Vector3D(
            math.cos(elev) * math.cos(angle) * mag,
            math.cos(elev) * math.sin(angle) * mag,
            math.sin(elev) * mag,
        )
