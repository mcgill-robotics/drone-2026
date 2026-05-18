# Mission 2: Building Perimeter Inspection

Autonomous building perimeter inspection using wall-following and depth sensor integration.

## Mission Overview

**Objective:** Autonomously fly around a building's perimeter while maintaining ~2m distance from walls, detecting walls using a depth camera and performing obstacle avoidance via lidar.

**Flight Phases:**
1. **Boot & OFFBOARD:** Initialize PX4/MAVROS, switch to OFFBOARD mode, wait for manual arm
2. **Takeoff:** Climb to 5m altitude
3. **Navigation:** Fly to building center (GPS coordinate)
4. **Wall Following:** Follow 4 walls of the building perimeter:
   - Approach wall (depth sensor feedback)
   - Align perpendicular to wall
   - Strafe along wall at constant distance (2m)
   - Detect wall end and turn corner
5. **Land & Recovery:** Land at final position

## Architecture

### Files

- **`movement.py`** — Low-level drone control interface
  - Boot/arm/disarm/mode switching (delegates to `px4_interface.py`)
  - Position-based navigation (takeoff to GPS coordinate)
  - Velocity-based movement (for wall strafing)
  - All commands use `send_velocity_setpoint()` for OFFBOARD mode

- **`depth_camera.py`** — Depth sensor interface
  - Subscribes to depth camera ROS topic (TODO: implement)
  - Maintains rolling history of readings (center, left, right distances)
  - Provides queries: `is_wall_ahead()`, `has_obstacle_on_side()`, `is_wall_ending()`
  - Distance filtering and validity checking

- **`wall_follower.py`** — Wall-following behavior
  - High-level perimeter inspection orchestration
  - Approach → Align → Strafe → Turn → Repeat
  - Integrates depth camera feedback
  - Maintains 2m target distance using distance error feedback
  - Detects wall ends via sudden distance jumps
  - Placeholder for OA bridge integration (lidar obstacle avoidance)

- **`test_mission2.py`** — Main mission script
  - Argument parsing (building center lat/lon)
  - Phase orchestration (boot → takeoff → navigate → inspect → land)
  - Exception handling and cleanup
  - Usage: `python3 test_mission2.py --lat 37.7749 --lon -122.4194 [--sitl]`

### Control Flow

```
test_mission2.main()
├── movement.setup_environment()  [Boot PX4/MAVROS]
├── movement.begin_phase()        [Enter OFFBOARD, takeoff to 5m]
├── movement.navigate_to_coordinate()  [Fly to building center]
├── DepthCameraInterface()        [Initialize depth subscriber]
├── WallFollower()                [Initialize wall follower]
├── wall_follower.inspect_perimeter()
│   ├── approach_wall()           [Forward until 2m from wall]
│   ├── align_with_wall()         [Rotate to perpendicular]
│   ├── strafe_along_wall()       [Sideways movement with distance maintenance]
│   │   └── [Optional] avoider_callback()  [OA bridge detour queries]
│   ├── turn_corner()             [Rotate 90° for next wall]
│   └── [Repeat for 4 walls]
└── movement.end_phase()          [Land]
```

## Key Features

### Depth Camera Integration

The depth sensor provides three key measurements:
- **Center distance:** Distance straight ahead (wall detection)
- **Left distance:** Distance to the left (obstacle detection during strafing)
- **Right distance:** Distance to the right (obstacle detection during strafing)

Used for:
1. **Wall approach:** Move forward until center distance = 2m
2. **Wall alignment:** Compare left vs right to detect roll/yaw error
3. **Strafing:** Maintain constant center distance with small forward adjustments
4. **Wall end detection:** Jump in center distance indicates wall disappearing
5. **Obstacle avoidance:** Side distance threshold triggers avoidance behavior

### Obstacle Avoidance

Two layers:
1. **Depth camera side obstacles:** If obstacle detected on strafing side (>1.5m away), reduce strafe speed
2. **Lidar via OA bridge (TODO):** Can query OA bridge for safe waypoints during strafing

### Wall Alignment

The drone maintains perpendicular alignment to walls:
- **Left distance < Right distance:** Tilted right, rotate left
- **Right distance < Left distance:** Tilted left, rotate right
- **Equal:** Perpendicular to wall

## Configuration

Modify `WallFollowingConfig` in `test_mission2.py`:

```python
wall_config = WallFollowingConfig(
    target_wall_distance_m=2.0,          # Maintain 2m from wall
    distance_tolerance_m=0.3,            # ±0.3m tolerance
    strafe_speed_mps=0.5,                # Move at 0.5 m/s along wall
    approach_speed_mps=0.3,              # Move at 0.3 m/s when approaching
    turn_speed_mps=0.2,                  # Move at 0.2 m/s when turning
    wall_end_threshold_m=4.0,            # Distance indicating wall ended
    corner_turn_angle_rad=math.pi/2,     # 90° corners
    max_strafing_time_s=120.0,           # 2 min max per wall
    obstacle_check_distance_m=1.5,       # Avoid obstacles >1.5m away
)
```

## Usage

### Basic Mission (Simulated)

```bash
# SITL simulation (PX4 running locally)
python3 tests/mission2/test_mission2.py \
  --lat 37.7749 \
  --lon -122.4194 \
  --sitl

# Dry run (show what would happen, no flight)
python3 tests/mission2/test_mission2.py \
  --lat 37.7749 \
  --lon -122.4194 \
  --dry-run
```

### Real Hardware

```bash
# Jetson hardware (serial connection to PX4)
python3 tests/mission2/test_mission2.py \
  --lat 37.7749 \
  --lon -122.4194
```

## TODO & Implementation Notes

### High Priority

1. **Depth Camera ROS Subscriber (depth_camera.py)**
   - Support `d455` and `sim` presets, with `--depth-topic` / `--depth-scale-m` overrides
   - Subscribe to `/camera/aligned_depth_to_color/image_raw` (Intel RealSense) or the Gazebo depth topic configured at runtime
   - Process depth frame to extract center/left/right regions
   - Handle NaN/invalid values (too far, too close, occlusions)
   - Maintain rolling history for trend detection
   - Implement `is_wall_ending()` and `get_distance_trend()`

2. **Wall Alignment (wall_follower.py::align_with_wall)**
   - Implement yaw control to rotate until left_distance ≈ right_distance
   - Once aligned perpendicular, rotate 90° to face along wall
   - Validate alignment before starting to strafe

3. **Corner Turning (wall_follower.py::turn_corner)**
   - Rotate 90° in place
   - Update `current_wall` (FRONT → RIGHT → BACK → LEFT → FRONT)
   - Verify new wall is ahead before continuing

### Medium Priority

4. **OA Bridge Integration (wall_follower.py::strafe_along_wall)**
   - Add lidar-based obstacle avoidance callback
   - When obstacle detected on strafing side:
     - Query OA bridge for safe detour waypoint
     - Move to waypoint (position-based)
     - Resume strafing
   - May need local coordinate ↔ GPS conversion

5. **Depth Frame Coordinate System**
   - Map depth pixel coordinates to real-world distances
   - Handle camera calibration (focal length, principal point)
   - Account for camera mounting (pitch/roll relative to drone body frame)

### Lower Priority

6. **Advanced Strafing**
   - Implement `strafe_direction` logic per wall (e.g., always move clockwise around building)
   - Add coverage checking to verify all walls were inspected
   - Handle non-rectangular buildings

7. **Failure Recovery**
   - Timeout detection → hold position and retry
   - Lost wall detection → search/recapture
   - Lidar conflict resolution → coordinate with both sensors

8. **Telemetry & Logging**
   - Log depth readings and positions to CSV
   - Record wall distances over time
   - Generate perimeter map visualization

## Dependencies

- **ROS 2 (Humble):** MAVROS, depth camera drivers
- **PX4 Autopilot:** Running on Jetson or simulation
- **Intel RealSense SDK** (if using RealSense camera): `pip install pyrealsense2`
- **OpenCV:** Already in environment, used for potential frame processing

## Safety Notes

- **Manual arm required:** Drone waits for manual RC arm before moving
- **Failsafe modes active:** If OFFBOARD connection lost, drone lands
- **Velocity limits:** Conservative speeds (0.3-0.5 m/s) for precision
- **Timeouts:** Each phase has explicit timeout to prevent infinite loops
- **Depth sensor failure:** Falls back to holding position if no valid readings

## Testing Strategy

1. **Dry run first:** Test argument parsing and phase structure
2. **SITL with mock depth:** Use `--dry-run` for unit testing wall-follower logic
3. **Simulated building:** Create virtual obstacle in Gazebo, test wall detection
4. **Real depth sensor:** Connect camera, verify frame rates and distance accuracy
5. **Real flight:** Test in open space first (far from walls), then near building

## Related Code

- **`mission_controller/px4_interface.py`:** Boot, arm, mode changes
- **`mission_controller/px4_setters.py`:** Velocity/position setpoints
- **`oa_bridge/oa_core/avoider.py`:** Obstacle avoidance interface
- **`tests/mission1/movement.py`:** Similar pattern for reference
