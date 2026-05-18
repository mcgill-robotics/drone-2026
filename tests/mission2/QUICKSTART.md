# Mission 2 Quick Start

## What is Mission 2?

Mission 2 is an **autonomous building perimeter inspection** system. The drone:
1. Takes off to 5 meters
2. Flies to a building center coordinate
3. Follows the building perimeter walls at ~2 meters distance
4. Uses depth camera to detect walls and maintain distance
5. Detects corners/wall ends and turns
6. Completes full perimeter scan and lands

## File Structure

```
tests/mission2/
├── __init__.py                 # Package init
├── test_mission2.py           # Main entry point (RUN THIS)
├── movement.py                 # Low-level flight control (uses px4_interface)
├── depth_camera.py            # Depth sensor interface (TODO: ROS subscriber)
├── wall_follower.py           # Wall-following logic
└── README.md                  # Full documentation
```

## Quick Test (Dry Run)

```bash
cd /Users/tylervuong/ros2_ws/drone-2026

# Test without flying
python3 tests/mission2/test_mission2.py \
  --lat 37.7749 \
  --lon -122.4194 \
  --dry-run
```

**Output:** Should show mission phases and what would happen (no actual flight)

## SITL Simulation

```bash
# Terminal 1: Start PX4 SITL and Gazebo
cd ~/PX4-Autopilot
make px4_sitl gazebo_x500

# Terminal 2: Run mission
python3 /path/to/test_mission2.py \
  --lat 37.7749 \
  --lon -122.4194 \
   --sitl \
   --depth-backend sim
```

**Note:** Use `--depth-backend sim` for Gazebo/PX4 depth topics, or `--depth-backend d455` for the real Intel camera. You can also override the topic with `--depth-topic` if your sim plugin publishes somewhere else.

## Real Hardware

```bash
# Jetson with PX4 firmware and MAVROS running

python3 tests/mission2/test_mission2.py \
  --lat 37.7749 \
  --lon -122.4194
```

To force a specific source, add `--depth-backend d455` or `--depth-backend sim`.

**Requirements:**
- PX4 running on FC (Pixhawk, etc.)
- MAVROS running on Jetson
- Depth camera mounted and ROS driver running
- Serial/UART connection between Jetson and flight controller

## Expected Flight

1. **Boot & Arm (~10s)**
   - Boots PX4/MAVROS
   - Enters OFFBOARD mode
   - Waits for manual RC arm (RC transmitter button)

2. **Takeoff (~15s)**
   - Climbs to 5m altitude
   - Locks heading

3. **Navigation (~30-60s)**
   - Flies to building center (GPS)
   - Holds position

4. **Wall Following (~5-10 min)**
   - Approaches wall (forward until 2m distance detected)
   - Aligns perpendicular
   - Strafes left along wall (detects wall end)
   - Turns corner 90°
   - Repeats for 4 walls

5. **Landing (~30s)**
   - Lands at final position

**Total Time:** ~10 minutes for full building

## What Needs to Be Implemented

### Critical (Blocking Flight)

1. **Depth Camera Subscriber** (`depth_camera.py`)
   - TODO: Subscribe to depth camera ROS topic
   - Extract center/left/right distances
   - Handle invalid/NaN values

### Important (Required for Functional Wall Following)

2. **Wall Alignment** (`wall_follower.py::align_with_wall()`)
   - TODO: Rotate to perpendicular using left vs right distances

3. **Corner Turning** (`wall_follower.py::turn_corner()`)
   - TODO: Rotate 90° and verify next wall

### Optional (Enhanced Features)

4. **OA Bridge Integration** (`wall_follower.py::strafe_along_wall()`)
   - TODO: Query lidar avoider for obstacle detours

## Troubleshooting

### "No valid depth reading"
**Cause:** Depth camera subscriber not implemented or camera not streaming
**Fix:** Implement ROS subscriber in `depth_camera.py`

### "Timeout approaching wall"
**Cause:** Depth camera not detecting walls (see above)
**Fix:** Verify camera topic (`rostopic list | grep depth`)

### "OFFBOARD mode timeout"
**Cause:** Setpoint stream not started or MAVROS not connected
**Fix:** Check MAVROS is running and connected to PX4

### "Manual arm timeout"
**Cause:** RC transmitter not arming drone
**Fix:** Press/hold RC arm button on transmitter

## Next Steps

1. **Test dry-run** to verify structure works
2. **Implement depth camera** (highest priority)
3. **Test SITL** with mock building obstacles
4. **Add wall alignment** logic
5. **Integrate OA bridge** for advanced obstacle avoidance
6. **Test on real hardware** (open area first, then building)

## Key Commands

```bash
# View depth camera topic
rostopic list | grep -i depth
rostopic echo /camera/aligned_depth_to_color/image_raw

# View drone state
rostopic echo /mavros/state

# Monitor velocities
rostopic echo /mavros/local_position/velocity_local
```

## References

- [Full Documentation](README.md)
- Mission 1 code: `tests/mission1/` (similar structure)
- PX4 interface: `mission_controller/px4_interface.py`
- OA Bridge: `oa_bridge/` (optional integration)
