import rclpy
import time
from px4_interface import init_px4


def test_mode_change(px4):
    """Test changing flight mode"""
    print("[TEST] Testing mode change...")

    # log mode before, attempt OFFBOARD switch, log result
    print_current_mode(px4, prefix="[TEST][BEFORE]")
    success = px4.change_mode("OFFBOARD")
    print(f"[TEST] change_mode('OFFBOARD') -> {success}")

    # spin a few times so state message has time to update
    for _ in range(10):
        rclpy.spin_once(px4, timeout_sec=0.1)
        time.sleep(0.1)

    # log mode after switch and print separator
    print_current_mode(px4, prefix="[TEST][AFTER]")
    print("-" * 60)


def test_position_setpoint(px4):
    """Test sending a local position setpoint"""
    print("[TEST] Testing position setpoint...")

    # fetch current location; skip test if unavailable
    loc = px4.get_location()
    if not loc:
        print("[TEST] Cannot test position setpoint because local position is unavailable")
        print("-" * 60)
        return

    # hold close to current position
    target_x = loc["x"]
    target_y = loc["y"]
    target_z = loc["z"]

    # send setpoint to current position and log result
    success = px4.send_position_setpoint(target_x, target_y, target_z)
    print(f"[TEST] send_position_setpoint({target_x}, {target_y}, {target_z}) -> {success}")
    print("-" * 60)


def test_velocity_setpoint(px4, duration_sec=3.0):
    """Test sending zero velocity setpoints continuously"""
    print("[TEST] Testing velocity setpoint stream...")
    print("[TEST] Sending zero velocity for a few seconds")

    # track elapsed time and successful publish count
    start = time.time()
    success_count = 0

    # stream zero velocity setpoints for duration_sec, counting successes
    while time.time() - start < duration_sec:
        rclpy.spin_once(px4, timeout_sec=0.1)
        success = px4.send_velocity_setpoint(0.0, 0.0, 0.0, 0.0)
        if success:
            success_count += 1
        time.sleep(0.1)

    # log total successful publishes and print separator
    print(f"[TEST] Velocity setpoint publish count: {success_count}")
    print("-" * 60)


def test_start_offboard(px4):
    """Test offboard warmup + mode switch"""
    print("[TEST] Testing start_offboard()...")

    
    print_current_mode(px4, prefix="[TEST][BEFORE]")
    success = px4.start_offboard()
    print(f"[TEST] start_offboard() -> {success}")

    # spin a few times so state message has time to update
    for _ in range(10):
        rclpy.spin_once(px4, timeout_sec=0.1)
        time.sleep(0.1)

    # log mode after switch and print separator
    print_current_mode(px4, prefix="[TEST][AFTER]")
    print("-" * 60)


def test_arm(px4):
    """Test arming the vehicle"""
    print("[TEST] Testing arm_vehicle()...")
    success = px4.arm_vehicle()
    print(f"[TEST] arm_vehicle() -> {success}")
    print("-" * 60)


def test_disarm(px4):
    """Test disarming the vehicle"""
    print("[TEST] Testing disarm_vehicle()...")
    success = px4.disarm_vehicle()
    print(f"[TEST] disarm_vehicle() -> {success}")
    print("-" * 60)


def test_hold_current_position(px4):
    """Test hold_current_position()"""
    print("[TEST] Testing hold_current_position()...")
    success = px4.hold_current_position()
    print(f"[TEST] hold_current_position() -> {success}")
    print("-" * 60)

def print_current_mode(px4, prefix="[MODE]"):
    """Print current flight mode"""
    if px4.current_state is None:
        print(f"{prefix} Current mode: <not available>")
    else:
        print(f"{prefix} Current mode: {px4.current_state.mode}")

def main():
    print("[MAIN] Starting PX4 setter test...")
    print("[MAIN] Assuming MAVROS is already running externally...")

    # initialize PX4 ROS2 node under the mavros namespace
    px4 = init_px4(namespace="mavros")

    # abort early if MAVROS connection failed; attempt clean shutdown
    if not px4.connected:
        print("[MAIN] Failed to connect to MAVROS")
        print("[MAIN] Make sure:")
        print("  1. MAVROS is already running in another terminal")
        print("  2. The drone / FCU is connected")
        print("  3. ros2 topic list shows /mavros/state")
        print("  4. The namespace is correct")
        try:
            px4.disconnect()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
        return

    print("[MAIN] Successfully connected to MAVROS")
    print("[MAIN] Waiting a few seconds to collect telemetry before testing setters...")

    # collect a little telemetry first
    warmup_start = time.time()
    while time.time() - warmup_start < 3.0:
        rclpy.spin_once(px4, timeout_sec=0.1)
        time.sleep(0.1)

    # snapshot telemetry health and current mode before running tests
    px4.print_telemetry_health()
    print_current_mode(px4, prefix="[MAIN]")

    # ---------------------------------------------------------
    # SAFE TESTS FIRST
    # ---------------------------------------------------------
    test_position_setpoint(px4)
    test_velocity_setpoint(px4, duration_sec=2.0)
    test_hold_current_position(px4)
    test_mode_change(px4)
    test_start_offboard(px4)

    # ---------------------------------------------------------
    # DANGEROUS TESTS
    # Uncomment only if safe to do so
    # ---------------------------------------------------------
    # test_arm(px4)
    # time.sleep(2.0)
    # test_disarm(px4)

    print("[MAIN] Setter test complete")
    px4.disconnect()

    #shutdown 
    if rclpy.ok():
        rclpy.shutdown()

    print("[MAIN] Shutdown complete")


if __name__ == "__main__":
    main()
