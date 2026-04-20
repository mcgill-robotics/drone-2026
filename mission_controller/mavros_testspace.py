import rclpy
import time
from px4_interface import init_px4, boot_px4, stop_px4
from px4_getters import get_jetson_ip


def print_jetson_info():
    """
    Print Jetson IP address at the very beginning of testspace execution.
    This retrieves the IP using the get_jetson_ip() function from px4_getters.
    """
    ip_address = get_jetson_ip()
    print("=" * 60)
    print("[JETSON INFO]")
    print(f"IP Address: {ip_address}")
    print("=" * 60)
    print()


def print_basic_telemetry(px4):
    """Print basic telemetry from PX4"""
    print("[TELEMETRY] Basic Status")
    
    # Basic state
    print(f"Connected:     {px4.connected}")
    print(f"Armed:         {px4.is_armed()}")
    
    # Local position (NED frame)
    loc = px4.get_location()
    if loc:
        print(f"Position (local):  X={loc['x']:.2f}m, Y={loc['y']:.2f}m, Z={loc['z']:.2f}m")
    else:
        print("Position (local):  <not available>")
    
    # Altitude
    alt = px4.get_altitude()
    print(f"Altitude:      {alt:.2f}m")
    
    # Battery
    battery = px4.get_battery_status()
    if battery:
        print(f"Battery:       {battery['percentage']:.1f}% ({battery['voltage']:.2f}V, {battery['current']:.2f}A)")
    else:
        print("Battery:       <not available>")
    
    # GPS
    gps = px4.get_gps_location()
    if gps:
        print(f"GPS Position:  Lat={gps['latitude']:.6f}°, Lon={gps['longitude']:.6f}°, Alt={gps['altitude']:.2f}m")
    else:
        print("GPS Position:  <not available>")
    
    # Velocity
    velocity = px4.get_velocity()
    if velocity:
        print(f"Velocity:      X={velocity['x']:.2f}m/s, Y={velocity['y']:.2f}m/s, Z={velocity['z']:.2f}m/s")
    else:
        print("Velocity:      <not available>")
    
    # Home position
    home = px4.get_home_location()
    if home:
        print(f"Home Position: Lat={home['latitude']:.6f}°, Lon={home['longitude']:.6f}°, Alt={home['altitude']:.2f}m")
    else:
        print("Home Position: <not available>")
    
    # Landed state
    landed = px4.is_landed()
    print(f"Landed:        {landed}")
    
    print("=" * 60)


def print_integrated_telemetry(px4):
    """Print all telemetry from the integrated telemetry APIs"""
    print("[TELEMETRY] Integrated Telemetry APIs")

    # Pose
    pose = px4.get_pose()
    if pose:
        print("[POSE]")
        if pose["position"]:
            p = pose["position"]
            print(f"  Position:            x={p['x']}, y={p['y']}, z={p['z']}")
        else:
            print("  Position:            <not available>")

        if pose["orientation_quaternion"]:
            q = pose["orientation_quaternion"]
            print(f"  Quaternion:          x={q['x']}, y={q['y']}, z={q['z']}, w={q['w']}")
        else:
            print("  Quaternion:          <not available>")

        if pose["orientation_euler_rad"]:
            e = pose["orientation_euler_rad"]
            print(f"  Euler (rad):         roll={e['roll']}, pitch={e['pitch']}, yaw={e['yaw']}")
        else:
            print("  Euler (rad):         <not available>")
    else:
        print("[POSE] <not available>")

    # Odometry
    odom = px4.get_odometry()
    if odom:
        print("[ODOMETRY]")
        if odom["position"]:
            p = odom["position"]
            print(f"  Position:            x={p['x']}, y={p['y']}, z={p['z']}")
        else:
            print("  Position:            <not available>")

        if odom["linear_velocity"]:
            lv = odom["linear_velocity"]
            print(f"  Linear velocity:     x={lv['x']}, y={lv['y']}, z={lv['z']}")
        else:
            print("  Linear velocity:     <not available>")

        if odom["angular_velocity"]:
            av = odom["angular_velocity"]
            print(f"  Angular velocity:    x={av['x']}, y={av['y']}, z={av['z']}")
        else:
            print("  Angular velocity:    <not available>")
    else:
        print("[ODOMETRY] <not available>")

    # Velocity local
    vel_local = px4.get_velocity_local()
    if vel_local:
        print("[VELOCITY LOCAL]")
        if vel_local["linear"]:
            lv = vel_local["linear"]
            print(f"  Linear:              x={lv['x']}, y={lv['y']}, z={lv['z']}")
        else:
            print("  Linear:              <not available>")

        if vel_local["angular"]:
            av = vel_local["angular"]
            print(f"  Angular:             x={av['x']}, y={av['y']}, z={av['z']}")
        else:
            print("  Angular:             <not available>")
    else:
        print("[VELOCITY LOCAL] <not available>")

    # Velocity body
    vel_body = px4.get_velocity_body()
    if vel_body:
        print("[VELOCITY BODY]")
        if vel_body["linear"]:
            lv = vel_body["linear"]
            print(f"  Linear:              x={lv['x']}, y={lv['y']}, z={lv['z']}")
        else:
            print("  Linear:              <not available>")

        if vel_body["angular"]:
            av = vel_body["angular"]
            print(f"  Angular:             x={av['x']}, y={av['y']}, z={av['z']}")
        else:
            print("  Angular:             <not available>")
    else:
        print("[VELOCITY BODY] <not available>")

    # Acceleration / IMU
    acc = px4.get_acceleration()
    if acc:
        print("[ACCELERATION / IMU]")
        if acc["linear_acceleration"]:
            la = acc["linear_acceleration"]
            print(f"  Linear acceleration: x={la['x']}, y={la['y']}, z={la['z']}")
        else:
            print("  Linear acceleration: <not available>")

        if acc["angular_velocity"]:
            av = acc["angular_velocity"]
            print(f"  Angular velocity:    x={av['x']}, y={av['y']}, z={av['z']}")
        else:
            print("  Angular velocity:    <not available>")

        if acc["orientation_euler_rad"]:
            e = acc["orientation_euler_rad"]
            print(f"  Orientation (rad):   roll={e['roll']}, pitch={e['pitch']}, yaw={e['yaw']}")
        else:
            print("  Orientation (rad):   <not available>")
    else:
        print("[ACCELERATION / IMU] <not available>")

    # Altitude
    alt = px4.get_altitude_data()
    if alt:
        print("[ALTITUDE]")
        print(f"  AMSL:                {alt['amsl']}")
        print(f"  Local:               {alt['local']}")
        print(f"  Relative:            {alt['relative']}")
        print(f"  Terrain:             {alt['terrain']}")
        print(f"  Bottom clearance:    {alt['bottom_clearance']}")
        print(f"  Monotonic:           {alt['monotonic']}")
    else:
        print("[ALTITUDE] <not available>")

    # GPS raw/fix
    gps_fix = px4.get_gps_raw_fix()
    if gps_fix:
        print("[GPS RAW FIX]")
        print(f"  Latitude:            {gps_fix['latitude']}")
        print(f"  Longitude:           {gps_fix['longitude']}")
        print(f"  Altitude:            {gps_fix['altitude']}")
    else:
        print("[GPS RAW FIX] <not available>")

    # GPS raw 1
    gps_raw_1 = px4.get_gps_raw(1)
    if gps_raw_1:
        print("[GPS1 RAW]")
        print(f"  Fix type:            {gps_raw_1['fix_type']}")
        print(f"  Lat:                 {gps_raw_1['lat']}")
        print(f"  Lon:                 {gps_raw_1['lon']}")
        print(f"  Alt:                 {gps_raw_1['alt']}")
        print(f"  Satellites visible:  {gps_raw_1['satellites_visible']}")
        print(f"  EPH:                 {gps_raw_1['eph']}")
        print(f"  EPV:                 {gps_raw_1['epv']}")
        print(f"  Velocity:            {gps_raw_1['vel']}")
        print(f"  COG:                 {gps_raw_1['cog']}")
    else:
        print("[GPS1 RAW] <not available>")

    # GPS raw 2
    gps_raw_2 = px4.get_gps_raw(2)
    if gps_raw_2:
        print("[GPS2 RAW]")
        print(f"  Fix type:            {gps_raw_2['fix_type']}")
        print(f"  Lat:                 {gps_raw_2['lat']}")
        print(f"  Lon:                 {gps_raw_2['lon']}")
        print(f"  Alt:                 {gps_raw_2['alt']}")
        print(f"  Satellites visible:  {gps_raw_2['satellites_visible']}")
        print(f"  EPH:                 {gps_raw_2['eph']}")
        print(f"  EPV:                 {gps_raw_2['epv']}")
        print(f"  Velocity:            {gps_raw_2['vel']}")
        print(f"  COG:                 {gps_raw_2['cog']}")
    else:
        print("[GPS2 RAW] <not available>")

    # GPS velocity
    gps_vel = px4.get_gps_velocity()
    if gps_vel:
        print("[GPS VELOCITY]")
        if gps_vel["linear"]:
            lv = gps_vel["linear"]
            print(f"  Linear:              x={lv['x']}, y={lv['y']}, z={lv['z']}")
        else:
            print("  Linear:              <not available>")

        if gps_vel["angular"]:
            av = gps_vel["angular"]
            print(f"  Angular:             x={av['x']}, y={av['y']}, z={av['z']}")
        else:
            print("  Angular:             <not available>")
    else:
        print("[GPS VELOCITY] <not available>")

    # RTK 1
    rtk_1 = px4.get_rtk_data(1)
    if rtk_1:
        print("[RTK 1]")
        for key, value in rtk_1.items():
            print(f"  {key}: {value}")
    else:
        print("[RTK 1] <not available>")

    # RTK 2
    rtk_2 = px4.get_rtk_data(2)
    if rtk_2:
        print("[RTK 2]")
        for key, value in rtk_2.items():
            print(f"  {key}: {value}")
    else:
        print("[RTK 2] <not available>")

    # RTK baseline
    rtk_baseline = px4.get_rtk_baseline()
    if rtk_baseline:
        print("[RTK BASELINE]")
        for key, value in rtk_baseline.items():
            print(f"  {key}: {value}")
    else:
        print("[RTK BASELINE] <not available>")

    print("=" * 60 + "\n")


def print_full_snapshot(px4):
    """Print one compact full snapshot object"""
    print("[TELEMETRY] Full Snapshot")
    snapshot = px4.get_full_telemetry_snapshot()
    for key, value in snapshot.items():
        print(f"{key}: {value}")
    print("=" * 60 + "\n")


def view_camera():
    """Display live camera feed from RealSense D455"""
    print("\n" + "="*70)
    print("CAMERA LIVE STREAM")
    print("="*70 + "\n")
    
    # Initialize ROS
    rclpy.init()
    
    # Initialize PX4 interface
    from px4_interface import PX4Getters
    px4 = PX4Getters()
    
    # Wait for camera frames
    print("[CAMERA] Waiting for camera frames...")
    for i in range(50):
        rclpy.spin_once(px4, timeout_sec=0.05)
        if px4.get_camera_frame() is not None:
            print("[CAMERA] ✓ Frames received!")
            break
    
    # Display camera
    print("[CAMERA] Press 'q' in the window to quit\n")
    px4.display_camera_frames()
    
    # Cleanup
    px4.disconnect()
    rclpy.shutdown()


def main():
    print("[MAIN] Starting PX4 interface test...")
    
    # Boot PX4 with SITL (simulation)
    # For hardware, use: boot_px4("serial:///dev/ttyUSB0:921600")
    print("[MAIN] Booting PX4...")
    # px4_proc = boot_px4()  # Uses default SITL URL: udp://127.0.0.1:14540
    px4_proc = None
    
    
    # Initialize and connect to MAVROS
    print("[MAIN] Initializing MAVROS interface...")
    px4 = init_px4(namespace="mavros")
    
    # Check if connected
    if not px4.connected:
        print("[MAIN] Failed to connect to MAVROS")
        print("[MAIN] Make sure:")
        print("  1. PX4 SITL is running or hardware is connected")
        print("  2. MAVROS is properly configured")
        print("[MAIN] Shutting down...")
        stop_px4()
        return
    
    print("[MAIN] Successfully connected to MAVROS")
    print("[MAIN] Collecting telemetry for 30 seconds")
    
    # Collect and display telemetry for 30 seconds
    start_time = time.time()
    iteration = 0
    
    while (time.time() - start_time) < 30:
        iteration += 1
        
        # Spin once to receive new messages
        rclpy.spin_once(px4, timeout_sec=0.1)
        
        # Print telemetry every 2 seconds
        if iteration % 20 == 0:
            print(f"[MAIN] Iteration {iteration}")
            px4.print_telemetry_health()
            print_basic_telemetry(px4)
            print_integrated_telemetry(px4)
        
        time.sleep(0.1)
    
    print("[MAIN] Test complete. Disconnecting...")
    
    # Final prints
    px4.print_telemetry_health()
    print_basic_telemetry(px4)
    print_integrated_telemetry(px4)
    print_full_snapshot(px4)
    
    # Cleanup
    px4.disconnect()
    stop_px4()
    rclpy.shutdown()
    print("[MAIN] Shutdown complete")


if __name__ == "__main__":
    boot_px4()
