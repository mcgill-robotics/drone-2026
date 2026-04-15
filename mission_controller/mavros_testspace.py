
import rclpy
import time
from px4_interface import init_px4, boot_px4, shutdown_px4

def print_telemetry(px4):
    """Print all available telemetry from PX4"""
    print("[TELEMETRY] Current Status")
    
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
    
    # GPS (RTK if available)
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
    
    print("="*60 + "\n")

def main():
    print("[MAIN] Starting PX4 interface test...")
    boot_px4();
    # Initialize and connect to MAVROS
    px4 = init_px4(namespace="mavros")
    
    # Check if connected
    if not px4.connected:
        print("[MAIN] Failed to connect to MAVROS")
        print("[MAIN] Make sure MAVROS is running!")
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
            print_telemetry(px4)
        
        time.sleep(0.1)
    
    print("[MAIN] Test complete. Disconnecting...")
    
    # Final telemetry print
    print_telemetry(px4)
    
    # Cleanup
    px4.disconnect()
    shutdown_px4()
    rclpy.shutdown()
    print("[MAIN] Shutdown complete")

if __name__ == "__main__":
    main()