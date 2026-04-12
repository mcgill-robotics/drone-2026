
import rclpy
from ardupilot_interface import init_px4

def main():
    print("[MAIN] Starting PX4 interface...")
    
    # Initialize and connect to MAVROS
    px4 = init_px4(namespace="mavros")
    
    # Check if connected
    if px4.connected:
        print("[MAIN] Successfully connected to MAVROS")
        print(f"[MAIN] Vehicle armed: {px4.is_armed()}")
        print(f"[MAIN] Current altitude: {px4.get_altitude()}")
        
        # Example: arm and takeoff
        # px4.arm_vehicle()
        # px4.takeoff(altitude=10)
        
    else:
        print("[MAIN] Failed to connect to MAVROS")
        print("[MAIN] Make sure MAVROS is running!")
    
    # Cleanup
    px4.disconnect()
    rclpy.shutdown()

if __name__ == "__main__":
    main()