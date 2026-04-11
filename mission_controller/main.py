
import rclpy
from ardupilot_interface import init_autopilot

def main():
    print("[MAIN] Starting ArduPilot interface...")
    
    # Initialize and connect to MAVROS
    autopilot = init_autopilot(namespace="mavros")
    
    # Check if connected
    if autopilot.connected:
        print("[MAIN] Successfully connected to MAVROS")
        print(f"[MAIN] Vehicle armed: {autopilot.is_armed()}")
        print(f"[MAIN] Current altitude: {autopilot.get_altitude()}")
        
        # Example: arm and takeoff
        # autopilot.arm_vehicle()
        # autopilot.takeoff(altitude=10)
        
    else:
        print("[MAIN] Failed to connect to MAVROS")
        print("[MAIN] Make sure MAVROS is running!")
    
    # Cleanup
    autopilot.disconnect()
    rclpy.shutdown()

if __name__ == "__main__":
    main()