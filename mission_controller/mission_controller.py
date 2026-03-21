"""
Main mission controller entry point with example usage
This file demonstrates how to use the mission controller system with ArduPilot
"""

from mission_controller import (
    Driver, MissionOne, MissionTwo, Point,
    ExtinguishObjective, SurveyObjective, PayloadDeliveryObjective,
    init_autopilot, get_autopilot
)


def main():
    """
    Example of how to use the mission controller system
    
    For real hardware operations:
    1. Initialize ArduPilot connection (WiFi, serial, or UDP telemetry)
    2. Create mission with strategy
    3. Execute mission
    4. Monitor telemetry
    5. Export logs
    
    For SITL (Software-In-The-Loop) testing:
    - Connect via UDP: "udp:127.0.0.1:14550" (default)
    
    For real hardware:
    - Via serial: "/dev/ttyUSB0" or "/dev/ttyAMA0" 
    - Via WiFi/UDP: "udp:192.168.1.100:14550"
    """
    
    # ================================================================
    # Initialize ArduPilot Connection
    # ================================================================
    print("\n" + "="*70)
    print("INITIALIZING ARDUPILOT CONNECTION")
    print("="*70 + "\n")
    
    # For SITL (Software-In-The-Loop) simulator:
    # autopilot = init_autopilot("udp:127.0.0.1:14550")
    
    # For real hardware via UDP telemetry:
    # autopilot = init_autopilot("udp:192.168.1.100:14550")
    
    # For real hardware via serial:
    # autopilot = init_autopilot("/dev/ttyUSB0")
    
    # Currently using SITL for example
    autopilot = init_autopilot("udp:127.0.0.1:14550")
    
    if not autopilot:
        print("[ERROR] Could not connect to autopilot")
        print("[INFO] Make sure ArduPilot is running or SITL is available")
        print("[INFO] For SITL: sim_vehicle.py -v ArduCopter --console")
        return
    
    # ================================================================
    # Create driver
    # ================================================================
    driver = Driver()
    
    # Define mission parameters
    home = Point(0, 0, 0)
    site = Point(100, 100, 50)
    boundary = ((-50, 150), (-50, 150))  # Example boundary
    
    # ================================================================
    # Mission One - Lap-based search mission
    # ================================================================
    print("\n" + "="*70)
    print("CREATING MISSION ONE - LAP-BASED SEARCH")
    print("="*70 + "\n")
    
    mission_one = driver.create_mission(
        mission_id=1,
        site_gps=site,
        mission_boundary=boundary,
        home_position=home,
        num_laps=3
    )
    
    # ================================================================
    # Mission Two - Water/Extinguisher based mission
    # ================================================================
    print("\n" + "="*70)
    print("CREATING MISSION TWO - WATER-BASED EXTINGUISHING")
    print("="*70 + "\n")
    
    water_strategy = MissionTwo(boundary, water_tank_capacity=50.0)
    mission_two = driver.create_mission(
        mission_id=2,
        site_gps=site,
        mission_boundary=boundary,
        home_position=home,
        num_laps=2,
        strategy=water_strategy
    )
    
    # Add fire extinguishing objectives
    water_strategy.add_objective(ExtinguishObjective(Point(110, 110, 50)))
    water_strategy.add_objective(ExtinguishObjective(Point(120, 120, 50)))
    
    # ================================================================
    # Example Survey Mission
    # ================================================================
    print("\n" + "="*70)
    print("CREATING SURVEY OBJECTIVE")
    print("="*70 + "\n")
    
    survey_obj = SurveyObjective(Point(100, 100, 25))
    mission_one.add_objective(survey_obj)
    
    # ================================================================
    # List all missions
    # ================================================================
    print("\n" + "="*70)
    print("REGISTERED MISSIONS")
    print("="*70 + "\n")
    driver.list_missions()
    
    # ================================================================
    # Telemetry Check
    # ================================================================
    print("\n" + "="*70)
    print("CHECKING AUTOPILOT TELEMETRY")
    print("="*70 + "\n")
    
    if autopilot.is_armed():
        print("[INFO] Vehicle is ARMED")
    else:
        print("[INFO] Vehicle is NOT armed (OK for ground testing)")
    
    location = autopilot.get_location()
    if location:
        print(f"[INFO] Vehicle location: {location['lat']:.6f}, {location['lon']:.6f}")
        print(f"[INFO] Altitude: {location['alt']:.1f}m (relative: {location['relative_alt']:.1f}m)")
    
    battery = autopilot.get_battery_status()
    if battery:
        print(f"[INFO] Battery: {battery['voltage']:.1f}V, {battery['current']:.1f}A, {battery['percentage']:.0f}%")
    
    # ================================================================
    # Execute Mission One
    # ================================================================
    print("\n" + "="*70)
    print("EXECUTING MISSION ONE")
    print("="*70 + "\n")
    print("[INFO] Starting mission execution...")
    print("[INFO] All flight commands will use ArduPilot via pymavlink")
    print("[INFO] Flight modes: GUIDED, AUTO, LAND, RTL")
    
    driver.start_mission(1)
    
    # ================================================================
    # Execute Mission Two (uncomment to run)
    # ================================================================
    # print("\n" + "="*70)
    # print("EXECUTING MISSION TWO")
    # print("="*70 + "\n")
    # driver.start_mission(2)
    
    # ================================================================
    # Export mission logs
    # ================================================================
    print("\n" + "="*70)
    print("EXPORTING MISSION LOGS")
    print("="*70 + "\n")
    
    driver.export_logs("mission_logs.json")
    
    # Cleanup
    if autopilot:
        autopilot.disconnect()
    
    print("\n" + "="*70)
    print("MISSION EXECUTION COMPLETE")
    print("="*70 + "\n")


if __name__ == "__main__":
    main()

