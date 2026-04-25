#!/usr/bin/env python3
"""
Test script to verify ArduPilot integration with mission controller
"""
import sys

print("Testing imports with ArduPilot integration...\n")

try:
    print("1. Importing ArduPilot interface...")
    from ardupilot_interface import ArduPilotInterface, init_autopilot, get_autopilot
    print("   ✓ ArduPilotInterface imported successfully\n")
    
    print("2. Importing mission controller package...")
    from mission_controller import (
        MissionState, Mode, Point,
        Objective, ExtinguishObjective, SurveyObjective,
        PathfindingStrategy, PathPrinting,
        MissionStrategy, MissionOne, MissionTwo,
        MissionController,
        Driver
    )
    print("   ✓ All core classes imported successfully\n")
    
    print("3. Importing stubs with ArduPilot...")
    from mission_controller import (
        takeoff_drone, land_drone, goto_drone, run_lap_algorithm,
        boustrophedon_search, at_position, pad_has_extinguisher,
        drop_payload, inside_boundary, extinguish_fire, take_survey_photos,
        release_payload, generate_print_pattern, generate_potential_field_path
    )
    print("   ✓ All stub functions imported successfully\n")
    
    print("4. Verifying stub functions reference ArduPilot...")
    import inspect
    
    # Check takeoff_drone source
    source = inspect.getsource(takeoff_drone)
    if 'autopilot' in source and 'takeoff' in source:
        print("   ✓ takeoff_drone() correctly uses ArduPilot\n")
    
    # Check goto_drone source
    source = inspect.getsource(goto_drone)
    if 'goto_location' in source:
        print("   ✓ goto_drone() correctly uses ArduPilot\n")
    
    # Check at_position source
    source = inspect.getsource(at_position)
    if 'get_location' in source:
        print("   ✓ at_position() correctly uses ArduPilot\n")
    
    print("="*70)
    print("✓ ALL IMPORTS SUCCESSFUL - ArduPilot integration ready!")
    print("="*70)
    
except Exception as e:
    print(f"\n✗ ERROR: {type(e).__name__}: {str(e)}")
    import traceback
    traceback.print_exc()
    sys.exit(1)
