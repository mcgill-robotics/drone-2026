#!/usr/bin/env python3
"""
Test script to validate boot_px4() and MAVROS launch

This script:
1. Calls boot_px4() to launch MAVROS
2. Waits for MAVROS to be ready
3. Validates that ROS 2 topics exist
4. Attempts to initialize PX4 interface
5. Checks connection status
"""

import sys
import os
import time
import subprocess

# Add parent directory to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import rclpy
from mission_controller.px4_interface import boot_px4, init_px4, stop_px4


#this runs ros2 topic list, extract all the topics and sees if they have mavros in them
def check_ros_topics(timeout=30):
    """Check if MAVROS topics are being published"""
    print("\n[VALIDATION] Checking for MAVROS ROS 2 topics...")
    
    start_time = time.time()
    while (time.time() - start_time) < timeout:
        try:
            # Try to list all topics
            result = subprocess.run(
                ["ros2", "topic", "list"],
                capture_output=True,
                text=True,
                timeout=5
            )
            
            topics = result.stdout.split('\n')
            
            # Check for key MAVROS topics
            required_topics = [
                '/mavros/state',
                '/mavros/local_position/pose',
                '/mavros/battery',
            ]
            
            found_topics = [t for t in required_topics if any(t in topic for topic in topics)]
            
            if len(found_topics) > 0:
                print(f"Found {len(found_topics)} MAVROS topics:")
                for topic in found_topics:
                    print(f"              - {topic}")
                return True
            else:
                elapsed = int(time.time() - start_time)
                print(f"Waiting for topics... ({elapsed}s)")
                time.sleep(2)
                
        except Exception as e:
            print(f"Error checking topics: {e}")
            time.sleep(2)
    
    print("No MAVROS topics found within timeout")
    return False


def check_px4_connection(timeout=30):
    """Check if PX4 interface can connect to MAVROS"""
    print("\n[VALIDATION] Checking PX4 interface connection...")
    
    if not rclpy.ok():
        rclpy.init()
    
    start_time = time.time()
    while (time.time() - start_time) < timeout:
        try:
            px4 = init_px4()
            time.sleep(1)  # Give it a moment to connect
            
            if px4 and px4.connected:
                print("PX4 interface connected")
                return True
            else:
                elapsed = int(time.time() - start_time)
                print(f"Waiting for connection... ({elapsed}s)")
                time.sleep(3)
                
        except Exception as e:
            elapsed = int(time.time() - start_time)
            print(f"Connection attempt failed ({elapsed}s): {str(e)}")
            time.sleep(2)
    
    print("Could not connect PX4 interface within timeout")
    return False


def main():
    print("\n" + "="*70)
    print("MAVROS BOOT VALIDATION TEST")
    print("="*70 + "\n")
    
    fcu_url = "serial:///dev/ttyTHS1:921600"
    print(f"[MAIN] Booting MAVROS with FCU URL: {fcu_url}")
    
    try:
        # Step 1: Boot MAVROS
        print("\n[STEP 1] Launching MAVROS...")
        px4_process = boot_px4(fcu_url=fcu_url)
        
        if px4_process is None:
            print("[STEP 1] Failed to start MAVROS process")
            return False
        
        print(f"MAVROS process started (PID: {px4_process.pid})")
        
        # Step 2: Wait for process to stabilize
        print("\n[STEP 2] Waiting for MAVROS to initialize...")
        time.sleep(10)
        
        # Check if process is still running
        if px4_process.poll() is not None:
            print(f"MAVROS process crashed (exit code: {px4_process.poll()})")
            print("Check MAVROS output for errors")
            return False
        
        # Step 3: Check ROS 2 topics, making sure they are there
        print("\nValidating MAVROS topics...")
        if not check_ros_topics(timeout=30):
            print("[MAVROS topics not found")
            return False
        print("MAVROS topics validated")
        
        # Step 4: Check PX4 interface connection
        print("\nValidating PX4 interface...")
        if not check_px4_connection(timeout=30):
            print("PX4 interface connection failed")
            return False
        print("PX4 interface connected")
        
        # Success
        print("\n" + "="*70)
        print("✓ ALL VALIDATIONS PASSED - MAVROS BOOT SUCCESSFUL")
        print("="*70 + "\n")
        
        return True
        
    except Exception as e:
        print(f"\n[MAIN] Exception: {str(e)}")
        import traceback
        traceback.print_exc()
        return False
    
    finally:
        print("\n[CLEANUP] Stopping MAVROS...")
        try:
            stop_px4()
            print("[CLEANUP] MAVROS stopped")
        except Exception as e:
            print(f"[CLEANUP] Error stopping MAVROS: {e}")
        
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == "__main__":
    success = main()
    exit(0 if success else 1)
