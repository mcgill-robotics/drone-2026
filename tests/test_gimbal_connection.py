"""
test_gimbal_connection.py

Purpose:
--------
Basic API-based gimbal bring-up test.

This script does NOT directly talk to the servo motors.

Instead, the flow is:

    test_gimbal_connection.py
        -> HTTP POST request
        -> api_server.py
        -> px4_interface.set_gimbal()
        -> PX4/MAVROS servo command
        -> PX4 servo output channels
        -> two gimbal servos

Use this script to verify:
--------------------------
1. api_server.py is running
2. /gimbal/set endpoint works
3. PX4/MAVROS accepts servo commands
4. The yaw servo responds
5. The pitch servo responds

Run order:
----------
Terminal 1:
    python3 api_server.py --no-boot

Terminal 2:
    python3 tests/test_gimbal_connection.py
"""

import time
import requests


# ============================================================
# API CONFIGURATION
# ============================================================

# If the API server runs on the same machine:
API_BASE_URL = "http://127.0.0.1:5000"

# If the API server runs on the Jetson and this script runs from another laptop,
# replace this with the Jetson IP address, for example:
#
# API_BASE_URL = "http://192.168.1.50:5000"


# ============================================================
# SERVO CHANNEL CONFIGURATION
# ============================================================

# PLACEHOLDER VALUES.
# These must be updated after checking the actual PX4 output wiring.
YAW_CHANNEL = 8
PITCH_CHANNEL = 9


# ============================================================
# PWM CONFIGURATION
# ============================================================

# 1500 is normally neutral/center for standard servos.
NEUTRAL_PWM = 1500

# Step used in this automatic test.
# Start small for safety.
STEP_PWM = 100


def send_gimbal(yaw_pwm, pitch_pwm):
    """
    Send one gimbal command to the API server.

    yaw_pwm:
        PWM value for yaw servo.

    pitch_pwm:
        PWM value for pitch servo.
    """

    payload = {
        "yaw_pwm": yaw_pwm,
        "pitch_pwm": pitch_pwm,
        "yaw_channel": YAW_CHANNEL,
        "pitch_channel": PITCH_CHANNEL,
    }

    response = requests.post(
        f"{API_BASE_URL}/gimbal/set",
        json=payload,
        timeout=5,
    )

    print("Status:", response.status_code)
    print("Response:", response.json())

    return response.ok


def main():
    """
    Run a simple automatic movement sequence.

    This is useful for first bring-up because it tests each direction once.
    """

    print("\n--- Center gimbal ---")
    send_gimbal(NEUTRAL_PWM, NEUTRAL_PWM)
    time.sleep(1)

    print("\n--- Yaw one direction ---")
    send_gimbal(NEUTRAL_PWM + STEP_PWM, NEUTRAL_PWM)
    time.sleep(1)

    print("\n--- Yaw opposite direction ---")
    send_gimbal(NEUTRAL_PWM - STEP_PWM, NEUTRAL_PWM)
    time.sleep(1)

    print("\n--- Pitch one direction ---")
    send_gimbal(NEUTRAL_PWM, NEUTRAL_PWM + STEP_PWM)
    time.sleep(1)

    print("\n--- Pitch opposite direction ---")
    send_gimbal(NEUTRAL_PWM, NEUTRAL_PWM - STEP_PWM)
    time.sleep(1)

    print("\n--- Return to center ---")
    send_gimbal(NEUTRAL_PWM, NEUTRAL_PWM)

    print("\nGimbal connection test complete.")


if __name__ == "__main__":
    main()
