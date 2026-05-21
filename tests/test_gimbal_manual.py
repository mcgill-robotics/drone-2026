"""
test_gimbal_manual.py

Purpose:
--------
Manual gimbal control using keyboard arrow keys through the API server.

This script does NOT directly control PX4 or the servo motors.

Instead, the flow is:

    keyboard arrow keys
        -> test_gimbal_manual.py
        -> HTTP POST /gimbal/set
        -> api_server.py
        -> px4_interface.set_gimbal()
        -> MAVROS CommandLong
        -> PX4 servo outputs
        -> two gimbal servos

Controls:
---------
LEFT arrow:
    decrease yaw PWM

RIGHT arrow:
    increase yaw PWM

UP arrow:
    increase pitch PWM

DOWN arrow:
    decrease pitch PWM

SPACE:
    return both servos to neutral

q:
    center gimbal and quit

Run order:
----------
Terminal 1:
    python3 api_server.py --no-boot

Terminal 2:
    python3 tests/test_gimbal_manual.py

Safety:
-------
Start with propellers removed if possible.
Start with small STEP_PWM values.
Confirm servo direction before increasing movement range.
"""

import curses
import time
import requests


# ============================================================
# API CONFIGURATION
# ============================================================

# If running on the same machine as api_server.py:
API_BASE_URL = "http://127.0.0.1:5000"

# If running from another laptop, replace with Jetson IP:
#
# API_BASE_URL = "http://192.168.1.50:5000"


# ============================================================
# SERVO CHANNEL CONFIGURATION
# ============================================================

# PLACEHOLDER VALUES.
# Update these once wiring is confirmed.
YAW_CHANNEL = 8
PITCH_CHANNEL = 9


# ============================================================
# PWM CONFIGURATION
# ============================================================

# Standard neutral value for many servos.
NEUTRAL_PWM = 1500

# Conservative safety range.
# Later you may narrow or widen these after testing.
MIN_PWM = 1000
MAX_PWM = 2000

# Amount changed per arrow-key press.
# Start small for safety.
STEP_PWM = 25


def clamp(value, min_value, max_value):
    """
    Keep a PWM value inside the allowed safety range.
    """

    return max(min_value, min(max_value, value))


def send_gimbal(yaw_pwm, pitch_pwm):
    """
    Send yaw/pitch PWM values to the API server.

    Returns:
        True if API request succeeded.
        False otherwise.
    """

    payload = {
        "yaw_pwm": yaw_pwm,
        "pitch_pwm": pitch_pwm,
        "yaw_channel": YAW_CHANNEL,
        "pitch_channel": PITCH_CHANNEL,
    }

    try:
        response = requests.post(
            f"{API_BASE_URL}/gimbal/set",
            json=payload,
            timeout=2,
        )

        return response.ok

    except requests.RequestException:
        return False


def run_keyboard_control(screen):
    """
    Main keyboard-control loop.

    This function continuously listens for keyboard input.
    The script stays running until the user presses q.
    """

    # Do not print typed characters to terminal.
    curses.noecho()

    # React to key presses immediately.
    curses.cbreak()

    # Allow curses to recognize arrow keys.
    screen.keypad(True)

    # Non-blocking input:
    # getch() returns -1 if no key is pressed.
    screen.nodelay(True)

    # Start both servos at neutral.
    yaw_pwm = NEUTRAL_PWM
    pitch_pwm = NEUTRAL_PWM

    # Send initial center command.
    last_ok = send_gimbal(yaw_pwm, pitch_pwm)

    while True:
        screen.clear()

        screen.addstr(0, 0, "Manual Gimbal Control through API")
        screen.addstr(2, 0, "Controls:")
        screen.addstr(3, 0, "  LEFT  arrow : yaw PWM -")
        screen.addstr(4, 0, "  RIGHT arrow : yaw PWM +")
        screen.addstr(5, 0, "  UP    arrow : pitch PWM +")
        screen.addstr(6, 0, "  DOWN  arrow : pitch PWM -")
        screen.addstr(7, 0, "  SPACE       : center")
        screen.addstr(8, 0, "  q           : center and quit")

        screen.addstr(10, 0, f"Yaw PWM:   {yaw_pwm}")
        screen.addstr(11, 0, f"Pitch PWM: {pitch_pwm}")
        screen.addstr(13, 0, f"Last API request OK: {last_ok}")

        key = screen.getch()

        should_send = False

        if key == curses.KEY_LEFT:
            yaw_pwm -= STEP_PWM
            should_send = True

        elif key == curses.KEY_RIGHT:
            yaw_pwm += STEP_PWM
            should_send = True

        elif key == curses.KEY_UP:
            pitch_pwm += STEP_PWM
            should_send = True

        elif key == curses.KEY_DOWN:
            pitch_pwm -= STEP_PWM
            should_send = True

        elif key == ord(" "):
            yaw_pwm = NEUTRAL_PWM
            pitch_pwm = NEUTRAL_PWM
            should_send = True

        elif key == ord("q"):
            send_gimbal(NEUTRAL_PWM, NEUTRAL_PWM)
            break

        # Clamp after key update.
        yaw_pwm = clamp(yaw_pwm, MIN_PWM, MAX_PWM)
        pitch_pwm = clamp(pitch_pwm, MIN_PWM, MAX_PWM)

        if should_send:
            last_ok = send_gimbal(yaw_pwm, pitch_pwm)

        screen.refresh()

        # Prevent sending/checking too aggressively.
        time.sleep(0.05)


def main():
    """
    Start curses safely.

    curses.wrapper() restores the terminal state when the script exits.
    """

    curses.wrapper(run_keyboard_control)


if __name__ == "__main__":
    main()
