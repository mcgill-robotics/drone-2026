"""
test_gimbal_manual.py

Manual gimbal control using keyboard arrow keys.

Controls:
---------
LEFT  arrow : yaw left
RIGHT arrow : yaw right
UP    arrow : pitch up
DOWN  arrow : pitch down

SPACE : center gimbal
s     : stop gimbal
q     : quit

This is for early hardware bring-up only.
"""

import curses
import time

from mission_controller.gimbal_interface import (
    GimbalConfig,
    GimbalInterface,
)


# Amount changed per key press, in degrees
STEP_DEG = 5.0


def clamp(value, min_value, max_value):
    """
    Keep value inside safety limits.
    """
    return max(min_value, min(max_value, value))


def run_keyboard_control(screen):
    """
    Main keyboard-control loop.
    """

    # Do not show typed characters
    curses.noecho()

    # React to keys immediately
    curses.cbreak()

    # Enable arrow-key detection
    screen.keypad(True)

    # Non-blocking input:
    # If no key is pressed, getch() returns -1
    screen.nodelay(True)

    config = GimbalConfig(
        # Placeholder port.
        # Change this later after checking Jetson port.
        port="/dev/ttyUSB0",
        baudrate=115200,
    )

    gimbal = GimbalInterface(config)

    if not gimbal.connect():
        screen.addstr(0, 0, "Failed to connect to gimbal board.")
        screen.refresh()
        time.sleep(2)
        return

    yaw = 0.0
    pitch = 0.0

    try:
        while True:
            screen.clear()

            screen.addstr(0, 0, "Manual Gimbal Keyboard Control")
            screen.addstr(2, 0, "Controls:")
            screen.addstr(3, 0, "  LEFT  arrow : yaw left")
            screen.addstr(4, 0, "  RIGHT arrow : yaw right")
            screen.addstr(5, 0, "  UP    arrow : pitch up")
            screen.addstr(6, 0, "  DOWN  arrow : pitch down")
            screen.addstr(7, 0, "  SPACE       : center")
            screen.addstr(8, 0, "  s           : stop")
            screen.addstr(9, 0, "  q           : quit")

            screen.addstr(11, 0, f"Current yaw:   {yaw:.2f} deg")
            screen.addstr(12, 0, f"Current pitch: {pitch:.2f} deg")

            key = screen.getch()

            if key == curses.KEY_LEFT:
                yaw -= STEP_DEG

            elif key == curses.KEY_RIGHT:
                yaw += STEP_DEG

            elif key == curses.KEY_UP:
                pitch += STEP_DEG

            elif key == curses.KEY_DOWN:
                pitch -= STEP_DEG

            elif key == ord(" "):
                yaw = 0.0
                pitch = 0.0
                gimbal.center()

            elif key == ord("s"):
                gimbal.stop()

            elif key == ord("q"):
                break

            # Clamp yaw/pitch to software safety limits
            yaw = clamp(
                yaw,
                config.min_yaw_deg,
                config.max_yaw_deg,
            )

            pitch = clamp(
                pitch,
                config.min_pitch_deg,
                config.max_pitch_deg,
            )

            # Send updated angles only when an arrow key was pressed
            if key in [
                curses.KEY_LEFT,
                curses.KEY_RIGHT,
                curses.KEY_UP,
                curses.KEY_DOWN,
            ]:
                gimbal.set_angles(
                    yaw_deg=yaw,
                    pitch_deg=pitch,
                )

            screen.refresh()

            # Small delay to avoid sending commands too fast
            time.sleep(0.05)

    finally:
        gimbal.disconnect()


def main():
    curses.wrapper(run_keyboard_control)


if __name__ == "__main__":
    main()
