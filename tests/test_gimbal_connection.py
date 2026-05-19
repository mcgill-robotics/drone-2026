"""
test_gimbal_connection.py

Purpose:
--------
Basic hardware bring-up test.

Use this BEFORE integrating with:
---------------------------------
- PX4
- MAVROS
- mission controller
- autonomy logic
- object detection

Current goal:
-------------
Verify:
- serial connection works
- commands are transmitted
- motors respond correctly

IMPORTANT:
-----------
Run WITHOUT propellers / dangerous hardware states
during early testing whenever possible.
"""

from mission_controller.gimbal_interface import (
    GimbalConfig,
    GimbalInterface,
)


def main():

    # ========================================================
    # CONFIGURATION
    # ========================================================

    # IMPORTANT:
    # Update port after identifying actual device.
    #
    # Useful Jetson commands:
    #
    #   ls /dev/ttyUSB*
    #   ls /dev/ttyACM*
    #   dmesg -w
    #
    config = GimbalConfig(
        port="/dev/ttyUSB0",
        baudrate=115200,
    )

    # ========================================================
    # CREATE INTERFACE
    # ========================================================

    gimbal = GimbalInterface(config)

    # ========================================================
    # CONNECT
    # ========================================================

    if not gimbal.connect():
        print("Failed to connect to gimbal board")
        return

    try:

        # ====================================================
        # BASIC TESTS
        # ====================================================

        print("\n--- Center ---")
        gimbal.center()

        print("\n--- Yaw Right ---")
        gimbal.set_angles(
            yaw_deg=20,
            pitch_deg=0,
        )

        print("\n--- Yaw Left ---")
        gimbal.set_angles(
            yaw_deg=-20,
            pitch_deg=0,
        )

        print("\n--- Pitch Up ---")
        gimbal.set_angles(
            yaw_deg=0,
            pitch_deg=10,
        )

        print("\n--- Pitch Down ---")
        gimbal.set_angles(
            yaw_deg=0,
            pitch_deg=-10,
        )

        print("\n--- Return Center ---")
        gimbal.center()

        print("\n--- Stop ---")
        gimbal.stop()

    finally:

        # ====================================================
        # DISCONNECT
        # ====================================================

        gimbal.disconnect()


if __name__ == "__main__":
    main()
