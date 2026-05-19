"""
gimbal_interface.py

Purpose:
--------
Simple boilerplate interface for:

Jetson -> serial board -> 2-axis gimbal

This file is intentionally SIMPLE and HEAVILY COMMENTED.

Current goals:
--------------
- establish serial communication
- provide safe structure
- allow future expansion
- avoid hardcoding hardware assumptions
- keep debugging easy

This is NOT:
-------------
- a stabilization controller
- a PX4 controller
- a MAVLink implementation
- a tracking algorithm

At this stage we ONLY care about:
---------------------------------
"Can Jetson communicate with the gimbal board reliably?"

Future hardware may change:
---------------------------
- serial port
- baudrate
- protocol format
- motor driver
- board firmware

Therefore:
-----------
Keep everything modular and easy to modify.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional

import serial


# ============================================================
# CONFIGURATION CLASS
# ============================================================

@dataclass
class GimbalConfig:
    """
    Stores communication + safety settings.

    These values are placeholders and should be updated
    once the real hardware is finalized.
    """

    # --------------------------------------------------------
    # SERIAL SETTINGS
    # --------------------------------------------------------

    # Placeholder serial port.
    #
    # Possible future examples:
    #   /dev/ttyUSB0
    #   /dev/ttyUSB1
    #   /dev/ttyACM0
    #   /dev/ttyTHS1
    #
    # We intentionally do NOT assume the final port yet.
    port: str = "/dev/ttyUSB0"

    # Serial baudrate.
    # Must match the board firmware later.
    baudrate: int = 115200

    # Serial timeout in seconds.
    timeout: float = 1.0

    # --------------------------------------------------------
    # SOFTWARE SAFETY LIMITS
    # --------------------------------------------------------

    # These are software angle clamps.
    # They should eventually match physical hardware limits.

    min_yaw_deg: float = -90.0
    max_yaw_deg: float = 90.0

    min_pitch_deg: float = -45.0
    max_pitch_deg: float = 45.0


# ============================================================
# GIMBAL INTERFACE
# ============================================================

class GimbalInterface:
    """
    Main Jetson ↔ gimbal interface.

    Responsibilities:
    -----------------
    - open serial connection
    - close serial connection
    - send commands
    - provide helper functions

    Non-responsibilities:
    ---------------------
    - mission planning
    - drone navigation
    - object tracking
    - stabilization
    - PX4 logic
    """

    def __init__(self, config: GimbalConfig):
        """
        Initialize interface.
        """

        self.config = config

        # Serial object.
        # None until connected.
        self.serial_conn: Optional[serial.Serial] = None

    # ========================================================
    # CONNECTION FUNCTIONS
    # ========================================================

    def connect(self) -> bool:
        """
        Attempt to connect to the serial board.

        Returns:
            True  -> success
            False -> failure
        """

        try:
            self.serial_conn = serial.Serial(
                port=self.config.port,
                baudrate=self.config.baudrate,
                timeout=self.config.timeout,
            )

            # Some boards reboot after serial opens.
            # Small delay helps avoid losing first commands.
            time.sleep(2.0)

            print(f"[Gimbal] Connected on {self.config.port}")

            return True

        except serial.SerialException as e:
            print(f"[Gimbal] Connection failed: {e}")

            return False

    def disconnect(self) -> None:
        """
        Safely close serial connection.
        """

        if self.serial_conn is not None:
            if self.serial_conn.is_open:
                self.serial_conn.close()

        print("[Gimbal] Disconnected")

    def is_connected(self) -> bool:
        """
        Check whether serial connection is active.
        """

        return (
            self.serial_conn is not None
            and self.serial_conn.is_open
        )

    # ========================================================
    # LOW LEVEL SERIAL COMMUNICATION
    # ========================================================

    def send_raw(self, message: str) -> None:
        """
        Send raw string command to board.

        IMPORTANT:
        ----------
        This is intentionally generic because
        the final board protocol is unknown.
        """

        if not self.is_connected():
            raise RuntimeError("Gimbal is not connected")

        # Add newline for easier serial parsing.
        payload = message.strip() + "\n"

        # Convert string -> bytes
        self.serial_conn.write(payload.encode("utf-8"))

        print(f"[Gimbal] TX -> {payload.strip()}")

    # ========================================================
    # HIGH LEVEL COMMANDS
    # ========================================================

    def set_angles(self, yaw_deg: float, pitch_deg: float) -> None:
        """
        Send yaw/pitch command.

        Current protocol:
        -----------------
        Placeholder ONLY.

        Replace later once board firmware is finalized.
        """

        # ----------------------------------------------------
        # SOFTWARE SAFETY CLAMPING
        # ----------------------------------------------------

        yaw_deg = max(
            self.config.min_yaw_deg,
            min(self.config.max_yaw_deg, yaw_deg)
        )

        pitch_deg = max(
            self.config.min_pitch_deg,
            min(self.config.max_pitch_deg, pitch_deg)
        )

        # ----------------------------------------------------
        # PLACEHOLDER COMMAND FORMAT
        # ----------------------------------------------------

        # Possible future protocols:
        #
        #   YAW:10,PITCH:5
        #   G1 X10 Y5
        #   binary packets
        #   MAVLink
        #   CAN frames
        #
        # Current placeholder:
        command = f"SET YAW {yaw_deg:.2f} PITCH {pitch_deg:.2f}"

        self.send_raw(command)

    def center(self) -> None:
        """
        Move gimbal to center position.
        """

        self.set_angles(
            yaw_deg=0.0,
            pitch_deg=0.0,
        )

    def stop(self) -> None:
        """
        Emergency stop placeholder.

        Actual implementation depends on:
        - board firmware
        - motor driver
        - communication protocol
        """

        self.send_raw("STOP")
