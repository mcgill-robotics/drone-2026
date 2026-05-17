#!/usr/bin/env python3
"""Mission 1 setup entry point."""

from __future__ import annotations

import argparse
import sys

import movement


def setup(*, sitl: bool = False, port: str | None = None, wait_seconds: float = 10.0) -> bool:
    return movement.setup_environment(sitl=sitl, port=port, boot=True, wait_seconds=wait_seconds)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Boot PX4/MAVROS for Mission 1")
    parser.add_argument("--sitl", action="store_true", help="Use SITL UDP connection")
    parser.add_argument("--hardware", action="store_true", help="Use /dev/ttyUSB0 hardware connection")
    parser.add_argument("--port", default=None, help="Custom hardware serial port")
    parser.add_argument("--wait-seconds", type=float, default=10.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    port = "/dev/ttyUSB0" if args.hardware else args.port
    return 0 if setup(sitl=args.sitl, port=port, wait_seconds=args.wait_seconds) else 1


if __name__ == "__main__":
    sys.exit(main())
