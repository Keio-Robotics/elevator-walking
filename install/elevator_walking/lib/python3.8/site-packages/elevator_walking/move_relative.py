#!/usr/bin/env python3
"""Send a relative pose target to a Unitree Go2.

The SDK call submits a target but does not report when motion completes.
Coordinates are interpreted as x/y offsets in meters and yaw in radians.
"""

import argparse
import math
import sys
from typing import List, Optional


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Move the Go2 by a relative x/y offset and yaw target."
    )
    parser.add_argument(
        "network_interface",
        help="network interface connected to the robot, for example enp3s0",
    )
    parser.add_argument("--x", type=float, default=0.0, help="relative x offset in meters")
    parser.add_argument("--y", type=float, default=0.0, help="relative y offset in meters")
    parser.add_argument("--yaw", type=float, default=0.0, help="relative yaw offset in radians")
    return parser


def main(argv: Optional[List[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    target = (args.x, args.y, args.yaw)
    if not all(math.isfinite(value) for value in target):
        parser.error("x, y, and yaw must be finite numbers")
    if all(value == 0.0 for value in target):
        parser.error("at least one of --x, --y, or --yaw must be non-zero")

    try:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from unitree_sdk2py.go2.obstacles_avoid.obstacles_avoid_client import (
            ObstaclesAvoidClient,
        )
    except ImportError as exc:
        print(
            "Unitree SDK 2 Python is required. Install the official "
            "unitree_sdk2_python package first.",
            file=sys.stderr,
        )
        print(f"Import error: {exc}", file=sys.stderr)
        return 2

    try:
        ChannelFactoryInitialize(0, args.network_interface)
        client = ObstaclesAvoidClient()
        client.SetTimeout(3.0)
        client.Init()

        code = client.MoveToIncrementPosition(args.x, args.y, args.yaw)
        if code != 0:
            print(f"Unitree API returned error code {code}.", file=sys.stderr)
            return 1

        print(
            "Relative movement command sent: "
            f"x={args.x:.3f} m, y={args.y:.3f} m, yaw={args.yaw:.3f} rad."
        )
        print("The SDK call does not confirm that the robot has reached the target.")
        return 0
    except KeyboardInterrupt:
        print("Interrupted.", file=sys.stderr)
        return 130
    except Exception as exc:  # SDK/network exceptions vary by SDK release.
        print(f"Could not send movement command: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
