"""Command-line entry point for the ROS2 Robotics Fundamentals demos.

    python -m ros2_robotics_lab.cli pubsub --rate 2 --seconds 3

Later phases add subcommands here (``navigate``, ``slam``, ``perceive`` ...).
"""

from __future__ import annotations

import argparse
import logging
import sys

from ros2_robotics_lab.bus import Bus
from ros2_robotics_lab.node import rclpy_available
from ros2_robotics_lab.nodes import HeartbeatPublisher, HeartbeatSubscriber


def _run_pubsub(rate_hz: float, seconds: float) -> int:
    # A dedicated bus so repeated runs in one process stay isolated.
    bus = Bus()
    pub = HeartbeatPublisher(rate_hz=rate_hz, bus=bus)
    sub = HeartbeatSubscriber(bus=bus)
    try:
        # Drive both nodes from one cooperative loop (single-threaded executor).
        deadline_ticks = int(seconds / 0.02)
        for _ in range(deadline_ticks):
            pub.spin_once(timeout_s=0.02)
            sub.spin_once(timeout_s=0.0)
    finally:
        pub.destroy_node()
        sub.destroy_node()
    print(
        f"pubsub demo done: published={pub.count} received={sub.received} "
        f"(backend={'rclpy' if rclpy_available() else 'in-process bus'})"
    )
    return 0 if sub.received > 0 else 1


def build_parser() -> argparse.ArgumentParser:
    # A shared parent so -v/--verbose works both before and after the subcommand.
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument(
        "-v", "--verbose", action="store_true", help="show node INFO logs"
    )

    parser = argparse.ArgumentParser(
        prog="ros2_robotics_lab", description=__doc__, parents=[common]
    )
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser(
        "pubsub",
        parents=[common],
        help="run the heartbeat publisher/subscriber demo",
    )
    p.add_argument("--rate", type=float, default=2.0, help="publish rate in Hz")
    p.add_argument("--seconds", type=float, default=3.0, help="run duration")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    logging.basicConfig(
        level=logging.INFO if args.verbose else logging.WARNING,
        format="%(name)s: %(message)s",
    )
    if args.command == "pubsub":
        return _run_pubsub(args.rate, args.seconds)
    return 2


if __name__ == "__main__":
    sys.exit(main())
