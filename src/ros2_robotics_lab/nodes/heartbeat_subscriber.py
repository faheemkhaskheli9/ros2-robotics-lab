"""Subscriber node: receives heartbeat messages and logs them.

Keeps the last message and a running count so tests (and later demos) can assert
that inter-node communication actually happened.
"""

from __future__ import annotations

from typing import Any

from ros2_robotics_lab.bus import Bus
from ros2_robotics_lab.node import MiniNode
from ros2_robotics_lab.nodes.heartbeat_publisher import TOPIC


class HeartbeatSubscriber(MiniNode):
    def __init__(self, *, bus: Bus | None = None) -> None:
        super().__init__("heartbeat_subscriber", bus=bus)
        self.received = 0
        self.last: Any | None = None
        self.create_subscription(TOPIC, self._on_message)

    def _on_message(self, message: Any) -> None:
        self.received += 1
        self.last = message
        data = message.get("data") if isinstance(message, dict) else message
        self.get_logger().info("received %s", data)
