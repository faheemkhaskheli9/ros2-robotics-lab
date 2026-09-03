"""Publisher node: emits an incrementing heartbeat message at a fixed rate.

ROS2 equivalent: a ``std_msgs/String`` publisher on ``/heartbeat`` driven by a
wall timer. Here the message is a plain ``dict`` so the demo has no message-gen
build step.
"""

from __future__ import annotations

import time

from ros2_robotics_lab.bus import Bus
from ros2_robotics_lab.node import MiniNode

TOPIC = "/heartbeat"


class HeartbeatPublisher(MiniNode):
    def __init__(self, rate_hz: float = 2.0, *, bus: Bus | None = None) -> None:
        super().__init__("heartbeat_publisher", bus=bus)
        if rate_hz <= 0:
            raise ValueError(f"rate_hz must be > 0, got {rate_hz}")
        self.count = 0
        self._pub = self.create_publisher(TOPIC)
        self.create_timer(1.0 / rate_hz, self._on_timer)

    def _on_timer(self) -> None:
        self.count += 1
        message = {
            "seq": self.count,
            "stamp": time.time(),
            "data": f"heartbeat {self.count}",
        }
        self._pub.publish(message)
        self.get_logger().info("published %s", message["data"])
