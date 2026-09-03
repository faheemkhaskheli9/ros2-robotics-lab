"""ROS2 Robotics Fundamentals — reusable demo building blocks.

The package is designed to run in two modes:

* **Native ROS2** — when ``rclpy`` is importable, nodes are thin wrappers over
  real ``rclpy`` publishers/subscribers and behave exactly like any other ROS2
  package (``ros2 run`` etc.).
* **Standalone** — when ``rclpy`` is not installed (e.g. a plain CPU dev box or
  CI), the same node classes fall back to an in-process publish/subscribe bus so
  the demos and their tests still run end-to-end.

The fallback is intentionally minimal: it only models the small slice of the
ROS2 API these demos use (create publisher/subscription, spin, timers).
"""

from ros2_robotics_lab.bus import Bus, Subscription
from ros2_robotics_lab.node import MiniNode, rclpy_available

__all__ = ["Bus", "Subscription", "MiniNode", "rclpy_available"]
