"""``MiniNode`` — the common node base used by every demo.

It exposes the handful of ROS2 concepts the demos need (publishers,
subscriptions, wall timers, a logger, ``spin``/``spin_once``) and transparently
routes them either to real ``rclpy`` or to the in-process :mod:`bus` fallback.
"""

from __future__ import annotations

import logging
import time
from typing import Any, Callable

from ros2_robotics_lab.bus import Bus, default_bus

try:  # pragma: no cover - exercised only where a full ROS2 stack is installed
    import rclpy  # type: ignore
    import rclpy.node  # type: ignore

    RCLPY_AVAILABLE = True
except Exception:  # ImportError, or a partial install that fails at import
    RCLPY_AVAILABLE = False


def rclpy_available() -> bool:
    return RCLPY_AVAILABLE


class _Timer:
    def __init__(self, period_s: float, callback: Callable[[], None]) -> None:
        self.period_s = period_s
        self.callback = callback
        self._next = time.monotonic() + period_s

    def due(self, now: float) -> bool:
        return now >= self._next

    def fire(self, now: float) -> None:
        self.callback()
        # Schedule from the deadline, not ``now``, so drift does not accumulate.
        self._next += self.period_s
        if self._next < now:  # we fell far behind; resync
            self._next = now + self.period_s


class MiniNode:
    """Minimal node abstraction.

    When ``rclpy`` is present this delegates to an ``rclpy.node.Node`` of the same
    name. Otherwise it uses ``bus`` for pub/sub and a cooperative timer loop.
    """

    def __init__(self, name: str, *, bus: Bus | None = None) -> None:
        self.name = name
        self._log = logging.getLogger(f"ros2_robotics_lab.{name}")
        self._timers: list[_Timer] = []
        self._subs: list[Any] = []
        self._native = None
        if RCLPY_AVAILABLE:
            if not rclpy.ok():
                rclpy.init()
            self._native = rclpy.node.Node(name)
            self._bus = None
        else:
            self._bus = bus or default_bus()

    # --- logging -----------------------------------------------------------
    def get_logger(self) -> logging.Logger:
        return self._log

    # --- publishers ------------------------------------------------------------
    def create_publisher(self, topic: str, msg_type: type | None = None):
        if self._native is not None:  # pragma: no cover
            return self._native.create_publisher(msg_type, topic, 10)
        bus = self._bus
        assert bus is not None

        class _Pub:
            def publish(self, message: Any) -> int:
                return bus.publish(topic, message)

        return _Pub()

    # --- subscriptions ------------------------------------------------------
    def create_subscription(
        self, topic: str, callback: Callable[[Any], None], msg_type: type | None = None
    ):
        if self._native is not None:  # pragma: no cover
            sub = self._native.create_subscription(msg_type, topic, callback, 10)
            self._subs.append(sub)
            return sub
        assert self._bus is not None
        sub = self._bus.subscribe(topic, callback)
        self._subs.append(sub)
        return sub

    # --- timers -----------------------------------------------------------
    def create_timer(self, period_s: float, callback: Callable[[], None]):
        if self._native is not None:  # pragma: no cover
            return self._native.create_timer(period_s, callback)
        timer = _Timer(period_s, callback)
        self._timers.append(timer)
        return timer

    # --- spinning -----------------------------------------------------------
    def spin_once(self, timeout_s: float = 0.05) -> None:
        if self._native is not None:  # pragma: no cover
            rclpy.spin_once(self._native, timeout_sec=timeout_s)
            return
        now = time.monotonic()
        for timer in self._timers:
            if timer.due(now):
                timer.fire(now)
        if timeout_s:
            time.sleep(timeout_s)

    def spin_for(self, duration_s: float, tick_s: float = 0.02) -> None:
        """Spin the node for ``duration_s`` seconds, then return."""
        end = time.monotonic() + duration_s
        while time.monotonic() < end:
            self.spin_once(timeout_s=tick_s)

    def destroy_node(self) -> None:
        for sub in self._subs:
            if hasattr(sub, "unsubscribe"):
                sub.unsubscribe()
        self._subs.clear()
        self._timers.clear()
        if self._native is not None:  # pragma: no cover
            self._native.destroy_node()
