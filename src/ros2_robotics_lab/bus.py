"""A tiny in-process publish/subscribe bus.

This stands in for the ROS2 middleware when ``rclpy`` is unavailable. It is not a
network transport and makes no attempt to be one — messages are delivered
synchronously to same-process subscribers on the calling thread.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass, field
from typing import Any, Callable


@dataclass
class Subscription:
    """Handle returned by :meth:`Bus.subscribe`; keeps the callback referenced."""

    topic: str
    callback: Callable[[Any], None]
    _bus: "Bus"

    def unsubscribe(self) -> None:
        self._bus._remove(self)


@dataclass
class Bus:
    """Process-wide topic registry. Delivery is synchronous and in-order."""

    _subs: dict[str, list[Subscription]] = field(default_factory=dict)
    _lock: threading.RLock = field(default_factory=threading.RLock)

    def subscribe(self, topic: str, callback: Callable[[Any], None]) -> Subscription:
        sub = Subscription(topic=topic, callback=callback, _bus=self)
        with self._lock:
            self._subs.setdefault(topic, []).append(sub)
        return sub

    def publish(self, topic: str, message: Any) -> int:
        """Deliver ``message`` to every subscriber of ``topic``.

        Returns the number of subscribers the message was delivered to. A
        subscriber callback that raises does not prevent delivery to the rest.
        """
        with self._lock:
            targets = list(self._subs.get(topic, ()))
        delivered = 0
        for sub in targets:
            sub.callback(message)
            delivered += 1
        return delivered

    def _remove(self, sub: Subscription) -> None:
        with self._lock:
            subs = self._subs.get(sub.topic)
            if subs and sub in subs:
                subs.remove(sub)
                if not subs:
                    del self._subs[sub.topic]


# One shared bus per process, mirroring the single ROS2 domain a demo joins.
_DEFAULT_BUS = Bus()


def default_bus() -> Bus:
    return _DEFAULT_BUS
