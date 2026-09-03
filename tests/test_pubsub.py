from ros2_robotics_lab.bus import Bus
from ros2_robotics_lab.cli import main
from ros2_robotics_lab.nodes import HeartbeatPublisher, HeartbeatSubscriber

import pytest


def test_subscriber_receives_publisher_messages():
    bus = Bus()
    pub = HeartbeatPublisher(rate_hz=50.0, bus=bus)
    sub = HeartbeatSubscriber(bus=bus)

    pub.spin_for(0.25)

    assert pub.count >= 3
    assert sub.received == pub.count
    assert sub.last is not None and sub.last["seq"] == pub.count
    assert sub.last["data"].startswith("heartbeat ")


def test_message_sequence_is_monotonic():
    bus = Bus()
    seqs = []
    pub = HeartbeatPublisher(rate_hz=100.0, bus=bus)
    HeartbeatSubscriber(bus=bus).create_subscription(
        "/heartbeat", lambda m: seqs.append(m["seq"])
    )
    pub.spin_for(0.2)
    assert seqs == list(range(1, len(seqs) + 1))


def test_invalid_rate_rejected():
    with pytest.raises(ValueError):
        HeartbeatPublisher(rate_hz=0.0)


def test_cli_pubsub_runs_and_reports_success(capsys):
    rc = main(["pubsub", "--rate", "50", "--seconds", "0.3"])
    out = capsys.readouterr().out
    assert rc == 0
    assert "pubsub demo done" in out
    assert "received=" in out
