from ros2_robotics_lab.bus import Bus


def test_publish_delivers_to_all_subscribers():
    bus = Bus()
    got_a, got_b = [], []
    bus.subscribe("/t", got_a.append)
    bus.subscribe("/t", got_b.append)

    delivered = bus.publish("/t", 42)

    assert delivered == 2
    assert got_a == [42]
    assert got_b == [42]


def test_publish_to_topic_with_no_subscribers_is_noop():
    assert Bus().publish("/empty", "x") == 0


def test_unsubscribe_stops_delivery():
    bus = Bus()
    seen = []
    sub = bus.subscribe("/t", seen.append)
    bus.publish("/t", 1)
    sub.unsubscribe()
    bus.publish("/t", 2)
    assert seen == [1]
