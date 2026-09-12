from __future__ import annotations

from pathlib import Path

import pytest

from ros2_robotics_lab.urdf import UrdfValidationError, load_urdf, validate_urdf

REPO_ROOT = Path(__file__).resolve().parents[1]
ROBOT_URDF = REPO_ROOT / "assets" / "urdf" / "mobile_robot.urdf"

VALID_URDF = """<?xml version="1.0"?>
<robot name="test_bot">
  <link name="base_link"/>
  <link name="sensor_link">
    <sensor name="cam" type="camera"/>
  </link>
  <joint name="sensor_joint" type="fixed">
    <parent link="base_link"/>
    <child link="sensor_link"/>
  </joint>
</robot>
"""


def test_shipped_robot_urdf_is_structurally_valid():
    description = validate_urdf(ROBOT_URDF)
    assert description.name == "mobile_robot"
    assert description.root_links == {"base_link"}


def test_shipped_robot_urdf_has_at_least_one_sensor_link():
    description = load_urdf(ROBOT_URDF)
    assert description.sensor_links, "expected at least one <sensor>-bearing link"
    assert "lidar_link" in description.sensor_links


def test_shipped_robot_urdf_wheel_joints_connect_to_base():
    description = load_urdf(ROBOT_URDF)
    assert description.joints["left_wheel_joint"] == ("base_link", "left_wheel")
    assert description.joints["right_wheel_joint"] == ("base_link", "right_wheel")


def test_valid_minimal_urdf_round_trips(tmp_path):
    path = tmp_path / "robot.urdf"
    path.write_text(VALID_URDF)
    description = validate_urdf(path)
    assert description.links == {"base_link", "sensor_link"}
    assert description.sensor_links == {"sensor_link"}


def test_missing_file_raises_file_not_found_error(tmp_path):
    with pytest.raises(FileNotFoundError):
        load_urdf(tmp_path / "nope.urdf")


def test_malformed_xml_raises_urdf_validation_error(tmp_path):
    path = tmp_path / "broken.urdf"
    path.write_text("<robot name='x'><link name='a'></robot>")  # unclosed <link>
    with pytest.raises(UrdfValidationError):
        load_urdf(path)


def test_joint_referencing_undefined_link_raises(tmp_path):
    path = tmp_path / "bad_ref.urdf"
    path.write_text(
        """<?xml version="1.0"?>
<robot name="bad">
  <link name="base_link"/>
  <joint name="j1" type="fixed">
    <parent link="base_link"/>
    <child link="ghost_link"/>
  </joint>
</robot>
"""
    )
    with pytest.raises(UrdfValidationError):
        load_urdf(path)


def test_link_with_no_joint_at_all_is_a_second_unconnected_root(tmp_path):
    """A link nobody's joint claims as a child is itself a root -- with the
    base_link also a root, that is two disconnected trees, caught as a
    "more than one root" error (the more specific "not reachable" case needs
    a joint cycle to construct, which URDF forbids by definition)."""
    path = tmp_path / "disconnected.urdf"
    path.write_text(
        """<?xml version="1.0"?>
<robot name="disconnected">
  <link name="base_link"/>
  <link name="sensor_link">
    <sensor name="cam" type="camera"/>
  </link>
  <link name="floating_link"/>
  <joint name="sensor_joint" type="fixed">
    <parent link="base_link"/>
    <child link="sensor_link"/>
  </joint>
</robot>
"""
    )
    with pytest.raises(UrdfValidationError, match="root link"):
        validate_urdf(path)


def test_unreachable_cyclic_island_fails_full_validation(tmp_path):
    """Two links that are each other's joint child (a cycle, disconnected
    from base_link) both count as non-roots, so the single-root check
    passes but reachability from that root does not -- exercises the
    "not reachable" branch specifically."""
    path = tmp_path / "cyclic_island.urdf"
    path.write_text(
        """<?xml version="1.0"?>
<robot name="cyclic_island">
  <link name="base_link">
    <sensor name="cam" type="camera"/>
  </link>
  <link name="island_a"/>
  <link name="island_b"/>
  <joint name="a_to_b" type="fixed">
    <parent link="island_a"/>
    <child link="island_b"/>
  </joint>
  <joint name="b_to_a" type="fixed">
    <parent link="island_b"/>
    <child link="island_a"/>
  </joint>
</robot>
"""
    )
    with pytest.raises(UrdfValidationError, match="not reachable"):
        validate_urdf(path)


def test_no_sensor_link_fails_full_validation(tmp_path):
    path = tmp_path / "no_sensor.urdf"
    path.write_text(
        """<?xml version="1.0"?>
<robot name="no_sensor">
  <link name="base_link"/>
  <link name="wheel"/>
  <joint name="wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel"/>
  </joint>
</robot>
"""
    )
    with pytest.raises(UrdfValidationError, match="sensor"):
        validate_urdf(path)


def test_multiple_root_links_fails_full_validation(tmp_path):
    path = tmp_path / "two_roots.urdf"
    path.write_text(
        """<?xml version="1.0"?>
<robot name="two_roots">
  <link name="base_link"/>
  <link name="other_root">
    <sensor name="cam" type="camera"/>
  </link>
</robot>
"""
    )
    with pytest.raises(UrdfValidationError, match="root link"):
        validate_urdf(path)
