"""Structural validation for the robot's URDF description.

Gazebo/RViz are not available in CI, so "loads without errors" is checked
here as a structural validity pass any real URDF loader would also reject on:
well-formed XML, every joint's parent/child references an existing link,
exactly one root link (the kinematic tree's base), every link reachable from
the root, and at least one sensor-bearing link (the acceptance criterion for
this issue). Actually loading it in Gazebo/RViz is a manual verification
step -- see ``docs/architecture.md``.
"""

from __future__ import annotations

import os
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path

__all__ = ["RobotDescription", "UrdfValidationError", "load_urdf", "validate_urdf"]


class UrdfValidationError(ValueError):
    """Raised when a URDF file is malformed or structurally invalid."""


@dataclass(frozen=True)
class RobotDescription:
    """Parsed links/joints of a URDF file, plus which links carry a sensor."""

    name: str
    links: frozenset[str]
    joints: dict[str, tuple[str, str]]  # joint name -> (parent_link, child_link)
    sensor_links: frozenset[str] = field(default_factory=frozenset)

    @property
    def root_links(self) -> frozenset[str]:
        """Links that are never a joint's child -- the tree's base(s)."""
        children = {child for _, child in self.joints.values()}
        return frozenset(self.links - children)


def load_urdf(path: str | os.PathLike[str]) -> RobotDescription:
    """Parse a URDF file into a :class:`RobotDescription`.

    Raises :class:`UrdfValidationError` for malformed XML or a joint
    referencing a link that isn't defined; does not check tree connectivity
    or sensor presence -- use :func:`validate_urdf` for the full acceptance
    check.
    """
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(f"URDF file not found: {path}")

    try:
        root = ET.parse(path).getroot()
    except ET.ParseError as exc:
        raise UrdfValidationError(f"{path} is not well-formed XML: {exc}") from exc

    if root.tag != "robot":
        raise UrdfValidationError(f"{path}: root element must be <robot>, got <{root.tag}>")

    links = frozenset(link.get("name") for link in root.findall("link") if link.get("name"))
    sensor_links = frozenset(
        link.get("name")
        for link in root.findall("link")
        if link.get("name") and link.find("sensor") is not None
    )

    joints: dict[str, tuple[str, str]] = {}
    for joint in root.findall("joint"):
        joint_name = joint.get("name")
        parent = joint.find("parent")
        child = joint.find("child")
        if not joint_name or parent is None or child is None:
            raise UrdfValidationError(
                f"{path}: joint missing name/parent/child: {ET.tostring(joint, encoding='unicode')}"
            )
        parent_link = parent.get("link")
        child_link = child.get("link")
        if parent_link not in links:
            raise UrdfValidationError(
                f"{path}: joint {joint_name!r} parent link {parent_link!r} is not defined"
            )
        if child_link not in links:
            raise UrdfValidationError(
                f"{path}: joint {joint_name!r} child link {child_link!r} is not defined"
            )
        joints[joint_name] = (parent_link, child_link)

    return RobotDescription(
        name=root.get("name", ""), links=links, joints=joints, sensor_links=sensor_links
    )


def validate_urdf(path: str | os.PathLike[str]) -> RobotDescription:
    """Load and fully validate a URDF: well-formed, one connected tree, >=1 sensor link.

    Raises :class:`UrdfValidationError` on any structural problem so a
    broken description fails loudly instead of silently loading as an
    incomplete robot.
    """
    description = load_urdf(path)

    roots = description.root_links
    if len(roots) != 1:
        raise UrdfValidationError(
            f"{path}: expected exactly one root link, found {sorted(roots)}"
        )

    reachable = set(roots)
    frontier = list(roots)
    while frontier:
        current = frontier.pop()
        for parent, child in description.joints.values():
            if parent == current and child not in reachable:
                reachable.add(child)
                frontier.append(child)

    unreachable = description.links - reachable
    if unreachable:
        raise UrdfValidationError(
            f"{path}: link(s) not reachable from root: {sorted(unreachable)}"
        )

    if not description.sensor_links:
        raise UrdfValidationError(
            f"{path}: no link defines a <sensor> element (need at least one, e.g. camera/lidar)"
        )

    return description
