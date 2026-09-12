# Architecture Notes: ROS2 Robotics Fundamentals

## Pipeline

```text
Sensors (sim or real) -> ROS2 Nodes (perception, planning, control) -> Navigation Stack -> Actuation
```

## Components

- Navigation stack demo
- Path planning demo
- Robot control demo
- SLAM demo
- RTAB-Map integration
- Robotic vision demo

## Robot Description (URDF)

`assets/urdf/mobile_robot.urdf` describes the simulated differential-drive
mobile robot used by every demo in this repo:

```text
base_link (chassis)
├── left_wheel   (continuous joint)
├── right_wheel  (continuous joint)
├── caster_wheel (fixed joint, unpowered support wheel)
└── lidar_link   (fixed joint) -- <sensor type="ray"> 360-sample 2D lidar
```

`base_link` is the single root of the kinematic tree; every other link is
reachable from it via exactly one joint. `lidar_link` is the sensor mount
required by this phase (a camera link can be added the same way -- a new
`<link>` with a `<sensor type="camera">` element, plus a fixed joint to
`base_link`).

Gazebo/RViz aren't available in CI, so "loads without errors" is enforced
structurally instead: `ros2_robotics_lab.urdf.validate_urdf()` parses the
file and checks well-formed XML, every joint's parent/child resolves to a
defined link, the tree has exactly one root and no disconnected links, and
at least one link carries a `<sensor>` element (`tests/test_urdf.py`).
Actually opening it in Gazebo/RViz is a manual smoke check once a full ROS2
install is available:

```bash
gz sdf -p assets/urdf/mobile_robot.urdf   # or: check_urdf assets/urdf/mobile_robot.urdf
rviz2 -d <config-that-loads-the-robot-model>
```

## Design Notes

- Keep provider/model choices swappable behind interfaces (see `multi-llm-router`
  and similar projects in this portfolio for the general pattern).
- Prefer configuration-driven pipelines (YAML/JSON in `configs/`) over hardcoded
  parameters so experiments are reproducible.
