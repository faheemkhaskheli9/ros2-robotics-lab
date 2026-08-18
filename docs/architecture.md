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

## Design Notes

- Keep provider/model choices swappable behind interfaces (see `multi-llm-router`
  and similar projects in this portfolio for the general pattern).
- Prefer configuration-driven pipelines (YAML/JSON in `configs/`) over hardcoded
  parameters so experiments are reproducible.
