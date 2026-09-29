# explo_multirobot

Nav2-based navigation and frontier exploration on top of the fused map produced by [map_merge_server](https://github.com/R1leMargoulin/map_merge_server). Adapted for multi-robot systems. Each robot runs its own Nav2 stack (no `map_server`/`amcl` — `slam_toolbox` or `cartographer` already provides live localization) and its own frontier-exploration node.

**Depends on [map_merge_server](https://github.com/R1leMargoulin/map_merge_server) already publishing `merged_map` for each robot,**.

---
# Acknowledgement
This work is built on top of what several interns provided during their internship @ CRIStAL Laboratory.
- Raphael Capelle
- Arthur Goddefroy

The exploration program is built on top of the implementation (for single robots) of Arka Gosh (AniArka) : (https://github.com/AniArka/Autonomous-Explorer-and-Mapper-ros2-nav2)

---

## Getting started

- The [Theory](theory.md), explain theorically what is made without implementation or commands details. A **short video demo** is also available.

- The [Getting started](getting_started.md) guide, explains how to run everything on a Agilex LIMO ROS2 robot.

Warning : The launchs file were for tests in simulation, the real robot implementation has currently no global launch file.

---

## explorer node explaination : for a robot named `robotX`

For a robot namespaced `robotX`:
- Nav2's global costmap subscribes to `/robotX/merged_map`.
- The explorer publishes navigation goals through `/robotX/navigate_to_pose`.
- Both rely on the TF chain `world → map → odom → base_footprint` already being live for `robotX`.

---
