# explo_multirobot

Nav2-based navigation and frontier exploration on top of the fused map produced by [map_merge_server](https://github.com/R1leMargoulin/map_merge_server). Adapted for multi-robot systems. Each robot runs its own Nav2 stack (no `map_server`/`amcl` — `slam_toolbox` already provides live localization) and its own frontier-exploration node.

**Depends on [map_merge_server](https://github.com/R1leMargoulin/map_merge_server) already publishing `merged_map` for each robot,**.

---

## Getting started
The [Getting started](getting_started.md) guide, explains how to run everything on a Agilex LIMO robot.

---

## explorer node explaination : for a robot named `robotX`

For a robot namespaced `robotX`:
- Nav2's global costmap subscribes to `/robotX/merged_map`.
- The explorer publishes navigation goals through `/robotX/navigate_to_pose`.
- Both rely on the TF chain `global_odom → map → odom → base_footprint` already being live for `robotX`.

---