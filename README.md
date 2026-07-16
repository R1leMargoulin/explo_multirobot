# explo_multirobot

Nav2-based navigation and frontier exploration on top of the fused map produced by [map_merge_server](https://github.com/R1leMargoulin/map_merge_server). Each robot runs its own Nav2 stack (no `map_server`/`amcl` — `slam_toolbox` already provides live localization) and its own frontier-exploration node.

**Depends on [map_merge_server](https://github.com/R1leMargoulin/map_merge_server) already publishing `merged_map` for each robot,** and currently on [multi_robot_slam_toolbox_simulation](https://github.com/R1leMargoulin/multi_robot_slam_toolbox_simulation)'s shared `global_odom` TF anchor. The second dependencie is currently being removed/modified for real robot deployement. (I will make a package with a launch for a correct slam toolbox without gazebo simulation)

---

## Components

### `nav2_multirobot_launch.py` + `nav2_multirobot_params.yaml`
Starts the minimal set of Nav2 nodes needed for point-to-point navigation: `controller_server`, `planner_server`, `smoother_server`, `behavior_server`, `bt_navigator`, `velocity_smoother`, plus a `lifecycle_manager`.

The global costmap uses `global_odom` as its reference frame (not the usual single-robot `map`), and its static layer reads from `merged_map` rather than a `map_server`-served static map.

**Implementation note:** this launch deliberately avoids `nav2_bringup`'s own `navigation_launch.py` and its namespace-rewriting mechanism, and instead starts each Nav2 node directly via `ExecuteProcess` with a manually-built, single-block `--ros-args` command line. It also relies on wrapping `nav2_multirobot_params.yaml` under a `/**:` wildcard namespace key.

### `explorer_node` + `explo_multirobot_launch.py`
Frontier-based exploration, one instance per robot. It:
- reads `merged_map` (not the raw local map);
- gets its real position via a live TF lookup (`global_odom` → `base_footprint`);
- groups frontier cells into connected clusters and targets the centroid of the largest unvisited one (blacklisting a radius around each visited centroid, not a single cell);
- sends goals via `NavigateToPose`, in the `global_odom` frame.

---

## The demo launches — simulation use only

Both `nav2_multirobot_launch.py` and `explo_multirobot_launch.py` loop over a hardcoded robot list (`ROBOT_NAMES`) and start every robot's instance from a single file. This is convenient in **simulation**, where everything runs on one machine.

**This is not the right model for a real deployment.** On real robots, each machine should start only its own robot's Nav2 stack and explorer node, with its own namespace, rather than a fixed list of every robot known ahead of time.

| Context | Recommendation |
|---|---|
| Simulation (single machine, multiple robots) | Both launches, as-is |
| Real deployment (one robot = one machine) | Launch only this robot's Nav2 stack + explorer, manually or via a single-robot-scoped launch |

---

## Manually launching the explorer for a single robot

```bash
ros2 run explo_multirobot explorer_node --ros-args \
  -r __ns:=/robot1 \
  -r /tf:=tf -r /tf_static:=tf_static \
  -p global_frame:=global_odom \
  -p robot_base_frame:=base_footprint \
  -p exploration_period_sec:=5.0
```

The `/tf`, `/tf_static` remaps are required.

For Nav2 itself, adapting `nav2_multirobot_launch.py` to take the robot name as a single launch argument (instead of looping over a hardcoded list) is the recommended path for a single-robot deployment, given the number of servers and remaps involved.

---

## Example: a robot named `robotX`

For a robot namespaced `robotX`:
- Nav2's global costmap subscribes to `/robotX/merged_map`.
- The explorer publishes navigation goals through `/robotX/navigate_to_pose`.
- Both rely on the TF chain `global_odom → map → odom → base_footprint` already being live for `robotX`.

---
