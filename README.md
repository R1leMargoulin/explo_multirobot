# explo_multirobot

Nav2-based navigation and frontier exploration on top of the fused map produced by [map_merge_server](https://github.com/R1leMargoulin/map_merge_server). Adapted for multi-robot systems. Each robot runs its own Nav2 stack (no `map_server`/`amcl` — `slam_toolbox` already provides live localization) and its own frontier-exploration node.

**Depends on [map_merge_server](https://github.com/R1leMargoulin/map_merge_server) already publishing `merged_map` for each robot,** and currently on [multi_robot_slam_toolbox_simulation](https://github.com/R1leMargoulin/multi_robot_slam_toolbox_simulation)'s shared `global_odom` TF anchor. The second dependency is currently being removed/modified for real robot deployment. (I will make a package with a launch for a correct slam toolbox without gazebo simulation)

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

### `single_robot_launch.py`
A single, parameterized launch for **one robot** — the recommended entry point outside simulation (see below).

---

## The demo launches — simulation use only

`nav2_multirobot_launch.py` and `explo_multirobot_launch.py` (used on their own, without `single_robot_launch.py`) loop over a hardcoded robot list (`ROBOT_NAMES`) and start every robot's instance from a single file. This is convenient in **simulation**, where everything runs on one machine.

**This is not the right model for a real deployment.** On real robots, each machine should start only its own robot's stack, with its own namespace, rather than a fixed list of every robot known ahead of time. That's what `single_robot_launch.py` is for.

| Context | Recommendation |
|---|---|
| Simulation (single machine, multiple robots) | `nav2_multirobot_launch.py` + `explo_multirobot_launch.py`, as-is |
| Real deployment (one robot = one machine) | `single_robot_launch.py`, run independently on each robot |

---

## Real deployment: `single_robot_launch.py`

One file, parameterized by namespace and known starting pose, that brings up everything needed for a single robot: static `global_odom → map` TF anchor, SLAM, this robot's map-merge server + gossip client(s), Nav2, and the explorer.

```bash
# On robot1
ros2 launch explo_multirobot single_robot_launch.py \
  namespace:=robot1 x_pose:=0.5 y_pose:=0.5 yaw_pose:=0.0 \
  peer_service:=/robot2/merge_map \
  slam_executable:=async_slam_toolbox_node \
  slam_params_file:=/path/to/your/mapper_params_online_async.yaml

# On robot2
ros2 launch explo_multirobot single_robot_launch.py \
  namespace:=robot2 x_pose:=-0.5 y_pose:=-0.5 yaw_pose:=1.5707 \
  peer_service:=/robot1/merge_map \
  slam_executable:=async_slam_toolbox_node \
  slam_params_file:=/path/to/your/mapper_params_online_async.yaml
```

Key arguments:
- `namespace`, `x_pose`/`y_pose`/`yaw_pose`: this robot's identity and known starting pose in the shared global frame.
- `peer_service`: **comma-separated list** of absolute `merge_map` service names to sync with (e.g. `"/robot2/merge_map,/robot3/merge_map"`). One `merge_client_example` instance is started per entry — a robot only needs at least one peer for convergence to propagate transitively across the rest of the network, but listing more than one (partial or full mesh) is supported too.
- `slam_executable` / `slam_params_file`: override these with whatever `slam_toolbox` variant and tuned params your own robot already uses (defaults to `sync_slam_toolbox_node` + stock params). Whatever file you point at, make sure `scan_topic` is relative (`scan`, not `/scan`) because an absolute value here breaks namespacing.
- `use_sim_time`: `false` by default (real robot). Set `true` only if this is still running against simulation.
- `nav2_params_file`: defaults to this package's `nav2_multirobot_params.yaml`; override if you need different Nav2 tuning per robot.

---

## explorer node explaination : for a robot named `robotX`

For a robot namespaced `robotX`:
- Nav2's global costmap subscribes to `/robotX/merged_map`.
- The explorer publishes navigation goals through `/robotX/navigate_to_pose`.
- Both rely on the TF chain `global_odom → map → odom → base_footprint` already being live for `robotX`.

---