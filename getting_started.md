# LIMO Multi-Robot Setup (SLAM + Nav2 + Zenoh Bridge)


The working approach run everything **locally without any ROS
namespace at all** (exactly like a normal single-robot setup), and use
`zenoh-bridge-ros2dds` to selectively expose only `merged_map` and
`merge_map` to other robots, with the `/robotX` prefix added **only** at
the bridge boundary.

---

# Prerequisites
ROS2 installed on both robot and computer (only for visualization purposes)

```bash
sudo apt update
sudo apt install ros-humble-rmw-cyclonedds-cpp
```

Add the following to `~/.bashrc` on every machine involved:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```


On the robot, build these packages in your workspace.
pull those repos in your src folder:
- [map_merge_interfaces](https://github.com/R1leMargoulin/map_merge_interfaces)
- [map_merge_server](https://github.com/R1leMargoulin/map_merge_server)
- [explo_multirobot](https://github.com/R1leMargoulin/explo_multirobot)

```bash
cd ~/ros2_ws #pick yours
colcon build --packages-select map_merge_interfaces map_merge_server explo_multirobot
source install/setup.bash
```


---

## Install `zenoh-bridge-ros2dds`
`zenoh-bridge-ros2dds` is a translation layer that sits on top of regular DDS (Cyclone DDS here) and
bridges it to Zenoh at the network boundary 

Eclipse Zenoh provides an apt repository with a dedicated package for this
exact use case:

```bash
echo "deb [trusted=yes] https://download.eclipse.org/zenoh/debian-repo/ /" | sudo tee /etc/apt/sources.list.d/zenoh.list > /dev/null
sudo apt update
sudo apt install zenoh-bridge-ros2dds
```

---

## Network setup: `ROS_LOCALHOST_ONLY`

**On each LIMO robot**, set this **before** launching anything (so that
local ROS traffic never leaks onto the network directly, bypassing the
bridge and causing duplicate/racing data):

```bash
export ROS_LOCALHOST_ONLY=1
sudo ip link set lo multicast on
```

---

## Zenoh bridge configuration

You can place those files in a `~/zenoh_conf/` folder.

### Robot config (`zenoh_conf.json5`)

```json5
{
  plugins: {
    ros2dds: {
      allow: {
        service_servers: [".*merge_map$"],
        service_clients: [".*merge_map$"],
      },
    },
  },
  mode: "peer",
  listen: { endpoints: ["tcp/0.0.0.0:7447"] },
  connect: {
    // Add every machine you want to share these topics/services with
    // (other robots, and/or the PC if you want to see the merged map there).
    endpoints: ["tcp/<PC_OR_OTHER_ROBOT_IP>:7447", "tcp/<PC_OR_OTHER_ROBOT_IP>:7447"],
  },
}
```

Adjust `namespace` per robot (`/robot1`, `/robot2`, ...).

### PC config (`zenoh_conf.json5`)

```json5
{
  plugins: {
    ros2dds: {},
  },
  mode: "peer",
  connect: {
    endpoints: ["tcp/<ROBOT_IP>:7447"],
  },
}
```

### Start the bridges

On the robot:
```bash
zenoh-bridge-ros2dds -c ~/zenoh_conf/zenoh_conf.json5
```

On the PC:
```bash
zenoh-bridge-ros2dds -c ~/zenoh_conf/zenoh_conf.json5
```
---

# Launch procedure 
Run each of the following in its own terminal, in this order.

You should `export ROS_LOCALHOST_ONLY=1` on each terminal, you can eventually place it at the end of your `.bashrc` but note that this will keep your robot running ros2 locally.
`

**Zenoh bridge**

```bash
sudo ip link set lo multicast on
zenoh-bridge-ros2dds -c zenoh_conf.json5
```

**Base robot bringup**
```bash
ros2 launch limo_bringup limo_start.launch.py
```

**SLAM**

```bash
ros2 launch limo_bringup limo_slam_box.launch.py
```

**Global reference frame anchor**

This robot's known pose in the frame shared across robots
(`global_odom -> map`). Adjust `--x --y --yaw` to this robot's actual
starting pose relative to the shared origin.

```bash
ros2 run tf2_ros static_transform_publisher --ros-args \
  -r __ns:=/ \
  -r /tf:=tf -r /tf_static:=tf_static \
  -p use_sim_time:=false \
  -- --x 0.5 --y 0.5 --z 0 --roll 0 --pitch 0 --yaw 0.0 \
  --frame-id global_odom --child-frame-id map
```

**Map merge server**  

Keep the same init pose (than the one from the previous step) as map origin., change the name of the robot as desired. it will publish the service as `/<robot_name>/merge_map`
```bash
#TO CHECK : -> au final ca j'ai mis a zero et je met global_odom en merged id frame, ca donne un truc plus cohérent.
ros2 run map_merge_server map_merge_server_node --ros-args \
  -p -p robot_name:=robot1 \
  -p map_origin_x:=0.5 \
  -p map_origin_y:=0.5 \
  -p map_origin_theta:=0.0 \
  -p merged_frame_id:=map
```

**Nav2**  

(change the path of the config depending on your workspace location)

```bash
ros2 launch limo_bringup limo_nav2_ackermann.launch.py \
  nav2_param_path:=~/ros2_ws/src/limo_ros2/explo_multirobot/config/nav2_limo_multirobot.yaml
```

**Frontier exploration**

```bash
ros2 run explo_multirobot explorer_node --ros-args \
  -p global_frame:=map \
  -p robot_base_frame:=base_link \
  -p exploration_period_sec:=5.0
```

---

**Multi-robot gossip client**

To sync this robot's fused map with a peer robot (once its own bridge is also up and both can see each other's `merge_map` service):

```bash
ros2 run map_merge_server merge_client_example --ros-args \
  -p -p robot_name:=robot1 \
  -p peer_service:=/robot2/merge_map \
  -p sync_period_sec:=5.0
```

`peer_service` accepts a comma-separated list if you want this robot to gossip with more than one peer (e.g. `"/robot2/merge_map,/robot3/merge_map"`).

---
