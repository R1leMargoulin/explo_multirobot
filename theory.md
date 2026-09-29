# Theory

## Basic robot movements
This solution uses the original core bringup of the robot. The bringup sets the robot control up, so that the sensor can be read and velocities command can be given that will be transformed into motors action.

We also use the cartographer node made by the robots manufacturer, as well as the nav2 config that we modify.

With all of these elements, the root is able to move and navigate autonomously to a given point(when a map is constructed).


## Frontier Exploration
We use a classic Frontier exploration. We identify here a frontier (the closest to th robot) of an explored part of the environment, and the unexplored part of it. The robot then moves at the selected location to expand the map with new sensors measurements.

We re-used as a base, the content of the package from of Arka Gosh (AniArka) : (https://github.com/AniArka/Autonomous-Explorer-and-Mapper-ros2-nav2).


## Multi-robot and map merging
### map merging
We launch a **map_merge** server, on the node we made : [map_merge_server](https://github.com/R1leMargoulin/map_merge_server). More details about merging can be found on that repo, including the limitations.

This node create a map **merged_map** on a global frame **world** we make for the robots. The global frame is created to make the robot calculate on the same origin.
Otherwise, we would have feed the shift of the two original maps, with obstacles duplication.

Here is an example of map merging : 

Before merge :
<img width="1803" height="628" alt="image" src="https://github.com/user-attachments/assets/a1476b67-fba0-408c-831e-c97b808a15dd" />

After merge : 
<img width="1803" height="628" alt="image" src="https://github.com/user-attachments/assets/eaffe48a-f9d8-4ea4-aaf8-0d7ecfb19db4" />

### multi-robot interactions
ROS2 can be tricky with multi robots, especially when it comes about communications. The most common issue is that every robot is publishing it's map, sensors, and any other information continuously. This causes a network overload that breaks entirely the system. 

An other issue is that manufacturers don't always take into account multi-robot use-case and hard-code the topics name, making difficult for the user to set proper namespaces.

This is why we made the choice to isolate every ROS component for each robot, and only chose what has to go in/out for the robots of the fleet.
The only thing that the robots needs to coordinate i our case is the **map_merge** service. Then we pass the robot on localhost only mode with the environment variable `ROS_LOCALHOST_ONLY` (set to 1). And we use **zenoh** for communication purpose.

`zenoh-bridge-ros2dds` is a translation layer that sits on top of regular DDS (Cyclone DDS here) and bridges it to Zenoh at the network boundary 

With a json configuration file as the one following, we can easily filter the elements we want to let out in the robot:

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

This solves the network overload problem for ROS2 multi-robot implementations. It also remove the need to use a namespace for every node, so we can only namespace the node/topics/sevices we want to share on zenoh.

## Overall
This figure represents the overall process:
<img width="1920" height="1080" alt="schematics explain" src="https://github.com/user-attachments/assets/d84988e0-1f7f-4d8a-907e-da072fe66a17" />

On the left part, we can see the whole blue rectangle as a Limo. In the red part, the ROS2 component (navigation, control, decision) are isolated. The zenoh bridge which is hybrid can communicate, expose topics/services and interact with topics/services exposed by other robots. This is then the only communication channel for robots.


# Video
Quick demo video

https://github.com/user-attachments/assets/41486e80-f545-44c3-babf-f17e4a545ebb

**Limitations :**
On the demo, I realize that I gave the possibility to the robot to go backward on the navigation. That is a mistake, because they can only sense what is at their front side. 
This will be corrected soon and the video will then be updated.
However, we can clearly see the two robots operating simultaneously and merge their maps. one other limitations we can see as mentionned in the "merge_map_server" repo, is that **robots see themselves as obstacles, which leads to ghost obstacle phenomenon**.


For any questions or suggestions, please contact : erwan.martin@univ-lille.fr



