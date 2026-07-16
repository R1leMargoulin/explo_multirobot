"""
Starts the minimal set of Nav2 nodes needed for point-to-point navigation
(controller, planner, smoother, behavior/recovery, bt_navigator, velocity
smoother) once per robot -- NO map_server, NO amcl (slam_toolbox already
provides live localization per robot), and NO route_server /
waypoint_follower / collision_monitor / opennav_docking (not needed for
basic NavigateToPose-based frontier exploration).

IMPORTANT -- history of why this file looks the way it does:

We tried, in order: nav2_bringup's own navigation_launch.py (RewrittenYaml),
then launch_ros LifecycleNode(parameters=[params_file]), then
LifecycleNode(arguments=['--params-file', ...]). ALL FOUR attempts reliably
corrupted nested parameters (FollowPath.critics came back empty: "No
critics defined for FollowPath").

Direct comparison of the actual process command lines finally found the
real cause: a manual `ros2 run nav2_controller controller_server --ros-args
--params-file <file>` (single --ros-args block) works correctly, but
launch_ros's Node/LifecycleNode actions ALWAYS append their own SEPARATE,
second `--ros-args` block for namespace/name/remappings, regardless of
whether params are passed via `parameters=` or `arguments=`. The resulting
command line looks like:

    controller_server --ros-args --params-file X --ros-args -r __ns:=... -r /tf:=tf ...

i.e. TWO separate --ros-args blocks. rcl_yaml_param_parser evidently loses
some nested list parameters (like FollowPath.critics) when the
--params-file argument sits in an earlier --ros-args block than the
node's remapping arguments, even though the same file loads perfectly
fine when everything is in a single block.

The fix: stop using Node/LifecycleNode for these six servers entirely, and
use plain ExecuteProcess instead, building the ENTIRE argv ourselves (via
`ros2 run <pkg> <executable>`) with exactly ONE --ros-args block --
mirroring the confirmed-working manual invocation exactly. ExecuteProcess
does not add any implicit namespace/remapping logic of its own, so the
robot namespace and /tf, /tf_static, cmd_vel remaps are embedded directly
in our own argument list instead of relying on PushRosNamespace (which has
no effect on a raw ExecuteProcess anyway).

lifecycle_manager itself is unaffected by this bug (its only parameters are
a small literal dict, no YAML file, no nested lists) and is kept as a
regular Node, wrapped in PushRosNamespace like the rest of this project.
"""
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, GroupAction
from launch_ros.actions import Node, PushRosNamespace


ROBOT_NAMES = ['robot1', 'robot2']

LIFECYCLE_NODE_NAMES = [
    'controller_server',
    'planner_server',
    'smoother_server',
    'behavior_server',
    'bt_navigator',
    'velocity_smoother',
]

# (package, executable) pairs, one ExecuteProcess per entry, per robot.
NAV2_SERVERS = [
    ('nav2_controller', 'controller_server'),
    ('nav2_planner', 'planner_server'),
    ('nav2_smoother', 'smoother_server'),
    ('nav2_behaviors', 'behavior_server'),
    ('nav2_bt_navigator', 'bt_navigator'),
    ('nav2_velocity_smoother', 'velocity_smoother'),
]

# Extra -r flags needed per executable, beyond the common TF ones every
# node gets. cmd_vel_nav/cmd_vel renaming matches nav2_bringup's own
# convention (controller/behavior publish on cmd_vel_nav, velocity_smoother
# republishes the smoothed result on the real cmd_vel).
EXTRA_REMAPS = {
    'controller_server': ['-r', 'cmd_vel:=cmd_vel_nav'],
    'behavior_server': ['-r', 'cmd_vel:=cmd_vel_nav'],
    'velocity_smoother': ['-r', 'cmd_vel:=cmd_vel_nav', '-r', 'cmd_vel_smoothed:=cmd_vel'],
}


def generate_launch_description():
    bringup_dir = get_package_share_directory('explo_multirobot')
    params_file_path = os.path.join(bringup_dir, 'config', 'nav2_multirobot_params.yaml')

    bt_xml_path = os.path.join(
        get_package_share_directory('nav2_bt_navigator'),
        'behavior_trees', 'navigate_to_pose_w_replanning_and_recovery.xml',
    )
    bt_override_fd, bt_override_path = tempfile.mkstemp(suffix='_bt_navigator_override.yaml')
    with os.fdopen(bt_override_fd, 'w') as f:
        f.write(
            '/**:\n'
            '  bt_navigator:\n'
            '    ros__parameters:\n'
            f'      default_nav_to_pose_bt_xml: "{bt_xml_path}"\n'
        )

    ld = LaunchDescription()

    for name in ROBOT_NAMES:
        # Costmap layer plugins (like StaticLayer) create their OWN
        # sub-namespace based on the layer name (e.g. "global_costmap"),
        # so a relative map_topic like "merged_map" resolves to
        # /robotN/global_costmap/merged_map -- NOT /robotN/merged_map,
        # where map_merge_server actually publishes. Nothing publishes to
        # the wrongly-resolved topic, hence "no map received". Fix: give
        # planner_server (which hosts global_costmap) an absolute,
        # per-robot-resolved override for this one parameter, the same
        # technique used for bt_navigator's BT xml path above.
        costmap_override_fd, costmap_override_path = tempfile.mkstemp(
            suffix=f'_{name}_global_costmap_override.yaml')
        with os.fdopen(costmap_override_fd, 'w') as f:
            f.write(
                '/**:\n'
                '  global_costmap:\n'
                '    global_costmap:\n'
                '      ros__parameters:\n'
                '        static_layer:\n'
                f'          map_topic: "/{name}/merged_map"\n'
            )

        for package, executable in NAV2_SERVERS:
            cmd = ['ros2', 'run', package, executable,
                  '--ros-args',
                  '--params-file', params_file_path]
            if executable == 'bt_navigator':
                cmd += ['--params-file', bt_override_path]
            if executable == 'planner_server':
                cmd += ['--params-file', costmap_override_path]
            cmd += ['-r', f'__ns:=/{name}',
                   '-r', '/tf:=tf',
                   '-r', '/tf_static:=tf_static']
            cmd += EXTRA_REMAPS.get(executable, [])

            ld.add_action(ExecuteProcess(
                cmd=cmd,
                name=executable,
                output='screen',
            ))

        ld.add_action(GroupAction([
            PushRosNamespace(name),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                namespace='',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'autostart': True,
                    'node_names': LIFECYCLE_NODE_NAMES,
                }],
            ),
        ]))

    return ld
