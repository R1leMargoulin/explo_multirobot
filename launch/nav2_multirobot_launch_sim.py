"""
Nav2 launch, note that a robot's own nav2 launch will be better if available, 
because the parameters will be fine tuned for that specific robots.
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
