"""
Starts one frontier-exploration node per robot, namespaced.

Adjust ROBOT_NAMES to match the robots you actually launched via
multi_tb3_simulation_launch.py.
"""
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import GroupAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


ROBOT_NAMES = ['robot1', 'robot2']


def generate_launch_description():
    declare_period_cmd = DeclareLaunchArgument(
        'exploration_period_sec', default_value='5.0',
        description='How often (seconds) each robot re-evaluates frontiers'
    )
    exploration_period = LaunchConfiguration('exploration_period_sec')

    ld = LaunchDescription()
    ld.add_action(declare_period_cmd)

    for name in ROBOT_NAMES:
        group = GroupAction([
            PushRosNamespace(name),
            Node(
                package='explo_multirobot',
                executable='explorer_node',
                name='explorer',
                output='screen',
                parameters=[{
                    'global_frame': 'global_odom',
                    'robot_base_frame': 'base_footprint',
                    'exploration_period_sec': exploration_period,
                    'use_sim_time': True,
                }],
                # tf2_ros's TransformListener subscribes on the absolute
                # /tf and /tf_static topics internally -- PushRosNamespace
                # alone does not affect it (same recurring issue already
                # fixed elsewhere in this project for slam_toolbox,
                # robot_state_publisher, etc.). Without this explicit
                # remap, this node's TF buffer stays permanently empty.
                remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            ),
        ])
        ld.add_action(group)
    return ld
