import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.launch_context import LaunchContext
from launch_ros.actions import Node


TF_REMAPPINGS = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

# Only the Nav2 nodes actually needed for NavigateToPose-based exploration.
# No map_server, no amcl: SLAM already provides live localization.
LIFECYCLE_NODE_NAMES = [
    'controller_server',
    'planner_server',
    'smoother_server',
    'behavior_server',
    'bt_navigator',
    'velocity_smoother',
]
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


def launch_setup(context: LaunchContext, *args, **kwargs):
    namespace = context.launch_configurations['namespace']
    x_pose = context.launch_configurations['x_pose']
    y_pose = context.launch_configurations['y_pose']
    yaw_pose = context.launch_configurations['yaw_pose']
    peer_service = context.launch_configurations['peer_service']
    use_sim_time = context.launch_configurations['use_sim_time'].lower() == 'true'
    sync_period_sec = context.launch_configurations['sync_period_sec']
    slam_executable = context.launch_configurations['slam_executable']
    slam_params_file = context.launch_configurations['slam_params_file']
    nav2_params_file_path = context.launch_configurations['nav2_params_file']

    actions = []

    # Static TF anchor: global_odom -> map, at this robot's known pose.
    actions.append(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='global_map_tf_broadcaster',
        namespace=namespace,
        remappings=TF_REMAPPINGS,
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '--x', x_pose, '--y', y_pose, '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', yaw_pose,
            '--frame-id', 'global_odom',
            '--child-frame-id', 'map',
        ],
    ))

    #    slam_toolbox's stock sync node/config -- override
    #    slam_executable/slam_params_file for your own robot's tuned setup.
    #    Whatever config you point at, make sure scan_topic is relative
    #    ("scan", not "/scan", "map" not "/map", etc.)
    actions.append(Node(
        package='slam_toolbox',
        executable=slam_executable,
        name='slam_toolbox',
        namespace=namespace,
        output='screen',
        remappings=TF_REMAPPINGS,
        parameters=[slam_params_file, {'use_sim_time': use_sim_time}],
    ))

    # This robot's map-merge server (persistent fused-map state).
    actions.append(Node(
        package='map_merge_server',
        executable='map_merge_server_node',
        name='map_merge_server',
        namespace=namespace,
        output='screen',
        parameters=[{
            'map_origin_x': float(x_pose),
            'map_origin_y': float(y_pose),
            'map_origin_theta': float(yaw_pose),
            'merged_frame_id': 'global_odom',
        }],
    ))

    # merge server clients
    peer_services = [p.strip() for p in peer_service.split(',') if p.strip()]
    for i, peer in enumerate(peer_services):
        actions.append(Node(
            package='map_merge_server',
            executable='merge_client_example',
            name=f'merge_client_{i}',
            namespace=namespace,
            output='screen',
            parameters=[{
                'peer_service': peer,
                'sync_period_sec': float(sync_period_sec),
            }],
        ))

    # Nav2
    bt_xml_path = os.path.join(
        get_package_share_directory('nav2_bt_navigator'),
        'behavior_trees', 'navigate_to_pose_w_replanning_and_recovery.xml',
    )
    bt_override_fd, bt_override_path = tempfile.mkstemp(
        suffix=f'_{namespace}_bt_navigator_override.yaml')
    with os.fdopen(bt_override_fd, 'w') as f:
        f.write(
            '/**:\n'
            '  bt_navigator:\n'
            '    ros__parameters:\n'
            f'      default_nav_to_pose_bt_xml: "{bt_xml_path}"\n'
        )

    costmap_override_fd, costmap_override_path = tempfile.mkstemp(
        suffix=f'_{namespace}_global_costmap_override.yaml')
    with os.fdopen(costmap_override_fd, 'w') as f:
        f.write(
            '/**:\n'
            '  global_costmap:\n'
            '    global_costmap:\n'
            '      ros__parameters:\n'
            '        static_layer:\n'
            f'          map_topic: "/{namespace}/merged_map"\n'
        )

    for package, executable in NAV2_SERVERS:
        cmd = ['ros2', 'run', package, executable,
              '--ros-args',
              '--params-file', nav2_params_file_path]
        if executable == 'bt_navigator':
            cmd += ['--params-file', bt_override_path]
        if executable == 'planner_server':
            cmd += ['--params-file', costmap_override_path]
        cmd += ['-r', f'__ns:=/{namespace}',
               '-r', '/tf:=tf',
               '-r', '/tf_static:=tf_static']
        cmd += EXTRA_REMAPS.get(executable, [])

        actions.append(ExecuteProcess(cmd=cmd, name=executable, output='screen'))

    actions.append(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        namespace=namespace,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': LIFECYCLE_NODE_NAMES,
        }],
    ))

    # Frontier exploration (local behavior)
    actions.append(Node(
        package='explo_multirobot',
        executable='explorer_node',
        name='explorer',
        namespace=namespace,
        output='screen',
        remappings=TF_REMAPPINGS,
        parameters=[{
            'global_frame': 'global_odom',
            'robot_base_frame': 'base_footprint',
            'exploration_period_sec': float(sync_period_sec),
            'use_sim_time': use_sim_time,
        }],
    ))

    return actions


def generate_launch_description():
    explo_dir = get_package_share_directory('explo_multirobot')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='robot1',
            description='Namespace for this robot (e.g. robot1, robot2)'),
        DeclareLaunchArgument(
            'x_pose', default_value='0.0',
            description="This robot's known starting X position in the shared global frame"),
        DeclareLaunchArgument(
            'y_pose', default_value='0.0',
            description="This robot's known starting Y position in the shared global frame"),
        DeclareLaunchArgument(
            'yaw_pose', default_value='0.0',
            description="This robot's known starting yaw in the shared global frame"),
        DeclareLaunchArgument(
            'peer_service', default_value='/robot2/merge_map',
            description="Comma-separated list of absolute merge_map service names to "
                        "sync with (e.g. '/robot2/merge_map,/robot3/merge_map'). One "
                        "merge_client_example instance is started per entry."),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock if true (false for real robots)'),
        DeclareLaunchArgument(
            'sync_period_sec', default_value='5.0',
            description='Gossip sync period AND exploration re-evaluation period'),
        DeclareLaunchArgument(
            'slam_executable', default_value='sync_slam_toolbox_node',
            description='slam_toolbox executable to use (sync_slam_toolbox_node, '
                        'async_slam_toolbox_node, ...) -- match whatever your robot uses'),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(
                get_package_share_directory('slam_toolbox'),
                'config', 'mapper_params_online_async.yaml'),
            description="Full path to slam_toolbox's params file -- override with your "
                        "own robot's tuned config. Make sure scan_topic is relative ('scan')."),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=os.path.join(explo_dir, 'config', 'nav2_multirobot_params.yaml'),
            description='Full path to the Nav2 parameters file for this robot'),
        OpaqueFunction(function=launch_setup),
    ])