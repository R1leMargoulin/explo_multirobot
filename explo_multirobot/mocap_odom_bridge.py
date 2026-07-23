#!/usr/bin/env python3
"""
Bridges a vrpn_mocap tracked pose into this robot's TF tree, replacing an
unreliable internal odometry source with ground-truth motion capture data.

Usage:
    ros2 run <your_package> mocap_odom_bridge --ros-args \\
        -p tracker_name:=robot1 \\
        -p base_frame:=base_link \\
        -p world_frame:=world \\
        -p odom_frame:=odom
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class MocapOdomBridge(Node):

    def __init__(self):
        super().__init__('mocap_odom_bridge')

        self.declare_parameter('tracker_name', 'robot1')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('odom_frame', 'odom')

        tracker_name = self.get_parameter('tracker_name').value
        self._base_frame = self.get_parameter('base_frame').value
        self._world_frame = self.get_parameter('world_frame').value
        self._odom_frame = self.get_parameter('odom_frame').value

        self._tf_broadcaster = TransformBroadcaster(self)
        self._static_tf_broadcaster = StaticTransformBroadcaster(self)

        # world -> odom, identity
        identity = TransformStamped()
        identity.header.stamp = self.get_clock().now().to_msg()
        identity.header.frame_id = self._world_frame
        identity.child_frame_id = self._odom_frame
        identity.transform.rotation.w = 1.0
        self._static_tf_broadcaster.sendTransform(identity)

        pose_topic = f'/vrpn_mocap/{tracker_name}/pose'
        
        self.create_subscription(
            PoseStamped, pose_topic, self._on_mocap_pose, qos_profile_sensor_data)

        self.get_logger().info(
            f'Bridging {pose_topic} -> TF "{self._odom_frame}" -> "{self._base_frame}" '
            f'(with "{self._world_frame}" == "{self._odom_frame}", identity)'
        )

    def _on_mocap_pose(self, msg: PoseStamped) -> None:
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self._odom_frame
        t.child_frame_id = self._base_frame
        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z
        t.transform.rotation = msg.pose.orientation
        self._tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = MocapOdomBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()