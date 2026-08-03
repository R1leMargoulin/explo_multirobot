#!/usr/bin/env python3
"""
Converts a vrpn_mocap PoseStamped into a PoseWithCovarianceStamped, which
is what robot_localization's ekf_filter_node expects for a `poseN` input
(vrpn_mocap itself only publishes plain PoseStamped, with no covariance).

Usage:
    ros2 run <your_package> mocap_pose_bridge --ros-args \\
        -p tracker_name:=robot1 \\
        -p output_topic:=/mocap_pose \\
        -p position_variance:=0.0001 \\
        -p orientation_variance:=0.0001
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped


class MocapPoseBridge(Node):

    def __init__(self):
        super().__init__('mocap_pose_bridge')

        self.declare_parameter('tracker_name', 'robot1')
        self.declare_parameter('output_topic', '/mocap_pose')
        
        self.declare_parameter('position_variance', 0.0001)     # m^2
        self.declare_parameter('orientation_variance', 0.0001)  # rad^2

        tracker_name = self.get_parameter('tracker_name').value
        output_topic = self.get_parameter('output_topic').value
        self._pos_var = self.get_parameter('position_variance').value
        self._orient_var = self.get_parameter('orientation_variance').value

        self._pub = self.create_publisher(PoseWithCovarianceStamped, output_topic, 10)

        pose_topic = f'/vrpn_mocap/{tracker_name}/pose'
        # vrpn_mocap publishes with best-effort QoS by default
        self.create_subscription(
            PoseStamped, pose_topic, self._on_mocap_pose, qos_profile_sensor_data)

        self.get_logger().info(f'Bridging {pose_topic} -> {output_topic}')

    def _on_mocap_pose(self, msg: PoseStamped) -> None:
        out = PoseWithCovarianceStamped()
        out.header = msg.header
        out.pose.pose = msg.pose

        cov = [0.0] * 36
        cov[0] = self._pos_var       # x
        cov[7] = self._pos_var       # y
        cov[14] = self._pos_var      # z
        cov[21] = self._orient_var   # roll
        cov[28] = self._orient_var   # pitch
        cov[35] = self._orient_var   # yaw
        out.pose.covariance = cov

        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = MocapPoseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()