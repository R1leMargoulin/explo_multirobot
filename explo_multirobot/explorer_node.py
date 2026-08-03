#!/usr/bin/env python3
"""
Frontier-based exploration node, adapted for our multi-robot merged-map setup.

Adapted from AniArka/Autonomous-Explorer-and-Mapper-ros2-nav2 (explorer.py).
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.duration import Duration

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, \
    ExtrapolationException

import numpy as np


class ExplorerNode(Node):

    def __init__(self):
        super().__init__('explorer')
        self.get_logger().info("Explorer Node Started")

        self.declare_parameter('global_frame', 'global_odom')
        self.declare_parameter('robot_base_frame', 'base_footprint')
        self.declare_parameter('exploration_period_sec', 5.0)

        self._global_frame = self.get_parameter('global_frame').value
        self._robot_base_frame = self.get_parameter('robot_base_frame').value
        exploration_period = self.get_parameter('exploration_period_sec').value

        # Must match map_merge_server's publisher QoS, or this subscription
        # silently never receives anything.
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = QoSReliabilityPolicy.RELIABLE
        map_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.map_sub = self.create_subscription(
            OccupancyGrid, 'merged_map', self.map_callback, map_qos)

        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.visited_frontiers = set()

        self.map_data = None
        # (row, col) in the merged map's own grid indices
        self.robot_position = (0, 0)

        self.timer = self.create_timer(exploration_period, self.explore)

    def map_callback(self, msg):
        self.map_data = msg
        self.get_logger().info("Map received")

    def _update_robot_position(self) -> bool:
        """
        robot's current pose in the shared global frame and
        convert it into the merged map's grid indices. 
        Returns False if the transform isn't available yet.
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                self._global_frame, self._robot_base_frame,
                rclpy.time.Time(), timeout=Duration(seconds=1.0))
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            self.get_logger().warning(
                f'Could not look up {self._global_frame} -> {self._robot_base_frame}: {exc}\n'
                f'Frames currently known to this buffer: '
                f'{self.tf_buffer.all_frames_as_yaml()}'
            )
            return False

        x = transform.transform.translation.x
        y = transform.transform.translation.y

        col = int(round((x - self.map_data.info.origin.position.x)
                        / self.map_data.info.resolution))
        row = int(round((y - self.map_data.info.origin.position.y)
                        / self.map_data.info.resolution))
        self.robot_position = (row, col)
        return True

    def navigate_to(self, x, y):
        """Send navigation goal to Nav2."""
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = self._global_frame
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.orientation.w = 1.0

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_msg

        self.get_logger().info(f"Navigating to goal: x={x}, y={y}")

        self.nav_to_pose_client.wait_for_server()

        send_goal_future = self.nav_to_pose_client.send_goal_async(nav_goal)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warning("Goal rejected!")
            return

        self.get_logger().info("Goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_complete_callback)

    def navigation_complete_callback(self, future):
        try:
            result = future.result().result
            self.get_logger().info(f"Navigation completed with result: {result}")
        except Exception as e:
            self.get_logger().error(f"Navigation failed: {e}")

    def find_frontiers(self, map_array):
        """Detect frontiers in the occupancy grid map."""
        frontiers = []
        rows, cols = map_array.shape

        for r in range(1, rows - 1):
            for c in range(1, cols - 1):
                if map_array[r, c] == 0:  # Free cell
                    neighbors = map_array[r-1:r+2, c-1:c+2].flatten()
                    if -1 in neighbors:
                        frontiers.append((r, c))

        self.get_logger().info(f"Found {len(frontiers)} frontiers")
        return frontiers

    def choose_frontier(self, frontiers):
        """Choose the closest frontier to the robot's actual position."""
        robot_row, robot_col = self.robot_position
        min_distance = float('inf')
        chosen_frontier = None

        for frontier in frontiers:
            if frontier in self.visited_frontiers:
                continue

            distance = np.sqrt((robot_row - frontier[0])**2 + (robot_col - frontier[1])**2)
            if distance < min_distance:
                min_distance = distance
                chosen_frontier = frontier

        if chosen_frontier:
            self.visited_frontiers.add(chosen_frontier)
            self.get_logger().info(f"Chosen frontier: {chosen_frontier}")
        else:
            self.get_logger().warning("No valid frontier found")

        return chosen_frontier

    def explore(self):
        if self.map_data is None:
            self.get_logger().warning("No map data available")
            return

        if not self._update_robot_position():
            return

        map_array = np.array(self.map_data.data).reshape(
            (self.map_data.info.height, self.map_data.info.width))

        frontiers = self.find_frontiers(map_array)

        if not frontiers:
            self.get_logger().info("No frontiers found. Exploration complete!")
            return

        chosen_frontier = self.choose_frontier(frontiers)

        if not chosen_frontier:
            self.get_logger().warning("No frontiers to explore")
            return

        goal_x = chosen_frontier[1] * self.map_data.info.resolution \
            + self.map_data.info.origin.position.x
        goal_y = chosen_frontier[0] * self.map_data.info.resolution \
            + self.map_data.info.origin.position.y

        self.navigate_to(goal_x, goal_y)


def main(args=None):
    rclpy.init(args=args)
    explorer_node = ExplorerNode()

    try:
        explorer_node.get_logger().info("Starting exploration...")
        rclpy.spin(explorer_node)
    except KeyboardInterrupt:
        explorer_node.get_logger().info("Exploration stopped by user")
    finally:
        explorer_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()