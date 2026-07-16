#!/usr/bin/env python3
"""
Frontier-based exploration node, adapted for our multi-robot merged-map setup.

Adapted from AniArka/Autonomous-Explorer-and-Mapper-ros2-nav2 (explorer.py).
Changes from the original, and why:

1. Subscribes to `merged_map` (relative) instead of the hard-coded absolute
   `/map`. The original subscribes to a global topic that simply doesn't
   exist in our setup -- our map source is each robot's own
   `map_merge_server` node, publishing on its own namespaced `merged_map`.

2. The map subscription now uses the QoS profile (RELIABLE +
   TRANSIENT_LOCAL) that matches what map_merge_server actually publishes
   with. Without this, the subscription silently never receives anything
   -- the exact same QoS-mismatch bug we spent a long time diagnosing
   earlier for RViz.

3. Navigation goals use `frame_id = 'global_odom'` instead of `'map'`.
   Our global_costmap's global_frame is `global_odom` (see the Nav2 params
   in this package's launch file), not the usual single-robot `map` frame,
   so goals must be expressed in that frame to be interpreted correctly.

4. `self.robot_position` is no longer a hard-coded, never-updated
   `(0, 0)` placeholder. It's now kept up to date via a real TF lookup
   (`global_odom` -> `base_footprint`), converted into the merged map's
   own row/col grid indices. Without this fix, frontier selection was
   always measuring distance from the grid's origin corner, never from
   the robot's actual position.
"""
import math

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
        # (row, col) in the merged map's own grid indices, kept up to date
        # from a live TF lookup in explore() below -- not a fixed placeholder.
        self.robot_position = (0, 0)

        self.timer = self.create_timer(exploration_period, self.explore)

    def map_callback(self, msg):
        self.map_data = msg
        self.get_logger().info("Map received")

    def _update_robot_position(self) -> bool:
        """
        Look up the robot's current pose in the shared global frame and
        convert it into the merged map's own row/col grid indices. Returns
        False (and leaves self.robot_position unchanged) if the transform
        isn't available yet.
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

    def find_frontier_clusters(self, map_array):
        """
        Detect frontier cells (free cells adjacent to unknown space) and
        group adjacent ones into connected clusters (8-connectivity, BFS).

        Treating every individual frontier cell as its own target (the
        original approach) makes the robot nibble one grid cell at a time
        along the edge of a single unexplored region, instead of jumping
        to genuinely different unexplored areas. Clustering first, then
        targeting a whole region's centroid, is the standard frontier-
        exploration approach (e.g. explore_lite) and fixes that.
        """
        rows, cols = map_array.shape
        is_frontier = np.zeros((rows, cols), dtype=bool)

        for r in range(1, rows - 1):
            for c in range(1, cols - 1):
                if map_array[r, c] == 0:
                    neighbors = map_array[r-1:r+2, c-1:c+2].flatten()
                    if -1 in neighbors:
                        is_frontier[r, c] = True

        visited = np.zeros((rows, cols), dtype=bool)
        clusters = []

        for r in range(rows):
            for c in range(cols):
                if not is_frontier[r, c] or visited[r, c]:
                    continue

                # BFS to collect the whole connected cluster.
                cluster_cells = []
                queue = [(r, c)]
                visited[r, c] = True
                while queue:
                    cr, cc = queue.pop()
                    cluster_cells.append((cr, cc))
                    for dr in (-1, 0, 1):
                        for dc in (-1, 0, 1):
                            if dr == 0 and dc == 0:
                                continue
                            nr, nc = cr + dr, cc + dc
                            if 0 <= nr < rows and 0 <= nc < cols \
                                    and is_frontier[nr, nc] and not visited[nr, nc]:
                                visited[nr, nc] = True
                                queue.append((nr, nc))

                clusters.append(cluster_cells)

        # Drop tiny clusters (noise / single stray cells).
        min_cluster_size = 5
        clusters = [c for c in clusters if len(c) >= min_cluster_size]

        self.get_logger().info(
            f"Found {len(clusters)} frontier clusters "
            f"(sizes: {sorted((len(c) for c in clusters), reverse=True)[:10]})"
        )
        return clusters

    def choose_frontier_cluster(self, clusters):
        """
        Pick the largest not-yet-visited cluster, by centroid.

        "Not yet visited" is a distance check against previously chosen
        centroids (self.visited_frontiers stores centroids, not exact
        cells) -- blacklisting a whole neighbourhood radius, rather than a
        single cell, is what actually prevents the robot from repeatedly
        re-targeting the same unexplored region one cell at a time.
        """
        blacklist_radius_cells = 10  # ~0.5 m at 0.05 m/cell resolution

        candidates = []
        for cluster in clusters:
            rows_arr = np.array([p[0] for p in cluster])
            cols_arr = np.array([p[1] for p in cluster])
            centroid = (float(rows_arr.mean()), float(cols_arr.mean()))

            too_close_to_visited = any(
                math.hypot(centroid[0] - v[0], centroid[1] - v[1]) < blacklist_radius_cells
                for v in self.visited_frontiers
            )
            if not too_close_to_visited:
                candidates.append((centroid, len(cluster)))

        if not candidates:
            self.get_logger().warning("No valid frontier cluster found")
            return None

        # Prioritize the largest unexplored region.
        candidates.sort(key=lambda item: item[1], reverse=True)
        chosen_centroid, chosen_size = candidates[0]

        self.visited_frontiers.add(chosen_centroid)
        self.get_logger().info(
            f"Chosen frontier cluster centroid: {chosen_centroid}, size: {chosen_size}"
        )
        return chosen_centroid

    def explore(self):
        if self.map_data is None:
            self.get_logger().warning("No map data available")
            return

        if not self._update_robot_position():
            return

        map_array = np.array(self.map_data.data).reshape(
            (self.map_data.info.height, self.map_data.info.width))

        clusters = self.find_frontier_clusters(map_array)

        if not clusters:
            self.get_logger().info("No frontiers found. Exploration complete!")
            return

        chosen_centroid = self.choose_frontier_cluster(clusters)

        if not chosen_centroid:
            self.get_logger().warning("No frontiers to explore")
            return

        goal_x = chosen_centroid[1] * self.map_data.info.resolution \
            + self.map_data.info.origin.position.x
        goal_y = chosen_centroid[0] * self.map_data.info.resolution \
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
