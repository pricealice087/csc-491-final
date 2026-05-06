#!/usr/bin/env python3

import sys
import os
sys.path.insert(0, os.path.dirname(__file__))

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

import numpy as np

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

from astar import astar


class AStarNode(Node):

    def __init__(self):
        super().__init__('astar_node')

        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, map_qos)

        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)

        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )

        self.path_pub = self.create_publisher(Path, '/astar_path', 10)

        self.map = None
        self.robot_pose = None

    # ----------------------------
    # CALLBACKS
    # ----------------------------

    def map_callback(self, msg):
        self.map = msg

    def pose_callback(self, msg):
        self.robot_pose = msg.pose.pose

    def goal_callback(self, goal_msg):
        if self.map is None or self.robot_pose is None:
            return

        grid, origin, res = self.convert_map(self.map)

        start = self.world_to_grid(
            self.robot_pose.position.x,
            self.robot_pose.position.y,
            origin,
            res
        )

        goal = self.world_to_grid(
            goal_msg.pose.position.x,
            goal_msg.pose.position.y,
            origin,
            res
        )

        path = astar(grid, start, goal)

        if path:
            self.publish_path(path, origin, res)
        
        if path is None:
            self.get_logger().warn("No path found")
            return

    # ----------------------------
    # MAP CONVERSION
    # ----------------------------

    def convert_map(self, map_msg):
        width = map_msg.info.width
        height = map_msg.info.height
        resolution = map_msg.info.resolution
        origin = map_msg.info.origin

        data = np.array(map_msg.data).reshape((height, width))
        data = np.flipud(data)

        grid = []
        for y in range(height):
            row = []
            for x in range(width):
                val = data[y][x]

                # 0 = free, 100 = obstacle
                if val >= 50:
                    row.append(1)
                else:
                    row.append(0)

            grid.append(row)

        return grid, origin, resolution

    # ----------------------------
    # WORLD ↔ GRID TRANSFORM
    # ----------------------------

    def world_to_grid(self, x, y, origin, res):
        gx = int((x - origin.position.x) / res)
        gy = int((y - origin.position.y) / res)
        return (gx, gy)

    def grid_to_world(self, x, y, origin, res):
        wx = x * res + origin.position.x
        wy = y * res + origin.position.y
        return wx, wy

    # ----------------------------
    # PATH PUBLISHING
    # ----------------------------

    def publish_path(self, path, origin, res):
        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()

        for x, y in path:
            wx, wy = self.grid_to_world(x, y, origin, res)

            pose = PoseStamped()
            pose.header = msg.header

            pose.pose.position.x = float(wx)
            pose.pose.position.y = float(wy)
            pose.pose.position.z = 0.0

            pose.pose.orientation.w = 1.0  # facing forward

            msg.poses.append(pose)

        self.path_pub.publish(msg)


def main():
    rclpy.init()
    node = AStarNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()