#!/usr/bin/env python3

from __future__ import annotations
import math
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from navigation_core import NavigationRLCore


class PolicyRunnerNode(Node):
    def __init__(self):
        super().__init__('policy_runner')

        self.core = NavigationRLCore()

        # Parameters
        default_policy = str(Path(__file__).parent / "policy.npy")
        self.policy_path = self.declare_parameter(
            "policy_path", default_policy).value

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )

        # Publisher (Nav2 goal)
        self.goal_pub = self.create_publisher(
            PoseStamped, '/goal_pose', 10)

        # Load Q-table
        self.q_table = np.load(self.policy_path)

        # Robot pose
        self.current_pose = None

        # Target (can later replace with perception)
        self.target_x = 5.0
        self.target_y = 1.0

    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose

    def get_yaw(self, pose):
        q = pose.orientation
        return math.atan2(
            2.0 * (q.w * q.z),
            1.0 - 2.0 * (q.z * q.z)
        )

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def compute_goal_angle(self):
        if self.current_pose is None:
            return 0.0

        dx = self.target_x - self.current_pose.position.x
        dy = self.target_y - self.current_pose.position.y

        robot_yaw = self.get_yaw(self.current_pose)
        angle_to_goal = math.atan2(dy, dx)

        return self.normalize_angle(angle_to_goal - robot_yaw)

    def scan_callback(self, msg):
        if self.current_pose is None:
            return

        goal_angle = self.compute_goal_angle()

        bins, _ = self.core.discretize_scan(msg, goal_angle)
        state = self.core.vector_to_state(bins)

        action_idx = int(np.argmax(self.q_table[state]))
        action = self.core.actions[action_idx]

        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()

        # Step forward in direction influenced by RL action
        step = 1.0
        yaw = self.get_yaw(self.current_pose)

        new_yaw = yaw + action.angular

        goal.pose.position.x = (
            self.current_pose.position.x +
            step * math.cos(new_yaw)
        )

        goal.pose.position.y = (
            self.current_pose.position.y +
            step * math.sin(new_yaw)
        )

        # Set orientation toward movement direction
        goal.pose.orientation.z = math.sin(new_yaw / 2.0)
        goal.pose.orientation.w = math.cos(new_yaw / 2.0)

        self.goal_pub.publish(goal)


def main():
    rclpy.init()
    node = PolicyRunnerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()