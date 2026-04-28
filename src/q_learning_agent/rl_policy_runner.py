#!/usr/bin/env python3

"""Common policy runner for executing learned Q-tables."""

from __future__ import annotations

from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from geometry_msgs.msg import PoseStamped

from navigation_core import NavigationRLCore


class PolicyRunnerNode(Node):
    def __init__(self, *, node_name: str, default_policy_filename: str):
        super().__init__(node_name)

        self.core = NavigationRLCore()

        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        default_dir = Path(__file__).resolve().parent / 'policies'
        default_policy = (default_dir / default_policy_filename).with_suffix('.npy')

        policy_param = self.declare_parameter('policy_path', str(default_policy)).value
        self.policy_path = Path(policy_param).expanduser()

        self.republish_period = float(self.declare_parameter('republish_period', 0.25).value)

        self.q_table = self._load_q_table(self.policy_path)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self._scan_callback, 10)
        self.republish_timer = self.create_timer(self.republish_period, self._republish_last_command)

        self.last_cmd: Twist = Twist()

        self.get_logger().info(f'Loaded Q-table from {self.policy_path}')

    def _load_q_table(self, path: Path) -> np.ndarray:
        if not path.exists():
            raise FileNotFoundError(f'Policy file {path} does not exist')
        data = np.load(path)
        if data.shape != (self.core.num_states, self.core.num_actions):
            raise ValueError(f'Policy file {path} has shape {data.shape}, expected {(self.core.num_states, self.core.num_actions)}')
        return data.astype(np.float32, copy=False)

    def _scan_callback(self, msg: LaserScan) -> None:
        bins, _ = self.core.discretize_scan(msg)
        state = self.core.vector_to_state(bins)

        row = self.q_table[state, :]
        action_idx = int(np.argmax(row))

        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()

        if action_idx == 0:
            goal.pose.position.x = 1.0
            goal.pose.position.y = 0.0
        elif action_idx == 1:
            goal.pose.position.x = 1.0
            goal.pose.position.y = 1.0
        elif action_idx == 2:
            goal.pose.position.x = 1.0
            goal.pose.position.y = -1.0
        elif action_idx == 3:
            goal.pose.position.x = 0.5
            goal.pose.position.y = 1.5
        else:
            goal.pose.position.x = 0.5
            goal.pose.position.y = -1.5

        self.goal_pub.publish(goal)
        
    def _republish_last_command(self) -> None:
        self.cmd_pub.publish(self.last_cmd)


__all__ = ['PolicyRunnerNode']
