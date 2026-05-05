#!/usr/bin/env python3

from __future__ import annotations
import math
from dataclasses import dataclass
from typing import Iterable, List, Sequence, Tuple

import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


@dataclass(frozen=True)
class Action:
    linear: float
    angular: float


class NavigationRLCore:
    def __init__(self, *, desired_wall_distance: float = 0.60, tolerance: float = 0.18):
        self.desired_wall_distance = desired_wall_distance
        self.tolerance = tolerance

        self.actions: Sequence[Action] = (
            Action(0.2, 0.0),
            Action(0.15, 0.5),
            Action(0.15, -0.5),
            Action(0.0, 1.0),
            Action(0.0, -1.0),
        )

        self.num_sectors = 4
        self.distance_bins = 5
        self.num_states = self.distance_bins ** self.num_sectors
        self.num_actions = len(self.actions)

    def action_to_twist(self, action_idx: int) -> Twist:
        action = self.actions[int(action_idx)]
        cmd = Twist()
        cmd.linear.x = action.linear
        cmd.angular.z = action.angular
        return cmd

    def _min_in_window(self, msg: LaserScan, center_deg: float, half_width_deg: float) -> float:
        idx = int((math.radians(center_deg) - msg.angle_min) / msg.angle_increment)
        width = int(math.radians(half_width_deg) / msg.angle_increment)

        values = []
        for i in range(idx - width, idx + width):
            if 0 <= i < len(msg.ranges):
                val = msg.ranges[i]
                if not math.isinf(val) and not math.isnan(val):
                    values.append(val)

        return min(values) if values else msg.range_max

    def _distance_bin(self, d: float) -> int:
        dw, tol = self.desired_wall_distance, self.tolerance
        if d < dw - 1.5 * tol: return 0
        if d < dw - 0.5 * tol: return 1
        if d <= dw + 0.5 * tol: return 2
        if d <= dw + 1.5 * tol: return 3
        return 4

    def discretize_scan(self, msg: LaserScan, goal_angle: float):
        # Goal direction
        if goal_angle < -0.5:
            goal_bin = 0
        elif goal_angle > 0.5:
            goal_bin = 2
        else:
            goal_bin = 1

        d_front = self._min_in_window(msg, 0, 15)
        d_left  = self._min_in_window(msg, 90, 20)
        d_right = self._min_in_window(msg, -90, 20)

        bins = [
            goal_bin,
            self._distance_bin(d_front),
            self._distance_bin(d_left),
            self._distance_bin(d_right)
        ]

        return bins, (d_front, d_left, d_right)

    def vector_to_state(self, vector: Iterable[int]) -> int:
        s = 0
        mult = 1
        for b in vector:
            s += int(b) * mult
            mult *= self.distance_bins
        return s

    def empty_q_table(self, fill: float = 0.0) -> np.ndarray:
        return np.full((self.num_states, self.num_actions), fill, dtype=np.float32)