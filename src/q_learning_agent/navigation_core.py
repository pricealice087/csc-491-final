#!/usr/bin/env python3

"""Shared utilities for wall-following reinforcement learning agents."""

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
    """State encoder/decoder and action helpers used by all RL agents."""

    def __init__(
        self,
        *,
        desired_wall_distance: float = 0.60,
        tolerance: float = 0.18,
    ) -> None:
        self.desired_wall_distance = desired_wall_distance
        self.tolerance = tolerance


        # TODO: Fill in 5 discrete actions (v, w). :
        self.actions: Sequence[Action] = (
            Action(0.2, 0.0),    # forward
            Action(0.15, 0.5),   # gentle left
            Action(0.15, -0.5),  # gentle right
            Action(0.0, 1.0),    # sharp left
            Action(0.0, -1.0),   # sharp right
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

    @staticmethod
    def _normalize_angle_into_scan(msg: LaserScan, rad: float) -> float:
        span = msg.angle_max - msg.angle_min
        if span <= 0.0:
            return msg.angle_min

        while rad < msg.angle_min:
            rad += span
        while rad >= msg.angle_max:
            rad -= span
        return rad

    def _idx_for_angle(self, msg: LaserScan, deg: float) -> int:
        rad = self._normalize_angle_into_scan(msg, math.radians(deg))
        idx = int(round((rad - msg.angle_min) / msg.angle_increment))
        return max(0, min(len(msg.ranges) - 1, idx))

    def _min_in_window(self, msg: LaserScan, center_deg: float, half_width_deg: float) -> float:
        rngs = list(msg.ranges)
        n = len(rngs)
        if n == 0:
            return msg.range_max if msg.range_max > 0.0 else 3.5

        c_idx = self._idx_for_angle(msg, center_deg)
        hw_idx = max(1, int(round(abs(half_width_deg) * math.pi / 180.0 / msg.angle_increment)))
        idxs = [(c_idx + k) % n for k in range(-hw_idx, hw_idx + 1)]

        rmin = msg.range_min if msg.range_min > 0.0 else 0.0
        rmax = msg.range_max if msg.range_max > 0.0 else 3.5

        vals = []
        saw_neg_inf = False
        for i in idxs:
            v = rngs[i]
            if v == float('-inf'):
                saw_neg_inf = True
            elif (not math.isnan(v)) and (not math.isinf(v)) and (rmin < v < rmax):
                vals.append(v)

        if vals:
            p10 = float(np.percentile(vals, 10))
            return max(rmin + 1e-3, min(p10, rmax))
        if saw_neg_inf:
            return max(rmin + 1e-3, 0.05)
        return rmax

    def _distance_bin(self, d: float) -> int:
        dw, tol = self.desired_wall_distance, self.tolerance
        if d < dw - 1.5 * tol:
            return 0
        if d < dw - 0.5 * tol:
            return 1
        if d <= dw + 0.5 * tol:
            return 2
        if d <= dw + 1.5 * tol:
            return 3
        return 4

    
    def discretize_scan(self, msg: LaserScan, goal_angle: float) -> Tuple[List[int], Tuple[float, ...]]:
        # Goal direction (discretized)
        if goal_angle < -0.5:
            goal_bin = 0   # goal to right
        elif goal_angle > 0.5:
            goal_bin = 2   # goal to left
        else:
            goal_bin = 1   # goal ahead

        # Obstacle distances
        d_front = self._min_in_window(msg, 0, 15)
        d_left  = self._min_in_window(msg, 90, 20)
        d_right = self._min_in_window(msg, -90, 20)

        b_front = self._distance_bin(d_front)
        b_left  = self._distance_bin(d_left)
        b_right = self._distance_bin(d_right)

        bins = [goal_bin, b_front, b_left, b_right]
        raw = (d_front, d_left, d_right)

        return bins, raw
        
    def compute_reward(
        self,
        raw: Sequence[float],
        steps_since_wall: int = 0,
    ) -> Tuple[float, bool]:
        """
        Compute (reward, done) for one transition.

        Inputs:
          - raw: (d_front, d_front_right, d_right, d_right_far) produced by discretize_scan().
          - steps_since_wall: how many consecutive steps we have been "far from the right wall".
            This counter is updated outside this function in rl_training_common.py by the training loop in _run_episode:
              * if right_dist is large -> steps_since_wall += 1
              * else -> steps_since_wall = 0
            and the episode may be force-terminated when it grows too big. 

        Returns:
          - reward: a scalar reward (can be clipped)
          - done: True if this step should terminate the while loop in _run_episode; otherwise False.
        """

        front, left, right = raw

        # (1) collision check
        if front < 0.2 or left < 0.15 or right < 0.15:
            return -100.0, True

        reward = 0.0

        # (2) obstacle penalty
        reward -= (1.0 / max(front, 0.1)) * 0.5

        # (3) goal progress (requires passing distance externally later)
        # placeholder:
        reward += 1.0

        # (4) time penalty
        reward -= 0.2

        return reward, False

    def vector_to_state(self, vector: Iterable[int]) -> int:
        s = 0
        mult = 1
        for b in vector:
            s += int(b) * mult
            mult *= self.distance_bins
        return s

    def state_to_vector(self, state: int) -> List[int]:
        v = []
        t = int(state)
        for _ in range(self.num_sectors):
            v.append(t % self.distance_bins)
            t //= self.distance_bins
        return v

    def empty_q_table(self, fill: float = 0.0) -> np.ndarray:
        return np.full((self.num_states, self.num_actions), fill, dtype=np.float32)

    def manual_seed_q_table(self) -> np.ndarray:
        """Sample handcrafted policy"""

        q_table = self.empty_q_table(fill=0.05)

        # Action indices (must match self.actions order):
        # A_FWD: forward, A_GL: gentle left, A_GR: gentle right,
        # A_SL: sharp left, A_SR: sharp right
        A_FWD, A_GL, A_GR, A_SL, A_SR = 0, 1, 2, 3, 4

        for state_idx in range(self.num_states):
            f_idx, fr_idx, right_idx, right_far_idx = self.state_to_vector(state_idx)
            q_row = [0.05, 0.05, 0.05, 0.05, 0.05]

            if f_idx <= 2:
                if right_idx <= 1 or fr_idx <= 2:
                    q_row[A_SL] = 1.0  
                else:
                    q_row[A_GL] = 1.0 
                q_table[state_idx, :] = q_row
                continue

            if fr_idx <= 2:
                q_row[A_GL] = 1.0
                q_row[A_SL] = 0.8 if right_idx <= 1 else 0.2
                q_table[state_idx, :] = q_row
                continue

            if right_idx == 0:
                q_row[A_SL] = 1.0

            elif right_idx == 1:
                if fr_idx <= 2:
                    q_row[A_SL] = 1.0
                else:
                    q_row[A_GL] = 0.9
                    q_row[A_FWD] = 0.1

            elif right_idx == 2:
                if fr_idx <= 2:
                    q_row[A_GL] = 0.9
                    q_row[A_FWD] = 0.1
                elif fr_idx >= 3 and f_idx >= 3:
                    # Check for outside corner
                    if right_far_idx >= 3:
                        q_row[A_SR] = 0.7  # Sharp right turn for outside corner
                        q_row[A_GR] = 0.3
                    else:
                        q_row[A_GR] = 0.8
                        q_row[A_FWD] = 0.2
                else:
                    q_row[A_FWD] = 1.0

            elif right_idx == 3:
                if fr_idx >= 2:
                    q_row[A_SR] = 0.9
                    q_row[A_GR] = 0.1
                else:
                    q_row[A_GR] = 0.8
                    q_row[A_FWD] = 0.2

            else:
                # Very far from wall - sharp right to reacquire
                q_row[A_SR] = 1.0

            q_table[state_idx, :] = q_row

        return q_table


__all__ = ['Action', 'WallFollowerCore']
