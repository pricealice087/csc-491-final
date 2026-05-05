#!/usr/bin/env python3

"""Base trainer node shared by Q-learning and SARSA implementations."""

from __future__ import annotations

import math
import time
from pathlib import Path
from typing import List, Sequence, Tuple

import matplotlib.pyplot as plt
import numpy as np
import rclpy

from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose

from .navigation_core import NavigationRLCore
# from .wall_geometry import WallGeometry


StartPose = Tuple[float, float, float]


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Convert quaternion to yaw angle."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

episodes = []
rewards = []

class DiscreteTrainerNode(Node):
    """Implements shared infrastructure for episodic TD control algorithms."""

    DEFAULT_START_POSES: Sequence[StartPose] = (
        (-3.4, -1.0, -0.5 * math.pi),
        (-2.0, -3.4, 0.0),
        (+3.4, +2.0, +0.5 * math.pi),
        (-2.0, +3.4, -math.pi),
        (-2.0, +2.6, 0.0),
        (+1.0, +2.6, 0.0),
        (+2.6, +1.0, -0.5 * math.pi),
        (-1.0, -1.4, 0.0),
    )


    def __init__(self, *, node_name: str, algorithm: str, default_policy_filename: str):
        super().__init__(node_name)

        self.algorithm = algorithm
        self.core = WallFollowerCore()

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self._scan_callback, 10)
        self.reset_client = self.create_client(SetEntityPose, '/world/default/set_pose')

        self.seed = int(self.declare_parameter('seed', 12345).value)
        self.rng = np.random.default_rng(self.seed)

        self.alpha = float(self.declare_parameter('alpha', 0.10).value)
        self.gamma = float(self.declare_parameter('gamma', 0.98).value)
        self.epsilon_start = float(self.declare_parameter('epsilon_start', 0.80).value)
        self.epsilon_end = float(self.declare_parameter('epsilon_end', 0.05).value)
        self.epsilon_decay = float(self.declare_parameter('epsilon_decay', 0.995).value)

        self.episodes = int(self.declare_parameter('episodes', 350).value)
        self.max_steps = int(self.declare_parameter('max_steps', 600).value)
        self.step_duration = float(self.declare_parameter('step_duration', 0.25).value)
        self.save_every = max(1, int(self.declare_parameter('save_every', 25).value))
        self.log_every = max(1, int(self.declare_parameter('log_every', 10).value))
        self.start_jitter = float(self.declare_parameter('start_pose_jitter', 0.20).value)
        self.reset_min_scans = max(1, int(self.declare_parameter('reset_min_scans', 3).value))

        self.entity_name = self.declare_parameter('entity_name', 'burger').value

        default_policy_dir = Path(__file__).resolve().parent.parent / 'policies'
        self.policy_dir = Path(self.declare_parameter('policy_dir', str(default_policy_dir)).value).expanduser()
        self.policy_dir.mkdir(parents=True, exist_ok=True)

        default_policy_filename = self.declare_parameter('policy_filename', default_policy_filename).value
        self.policy_path = (self.policy_dir / default_policy_filename).with_suffix('.npy')
        self.metrics_path = self.policy_dir / f'{self.policy_path.stem}_training_stats.npz'

        world_file_param = self.declare_parameter('world_file', '').value
        self.wall_geometry = self._load_wall_geometry(world_file_param)

        generated = self._generate_start_poses_from_geometry()
        if generated:
            self.start_poses = generated
            self.get_logger().info(f'Using {len(self.start_poses)} geometry-based start poses')
        else:
            self.start_poses = list(self.DEFAULT_START_POSES)

        self.init_mode = self.declare_parameter('init_mode', 'manual').value
        init_q_path_param = self.declare_parameter('init_q_path', '').value
        self.q_table = self._initialize_q_table(init_q_path_param)

        self.episode_rewards: List[float] = []
        self.episode_lengths: List[int] = []

        self.last_scan: LaserScan | None = None
        self.last_scan_stamp: int | None = None
        self.steps_since_wall: int = 0

    @staticmethod
    def _wrap_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def _resolve_world_file(self, world_file_param: str) -> Path | None:
        if world_file_param:
            path = Path(world_file_param).expanduser().resolve()
            if path.exists():
                return path
            self.get_logger().warning(f'world_file parameter set to {world_file_param} but file not found')

        pkg_root = Path(__file__).resolve().parent.parent
        candidate = (pkg_root / 'worlds' / 'largemaze.world').resolve()
        if candidate.exists():
            return candidate
        self.get_logger().warning(f'Could not find default world file at {candidate}')
        return None


    def _load_wall_geometry(self, world_file_param: str) -> WallGeometry | None:
        world_path = self._resolve_world_file(world_file_param)
        if world_path is None:
            return None
        try:
            geometry = WallGeometry(str(world_path))
            self.get_logger().info(f'Loaded {len(geometry.walls)} walls from {world_path}')
            return geometry
        except Exception as exc: 
            self.get_logger().warning(f'Failed to load wall geometry from {world_path}: {exc}')
            return None

    def _generate_start_poses_from_geometry(self) -> List[StartPose]:
        if self.wall_geometry is None:
            return []

        poses: List[StartPose] = []
        desired = self.core.desired_wall_distance
        clearance = 0.05

        for wall in self.wall_geometry.walls:
            heading = self._wrap_angle(wall.yaw)
            right_vec = np.array([math.sin(heading), -math.cos(heading)])
            offset = desired + (wall.thickness / 2.0) + clearance

            p1, p2 = wall.get_segment_endpoints()
            samples = (0.2, 0.8) if np.linalg.norm(p2 - p1) > 0.5 else (0.5,)

            for frac in samples:
                anchor = p1 + frac * (p2 - p1)

                pos_primary = anchor - right_vec * offset
                heading_primary = heading
                if self.wall_geometry.is_in_track(pos_primary, tolerance=0.05):
                    poses.append((float(pos_primary[0]), float(pos_primary[1]), heading_primary))

                pos_alt = anchor + right_vec * offset
                heading_alt = self._wrap_angle(heading + math.pi)
                if self.wall_geometry.is_in_track(pos_alt, tolerance=0.05):
                    poses.append((float(pos_alt[0]), float(pos_alt[1]), heading_alt))

        unique: List[StartPose] = []
        seen: set[tuple[float, float, float]] = set()
        for pose in poses:
            key = (
                round(pose[0], 2),
                round(pose[1], 2),
                round(self._wrap_angle(pose[2]), 2),
            )
            if key in seen:
                continue
            seen.add(key)
            unique.append((pose[0], pose[1], self._wrap_angle(pose[2])))

        return unique

    def _initialize_q_table(self, init_q_path_param: str) -> np.ndarray:
        init_mode = (self.init_mode or '').lower()
        if init_mode == 'manual':
            return self.core.manual_seed_q_table()
        if init_mode == 'zeros':
            return self.core.empty_q_table()
        if init_mode == 'file':
            candidate = Path(init_q_path_param).expanduser() if init_q_path_param else self.policy_path
            if not candidate.exists():
                self.get_logger().warning(f'init_mode=file but {candidate} does not exist; using zeros')
                return self.core.empty_q_table()
            data = np.load(candidate)
            if data.shape != (self.core.num_states, self.core.num_actions):
                raise ValueError(f'Loaded Q-table shape {data.shape} does not match expected {(self.core.num_states, self.core.num_actions)}')
            return data.astype(np.float32, copy=False)
        raise ValueError(f'Unknown init_mode: {self.init_mode}')

    def _scan_callback(self, msg: LaserScan) -> None:
        stamp = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        self.last_scan = msg
        self.last_scan_stamp = stamp

    def wait_for_scan(self, *, timeout_sec: float, require_new: bool = False, last_stamp: int | None = None) -> LaserScan | None:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            remaining = max(0.0, deadline - time.monotonic())
            rclpy.spin_once(self, timeout_sec=min(0.05, remaining))
            if self.last_scan is None:
                continue
            if require_new and last_stamp is not None and self.last_scan_stamp == last_stamp:
                continue
            return self.last_scan
        return None

    def stop_robot(self) -> None:
        self.cmd_pub.publish(Twist())

    def _sample_start_pose(self, episode_idx: int) -> StartPose:
        base = self.start_poses[episode_idx % len(self.start_poses)]
        jitter_xy = self.start_jitter
        x = base[0] + self.rng.uniform(-jitter_xy, jitter_xy)
        y = base[1] + self.rng.uniform(-jitter_xy, jitter_xy)
        yaw = base[2] + self.rng.uniform(-0.15, 0.15)
        return (x, y, yaw)

    def reset_environment(self, episode_idx: int) -> None:
        pose = self._sample_start_pose(episode_idx)
        if not self.reset_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('SetEntityPose service not available')
            return

        req = SetEntityPose.Request()
        req.entity = Entity(name=self.entity_name, type=Entity.MODEL)
        req.pose.position.x = pose[0]
        req.pose.position.y = pose[1]
        req.pose.position.z = 0.05
        qx, qy, qz, qw = yaw_to_quaternion(pose[2])
        req.pose.orientation.x = qx
        req.pose.orientation.y = qy
        req.pose.orientation.z = qz
        req.pose.orientation.w = qw

        future = self.reset_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        result = future.result()
        if not result or not bool(result.success):
            self.get_logger().warning(f'Failed to reset robot pose for episode {episode_idx}')

        self.stop_robot()
        time.sleep(0.1)

        self.last_scan = None
        self.last_scan_stamp = None

        self._wait_for_reset_observations(required_scans=self.reset_min_scans)
        self.steps_since_wall = 0

    def epsilon_for_episode(self, episode_idx: int) -> float:
        eps = self.epsilon_start * (self.epsilon_decay ** episode_idx)
        return float(max(self.epsilon_end, eps))
    
    def select_action(self, state_idx: int, epsilon: float) -> int:
        # TODO (epsilon-greedy action selection):
        # - With probability `epsilon`, pick a random action index in [0, num_actions).
        # - Otherwise, pick a greedy action using the Q-values for this state:
        #     row = q_table[state_idx, :]
        #   Choose among all actions that achieve the maximum Q-value (tie-break randomly).
        # - Return an int action index.
        # exploration
        if self.rng.random() < epsilon:
            return int(self.rng.integers(self.core.num_actions))

        # exploitation
        row = self.q_table[state_idx, :]
        max_q = np.max(row)

        best_actions = np.flatnonzero(row == max_q)

        return int(self.rng.choice(best_actions))


    def save_policy(self) -> None:
        np.save(self.policy_path, self.q_table)

    def save_metrics(self) -> None:
        np.savez(
            self.metrics_path,
            rewards=np.asarray(self.episode_rewards, dtype=np.float32),
            lengths=np.asarray(self.episode_lengths, dtype=np.int32),
            alpha=self.alpha,
            gamma=self.gamma,
            epsilon_start=self.epsilon_start,
            epsilon_end=self.epsilon_end,
            epsilon_decay=self.epsilon_decay,
            algorithm=self.algorithm,
            step_duration=self.step_duration,
        )

    def run_training(self) -> None:
        self.get_logger().info(f'Starting {self.algorithm} training for {self.episodes} episodes')

        warmup_scan = self.wait_for_scan(timeout_sec=3.0)
        if warmup_scan is None:
            self.get_logger().warning('Did not receive initial LaserScan before training start')

        for episode_idx in range(self.episodes):
            epsilon = self.epsilon_for_episode(episode_idx)
            reward, steps = self._run_episode(episode_idx, epsilon)
            self.episode_rewards.append(reward)
            self.episode_lengths.append(steps)

            if (episode_idx + 1) % self.log_every == 0:
                self.get_logger().info(
                    f'Episode {episode_idx + 1}/{self.episodes} | '
                    f'reward={reward:.3f} steps={steps} epsilon={epsilon:.3f}'
                )

            if (episode_idx + 1) % self.save_every == 0:
                self.save_policy()

        self.save_policy()
        self.save_metrics()
        self.stop_robot()
        self.get_logger().info(f'Training complete. Policy saved to {self.policy_path}')

    def _run_episode(self, episode_idx: int, epsilon: float) -> Tuple[float, int]:
        # TODO(summary):
        # In this episode loop, you will fill in the key logic for:
        # - Q-learning vs. SARSA action selection and table updates
        # - executing an action and computing the next state
        # - computing reward/termination and applying extra penalties (stagnation, lost-wall)
        # Please follow TODO(1), TODO(2), ... in order to complete this function.

        self.reset_environment(episode_idx)

        observation = self.wait_for_scan(timeout_sec=2.0)
        if observation is None:
            self.get_logger().error(f'Episode {episode_idx}: no observation after reset - sensor stream may have died')
            self.get_logger().error('Stopping training. Please restart Gazebo and try again.')
            raise RuntimeError('LaserScan topic stopped publishing')

        bins, _ = self.core.discretize_scan(observation)
        state = self.core.vector_to_state(bins)
        self.steps_since_wall = 0

        total_reward = 0.0
        steps = 0
        forward_progress = 0.0

        self.current_goal = None
        goal_progress_reward = 0.0

        #TODO:
        # if self.detect_goal:
        #     goal_progress_reward = +10.0 if next_state is closer to goal else -5.0

        STAGNATION_STEPS = 40
        MIN_PROGRESS_PER_CHECK = 0.1
        last_stagnation_check_step = 0
        progress_at_last_check = 0.0

        # TODO(1): If SARSA, choose the initial action now; otherwise set action=None (Q-learning chooses per step).
        if self.algorithm == 'sarsa':
            action = self.select_action(state, epsilon)
        else:
            action = None

        done = False
        while steps < self.max_steps and not done and rclpy.ok():
            steps += 1

            # TODO(2): Select an action when needed:
            # - Q-learning: pick every step
            # - SARSA: reuse `action` unless it is None
            if self.algorithm == 'q_learning':
                action = self.select_action(state, epsilon)
            elif action is None:
                action = self.select_action(state, epsilon)


            prev_stamp = self.last_scan_stamp
            next_obs = self.execute_action(action, prev_stamp)
            if next_obs is None:
                self.get_logger().error(
                    f'Episode {episode_idx} step {steps}: no new scan received - sensor stream may have died'
                )
                self.get_logger().error('Stopping training. Please restart Gazebo and try again.')
                raise RuntimeError('LaserScan topic stopped publishing during episode')

            next_bins, next_raw = self.core.discretize_scan(next_obs)
            # TODO(3): Convert next_bins into a single integer state index for the Q-table.
            next_state = self.core.vector_to_state(next_bins)

            # TODO(4): Compute (reward, done) by using one of the methods provided in rl_core.py, remember to also send the relevant parameters.
            reward, done = self.core.compute_reward(next_raw, self.steps_since_wall)

            action_obj = self.core.actions[action]
            forward_progress += action_obj.linear * self.step_duration

            if steps - last_stagnation_check_step > STAGNATION_STEPS and not done:
                if forward_progress - progress_at_last_check < MIN_PROGRESS_PER_CHECK:
                    self.get_logger().info(
                        f'Episode {episode_idx} ended due to stagnation '
                        f'(progress < {MIN_PROGRESS_PER_CHECK:.2f}m in {STAGNATION_STEPS} steps)'
                    )
                    # TODO(5): Adjust `total_reward` when ending due to stagnation.
                    total_reward -= 5.0
                    reward -= 5.0
                    done = True

                # TODO(6): Update last_stagnation_check_step and progress_at_last_check.
                last_stagnation_check_step = steps
                progress_at_last_check = forward_progress

            right_dist = next_raw[2] if len(next_raw) > 2 else float('inf')

            if right_dist > self.core.desired_wall_distance + 0.3:
                self.steps_since_wall += 1
            else:
                self.steps_since_wall = 0

            if self.steps_since_wall >= 15 and not done:
                # TODO(7): If wall has been lost too long, terminate the episode and add a large penalty.
                # Add it to both total_reward and the current-step reward.
                penalty = -8.0
                total_reward += penalty
                reward += penalty
                done = True

            if self.algorithm == 'sarsa':
                next_action = self.select_action(next_state, epsilon) if not done else None
                self._sarsa_update(state, action, reward, next_state, next_action, done)
                state = next_state
                action = next_action
            else:
                self._q_learning_update(state, action, reward, next_state, done)
                state = next_state
                action = None
            

        episodes.append(episode_idx)
        rewards.append(total_reward)

        plt.clf()
        plt.plot(episodes, rewards)
        plt.xlabel("Episode")
        plt.ylabel("Reward")
        plt.pause(0.01)  # small pause to refresh plot
        plt.show(block=False)
        
        self.stop_robot()
        return float(total_reward), steps

    def execute_action(self, action_idx: int, prev_stamp: int | None) -> LaserScan | None:
        cmd = self.core.action_to_twist(action_idx)
        self.cmd_pub.publish(cmd)
        scan = self.wait_for_scan(timeout_sec=self.step_duration + 0.2, require_new=True, last_stamp=prev_stamp)
        if scan is None:
            scan = self.wait_for_scan(timeout_sec=0.5, require_new=True, last_stamp=prev_stamp)
        return scan

    def _wait_for_reset_observations(self, *, required_scans: int) -> None:
        """Wait for a few fresh scans after teleporting the robot."""
        max_wait = max(2.5, required_scans * (self.step_duration + 0.15))
        deadline = time.monotonic() + max_wait

        scans_seen = 0
        last_seen_stamp: int | None = None

        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

            if self.last_scan_stamp is None:
                continue

            if last_seen_stamp is None or self.last_scan_stamp != last_seen_stamp:
                last_seen_stamp = self.last_scan_stamp
                scans_seen += 1
                if scans_seen >= required_scans:
                    break

        if scans_seen == 0:
            self.get_logger().error(
                f'reset_environment: received 0 LaserScans after {max_wait:.1f}s - sensor stream has died'
            )
            self.get_logger().error('Stopping training. Please restart Gazebo and try again.')
            raise RuntimeError('LaserScan topic stopped publishing after environment reset')
        elif scans_seen < required_scans:
            self.get_logger().warning(
                f'reset_environment: only observed {scans_seen}/{required_scans} fresh LaserScans before episode restart'
            )

    def _q_learning_update(self, state: int, action: int, reward: float, next_state: int, done: bool) -> None:
        # TODO: Update self.q_table[state, action] using Q-learning.
        current = self.q_table[state, action]

        if done:
            target = reward
        else:
            target = reward + self.gamma * np.max(self.q_table[next_state])

        self.q_table[state, action] = current + self.alpha * (target - current)

    def _sarsa_update(self, state: int, action: int, reward: float, next_state: int, next_action: int | None, done: bool) -> None:
        # TODO: Update self.q_table[state, action] using SARSA.
        current = self.q_table[state, action]

        if done or next_action is None:
            target = reward 
        else:
            target = reward + self.gamma * self.q_table[next_state, next_action]

        self.q_table[state, action] = current + self.alpha * (target - current)


__all__ = ['DiscreteTrainerNode']
