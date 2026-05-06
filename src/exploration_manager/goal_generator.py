from geometry_msgs.msg import PoseStamped
import numpy as np
import math


class GoalGenerator:
    def __init__(self):
        self.last_goal = None

    def sample_frontier_goal(self, map_msg):
        """
        Pick a simple frontier-like goal:
        (placeholder version using random free space)
        """

        width = map_msg.info.width
        height = map_msg.info.height
        res = map_msg.info.resolution
        origin = map_msg.info.origin.position

        data = np.array(map_msg.data).reshape((height, width))

        free_cells = np.argwhere(data == 0)

        if len(free_cells) == 0:
            return None

        y, x = free_cells[np.random.randint(len(free_cells))]

        goal = PoseStamped()
        goal.header.frame_id = "map"

        goal.pose.position.x = float(x * res + origin.x)
        goal.pose.position.y = float(y * res + origin.y)
        goal.pose.position.z = 0.0

        goal.pose.orientation.w = 1.0

        return goal

    def convert_rl_action_to_goal(self, action, current_pose):

        step_size = 1.0

        yaw = self.get_yaw(current_pose)

        new_yaw = yaw + action.angular

        goal = PoseStamped()
        goal.header.frame_id = "map"

        goal.pose.position.x = (
            current_pose.position.x +
            step_size * math.cos(new_yaw)
        )

        goal.pose.position.y = (
            current_pose.position.y +
            step_size * math.sin(new_yaw)
        )

        goal.pose.position.z = 0.0

        goal.pose.orientation.z = math.sin(new_yaw / 2.0)
        goal.pose.orientation.w = math.cos(new_yaw / 2.0)

        return goal

    def get_yaw(self, pose):
        q = pose.orientation
        return math.atan2(
            2.0 * (q.w * q.z),
            1.0 - 2.0 * (q.z * q.z)
        )