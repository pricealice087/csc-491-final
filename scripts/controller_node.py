import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Point, PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Path

# LeBron's world position in room1.world
LEBRON_X = 2.0
LEBRON_Y = 0.0

CONFIDENCE_THRESHOLD = 0.3
ARRIVAL_RADIUS_BALL = 1.0    # meters — stop here and "pick up"
ARRIVAL_RADIUS_LEBRON = 1.0  # meters — stop here and declare delivery
WAYPOINT_RADIUS = 0.5        # meters — advance to next A* waypoint

SEARCH_ANGULAR = 0.5   # rad/s spin speed while searching
DRIVE_LINEAR = 1.0     # m/s forward speed
KP_ANGULAR = 1.5       # proportional gain for heading correction
MAX_ANGULAR = 1.0      # rad/s cap on angular command
ALIGN_THRESHOLD = 0.15 # rad — must be within this before driving forward

PICKUP_DURATION = 2.0  # seconds to pause at ball before delivering


class State:
    SEARCHING = 'SEARCHING'
    APPROACHING_BALL = 'APPROACHING_BALL'
    PICKUP = 'PICKUP'
    DELIVERING = 'DELIVERING'
    DELIVERED = 'DELIVERED'


class DeliveryController(Node):
    def __init__(self):
        super().__init__('delivery_controller')

        self.detection_sub = self.create_subscription(
            Point, '/basketball/detection', self.detection_callback, 10
        )
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10
        )
        self.path_sub = self.create_subscription(
            Path, '/astar_path', self.path_callback, 10
        )

        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        self.state = State.SEARCHING

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        self.ball_world_x = None
        self.ball_world_y = None
        self.pickup_start_time = None

        self.current_path = []
        self.path_index = 0

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Delivery controller started | state=SEARCHING')

    def pose_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self.robot_x = pos.x
        self.robot_y = pos.y
        self.robot_yaw = self._yaw_from_quaternion(ori)

    def path_callback(self, msg):
        if self.state in (State.APPROACHING_BALL, State.DELIVERING):
            self.current_path = msg.poses
            self.path_index = 0
            self.get_logger().info(f'A* path received with {len(self.current_path)} waypoints')

    def detection_callback(self, msg):
        # msg.x = distance_m, msg.y = heading_deg, msg.z = confidence
        if self.state != State.SEARCHING:
            return

        confidence = msg.z
        if confidence < CONFIDENCE_THRESHOLD:
            return

        distance_m = msg.x
        heading_rad = math.radians(msg.y)

        self.ball_world_x = self.robot_x + distance_m * math.cos(self.robot_yaw + heading_rad)
        self.ball_world_y = self.robot_y + distance_m * math.sin(self.robot_yaw + heading_rad)

        self.get_logger().info(
            f'Ball locked | world=({self.ball_world_x:.2f}, {self.ball_world_y:.2f}) '
            f'| confidence={confidence:.2f} | distance={distance_m:.2f}m'
        )

        self.current_path = []
        self.path_index = 0
        self._publish_goal(self.ball_world_x, self.ball_world_y)
        self._set_state(State.APPROACHING_BALL)

    def control_loop(self):
        if self.state == State.SEARCHING:
            self._do_searching()
        elif self.state == State.APPROACHING_BALL:
            self._do_approaching_ball()
        elif self.state == State.PICKUP:
            self._do_pickup()
        elif self.state == State.DELIVERING:
            self._do_delivering()
        elif self.state == State.DELIVERED:
            self._stop()

    def _do_searching(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.angular.z = SEARCH_ANGULAR
        self.cmd_pub.publish(msg)

    def _do_approaching_ball(self):
        if self.ball_world_x is None:
            self._set_state(State.SEARCHING)
            return

        if self._distance_to(self.ball_world_x, self.ball_world_y) < ARRIVAL_RADIUS_BALL:
            self._stop()
            self.pickup_start_time = self.get_clock().now()
            self._set_state(State.PICKUP)
            return

        # Follow A* path if available
        if self.current_path:
            if self.path_index >= len(self.current_path):
                # Exhausted all waypoints — we're at the ball
                self._stop()
                self.pickup_start_time = self.get_clock().now()
                self._set_state(State.PICKUP)
                return

            waypoint = self.current_path[self.path_index]
            wp_x = waypoint.pose.position.x
            wp_y = waypoint.pose.position.y

            if self._distance_to(wp_x, wp_y) < WAYPOINT_RADIUS:
                self.path_index += 1
                return

            self._navigate_to(wp_x, wp_y)
        else:
            # No path yet — drive directly until A* responds
            self._navigate_to(self.ball_world_x, self.ball_world_y)

    def _do_pickup(self):
        self._stop()
        elapsed = (self.get_clock().now() - self.pickup_start_time).nanoseconds / 1e9
        if elapsed >= PICKUP_DURATION:
            self._publish_goal(LEBRON_X, LEBRON_Y)
            self.current_path = []
            self.path_index = 0
            self._set_state(State.DELIVERING)

    def _do_delivering(self):
        if self._distance_to(LEBRON_X, LEBRON_Y) < ARRIVAL_RADIUS_LEBRON:
            self._stop()
            self._set_state(State.DELIVERED)
            self.get_logger().info('Ball delivered to LeBron!')
            return

        if self.current_path and self.path_index < len(self.current_path):
            waypoint = self.current_path[self.path_index]
            wp_x = waypoint.pose.position.x
            wp_y = waypoint.pose.position.y

            if self._distance_to(wp_x, wp_y) < WAYPOINT_RADIUS:
                self.path_index += 1
                return

            self._navigate_to(wp_x, wp_y)
        else:
            self._navigate_to(LEBRON_X, LEBRON_Y)

    def _publish_goal(self, x, y):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)

    def _navigate_to(self, target_x, target_y):
        dx = target_x - self.robot_x
        dy = target_y - self.robot_y
        angle_to_target = math.atan2(dy, dx)
        angle_error = self._normalize_angle(angle_to_target - self.robot_yaw)

        angular_cmd = max(-MAX_ANGULAR, min(MAX_ANGULAR, KP_ANGULAR * angle_error))

        # Always drive forward — scale speed down when sharply off-course
        speed_scale = max(0.3, 1.0 - abs(angle_error) / math.pi)

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = DRIVE_LINEAR * speed_scale
        msg.twist.angular.z = angular_cmd

        self.cmd_pub.publish(msg)

    def _stop(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        self.cmd_pub.publish(msg)

    def _set_state(self, new_state):
        self.get_logger().info(f'State: {self.state} → {new_state}')
        self.state = new_state

    def _distance_to(self, x, y):
        return math.sqrt((x - self.robot_x) ** 2 + (y - self.robot_y) ** 2)

    @staticmethod
    def _normalize_angle(angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    @staticmethod
    def _yaw_from_quaternion(q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )


def main(args=None):
    rclpy.init(args=args)
    node = DeliveryController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
