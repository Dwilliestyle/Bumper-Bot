#!/usr/bin/env python3


import math
import threading
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionServer
from rclpy.action import GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile

from bumperbot_msgs.action import Patrol


class BumperbotPatrolServer(Node):

    def __init__(self):
        super().__init__('bumperbot_patrol_server')

        print('Bumperbot Patrol Server')
        print('----------------------------------------------')

        self._action_server = ActionServer(
            self,
            Patrol,
            'bumperbot',
            self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback)

        self.goal_msg = Patrol.Goal()
        self.twist = Twist()
        self.odom = Odometry()

        self.linear_x = 0.2
        self.angular_z = 1.5

        qos = QoSProfile(depth=10)

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)

        self.odom_sub = self.create_subscription(
            Odometry, '/bumperbot_controller/odom', self.odom_callback, qos)

    def init_twist(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)

    def odom_callback(self, msg):
        self.odom = msg

    def get_yaw(self):
        q = self.odom.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    def go_front(self, length):
        start_x = self.odom.pose.pose.position.x
        start_y = self.odom.pose.pose.position.y
        target_heading = self.get_yaw()  # hold this heading

        while True:
            current_x = self.odom.pose.pose.position.x
            current_y = self.odom.pose.pose.position.y
            dist = math.sqrt(
                (current_x - start_x) ** 2 +
                (current_y - start_y) ** 2
            )

            if dist >= length:
                break

            # Small correction to hold heading
            heading_error = math.atan2(
                math.sin(target_heading - self.get_yaw()),
                math.cos(target_heading - self.get_yaw())
            )

            self.twist.linear.x = self.linear_x
            self.twist.angular.z = heading_error * 0.5  # proportional correction
            self.cmd_vel_pub.publish(self.twist)
            time.sleep(0.05)

        self.init_twist()
        time.sleep(0.3)

    def turn(self, target_angle):
        initial_yaw = self.get_yaw()
        target_yaw = initial_yaw + (target_angle * math.pi / 180.0)

        # Normalize target_yaw to [-pi, pi]
        target_yaw = math.atan2(math.sin(target_yaw), math.cos(target_yaw))

        self.get_logger().info(
            f'Turn start — initial_yaw: {math.degrees(initial_yaw):.1f}°  '
            f'target_yaw: {math.degrees(target_yaw):.1f}°'
        )

        loop_count = 0
        while True:
            time.sleep(0.1)
            loop_count += 1

            current_yaw = self.get_yaw()

            # Signed error — tells us direction AND magnitude
            error = math.atan2(
                math.sin(target_yaw - current_yaw),
                math.cos(target_yaw - current_yaw)
            )

            self.get_logger().info(
                f'  loop {loop_count}: current_yaw={math.degrees(current_yaw):.1f}°  '
                f'error={math.degrees(error):.1f}°'
            )

            if abs(error) < 0.05:
                self.get_logger().info(f'  Turn complete after {loop_count} loops')
                break

            if loop_count > 200:
                self.get_logger().warn('Turn timeout — forcing exit')
                break

            # Proportional speed, direction controlled by sign of error
            speed = max(0.5, min(1.0, abs(error) * 1.2))
            self.twist.linear.x = 0.0
            self.twist.angular.z = math.copysign(speed, error)
            self.cmd_vel_pub.publish(self.twist)

        self.init_twist()
        time.sleep(0.3)

    def goal_callback(self, goal_request):
        self.goal_msg = goal_request
        return GoalResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        feedback_msg = Patrol.Feedback()

        length = self.goal_msg.goal.y
        iteration = int(self.goal_msg.goal.z)

        if self.goal_msg.goal.x == 1:
            for count in range(iteration):
                self.square(feedback_msg, goal_handle, length)
            feedback_msg.state = 'square patrol complete!!'
        elif self.goal_msg.goal.x == 2:
            for count in range(iteration):
                self.triangle(feedback_msg, goal_handle, length)
            feedback_msg.state = 'triangle patrol complete!!'

        goal_handle.succeed()
        result = Patrol.Result()
        result.result = feedback_msg.state

        self.init_twist()
        self.get_logger().info('Patrol complete.')
        threading.Timer(0.1, rclpy.shutdown).start()

        return result

    def square(self, feedback_msg, goal_handle, length):
        self.linear_x = 0.2

        for i in range(4):
            self.go_front(length)
            self.turn(90.0)

            feedback_msg.state = 'line ' + str(i + 1)
            goal_handle.publish_feedback(feedback_msg)

    def triangle(self, feedback_msg, goal_handle, length):
        self.linear_x = 0.2
        self.angular_z = 1.5

        for i in range(3):
            self.go_front(length)
            self.turn(120.0)

            feedback_msg.state = 'line ' + str(i + 1)
            goal_handle.publish_feedback(feedback_msg)


def main(args=None):
    rclpy.init(args=args)

    bumperbot_patrol_server = BumperbotPatrolServer()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(bumperbot_patrol_server)
    executor.spin()


if __name__ == '__main__':
    main()