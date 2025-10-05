#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class PIDcircle(Node):
    def __init__(self):
        super().__init__('pid_circle')

        self.Kp = 2.0
        self.Ki = 0.0
        self.Kd = 0.5

        self.target_angular_velocity = 5.0  
        self.linear_velocity = 1.0          

        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = self.get_clock().now()
        self.pose = None

        self.publisher_new = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscriber_new = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("PID Circle Controller node started.")

    def pose_callback(self, msg):
        self.pose = msg

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def control_loop(self):
        if self.pose is None:
            return

        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds / 1e9
        self.prev_time = current_time

        theta_target = self.pose.theta + self.target_angular_velocity * dt
        theta_target = self.normalize_angle(theta_target)

        error = self.normalize_angle(theta_target - self.pose.theta)

        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error

        angular_z = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        cmd = Twist()
        cmd.linear.x = self.linear_velocity
        cmd.angular.z = angular_z
        self.publisher_new.publish(cmd)
    
def main(args=None):
    rclpy.init(args=args)
    node = PIDcircle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


