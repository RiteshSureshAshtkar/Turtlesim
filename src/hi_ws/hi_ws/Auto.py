#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class Auto(Node):
    def __init__(self):
        super().__init__('auto')        

        self.prev_time = self.get_clock().now()
        self.pose = None
        self.target=None

        self.publisher_new = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscriber_new = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.subscriber_new1 = self.create_subscription(Pose, '/turtle1/target', self.target_callback, 10)

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Auto node started.")

    def pose_callback(self, msg):
        self.pose = msg
    def target_callback(self, msg):
        self.target = msg

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def control_loop(self):
        if self.pose is None or self.target is None:
            return

        dx = self.target.x - self.pose.x
        dy = self.target.y - self.pose.y
        distance = math.sqrt(dx**2 + dy**2)

        k_linear = 1.5
        k_angular = 4.0
        k_theta_final = 3.0

        cmd = Twist()

        if distance > 0.0001:
            theta_target_pos = math.atan2(dy, dx)
            angle_error_pos = self.normalize_angle(theta_target_pos - self.pose.theta)

            cmd.linear.x = k_linear * distance
            cmd.angular.z = k_angular * angle_error_pos
        else:
            angle_error_final = self.normalize_angle(self.target.theta - self.pose.theta)

            if abs(angle_error_final) > 0.0001:
                cmd.linear.x = 0.0
                cmd.angular.z = k_theta_final * angle_error_final
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0

        self.publisher_new.publish(cmd)

    
def main(args=None):
    rclpy.init(args=args)
    node = Auto()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


