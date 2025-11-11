#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np
import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class BugNavigator(Node):
    def __init__(self):
        super().__init__('ebot_navigator')
        
        # QoS Profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, qos_profile)

        # Pause subscriber (from shape detector)
        self.paused = False
        self.pause_sub = self.create_subscription(
            Bool, '/detection_pause', self.pause_cb, qos_profile)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Waypoints: [x, y, yaw]
        self.waypoints = [
            [ 0.26, -5.0, 0.66 ],
            [ 0.26, -1.95, 1.57 ],   # P1 - Dock station
            [ -1.48, -0.67, -1.57],  # P2
            [ -1.53, -6.61, -1.57]   # P3
        ]
        self.current_waypoint_idx = 0
        
        # Tolerances
        self.pos_tolerance = 0.2
        self.ang_tolerance = np.radians(10)
        
        # Robot state
        self.robot_x = -1.5339
        self.robot_y = -6.6156
        self.robot_yaw = 1.57
        self.lidar_data = None
        self.obstacle_detected = False
        self.obstacle_direction = None
        
        # Control parameters
        self.linear_vel_max = 0.5
        self.angular_vel_max = 3.0
        self.angular_vel_max_emergency = 5.0
        self.kp_linear = 1.5
        self.kp_angular = 2.0
        self.min_obstacle_distance = 0.600
        self.min_front_distance = 0.5
        self.kp_wall_follow = 1.0
        
        # Tangent bug state
        self.following_wall = False
        self.wall_follow_start = None

        # Previous pose for stuck detection
        self.prev_x = None
        self.prev_y = None
        self.prev_theta = None
        
        # Timer
        self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('eBot Navigator initialized')
        self.get_logger().info(f'Starting position: [{self.robot_x}, {self.robot_y}, {self.robot_yaw}]')
    
    def scan_callback(self, msg):
        """Process LiDAR scan data"""
        self.lidar_data = msg
        self.detect_obstacles()
    
    def odom_callback(self, msg):
        """Update robot position and orientation from odometry"""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        orientation = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        self.robot_yaw = np.arctan2(siny_cosp, cosy_cosp)
    
    def pause_cb(self, msg: Bool):
        """Callback to receive pause/unpause requests from the shape detector."""
        new_state = bool(msg.data)
        if new_state != self.paused:
            self.paused = new_state
            if self.paused:
                self.get_logger().info('Navigation PAUSED for shape detection')
                # Immediately stop the robot
                self.cmd_vel_pub.publish(Twist())
            else:
                self.get_logger().info('Navigation RESUMED after detection')
    
    def detect_obstacles(self):
        """Detect obstacles from LiDAR data"""
        if self.lidar_data is None:
            return
        
        ranges = self.lidar_data.ranges
        angles = np.linspace(self.lidar_data.angle_min, 
                           self.lidar_data.angle_max, 
                           len(ranges))
        
        self.obstacle_detected = False
        min_distance = float('inf')
        obstacle_angle = None
        
        for i, (r, angle) in enumerate(zip(ranges, angles)):
            if r < float('inf') and not math.isnan(r):
                if r < self.min_obstacle_distance:
                    self.obstacle_detected = True
                    if r < min_distance:
                        min_distance = r
                        obstacle_angle = angle
        
        self.obstacle_direction = obstacle_angle
    
    def detect_front_wall(self):
        """Detect if there is a wall in front"""
        if self.lidar_data is None:
            return None
        ranges = self.lidar_data.ranges
        angles = np.linspace(self.lidar_data.angle_min, self.lidar_data.angle_max, len(ranges))
        
        alpha = 0.6
        closest_front = float('inf')
        side = None
        for r, angle in zip(ranges, angles):
            if r is None or math.isnan(r) or r == float('inf'):
                continue
            if -alpha <= angle <= alpha:
                if r < closest_front:
                    closest_front = r
                    if angle >= 0:
                        side = 'left'
                    else:
                        side = 'right'
        
        if closest_front < self.min_front_distance:
            return side
        return None
    
    def is_waypoint_reached(self):
        """Check if current waypoint is reached within tolerance"""
        target = self.waypoints[self.current_waypoint_idx]
        
        # Position check
        dist_to_target = np.sqrt(
            (self.robot_x - target[0])**2 + 
            (self.robot_y - target[1])**2
        )
        
        if dist_to_target > self.pos_tolerance:
            return False
        
        # Orientation check
        target_yaw = target[2]
        yaw_error = self.normalize_angle(target_yaw - self.robot_yaw)
        
        if abs(yaw_error) > self.ang_tolerance:
            return False
        
        return True
    
    def navigate_to_waypoint(self):
        """Navigate to current waypoint"""
        target = self.waypoints[self.current_waypoint_idx]
        
        # Calculate direction to target
        dx = target[0] - self.robot_x
        dy = target[1] - self.robot_y
        dist_to_target = np.sqrt(dx**2 + dy**2)
        target_direction = np.arctan2(dy, dx)
        
        # Check if we should switch to wall following
        if self.obstacle_detected and not self.following_wall:
            self.following_wall = True
            self.wall_follow_start = (self.robot_x, self.robot_y)
            self.get_logger().warn('Obstacle detected! Switching to wall following mode')
        
        # Wall following mode
        if self.following_wall:
            if self.can_reach_target_directly(target):
                self.following_wall = False
                self.get_logger().info('Resumed direct navigation')
            else:
                return self.wall_follow_control()
        
        # Direct navigation mode
        return self.direct_navigation_control(dist_to_target, target_direction, target[2])
    
    def can_reach_target_directly(self, target):
        """Check if target can be reached without obstacles"""
        if not self.obstacle_detected:
            return True
        
        dx = target[0] - self.robot_x
        dy = target[1] - self.robot_y
        dist = np.sqrt(dx**2 + dy**2)
        
        return dist < 0.5 or not self.obstacle_detected
    
    def wall_follow_control(self):
        """Control law for wall following"""
        twist = Twist()
        
        if self.obstacle_direction is not None:
            obstacle_relative_angle = self.normalize_angle(
                self.obstacle_direction - self.robot_yaw
            )
            
            if obstacle_relative_angle > 0:
                desired_yaw = self.robot_yaw - np.pi/7
            else:
                desired_yaw = self.robot_yaw + np.pi/7
            
            yaw_error = self.normalize_angle(desired_yaw - self.robot_yaw)
            
            twist.linear.x = 0.5
            twist.angular.z = self.kp_wall_follow * yaw_error
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        
        self.saturate_velocities(twist)
        return twist
    
    def direct_navigation_control(self, dist_to_target, target_direction, target_yaw):
        """Control law for direct navigation to waypoint"""
        twist = Twist()
        
        if dist_to_target > self.pos_tolerance:
            linear_error = dist_to_target
            twist.linear.x = min(self.kp_linear * linear_error, self.linear_vel_max)
            
            yaw_error = self.normalize_angle(target_direction - self.robot_yaw)
            twist.angular.z = self.kp_angular * yaw_error
        else:
            yaw_error = self.normalize_angle(target_yaw - self.robot_yaw)
            
            if abs(yaw_error) > self.ang_tolerance:
                twist.linear.x = 0.0
                twist.angular.z = self.kp_angular * yaw_error
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
        
        self.saturate_velocities(twist)
        return twist
    
    def apply_front_wall_turn(self, twist: Twist) -> Twist:
        """Apply emergency turn if wall detected in front"""
        side = self.detect_front_wall()
        if side is None:
            return twist
        if side == 'right':
            twist.angular.z += 5.0
        elif side == 'left':
            twist.angular.z -= 5.0
        twist.angular.z = np.clip(twist.angular.z, -self.angular_vel_max_emergency, self.angular_vel_max_emergency)
        return twist
    
    def saturate_velocities(self, twist):
        """Saturate velocities to normal max limits"""
        twist.linear.x = np.clip(twist.linear.x, -self.linear_vel_max, self.linear_vel_max)
        twist.angular.z = np.clip(twist.angular.z, -self.angular_vel_max, self.angular_vel_max)
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle
    
    def control_loop(self):
        """Main control loop"""
        # If paused by shape detector, keep robot stopped
        if self.paused:
            self.cmd_vel_pub.publish(Twist())
            return

        if self.lidar_data is None:
            return
        
        if self.is_waypoint_reached():
            if self.current_waypoint_idx < len(self.waypoints) - 1:
                self.current_waypoint_idx += 1
                self.get_logger().info(
                    f'Waypoint {self.current_waypoint_idx} reached! Moving to waypoint {self.current_waypoint_idx + 1}'
                )
            else:
                self.get_logger().info('All waypoints reached! Mission complete.')
                self.cmd_vel_pub.publish(Twist())
                return
        
        # Generate control command
        twist = self.navigate_to_waypoint()

        # Apply emergency turns if needed
        twist = self.apply_front_wall_turn(twist)

        # Stuck detection
        tol = 1e-3
        if (self.prev_x is not None and
            abs(self.robot_x - self.prev_x) < tol and
            abs(self.robot_y - self.prev_y) < tol and
            abs(self.robot_yaw - self.prev_theta) < tol):
            twist.linear.x = -0.1
            twist.angular.z = -0.5
        
        self.cmd_vel_pub.publish(twist)
        
        # Log current state
        target = self.waypoints[self.current_waypoint_idx]
        dist = np.sqrt(
            (self.robot_x - target[0])**2 + 
            (self.robot_y - target[1])**2
        )
        
        # Update previous pose
        self.prev_x = self.robot_x
        self.prev_y = self.robot_y
        self.prev_theta = self.robot_yaw


def main(args=None):
    rclpy.init(args=args)
    navigator = BugNavigator()
    
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        navigator.get_logger().info('Navigation stopped by user')
    finally:
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()