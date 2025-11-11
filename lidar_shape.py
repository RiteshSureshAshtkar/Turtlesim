#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool
from collections import deque


class ShapeDetector(Node):
    def __init__(self):
        super().__init__('shape_detector')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile)
        self.detection_pub = self.create_publisher(String, '/detection_status', 10)
        self.pause_pub = self.create_publisher(Bool, '/detection_pause', 10)

        self.robot_x = -1.5339
        self.robot_y = -6.6156
        self.robot_yaw = 1.57
        self.lidar_data = None

        self.detected_positions = []
        self.is_paused = False
        self.pause_start_time = None

        self.dock_waypoint = (0.26, -1.95)
        self.dock_detected = False
        self.dock_tolerance = 0.22

        self.scan_range_min = 0.3
        self.scan_range_max = 1.8
        self.cluster_distance = 0.10
        self.min_cluster_size = 15

        self.ransac_iterations = 200
        self.ransac_distance_threshold = 0.04
        self.min_line_points = 6
        self.min_line_length = 0.18
        self.max_line_length = 1.3

        self.max_gap_between_lines = 0.20
        self.angle_merge_threshold = 0.15
        self.distance_merge_threshold = 0.18

        self.last_detection_time = 0
        self.detection_interval = 2.5

        self.create_timer(0.1, self.detection_loop)

        self.get_logger().info('Shape detector initialized.')

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = np.arctan2(siny_cosp, cosy_cosp)

    def scan_callback(self, msg):
        self.lidar_data = msg

    def get_world_points(self):
        if self.lidar_data is None:
            return np.array([])

        ranges = np.array(self.lidar_data.ranges)
        angles = np.linspace(self.lidar_data.angle_min, self.lidar_data.angle_max, len(ranges))
        valid = (ranges >= self.scan_range_min) & (ranges <= self.scan_range_max) & (~np.isnan(ranges)) & (~np.isinf(ranges))
        ranges = ranges[valid]
        angles = angles[valid]

        if len(ranges) == 0:
            return np.array([])

        x_robot = ranges * np.cos(angles)
        y_robot = ranges * np.sin(angles)
        cos_yaw = np.cos(self.robot_yaw)
        sin_yaw = np.sin(self.robot_yaw)
        x_world = self.robot_x + x_robot * cos_yaw - y_robot * sin_yaw
        y_world = self.robot_y + x_robot * sin_yaw + y_robot * cos_yaw
        return np.column_stack([x_world, y_world])

    def euclidean_cluster(self, points):
        if len(points) == 0:
            return []
        clusters = []
        visited = np.zeros(len(points), dtype=bool)

        for i in range(len(points)):
            if visited[i]:
                continue
            cluster_indices = [i]
            visited[i] = True
            queue = deque([i])
            while queue:
                current_idx = queue.popleft()
                current_point = points[current_idx]
                for j in range(len(points)):
                    if visited[j]:
                        continue
                    if np.linalg.norm(points[j] - current_point) < self.cluster_distance:
                        visited[j] = True
                        cluster_indices.append(j)
                        queue.append(j)
            if len(cluster_indices) >= self.min_cluster_size:
                clusters.append(points[cluster_indices])
        return clusters

    def fit_line_ransac(self, points):
        if len(points) < 2:
            return None, None
        best_inliers, best_model = None, None
        max_inliers = 0
        for _ in range(min(self.ransac_iterations, len(points) * 20)):
            i1, i2 = np.random.choice(len(points), 2, replace=False)
            p1, p2 = points[i1], points[i2]
            dx, dy = p2[0] - p1[0], p2[1] - p1[1]
            length = np.sqrt(dx**2 + dy**2)
            if length < 0.01:
                continue
            a, b = -dy / length, dx / length
            c = -(a * p1[0] + b * p1[1])
            distances = np.abs(a * points[:, 0] + b * points[:, 1] + c)
            inliers = distances < self.ransac_distance_threshold
            n_inliers = np.sum(inliers)
            if n_inliers > max_inliers:
                max_inliers = n_inliers
                best_inliers = inliers
                best_model = (a, b, c)
        if max_inliers < self.min_line_points:
            return None, None
        return best_inliers, best_model

    def extract_line_segment(self, points):
        if len(points) < 2:
            return None
        center = np.mean(points, axis=0)
        centered = points - center
        cov = np.cov(centered.T)
        eigenvalues, eigenvectors = np.linalg.eig(cov)
        principal = eigenvectors[:, np.argmax(eigenvalues)]
        projections = np.dot(centered, principal)
        p1 = points[np.argmin(projections)]
        p2 = points[np.argmax(projections)]
        length = np.linalg.norm(p2 - p1)
        if length < self.min_line_length or length > self.max_line_length:
            return None
        angle = np.arctan2(p2[1] - p1[1], p2[0] - p1[0])
        return {'p1': p1, 'p2': p2, 'length': length, 'angle': angle, 'center': (p1 + p2) / 2}

    def detect_lines_in_cluster(self, points):
        lines, remaining = [], points.copy()
        for _ in range(8):
            if len(remaining) < self.min_line_points:
                break
            inliers, model = self.fit_line_ransac(remaining)
            if inliers is None:
                break
            line = self.extract_line_segment(remaining[inliers])
            if line:
                lines.append(line)
            remaining = remaining[~inliers]
        return lines

    def are_collinear(self, l1, l2):
        angle_diff = abs(self.normalize_angle(l1['angle'] - l2['angle']))
        if angle_diff > self.angle_merge_threshold and angle_diff < (np.pi - self.angle_merge_threshold):
            return False
        dist = np.linalg.norm(l1['center'] - l2['center'])
        min_end_dist = min(
            np.linalg.norm(l1['p1'] - l2['p1']),
            np.linalg.norm(l1['p1'] - l2['p2']),
            np.linalg.norm(l1['p2'] - l2['p1']),
            np.linalg.norm(l1['p2'] - l2['p2'])
        )
        return dist < self.distance_merge_threshold or min_end_dist < self.distance_merge_threshold

    def merge_collinear_lines(self, lines):
        if len(lines) < 2:
            return lines
        merged, used = [], set()
        for i in range(len(lines)):
            if i in used:
                continue
            group = [lines[i]]
            used.add(i)
            for j in range(i + 1, len(lines)):
                if j not in used and self.are_collinear(lines[i], lines[j]):
                    group.append(lines[j])
                    used.add(j)
            if len(group) == 1:
                merged.append(lines[i])
                continue
            points = np.array([p for line in group for p in [line['p1'], line['p2']]])
            dists = np.linalg.norm(points[:, None] - points, axis=2)
            i, j = np.unravel_index(np.argmax(dists), dists.shape)
            p1, p2 = points[i], points[j]
            merged.append({'p1': p1, 'p2': p2, 'length': np.linalg.norm(p2 - p1),
                           'angle': np.arctan2(p2[1] - p1[1], p2[0] - p1[0]), 'center': (p1 + p2) / 2})
        return merged

    def order_lines_to_polygon(self, lines):
        if len(lines) < 3:
            return None
        ordered = [lines[0]]
        remaining = list(lines[1:])
        while remaining:
            last_end = ordered[-1]['p2']
            best_dist, best_idx, flip = float('inf'), -1, False
            for idx, line in enumerate(remaining):
                d1, d2 = np.linalg.norm(line['p1'] - last_end), np.linalg.norm(line['p2'] - last_end)
                if d1 < best_dist:
                    best_dist, best_idx, flip = d1, idx, False
                if d2 < best_dist:
                    best_dist, best_idx, flip = d2, idx, True
            if best_idx == -1 or best_dist > self.max_gap_between_lines:
                return None
            next_line = remaining.pop(best_idx)
            if flip:
                next_line = {'p1': next_line['p2'], 'p2': next_line['p1'],
                             'length': next_line['length'], 'angle': self.normalize_angle(next_line['angle'] + np.pi),
                             'center': next_line['center']}
            ordered.append(next_line)
        if np.linalg.norm(ordered[-1]['p2'] - ordered[0]['p1']) > self.max_gap_between_lines:
            return None
        return ordered

    def compute_shape_properties(self, lines):
        n = len(lines)
        vertices = [l['p1'] for l in lines]
        center = np.mean(vertices, axis=0)
        sides = [l['length'] for l in lines]
        angles = []
        for i in range(n):
            v1 = lines[i]['p2'] - lines[i]['p1']
            v2 = lines[(i + 1) % n]['p2'] - lines[(i + 1) % n]['p1']
            v1 /= np.linalg.norm(v1) + 1e-8
            v2 /= np.linalg.norm(v2) + 1e-8
            angles.append(np.arccos(np.clip(np.dot(v1, v2), -1, 1)))
        return {'n_sides': n, 'center': center, 'side_lengths': sides, 'angles': angles,
                'avg_side': np.mean(sides), 'std_side': np.std(sides),
                'min_side': np.min(sides), 'max_side': np.max(sides)}

    def classify_polygon(self, props):
        n = props['n_sides']
        if n == 3:
            if props['min_side'] < 0.25 or props['max_side'] > 1.0:
                return None
            if props['max_side'] / props['min_side'] > 2.8:
                return None
            sides = sorted(props['side_lengths'])
            if sides[0] + sides[1] < sides[2] * 0.9:
                return None
            return 'triangle'
        elif n == 4:
            if props['min_side'] < 0.25 or props['max_side'] > 1.2:
                return None
            if props['std_side'] / (props['avg_side'] + 1e-6) > 0.45:
                return None
            right_angles = sum(1 for a in np.degrees(props['angles']) if 75 < a < 105)
            if right_angles < 2:
                return None
            return 'square'
        return None

    def normalize_angle(self, a):
        while a > np.pi:
            a -= 2 * np.pi
        while a < -np.pi:
            a += 2 * np.pi
        return a

    def already_detected(self, center, shape_type):
        for x, y, t in self.detected_positions:
            if t == shape_type and np.linalg.norm([center[0] - x, center[1] - y]) < 0.65:
                return True
        return False

    def is_near_dock(self):
        return np.linalg.norm([self.robot_x - self.dock_waypoint[0], self.robot_y - self.dock_waypoint[1]]) < self.dock_tolerance

    def handle_pause(self):
        if not self.is_paused or self.pause_start_time is None:
            return
        elapsed = self.get_clock().now().nanoseconds / 1e9 - self.pause_start_time
        if elapsed >= 2.0:
            self.pause_pub.publish(Bool(data=False))
            self.is_paused = False
            self.pause_start_time = None

    def detection_loop(self):
        if self.is_paused:
            self.handle_pause()
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_detection_time < self.detection_interval:
            return
        if not self.dock_detected and self.is_near_dock():
            self.dock_detected = True
            msg = f"DOCK_STATION,{self.dock_waypoint[0]:.2f},{self.dock_waypoint[1]:.2f}"
            self.detection_pub.publish(String(data=msg))
            self.detected_positions.append((*self.dock_waypoint, 'dock'))
            self.last_detection_time = now
            self.pause_pub.publish(Bool(data=True))
            self.is_paused = True
            self.pause_start_time = now
            return
        points = self.get_world_points()
        if len(points) < 15:
            return
        for cluster in self.euclidean_cluster(points):
            lines = self.detect_lines_in_cluster(cluster)
            if len(lines) < 3:
                continue
            merged = self.merge_collinear_lines(lines)
            if len(merged) not in [3, 4]:
                continue
            polygon = self.order_lines_to_polygon(merged)
            if polygon is None:
                continue
            props = self.compute_shape_properties(polygon)
            shape = self.classify_polygon(props)
            if not shape:
                continue
            c = props['center']
            if self.already_detected(c, shape):
                continue
            if np.linalg.norm([self.robot_x - c[0], self.robot_y - c[1]]) > 1.5:
                continue
            status = 'FERTILIZER_REQUIRED' if shape == 'triangle' else 'BAD_HEALTH'
            msg = f"{status},{c[0]:.2f},{c[1]:.2f}"
            self.detection_pub.publish(String(data=msg))
            self.detected_positions.append((c[0], c[1], shape))
            self.last_detection_time = now
            self.pause_pub.publish(Bool(data=True))
            self.is_paused = True
            self.pause_start_time = now
            break


def main(args=None):
    rclpy.init(args=args)
    node = ShapeDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
