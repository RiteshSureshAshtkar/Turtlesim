import rclpy
from rclpy.node import Node
from example_interfaces.srv import Trigger
from geometry_msgs.msg import Twist
import time
import math

class MoveCircleServer(Node):
    def __init__(self):
        super().__init__('move_circle_server')
        self.srv = self.create_service(Trigger, 'move_circle', self.handle_move_circle)
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.get_logger().info("move_circle service ready. Call it to make a circle of radius 3.")

    def handle_move_circle(self, request, response):
        r = 3.0  
        v = 1.0  
        w = v / r 
        duration = 2.0 * math.pi / w
        self.get_logger().info(f"Starting circle: radius={r}, v={v}, w={w:.4f}, duration={duration:.3f}s")
        start = self.get_clock().now().nanoseconds / 1e9
        rate_hz = 20.0
        period = 1.0 / rate_hz

        while (self.get_clock().now().nanoseconds / 1e9 - start) < duration and rclpy.ok():
            twist = Twist()
            twist.linear.x = float(v)
            twist.angular.z = float(w)
            self.pub.publish(twist)
            # allow executor to process other callbacks (and wait up to period)
            rclpy.spin_once(self, timeout_sec=period)

        self.pub.publish(Twist())
        response.success = True
        response.message = f"Completed one circle of radius {r} in {duration:.2f}s"
        self.get_logger().info("Circle completed; responding to client.")
        return response

def main(args=None):
    rclpy.init(args=args)
    node = MoveCircleServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
