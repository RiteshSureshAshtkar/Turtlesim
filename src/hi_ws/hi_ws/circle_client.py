import rclpy
from rclpy.node import Node
from example_interfaces.srv import Trigger
from functools import partial

class MoveCircleClient(Node):
    def __init__(self):
        super().__init__('move_circle_client')
        self.cli = self.create_client(Trigger, 'move_circle')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for move_circle service...')
        self.req = Trigger.Request()

    def send_request(self):
        future = self.cli.call_async(self.req)
        self.get_logger().info('Move circle request sent — waiting for completion...')
        future.add_done_callback(partial(self.move_circle_callback))
    def move_circle_callback(self,future):
        try:
            response = future.result()
            self.get_logger().info(f"Service returned: success={response.success}, message='{response.message}'")
        except Exception as e:
            self.get_logger().error('Service call failed %r' % (e,))

def main(args=None):
    rclpy.init(args=args)
    node = MoveCircleClient()
    node.send_request()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
