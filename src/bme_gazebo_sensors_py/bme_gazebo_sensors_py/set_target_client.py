import rclpy
from rclpy.node import Node
from videeps_pkg.srv import SetTargetObject

class SetTargetClient(Node):
    def __init__(self):
        super().__init__('set_target_client')
        self.cli = self.create_client(SetTargetObject, 'set_target_object')

    def send_request(self, object_name):
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')
        
        req = SetTargetObject.Request()
        req.object_name = object_name

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result():
            self.get_logger().info(f"Response: {future.result().message}")
        else:
            self.get_logger().error('Service call failed.')

def main(args=None):
    rclpy.init(args=args)
    node = SetTargetClient()
    object_to_follow = input("Enter object to follow: ")
    node.send_request(object_to_follow)
    node.destroy_node()
    rclpy.shutdown()
