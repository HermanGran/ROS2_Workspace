import sys

from pid_controller_msgs import SetReference
import rclpy
from rclpy.node import Node


class ReferenceInputNode(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(SetReference, 'set_reference')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetReference.Request()

    def send_request(self, reference):
        self.req.a = reference
        return self.cli.call_async(self.req)


def main():
    rclpy.init()
    node = ReferenceInputNode()

    try:
        while rclpy.ok():
            user_input = input("Set new reference\n")
            try:
                reference = float(user_input)
            except ValueError:
                print("Invalid input, try again:")
                continue
            
            future = node.send_request(reference)
            rclpy.spin_until_future_complete(node, future)
            response = future.result()
            if response.success:
                node.get_logger().info(f"Reference updated to: {reference:.2f}")
            else:
                node.get_logger().warn(f"Invalid reference value: {reference:.2f} (outside [-π, π])")
    except KeyboardInterrupt:
        print("Shutting down...")

    node.destroy_node
    rclpy.shutdown()


if __name__ == '__main__':
    main()