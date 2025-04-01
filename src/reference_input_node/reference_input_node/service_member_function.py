from pid_controller_msgs import SetReference

import rclpy
from rclpy.node import Node
import math


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(SetReference, 'set_reference', self.set_reference_callback)

    def set_reference_callback(self, request, response):
        value = request.request
        if -math.pi <= value <= math.pi:
            self.reference = value
            self.get_logger().info(f"Reference updated via service to: {value:.2f}")
            response.success = True
        else:
            self.get_logger().warn(f"Invalid reference value: {value:.2f} (outside [-π, π])")
            response.success = False
        return response


def main():
    rclpy.init()

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()