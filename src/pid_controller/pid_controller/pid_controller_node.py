import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from rcl_interfaces.msg import SetParametersResult
from pid_controller_msgs.srv import SetReference  
import math

class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('pid_controller_node')

        # PID-parametere og referanse
        self.declare_parameter('p', 1.0)
        self.declare_parameter('i', 0.0)
        self.declare_parameter('d', 0.0)
        self.declare_parameter('reference', 3.0)

        self.p = self.get_parameter('p').value
        self.i = self.get_parameter('i').value
        self.d = self.get_parameter('d').value
        self.reference = self.get_parameter('reference').value

        self.add_on_set_parameters_callback(self.parameter_callback)

        self.voltage = 0.0
        self.previous_error = 0.0
        self.integral = 0.0

        # Service-server
        self.srv = self.create_service(SetReference, 'set_reference', self.set_reference_callback)

        # ROS2 Publisher og Subscriber
        self.publisher_ = self.create_publisher(Float64, 'voltage', 10)
        self.subscription = self.create_subscription(
            Float64,
            'Angle',
            self.measurement_listener,
            10)
        
        self.get_logger().info("PID Controller Node started!")

    def measurement_listener(self, msg):
        measured_angle = msg.data
        error = self.reference - measured_angle

        self.integral += error
        derivative = error - self.previous_error
        self.voltage = self.p * error + self.i * self.integral + self.d * derivative
        self.previous_error = error

        voltage_msg = Float64()
        voltage_msg.data = self.voltage
        self.publisher_.publish(voltage_msg)
        self.get_logger().info(f'Published voltage: {self.voltage:.2f}')
        
    def parameter_callback(self, params):
        for param in params:
            if param.name == 'p' and param.value >= 0.0:
                self.p = param.value
                self.get_logger().info(f'P updated to: {self.p}')
            elif param.name == 'i' and param.value >= 0.0:
                self.i = param.value
                self.get_logger().info(f'I updated to: {self.i}')
            elif param.name == 'd' and param.value >= 0.0:
                self.d = param.value
                self.get_logger().info(f'D updated to: {self.d}')
            elif param.name == 'reference' and param.value >= 0.0:
                self.reference = param.value
                self.get_logger().info(f'Reference updated to: {self.reference}')    
        return SetParametersResult(successful=True)

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

def main(args=None):
    rclpy.init(args=args)
    node = PIDControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


