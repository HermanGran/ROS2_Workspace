import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class jointSimulator:
    
    def __init__(self, K=230.0, T=0.15, noise=0):
        self.angle = 0
        self.angular_velocity = 0
        self.voltage = 0
        self.noise = noise
        self.T = T
        self.K = K

    
    def update(self, dt=0.01):

        dvel = dt * ( - (1/self.T) * self.angular_velocity + (self.K / self.T) * self.voltage)
        self.angular_velocity += dvel

        # Integrate velocity -> angle
        self.angle += self.angular_velocity * dt



class jointSimulatorNode(Node):
    
    def __init__(self):
        super().__init__("joint_simulator")

        # Creating instance of simulator
        self.simulator = jointSimulator(K=230.0, T=0.15, noise=0.0)

        # Creates publisher
        self.publisher_ = self.create_publisher(Float64, "Angle", 10)

        self.subscription_ = self.create_subscription(
            Float64,
            "inupt_voltage",
            self.voltage_listener,
            10
        )

        self.timer_period = 0.01
        self.timer_ = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info("JointSimulatorNode has been started")


    def voltage_listener(self, msg):
        self.simulator.voltage = msg.data

    def timer_callback(self):
        self.simulator.update(dt=self.timer_period)

        angle_msg = Float64()

        angle_msg.data = self.simulator.angle

        self.publisher_.publish(angle_msg)
        # self.get_logger().info(f"Publishing angle: {angle_msg.data}")
    



def main(args=None):
    rclpy.init(args=args)
    node = jointSimulatorNode()
    rclpy.spin(node)
    node.destroy_node
    rclpy.shutdown



if __name__ == '__main__':
    main()
