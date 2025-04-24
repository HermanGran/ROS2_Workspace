import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CannyEdgeNode(Node):
    def __init__(self):
        super().__init__('canny_edge')

        self.subscription = self.create_subscription(
            Image,
            "image_blurred",
            self.image_callback,
            10)

        self.publisher = self.create_publisher(
            Image,
            'image_output',
            10)

        self.bridge = CvBridge()
    
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error('Failed to convert image: %s' % str(e))
            return

        # Canny Edge processing
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        cv_edge = cv2.Canny(gray_image, threshold1=50, threshold2=150)
        cv_edge_bgr = cv2.cvtColor(cv_edge, cv2.COLOR_GRAY2BGR)

        try:
            edge_msg = self.bridge.cv2_to_imgmsg(cv_edge_bgr, "bgr8")
        except Exception as e:
            self.get_logger().error('Failed to convert image: %s' % str(e))
            return

        self.publisher.publish(edge_msg)

def main(args=None):
    rclpy.init(args=args)
    canny_edge_node = CannyEdgeNode()
    rclpy.spin(canny_edge_node)
    canny_edge_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
