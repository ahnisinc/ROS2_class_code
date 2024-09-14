import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class ImageProcessor(Node):
    def __init__(self):
        super().__init__("webcam_sub")

        # Create a publisher for processed images
        self.publisher_ = self.create_publisher(Image, "topic_sub", 10)

        # Create a subscription to the raw image topic
        self.subscription = self.create_subscription(
            Image, "camera/image_raw", self.listener_callback, 10
        )

        # Create a CvBridge instance to convert between ROS and OpenCV images
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        try:
            # Convert ROS image to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # Process the image with OpenCV (convert to grayscale here)
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # Convert the processed image back to ROS format
            ros_image = self.bridge.cv2_to_imgmsg(gray_image, encoding="mono8")

            # Publish the processed image
            self.publisher_.publish(ros_image)

        except CvBridgeError as e:
            self.get_logger().info(f"Error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
