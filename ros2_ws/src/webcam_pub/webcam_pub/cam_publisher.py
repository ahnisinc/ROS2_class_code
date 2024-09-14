import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
#from image_transport import ImageTransport


class ImagePublisher(Node):
    def __init__(self):
        super().__init__("image_publisher")
        self.publisher_ = self.create_publisher(Image, "image_topic", 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(1 / 30.0, self.timer_callback)  # 30 fps
        self.cap = cv2.VideoCapture(0)  # 0: default camera

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("Failed to capture image")
            return

        # Convert the image to ROS format
        #ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")

        # -------------This is modified by brian lee
        # Process the image with OpenCV (convert to grayscale here)
        gray_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Convert the processed image back to ROS format
        toros_image = self.bridge.cv2_to_imgmsg(gray_image, encoding="mono8")
        # -------------This is modified by brian lee
        self.publisher_.publish(toros_image)


def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    rclpy.spin(node)
    node.cap.release()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
