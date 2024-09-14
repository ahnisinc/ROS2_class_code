import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuSubscriberNode(Node):

    def __init__(self):
        super().__init__('imu_subscriber')
        self.subscription = self.create_subscription(
            Imu,
            'imu/data',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('Orientation: [%f, %f, %f, %f]' % (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ))
        self.get_logger().info('Angular Velocity: [%f, %f, %f]' % (
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ))
        self.get_logger().info('Linear Acceleration: [%f, %f, %f]' % (
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ))

def main(args=None):
    rclpy.init(args=args)
    node = ImuSubscriberNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
