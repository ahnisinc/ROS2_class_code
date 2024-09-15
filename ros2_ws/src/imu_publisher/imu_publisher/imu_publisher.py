import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
import time

class IMUPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.publisher_ = self.create_publisher(Imu, 'imu/data', 100)
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)  # Adjust port name
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.get_logger().info('IMU Publisher node has been started.')

    def timer_callback(self):
        if self.serial_port.in_waiting > 0:
            try:
                # Read line from serial
                line = self.serial_port.readline().decode('utf-8').strip()
                # Parse the line into sensor data
                data = list(map(float, line.split(',')))
                
                if len(data) == 6:
                    ax, ay, az, gx, gy, gz = data

                    # Create and populate IMU message
                    imu_msg = Imu()
                    imu_msg.header.stamp = self.get_clock().now().to_msg()
                    imu_msg.header.frame_id = 'imu_frame'
                    imu_msg.linear_acceleration.x = ax / 16384.0
                    imu_msg.linear_acceleration.y = ay / 16384.0
                    imu_msg.linear_acceleration.z = az / 16384.0
                    imu_msg.angular_velocity.x = gx / 131.0
                    imu_msg.angular_velocity.y = gy / 131.0
                    imu_msg.angular_velocity.z = gz / 131.0

                    # Fill in covariance (optional, can be adjusted)
                    imu_msg.linear_acceleration_covariance = [0.0] * 9
                    imu_msg.angular_velocity_covariance = [0.0] * 9
                    imu_msg.orientation_covariance = [0.0] * 9
                    
                    self.publisher_.publish(imu_msg)
                    self.get_logger().info(f'Published IMU data: {line}')

            except Exception as e:
                self.get_logger().error(f'Error reading from serial: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = IMUPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
