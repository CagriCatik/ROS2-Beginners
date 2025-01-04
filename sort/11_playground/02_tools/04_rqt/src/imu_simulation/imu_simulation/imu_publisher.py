#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import random
import math

class IMUPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.publisher_ = self.create_publisher(Imu, 'imu_data', 10)
        self.timer = self.create_timer(0.25, self.publish_imu_data)

        # IMU characteristics
        self.accel_noise_stddev = 0.2  # m/s^2
        self.gyro_noise_stddev = 0.01  # rad/s
        self.accel_bias = [0.1, -0.1, 0.05]  # constant bias
        self.gyro_bias = [0.01, -0.02, 0.005]  # constant bias
        self.gravity = 9.81  # m/s^2

    def publish_imu_data(self):
        imu_msg = Imu()
        
        # Simulate linear acceleration with noise and bias
        imu_msg.linear_acceleration.x = random.gauss(0, self.accel_noise_stddev) + self.accel_bias[0]
        imu_msg.linear_acceleration.y = random.gauss(0, self.accel_noise_stddev) + self.accel_bias[1]
        imu_msg.linear_acceleration.z = random.gauss(0, self.accel_noise_stddev) + self.accel_bias[2] + self.gravity
        
        # Simulate angular velocity with noise and bias
        imu_msg.angular_velocity.x = random.gauss(0, self.gyro_noise_stddev) + self.gyro_bias[0]
        imu_msg.angular_velocity.y = random.gauss(0, self.gyro_noise_stddev) + self.gyro_bias[1]
        imu_msg.angular_velocity.z = random.gauss(0, self.gyro_noise_stddev) + self.gyro_bias[2]

        # Generate a realistic orientation (Quaternion)
        # Let's assume the IMU is not rotating for simplicity and just simulate small random changes
        imu_msg.orientation.x = random.uniform(-0.01, 0.01)
        imu_msg.orientation.y = random.uniform(-0.01, 0.01)
        imu_msg.orientation.z = random.uniform(-0.01, 0.01)
        imu_msg.orientation.w = math.sqrt(1 - (imu_msg.orientation.x**2 + imu_msg.orientation.y**2 + imu_msg.orientation.z**2))

        self.publisher_.publish(imu_msg)
        self.get_logger().info('Publishing IMU data: linear_acc: (%f, %f, %f), angular_vel: (%f, %f, %f)' %
                               (imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z,
                                imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z))

def main(args=None):
    rclpy.init(args=args)
    node = IMUPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
