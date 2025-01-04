#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class IMUSubscriber(Node):
    def __init__(self):
        super().__init__('imu_subscriber')
        self.subscription = self.create_subscription(Imu, 'imu_data', self.imu_callback, 10)
        self.subscription  # prevent unused variable warning

    def imu_callback(self, msg):
        self.get_logger().info('Received IMU data: linear_acc: (%f, %f, %f), angular_vel: (%f, %f, %f)' %
                               (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
                                msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z))

def main(args=None):
    rclpy.init(args=args)
    node = IMUSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
