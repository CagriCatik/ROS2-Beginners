# IMU Simulation

## Introduction

In this project, we simulate an Inertial Measurement Unit (IMU) sensor using ROS2. The IMU sensor typically provides data such as linear acceleration, angular velocity, and orientation (as a quaternion). The goal is to generate realistic IMU data, visualize it using RQT, and automate the visualization setup.

## Project Setup

### 1. Creating a ROS2 Workspace

Start by creating a ROS2 workspace and a new package named `imu_simulation`:

```bash
mkdir -p ~/ros2_imu_ws/src
cd ~/ros2_imu_ws/
colcon build
source install/setup.bash

cd src
ros2 pkg create --build-type ament_python imu_simulation
```

### 2. Implementing the IMU Data Publisher

Create a Python file named `imu_publisher.py` within the `imu_simulation` package:

```bash
cd ~/ros2_imu_ws/src/imu_simulation/imu_simulation/
```

Here's the content of `imu_publisher.py`:

```python
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
```

This script simulates IMU data with realistic characteristics such as noise, bias, and gravity effects.

## Realistic IMU Data Generation

### 1. **Noise Addition**

- **Linear Acceleration Noise**: `self.accel_noise_stddev` adds Gaussian noise to simulate the random variations in acceleration data.
- **Angular Velocity Noise**: `self.gyro_noise_stddev` adds Gaussian noise to simulate the random variations in angular velocity data.

### 2. **Bias**

- **Linear Acceleration Bias**: A constant offset is added to the acceleration readings (`self.accel_bias`).
- **Angular Velocity Bias**: A constant offset is added to the angular velocity readings (`self.gyro_bias`).

### 3. **Gravity**

- The gravity component is added to the Z-axis of the linear acceleration to simulate the effect of gravity when the IMU is stationary or in a specific orientation.

### 4. **Quaternion Orientation**

- The orientation is slightly randomized to simulate small perturbations, with the quaternion being normalized to ensure it remains valid.

## Visualizing IMU Data with RQT

To visualize the IMU data in `rqt`, follow these steps:

1. **Run the IMU Publisher**:

   ```bash
   ros2 run imu_simulation imu_publisher
   ```

2. **Launch RQT**:

   ```bash
   rqt
   ```

3. **Add Visualization Plugins**:
   - Use `Plugins > Visualization > Plot` to add real-time plots for topics such as:
     - `/imu_data/linear_acceleration/x`
     - `/imu_data/linear_acceleration/y`
     - `/imu_data/linear_acceleration/z`
     - `/imu_data/angular_velocity/x`
     - `/imu_data/angular_velocity/y`
     - `/imu_data/angular_velocity/z`

4. **Customize Visualization**:
   - Adjust axes, scales, and other settings to suit your needs.

## Automating RQT Configuration

You can automate the opening of your `rqt` setup with a pre-saved configuration:

### 1. Save the RQT Configuration

- After setting up the desired plots in `rqt`, go to `File > Save Perspective As...` and save it as `imu_visualization.perspective`.

### 2. Create a Script to Open RQT with the Configuration

Create a script `open_imu_visualization.sh`:

```bash
#!/bin/bash

# Source ROS2 setup
source /opt/ros/humble/setup.bash
source ~/ros2_imu_ws/install/setup.bash

# Open rqt with the saved perspective
rqt --perspective-file ~/ros2_imu_ws/imu_visualization.perspective
```

- Make the script executable:

  ```bash
  chmod +x open_imu_visualization.sh
  ```

- Run the script to automatically launch `rqt` with your saved configuration:

  ```bash
  ./open_imu_visualization.sh
  ```
