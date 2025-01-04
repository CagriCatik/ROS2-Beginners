# Launching Multiple Instances of a Node with Different Configurations

This project demonstrates how to launch multiple instances of a ROS 2 node, each with unique configurations. The scenario involves launching five instances of a temperature sensor node, each with a distinct name and configuration. This setup is particularly useful for managing multiple sensors or devices in a ROS 2-based robotic system.

## Project Structure

The project is organized into a ROS 2 package named `my_sensor_package`, with the following structure:

```
my_sensor_package/
├── launch/
│   └── multiple_sensors_launch.py
├── my_sensor_package/
│   └── sensor_node.py
├── package.xml
└── setup.py
```

- **`sensor_node.py`**: Python script for the sensor node that simulates temperature readings.
- **`multiple_sensors_launch.py`**: Launch file that starts five instances of the sensor node with unique names.
- **`package.xml`**: Defines package metadata and dependencies.
- **`setup.py`**: Setup script that ensures the launch files are included during installation.

## Installation

Follow these steps to install and set up the package:

### 1. Clone the Repository

Clone this repository into your ROS 2 workspace's `src` directory:

```sh
cd ~/ros2_ws/src
git clone <repository_url> my_sensor_package
```

### 2. Build the Package

Navigate to the workspace root and build the package using `colcon`:

```sh
cd ~/ros2_ws
colcon build --packages-select my_sensor_package
```

### 3. Source the Workspace

After building, source your workspace to update your environment:

```sh
source install/setup.bash
```

## Usage

### Launching the Nodes

To launch the multiple sensor nodes, run the following command:

```sh
ros2 launch my_sensor_package multiple_sensors_launch.py
```

This command will start five instances of the `sensor_node`, each named `sensor_node_1`, `sensor_node_2`, `sensor_node_3`, `sensor_node_4`, and `sensor_node_5`.

### Verifying Node Operation

1. **List Active Nodes**:
   To verify that all nodes are running, list the active nodes:

   ```sh
   ros2 node list
   ```

   You should see all five nodes listed.

2. **Query Node Information**:
   To check the details of a specific node, use the `ros2 node info` command:

   ```sh
   ros2 node info /sensor_node_1
   ```

   Ensure you use the correct and full node name, including any prefix (`/`).

## Customization

### Adding More Nodes

To add additional sensor nodes, modify the `multiple_sensors_launch.py` file in the `launch` directory. Copy an existing `Node` block and update the `name` and `parameters` fields accordingly.

### Modifying the Node Behavior

If you need to adjust how the sensor node operates (e.g., change the simulated temperature range), edit the `sensor_node.py` file in the `my_sensor_package` directory.

## Best Practices

1. **Unique Naming**: Always assign unique names to each node instance to avoid conflicts and ensure smooth operation.
2. **Consistent Naming Conventions**: Use meaningful and consistent names for easier identification and management of nodes.
3. **Dynamic Configuration**: Leverage ROS 2’s dynamic remapping capabilities to configure nodes without altering the source code.
4. **Testing**: Regularly verify the operation of each node to ensure they are correctly configured and functioning as expected.

## Troubleshooting

### Node Not Found

If you encounter an "Unable to find node" error when using `ros2 node info`, ensure you are using the correct and full node name (e.g., `/sensor_node_1`).

If the issue persists, try restarting the ROS 2 daemon:

```sh
ros2 daemon stop
ros2 daemon start
```

### Launch File Not Found

If you receive an error indicating that the launch file is not found, ensure that:

- The `launch` directory is correctly placed in your package's root directory.
- The `setup.py` file includes the launch directory as described in the Installation section.