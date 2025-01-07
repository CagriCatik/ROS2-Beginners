# Customizing Node Configurations

In ROS 2, launch files provide a powerful and flexible way to start and manage multiple nodes within an application. This tutorial will demonstrate how to customize your application using launch files by renaming nodes, remapping topics, and setting parameters. The instructions will be precise and scientific, ensuring clarity and accuracy.

## Introduction to Launch Files

Launch files in ROS 2 are written in Python, allowing for extensive customization through Python scripting. They provide a convenient way to automate the process of starting multiple nodes with specific configurations.

### Creating a Basic Launch File

Before diving into customization, let's create a basic launch file to start two nodes: `number_publisher` and `number_counter`.

```python
import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='your_package_name',
            executable='number_publisher',
            name='number_publisher',
            output='screen'
        ),
        Node(
            package='your_package_name',
            executable='number_counter',
            name='number_counter',
            output='screen'
        )
    ])

if __name__ == '__main__':
    generate_launch_description()
```

This launch file will start the `number_publisher` and `number_counter` nodes with default names and configurations.

## Renaming Nodes

To rename nodes, you simply add the `name` argument to the `Node` action. This changes the default node names to more descriptive ones.

```python
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='your_package_name',
            executable='number_publisher',
            name='my_number_publisher',
            output='screen'
        ),
        Node(
            package='your_package_name',
            executable='number_counter',
            name='my_number_counter',
            output='screen'
        )
    ])
```

By setting the `name` argument, `number_publisher` is renamed to `my_number_publisher` and `number_counter` to `my_number_counter`.

## Remapping Topics

Remapping topics allows nodes to communicate over different topic names. This is useful when integrating nodes that expect different topic names.

```python
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='your_package_name',
            executable='number_publisher',
            name='my_number_publisher',
            output='screen',
            remappings=[
                ('/number', '/my_number')
            ]
        ),
        Node(
            package='your_package_name',
            executable='number_counter',
            name='my_number_counter',
            output='screen',
            remappings=[
                ('/number', '/my_number'),
                ('/number_count', '/my_number_count')
            ]
        )
    ])
```

In this example, the `number_publisher` node publishes to `/my_number` instead of `/number`. Similarly, `number_counter` subscribes to `/my_number` and publishes to `/my_number_count`.

## Setting Parameters

Parameters can be set within the launch file, providing a way to configure node behavior without modifying the node code.

```python
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='your_package_name',
            executable='number_publisher',
            name='my_number_publisher',
            output='screen',
            parameters=[{
                'number_to_publish': 4,
                'publish_frequency': 5.0
            }]
        ),
        Node(
            package='your_package_name',
            executable='number_counter',
            name='my_number_counter',
            output='screen'
        )
    ])
```

Here, the `number_publisher` node is configured to publish the number `4` at a frequency of `5.0 Hz`.

## Comprehensive Example

Combining all the elements above, we get a comprehensive launch file:

```python
import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    remap_number_topic = ('/number', '/my_number')
    remap_count_topic = ('/number_count', '/my_number_count')

    return LaunchDescription([
        Node(
            package='your_package_name',
            executable='number_publisher',
            name='my_number_publisher',
            output='screen',
            remappings=[remap_number_topic],
            parameters=[{
                'number_to_publish': 4,
                'publish_frequency': 5.0
            }]
        ),
        Node(
            package='your_package_name',
            executable='number_counter',
            name='my_number_counter',
            output='screen',
            remappings=[remap_number_topic, remap_count_topic]
        )
    ])

if __name__ == '__main__':
    generate_launch_description()
```

This launch file:

1. Renames the nodes `number_publisher` to `my_number_publisher` and `number_counter` to `my_number_counter`.
2. Remaps the `/number` topic to `/my_number` and `/number_count` to `/my_number_count`.
3. Sets the parameters for `number_publisher`.

## Running the Launch File

To execute the launch file, use the following command:

```bash
ros2 launch your_package_name your_launch_file_name.py
```

Replace `your_package_name` with the actual package name and `your_launch_file_name.py` with the name of your launch file.

## Conclusion

This tutorial has demonstrated how to customize ROS 2 applications using launch files. By renaming nodes, remapping topics, and setting parameters, you can configure your application to meet specific requirements efficiently. The flexibility provided by Python-based launch files allows for sophisticated configurations and logical operations, enhancing the capability of your ROS 2 applications.