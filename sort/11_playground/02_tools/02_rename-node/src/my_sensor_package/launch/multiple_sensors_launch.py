from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_sensor_package',
            executable='sensor_node',
            name='sensor_node_1',
            output='screen',
            parameters=[{'sensor_id': 1}]
        ),
        Node(
            package='my_sensor_package',
            executable='sensor_node',
            name='sensor_node_2',
            output='screen',
            parameters=[{'sensor_id': 2}]
        ),
        Node(
            package='my_sensor_package',
            executable='sensor_node',
            name='sensor_node_3',
            output='screen',
            parameters=[{'sensor_id': 3}]
        ),
        Node(
            package='my_sensor_package',
            executable='sensor_node',
            name='sensor_node_4',
            output='screen',
            parameters=[{'sensor_id': 4}]
        ),
        Node(
            package='my_sensor_package',
            executable='sensor_node',
            name='sensor_node_5',
            output='screen',
            parameters=[{'sensor_id': 5}]
        ),
    ])
