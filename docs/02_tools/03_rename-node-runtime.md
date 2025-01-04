# Launching Multiple Instances of a Node with Different Configurations

In many robotic applications, there is often a need to launch multiple instances of the same node, each with different configurations. This tutorial will guide you through the process of launching the same node multiple times with unique configurations in ROS 2. We will address potential issues and explain best practices to ensure smooth operation of your nodes.

## Background

In ROS 1, it was not possible to launch multiple nodes with the same name. However, in ROS 2, it is technically possible, though not advisable due to the complications it introduces. Launching multiple nodes with the same name can lead to unintended side effects and communication problems within your ROS graph.

## Practical Example: Multiple Temperature Sensors

Consider a scenario where you have a temperature sensor node and you need to launch this node five times, once for each of your five temperature sensors. Each instance must have a unique name to function correctly.

## Step-by-Step Tutorial

### 1. Launching Nodes with Identical Names

First, let's explore what happens if you launch nodes with the same name. In a terminal, run:

```sh
ros2 run my_python_package my_node
```

In a different terminal, run the same command again:

```sh
ros2 run my_python_package my_node
```

Then list the active nodes:

```sh
ros2 node list
```

You will see a warning:

```sh
Be aware that some nodes in the graph share an exact name and this can have unintended side effects.
```

Both nodes will appear in the list, but this setup is problematic. Interactions with these nodes, such as retrieving node information, will yield warnings and ambiguous results.

### 2. Properly Launching Multiple Instances

To avoid issues, you must assign a unique name to each node instance. ROS 2 allows you to remap node names dynamically at launch. Here’s how to do it.

In one terminal, start the first node with a unique name:

```sh
ros2 run my_python_package my_node --ros-args -r __node:=sensor_node_1
```

In another terminal, start the second node with a different unique name:

```sh
ros2 run my_python_package my_node --ros-args -r __node:=sensor_node_2
```

Repeat this process for additional nodes, ensuring each has a unique name:

```sh
ros2 run my_python_package my_node --ros-args -r __node:=sensor_node_3
ros2 run my_python_package my_node --ros-args -r __node:=sensor_node_4
ros2 run my_python_package my_node --ros-args -r __node:=sensor_node_5
```

Now, if you list the active nodes:

```sh
ros2 node list
```

You will see all five nodes, each with a unique name, indicating that they are correctly set up.

### 3. Verifying Node Configurations

To verify that each node is functioning as expected, you can query the node information. For instance:

```sh
ros2 node info sensor_node_1
ros2 node info sensor_node_2
```

Ensure that each command returns information specific to the respective node instance.

### 4. Extending the Example: Remapping Topics and Services

Beyond renaming nodes, ROS 2 allows you to remap topics, services, and parameters at runtime. This feature is useful for configuring nodes dynamically without changing the source code.

For example, to remap a topic:

```sh
ros2 run my_python_package my_node --ros-args -r __node:=sensor_node_1 -r temperature:=sensor_1/temperature
```

This command remaps the `temperature` topic to `sensor_1/temperature` for the `sensor_node_1`.

## Best Practices

1. **Unique Naming**: Always assign unique names to node instances to avoid conflicts and unintended behaviors.
2. **Consistent Naming Conventions**: Use meaningful and consistent naming conventions to easily identify and manage your nodes.
3. **Dynamic Configuration**: Take advantage of ROS 2’s dynamic remapping capabilities to flexibly configure nodes at runtime.
4. **Testing and Verification**: Regularly verify the configurations and interactions of your nodes to ensure correct operation.

## Conclusion

Launching multiple instances of a node with different configurations in ROS 2 is a powerful capability that can streamline many robotic applications. By following the best practices outlined in this tutorial, you can ensure robust and error-free operation of your nodes. Proper node management and dynamic configuration are key to leveraging the full potential of ROS 2 in complex systems.
