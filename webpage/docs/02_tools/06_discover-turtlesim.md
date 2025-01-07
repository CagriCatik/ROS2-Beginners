# Working with the `turtlesim` Package

This tutorial provides a detailed guide to using the `turtlesim` package in ROS2. The `turtlesim` package is a simplified simulation of a robot that serves as an excellent introduction to the principles of ROS2. By following this guide, you will gain practical experience in installing, running, and manipulating nodes within ROS2.

## Prerequisites

Ensure you have a working ROS2 installation on your system. This tutorial assumes you are familiar with basic ROS2 concepts, such as nodes, topics, and the ROS2 command-line interface.

## Installing the `turtlesim` Package

To begin, you need to install the `turtlesim` package. Open a terminal and use the following command, replacing `<distro>` with your specific ROS2 distribution (e.g., `foxy`, `galactic`, `humble`):

```bash
sudo apt install ros-<distro>-turtlesim
```

For instance, if you are using ROS2 Foxy, the command would be:

```bash
sudo apt install ros-foxy-turtlesim
```

## Setting Up the Environment

After installing `turtlesim`, you need to source your ROS2 workspace. This step is crucial for making ROS2 aware of the newly installed packages. In a terminal, run:

```bash
source /opt/ros/<distro>/setup.bash
```

Replace `<distro>` with your ROS2 distribution.

## Running the `turtlesim` Node

Now that the environment is set up, you can launch the `turtlesim` node. Open a new terminal and execute:

```bash
ros2 run turtlesim turtlesim_node
```

This command will open a graphical window displaying a turtle in the center. The `turtlesim_node` is now running and ready to receive commands.

## Interacting with `turtlesim`

### Listing Active Nodes

To see the active nodes, use the following command:

```bash
ros2 node list
```

You should see `turtlesim_node` in the list of active nodes.

### Controlling the Turtle

You can control the turtle using another node called `turtle_teleop_key`. Open a new terminal and run:

```bash
ros2 run turtlesim turtle_teleop_key
```

This node allows you to control the turtle using the arrow keys on your keyboard. The `turtle_teleop_key` node captures your key inputs and sends them to the `turtlesim_node`.

### Understanding Node Communication

To visualize the communication between nodes, you can use the `rqt_graph` tool. First, ensure `rqt` is installed:

```bash
sudo apt install ros-<distro>-rqt-graph
```

Then, in a new terminal, run:

```bash
rqt_graph
```

This command opens a graphical interface displaying the nodes and topics involved in the current ROS2 system. You should see `turtle_teleop_key` communicating with `turtlesim_node`.

### Commanding the Turtle Programmatically

Nodes in ROS2 communicate via topics. The `turtlesim_node` listens to a topic named `/turtle1/cmd_vel` for velocity commands. You can publish messages to this topic to control the turtle. For example, to make the turtle move forward, open a new terminal and run:

```bash
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

This command publishes a `Twist` message to the `/turtle1/cmd_vel` topic, causing the turtle to move forward with a linear velocity of 2.0 units.

### Renaming Nodes

You can also rename nodes using the `--ros-args --remap` option. For instance, to rename `turtlesim_node` to `my_turtle`, use the following command:

```bash
ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle
```

After running this command, if you list the active nodes again using `ros2 node list`, you will see `my_turtle` instead of `turtlesim_node`.

## Conclusion

In this tutorial, you have learned how to install, run, and interact with the `turtlesim` package in ROS2. You have also explored node communication, controlling nodes programmatically, and renaming nodes. These skills provide a solid foundation for further exploration and development within the ROS2 ecosystem.
