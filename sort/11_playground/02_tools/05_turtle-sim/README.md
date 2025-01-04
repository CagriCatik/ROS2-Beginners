# Exploring the `turtlesim` Package in ROS2

This project will walk you through the steps to set up and interact with the `turtlesim` package in ROS2. We will cover installation, launching nodes, controlling the turtle, and understanding node communication within ROS2.

## Step 1: Install the `turtlesim` Package

1. **Open a Terminal**: Start by opening a terminal window.

2. **Install `turtlesim`**: Run the following command to install the `turtlesim` package. Replace `humble` with your ROS2 distribution name, such as `foxy`, `galactic`, or `humble`.

   ```bash
   sudo apt install ros-humble-turtlesim
   ```

## Step 2: Set Up the ROS2 Environment

1. **Source the Workspace**: Ensure ROS2 can locate the newly installed packages by sourcing your workspace:

   ```bash
   source /opt/ros/humble/setup.bash
   ```

   Replace `humble` with your ROS2 distribution.

## Step 3: Launch the `turtlesim` Node

1. **Start the `turtlesim` Node**: In a new terminal, run the following command to launch the `turtlesim` node:

   ```bash
   ros2 run turtlesim turtlesim_node
   ```

   A graphical window will appear with a turtle in the center. This turtle is now ready to receive commands.

## Step 4: Control the Turtle

1. **List Active Nodes**: Check the running nodes by executing the following command in a terminal:

   ```bash
   ros2 node list
   ```

   You should see `turtlesim_node` listed.

2. **Control the Turtle with Keyboard**: To manually control the turtle, open another terminal and run:

   ```bash
   ros2 run turtlesim turtle_teleop_key
   ```

   Use the arrow keys to move the turtle around the screen.

## Step 5: Understand Node Communication

1. **Visualize Node Communication**: To see how nodes communicate, you can use the `rqt_graph` tool. First, install it if necessary:

   ```bash
   sudo apt install ros-humble-rqt-graph
   ```

   Then, run the tool:

   ```bash
   rqt_graph
   ```

   This will open a graphical interface showing the communication between the `turtlesim_node` and `turtle_teleop_key`.

## Step 6: Control the Turtle Programmatically

1. **Publish Velocity Commands**: You can also control the turtle by publishing messages to its velocity topic. In a new terminal, run:

   ```bash
   ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
   ```

   This command will make the turtle move forward.

## Step 7: Rename the `turtlesim` Node

1. **Rename the Node**: To rename the `turtlesim_node`, run the following command:

   ```bash
   ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle
   ```

   After renaming, list the active nodes again using `ros2 node list`, and you will see `my_turtle` instead of `turtlesim_node`.
