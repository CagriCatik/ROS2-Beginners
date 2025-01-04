# Launching Your First ROS 2 Program

## Prerequisites

Before proceeding, ensure that you have successfully installed ROS 2 and correctly set up your environment. This tutorial assumes that you have followed the installation instructions and configured your environment as described in the ROS 2 documentation. If you encounter issues, refer back to the installation steps to verify that everything was set up correctly.

## Verifying ROS 2 Installation

To confirm that ROS 2 is properly installed and configured, you can perform a simple test by launching a pre-existing example. This test will help you verify that your installation is functional before you proceed to writing your own ROS 2 programs.

## Step-by-Step Guide

1. **Open a Terminal**
   Begin by opening a terminal window. This will be your interface for executing ROS 2 commands.
2. **Source the ROS 2 Environment**
   Before running any ROS 2 commands, you need to source the ROS 2 setup file to ensure the environment is correctly configured. This step is crucial as it sets up the necessary environment variables.

   ```sh
   source /opt/ros/<distro>/setup.bash
   ```

   Replace `<distro>` with the name of your ROS 2 distribution (e.g., `humble`, `galactic`).

   To avoid having to source the setup file every time you open a terminal, add the sourcing command to your `~/.bashrc` file:

   ```sh
   echo "source /opt/ros/<distro>/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

3. **Verify ROS 2 Command Availability**
   To verify that ROS 2 commands are available, type `ros2` followed by pressing the `Tab` key twice. You should see a list of ROS 2 commands if everything is set up correctly. If no commands appear, check your installation and environment setup.

   ```sh
   ros2 <Tab><Tab>
   ```

4. **Running the Talker Example**
   Now, let's run an example node called `talker`. This node is part of the `demo_nodes_cpp` package and it publishes messages.

   ```sh
   ros2 run demo_nodes_cpp talker
   ```

   You should see output indicating that the `talker` node is publishing messages:

   ```
   [INFO] [talker]: Publishing: 'Hello World: <counter>'
   ```

5. **Running the Listener Example**
   Open another terminal window, source the ROS 2 setup file again, and then run the `listener` node from the same package. This node will subscribe to the messages published by the `talker` node and print them.

   ```sh
   source /opt/ros/<distro>/setup.bash
   ros2 run demo_nodes_cpp listener
   ```

   The `listener` node should output the messages it receives from the `talker` node:

   ```
   [INFO] [listener]: I heard: 'Hello World: <counter>'
   ```

6. **Stopping the Nodes**
   To stop the nodes, use `Ctrl+C` in each terminal window where the nodes are running. This will gracefully terminate the nodes.

   ```sh
   Ctrl+C
   ```

## Troubleshooting

- **No Output from `ros2 run` Command:**
  Ensure that the ROS 2 setup file has been sourced correctly in both terminal windows.
- **Error Messages:**
  Verify that the `demo_nodes_cpp` package is correctly installed. If it is not, reinstall ROS 2 and ensure all default packages are included.
