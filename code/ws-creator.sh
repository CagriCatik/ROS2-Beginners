#!/bin/bash

# Define the workspace name
WORKSPACE_NAME="my_ros2_workspace"

# Create the workspace directory
mkdir -p ~/$WORKSPACE_NAME/src
cd ~/$WORKSPACE_NAME

# Initialize the workspace
source /opt/ros/humble/setup.bash
colcon build

# Source the workspace
echo "source ~/$WORKSPACE_NAME/install/setup.bash" >> ~/.bashrc
source ~/.bashrc

echo "ROS2 workspace '$WORKSPACE_NAME' created and initialized."
