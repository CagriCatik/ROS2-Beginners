import subprocess
import os

# Define the workspace name
workspace_name = "my_ros2_workspace"

# Create the workspace directory
os.makedirs(os.path.expanduser(f"~/{workspace_name}/src"), exist_ok=True)

# Initialize the workspace
subprocess.run(["source", "/opt/ros/foxy/setup.bash"], shell=True, check=True)
subprocess.run(["colcon", "build"], cwd=os.path.expanduser(f"~/{workspace_name}"), check=True)

# Source the workspace
with open(os.path.expanduser("~/.bashrc"), "a") as bashrc:
    bashrc.write(f"source ~/{workspace_name}/install/setup.bash\n")

subprocess.run(["source", os.path.expanduser("~/.bashrc")], shell=True, check=True)

print(f"ROS2 workspace '{workspace_name}' created and initialized.")
