# Installing ROS 2 Humble

This tutorial provides a detailed, step-by-step guide to installing ROS 2 Humble on Kubuntu 22.04. Whether you are using a dual-boot setup or a virtual machine, these instructions will guide you through the process accurately and efficiently.

## Prerequisites

Before beginning the installation, ensure that you have Kubuntu 22.04 installed and that your system is up to date. Open a terminal and update your package lists:

```bash
sudo apt update
sudo apt upgrade
```

## Setting Up Locale

ROS 2 requires UTF-8 locale support. Verify your locale settings and set them if necessary:

```bash
locale  # Check the current locale

# If the output does not contain 'en_US.UTF-8', set it up as follows:
sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8  # Set the environment variable
```

## Adding ROS 2 Package Sources

To install ROS 2, you need to add the ROS 2 package sources to your system. Begin by ensuring the `universe` repository is enabled:

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

Next, add the ROS 2 GPG key and repository:

```bash
sudo apt update
sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [trusted=yes] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt update
```

## Installing ROS 2

With the sources added, you can now install ROS 2. There are different configurations available, but for a comprehensive setup, install the Desktop version, which includes the ROS base along with various development tools:

```bash
sudo apt install ros-humble-desktop
```

This installation may take some time as it involves downloading several packages. Confirm the installation when prompted.

## Setting Up the Environment

After installation, you need to source the ROS 2 environment. To make this automatic for every terminal session, add the following line to your `.bashrc` file:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Verifying the Installation

To verify that ROS 2 is installed correctly, open a new terminal and run:

```bash
ros2 --version
```

This should output the version of ROS 2 you have installed, indicating a successful installation.

## Installing Additional ROS 2 Packages

To install additional ROS 2 packages, use the following command pattern:

```bash
sudo apt install ros-humble-<package_name>
```

Replace `<package_name>` with the specific package you need. For instance, to install the `turtlesim` package, you would run:

```bash
sudo apt install ros-humble-turtlesim
```