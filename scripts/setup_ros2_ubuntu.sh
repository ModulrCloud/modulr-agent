#!/bin/bash

# ROS2 Setup Script for Ubuntu
# This script installs ROS2 and necessary dependencies for the Modulr Agent

set -e  # Exit on error

echo "=========================================="
echo "ROS2 Setup for Modulr Agent"
echo "=========================================="

# Detect Ubuntu version
UBUNTU_VERSION=$(lsb_release -rs)
echo "Detected Ubuntu version: $UBUNTU_VERSION"

# Determine ROS2 distribution based on Ubuntu version
if [[ "$UBUNTU_VERSION" == "22.04" ]]; then
    ROS_DISTRO="humble"
    echo "Using ROS2 Humble (recommended for Ubuntu 22.04)"
elif [[ "$UBUNTU_VERSION" == "24.04" ]]; then
    ROS_DISTRO="jade"
    echo "Using ROS2 Jade (recommended for Ubuntu 24.04)"
else
    echo "Warning: Ubuntu version $UBUNTU_VERSION detected."
    echo "Defaulting to ROS2 Humble. You may need to adjust this."
    ROS_DISTRO="humble"
fi

echo ""
echo "Step 1: Updating system packages..."
sudo apt update && sudo apt upgrade -y

echo ""
echo "Step 2: Installing prerequisites..."
sudo apt install -y \
    software-properties-common \
    curl \
    gnupg \
    lsb-release

echo ""
echo "Step 3: Adding ROS2 repository..."
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2-latest.list > /dev/null

echo ""
echo "Step 4: Installing ROS2..."
sudo apt update
sudo apt install -y ros-$ROS_DISTRO-desktop

echo ""
echo "Step 5: Installing ROS2 development tools..."
sudo apt install -y \
    python3-rosdep \
    python3-colcon-common-extensions \
    ros-dev-tools

echo ""
echo "Step 6: Initializing rosdep..."
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update

echo ""
echo "Step 7: Installing rosbridge-suite (required for ROS2 bridge)..."
source /opt/ros/$ROS_DISTRO/setup.bash
sudo apt install -y ros-$ROS_DISTRO-rosbridge-suite

echo ""
echo "Step 8: Installing camera-related packages..."
sudo apt install -y \
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-image-transport \
    python3-opencv \
    v4l-utils

echo ""
echo "Step 9: Setting up environment..."
if ! grep -q "source /opt/ros/$ROS_DISTRO/setup.bash" ~/.bashrc; then
    echo "" >> ~/.bashrc
    echo "# ROS2 Setup" >> ~/.bashrc
    echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
    echo "export ROS_DISTRO=$ROS_DISTRO" >> ~/.bashrc
fi

echo ""
echo "=========================================="
echo "ROS2 Setup Complete!"
echo "=========================================="
echo ""
echo "To use ROS2 in this terminal, run:"
echo "  source /opt/ros/$ROS_DISTRO/setup.bash"
echo ""
echo "Or open a new terminal (it will be sourced automatically)."
echo ""
echo "Next steps:"
echo "1. Test ROS2 installation: ros2 --help"
echo "2. List available cameras: v4l2-ctl --list-devices"
echo "3. Run the camera node: python3 scripts/camera_node.py"
echo "4. Start rosbridge: ros2 launch rosbridge_server rosbridge_websocket_launch.xml"
echo ""

