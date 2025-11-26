# ROS2 Setup Guide for Ubuntu Laptop

This guide will help you set up ROS2 on your Ubuntu laptop and configure a camera node to simulate a robot for the Modulr Agent.

## Prerequisites

- Ubuntu 20.04, 22.04, or 24.04
- A working webcam/camera
- Internet connection

## Quick Setup

### Option 1: Automated Setup Script

Run the automated setup script:

```bash
cd scripts
chmod +x setup_ros2_ubuntu.sh
./setup_ros2_ubuntu.sh
```

After the script completes, open a new terminal or source ROS2:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
```

### Option 2: Manual Setup

#### 1. Install ROS2

For Ubuntu 22.04 (recommended):
```bash
sudo apt update && sudo apt upgrade -y
sudo apt install software-properties-common curl gnupg lsb-release

# Add ROS2 repository
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2-latest.list > /dev/null

# Install ROS2 Humble
sudo apt update
sudo apt install -y ros-humble-desktop

# Install development tools
sudo apt install -y python3-rosdep python3-colcon-common-extensions ros-dev-tools

# Initialize rosdep
sudo rosdep init
rosdep update
```

#### 2. Install Required Packages

```bash
source /opt/ros/humble/setup.bash

# Install rosbridge (required for ROS2 bridge)
sudo apt install -y ros-humble-rosbridge-suite

# Install camera and image processing packages
sudo apt install -y ros-humble-cv-bridge ros-humble-image-transport python3-opencv v4l-utils
```

#### 3. Set Up Environment

Add ROS2 to your `.bashrc`:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "export ROS_DISTRO=humble" >> ~/.bashrc
source ~/.bashrc
```

## Testing Your Setup

### 1. Verify ROS2 Installation

```bash
ros2 --help
```

You should see ROS2 command help.

### 2. List Available Cameras

```bash
v4l2-ctl --list-devices
```

This will show all video devices. Note the device path (e.g., `/dev/video0`).

### 3. Test Camera Access

```bash
# Test if you can read from the camera
v4l2-ctl --device=/dev/video0 --stream-mmap --stream-to=/tmp/test.raw --stream-count=1
```

## Running the Camera Node

The camera node publishes images to `/camera/image_raw`, which is what the Modulr Agent expects.

### 1. Make the Script Executable

```bash
chmod +x scripts/camera_node.py
```

### 2. Run the Camera Node

```bash
# Source ROS2 (if not already sourced)
source /opt/ros/humble/setup.bash

# Run the camera node (defaults to camera index 0)
python3 scripts/camera_node.py

# Or specify a different camera index
python3 scripts/camera_node.py 1
```

You should see output like:
```
[INFO] [camera_publisher]: Camera opened successfully. FPS: 30.0
[INFO] [camera_publisher]: Camera node started. Publishing to /camera/image_raw
```

### 3. Verify the Topic is Publishing

In another terminal:

```bash
source /opt/ros/humble/setup.bash

# List topics
ros2 topic list

# You should see /camera/image_raw in the list

# Check the topic info
ros2 topic info /camera/image_raw

# View the images (optional, requires rqt_image_view)
sudo apt install -y ros-humble-rqt-image-view
rqt_image_view
```

## Running the Complete System

To run the Modulr Agent with your camera:

### Terminal 1: Start rosbridge

```bash
source /opt/ros/humble/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

### Terminal 2: Start the Camera Node

```bash
source /opt/ros/humble/setup.bash
python3 scripts/camera_node.py
```

### Terminal 3: Run the Modulr Agent

```bash
# From the modulr-agent directory
source /opt/ros/humble/setup.bash

# If first time setup
cargo run -- initial-setup --robot-id my_laptop_robot --signaling-url wss://your-signaling-server:8765

# Start the agent
cargo run -- start
```

## Troubleshooting

### Camera Not Found

If the camera node can't find your camera:

1. Check available devices:
   ```bash
   v4l2-ctl --list-devices
   ```

2. Try different camera indices:
   ```bash
   python3 scripts/camera_node.py 0  # First camera
   python3 scripts/camera_node.py 1  # Second camera
   ```

3. Check camera permissions:
   ```bash
   ls -l /dev/video*
   # If needed, add yourself to video group:
   sudo usermod -a -G video $USER
   # Then log out and back in
   ```

### ROS2 Topics Not Visible

1. Make sure ROS2 is sourced:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. Check if the camera node is running:
   ```bash
   ros2 node list
   ```

3. Verify topic exists:
   ```bash
   ros2 topic list
   ros2 topic echo /camera/image_raw --once
   ```

### rosbridge Connection Issues

1. Verify rosbridge is running:
   ```bash
   ros2 node list | grep rosbridge
   ```

2. Check if port 9090 is in use:
   ```bash
   netstat -tuln | grep 9090
   # or
   ss -tuln | grep 9090
   ```

3. Test rosbridge connection:
   ```bash
   # In a browser, go to: http://localhost:9090
   # You should see rosbridge websocket server info
   ```

## Additional Resources

- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [ROS2 Camera Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [OpenCV Python Documentation](https://docs.opencv.org/)

## Notes

- The camera node publishes at 30 FPS by default. You can modify the `frame_rate` parameter in `camera_node.py` if needed.
- The image format is BGR8 (standard OpenCV format), which is compatible with ROS2 sensor_msgs/Image.
- The topic name `/camera/image_raw` matches what the Modulr Agent expects (hardcoded in the ROS2 bridge).

