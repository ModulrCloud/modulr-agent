# Fresh Ubuntu Laptop Setup Guide - Step by Step

This guide will walk you through setting up a completely fresh Ubuntu laptop to simulate a robot using your camera with the Modulr Agent.

## Prerequisites Check

Before we start, let's check what you already have installed.

### Step 1: Check Your System

Open a terminal and run these commands to check your current setup:

```bash
# Check Ubuntu version
lsb_release -a

# Check if Rust is installed
rustc --version

# Check if ROS2 is installed
ros2 --help 2>/dev/null && echo "ROS2 is installed" || echo "ROS2 is NOT installed"

# Check if camera is available
v4l2-ctl --list-devices 2>/dev/null || echo "v4l2-utils not installed (we'll install it)"
```

**Note:** If ROS2 is not installed, the `ros2 --help` command will give an error - that's fine, we'll install it.

---

## Part 1: Install Rust

### Step 2: Install Rust

Rust is required to build the Modulr Agent. Install it using rustup:

```bash
# Download and run rustup installer
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

# Follow the prompts (press Enter to accept defaults)
# After installation, you'll need to reload your shell
source ~/.cargo/env

# Verify installation
rustc --version
cargo --version
```

You should see version numbers for both commands. If you see "command not found", try:
```bash
source ~/.bashrc
```

---

## Part 2: Install ROS2

### Step 3: Determine Your Ubuntu Version

```bash
lsb_release -rs
```

This will show your Ubuntu version (e.g., 22.04, 24.04).

### Step 4: Install ROS2

**For Ubuntu 22.04 (recommended - use ROS2 Humble):**

```bash
# Update system packages
sudo apt update && sudo apt upgrade -y

# Install prerequisites
sudo apt install -y software-properties-common curl gnupg lsb-release

# Add ROS2 repository
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2-latest.list > /dev/null

# Install ROS2 Humble Desktop
sudo apt update
sudo apt install -y ros-humble-desktop

# Install development tools
sudo apt install -y python3-rosdep python3-colcon-common-extensions ros-dev-tools

# Initialize rosdep (only needed once)
sudo rosdep init
rosdep update
```

**For Ubuntu 24.04 (use ROS2 Jade):**

```bash
# Update system packages
sudo apt update && sudo apt upgrade -y

# Install prerequisites
sudo apt install -y software-properties-common curl gnupg lsb-release

# Add ROS2 repository
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2-latest.list > /dev/null

# Install ROS2 Jade Desktop
sudo apt update
sudo apt install -y ros-jade-desktop

# Install development tools
sudo apt install -y python3-rosdep python3-colcon-common-extensions ros-dev-tools

# Initialize rosdep (only needed once)
sudo rosdep init
rosdep update
```

**Note:** If `sudo rosdep init` says the file already exists, that's fine - skip that step and just run `rosdep update`.

### Step 5: Set Up ROS2 Environment

Add ROS2 to your shell configuration so it's available every time you open a terminal:

```bash
# For ROS2 Humble (Ubuntu 22.04)
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "export ROS_DISTRO=humble" >> ~/.bashrc

# OR for ROS2 Jade (Ubuntu 24.04)
# echo "source /opt/ros/jade/setup.bash" >> ~/.bashrc
# echo "export ROS_DISTRO=jade" >> ~/.bashrc

# Reload your shell configuration
source ~/.bashrc

# Verify ROS2 is working
ros2 --help
```

You should see ROS2 help text. If you get "command not found", try opening a new terminal window.

---

## Part 3: Install Required Dependencies

### Step 6: Install GStreamer (for video streaming)

```bash
sudo apt install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav libgstrtspserver-1.0-dev libges-1.0-dev
```

### Step 7: Install ROS2 Bridge and Camera Packages

```bash
# Make sure ROS2 is sourced
source /opt/ros/$ROS_DISTRO/setup.bash

# Install rosbridge (required for ROS2 communication)
sudo apt install -y ros-$ROS_DISTRO-rosbridge-suite

# Install camera and image processing packages
sudo apt install -y ros-$ROS_DISTRO-cv-bridge ros-$ROS_DISTRO-image-transport python3-opencv v4l-utils
```

---

## Part 4: Clone and Build the Modulr Agent

### Step 8: Clone the Repository

```bash
# Navigate to where you want to clone the project (e.g., Documents or home)
cd ~
# or
cd ~/Documents

# Clone the repository
git clone --recurse-submodules https://github.com/ModulrCloud/modulr-agent
# If you're using your own fork/branch:
# git clone --recurse-submodules <your-repo-url>
# cd modulr-agent
# git checkout webTest  # or your branch name

cd modulr-agent
```

### Step 9: Build the Modulr Agent

```bash
# Make sure ROS2 is sourced
source /opt/ros/$ROS_DISTRO/setup.bash

# Build in debug mode (faster compilation, larger binary)
cargo build

# OR build in release mode (slower compilation, optimized binary)
# cargo build --release
```

This will take several minutes the first time as it downloads and compiles all dependencies.

---

## Part 5: Test Your Camera

### Step 10: Check Camera Availability

```bash
# List all video devices
v4l2-ctl --list-devices
```

You should see your camera listed (e.g., "Integrated Camera" or similar). Note the device path (usually `/dev/video0`).

### Step 11: Test Camera Access

```bash
# Test reading from camera (this will create a test file)
v4l2-ctl --device=/dev/video0 --stream-mmap --stream-to=/tmp/test.raw --stream-count=1

# If this works without errors, your camera is accessible
```

If you get permission errors, you may need to add yourself to the video group:

```bash
sudo usermod -a -G video $USER
# Log out and back in for this to take effect
```

---

## Part 6: Set Up Camera Node

### Step 12: Make Scripts Executable

```bash
cd ~/modulr-agent  # or wherever you cloned it
chmod +x scripts/camera_node.py
chmod +x scripts/setup_ros2_ubuntu.sh
chmod +x scripts/start_camera_simulation.sh
```

### Step 13: Test the Camera Node

```bash
# Make sure ROS2 is sourced
source /opt/ros/$ROS_DISTRO/setup.bash

# Run the camera node
python3 scripts/camera_node.py
```

You should see output like:
```
[INFO] [camera_publisher]: Camera opened successfully. FPS: 30.0
[INFO] [camera_publisher]: Camera node started. Publishing to /camera/image_raw
```

**Press Ctrl+C to stop it.**

In another terminal, verify the topic is publishing:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash

# List topics
ros2 topic list

# You should see /camera/image_raw

# Check topic info
ros2 topic info /camera/image_raw

# View one message (optional)
ros2 topic echo /camera/image_raw --once
```

---

## Part 7: Run the Complete System

### Step 14: Start Everything

You'll need **three terminals** running simultaneously:

**Terminal 1 - Start rosbridge:**
```bash
source /opt/ros/$ROS_DISTRO/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

You should see:
```
[INFO] [rosbridge_websocket]: Rosbridge WebSocket server started on port 9090
```

**Terminal 2 - Start camera node:**
```bash
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/modulr-agent  # or your path
python3 scripts/camera_node.py
```

You should see camera node output.

**Terminal 3 - Run the Modulr Agent:**
```bash
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/modulr-agent

# First-time setup (replace with your values)
cargo run -- initial-setup --robot-id my_laptop_robot --signaling-url wss://your-signaling-server:8765

# Then start the agent
cargo run -- start

# Or with verbose logging
cargo run -- -vvv start
```

### Alternative: Use the Convenience Script

Instead of running rosbridge and camera node separately, you can use the convenience script:

**Terminal 1 - Start rosbridge and camera together:**
```bash
cd ~/modulr-agent
./scripts/start_camera_simulation.sh
```

**Terminal 2 - Run the agent:**
```bash
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/modulr-agent
cargo run -- start
```

---

## Troubleshooting

### Rust Installation Issues

If `rustc --version` doesn't work after installation:
```bash
source ~/.cargo/env
source ~/.bashrc
```

### ROS2 Not Found

If `ros2 --help` doesn't work:
```bash
source /opt/ros/humble/setup.bash  # or jade
ros2 --help
```

If it still doesn't work, check if ROS2 was installed:
```bash
ls /opt/ros/
```

### Camera Permission Denied

```bash
# Add yourself to video group
sudo usermod -a -G video $USER
# Log out and log back in
```

### Camera Node Can't Find Camera

Try different camera indices:
```bash
python3 scripts/camera_node.py 0  # First camera
python3 scripts/camera_node.py 1  # Second camera (if you have multiple)
```

### rosbridge Connection Failed

Make sure rosbridge is running and check the port:
```bash
# Check if port 9090 is in use
netstat -tuln | grep 9090
# or
ss -tuln | grep 9090
```

### Build Errors

If `cargo build` fails:
1. Make sure Rust is properly installed: `rustc --version`
2. Make sure ROS2 is sourced: `source /opt/ros/$ROS_DISTRO/setup.bash`
3. Check error messages for missing dependencies

---

## Quick Reference Commands

Once everything is set up, here are the key commands:

```bash
# Source ROS2 (add to ~/.bashrc to make permanent)
source /opt/ros/$ROS_DISTRO/setup.bash

# Start rosbridge
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# Start camera node
python3 scripts/camera_node.py

# Run agent
cargo run -- start

# Or use convenience script
./scripts/start_camera_simulation.sh
```

---

## Next Steps

Once everything is working:

1. **Set up signaling server** (if testing locally) - see README.md
2. **Configure robot ID and signaling URL** - use `initial-setup` command
3. **Test the connection** - verify camera feed appears in your teleop interface

---

## Summary Checklist

- [ ] Rust installed (`rustc --version` works)
- [ ] ROS2 installed (`ros2 --help` works)
- [ ] GStreamer installed
- [ ] rosbridge-suite installed
- [ ] Camera packages installed (cv-bridge, opencv)
- [ ] Modulr Agent cloned and built
- [ ] Camera accessible (`v4l2-ctl --list-devices` shows camera)
- [ ] Camera node runs successfully
- [ ] rosbridge runs successfully
- [ ] Agent runs successfully

If all checkboxes are checked, you're ready to simulate a robot! 🎉

