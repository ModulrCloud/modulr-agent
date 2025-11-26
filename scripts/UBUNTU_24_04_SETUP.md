# Ubuntu 24.04.3 Setup Guide - Step by Step

This guide is specifically tailored for **Ubuntu 24.04.3** and will walk you through setting up everything needed to simulate a robot using your laptop camera.

## Quick Overview

For Ubuntu 24.04.3, you'll be using **ROS2 Jazzy Jalisco** (the correct distribution name, not Jade).

The **server URL and robot ID are only needed when running the agent**, not for building. You can build and test everything first, then get those values from Alex later.

---

## Step 1: Verify Your System

```bash
# Check Ubuntu version (should show 24.04)
lsb_release -a

# Update system
sudo apt update && sudo apt upgrade -y
```

---

## Step 2: Install Rust

```bash
# Download and install Rust
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

# Press Enter to accept defaults
# Then reload your shell
source ~/.cargo/env
source ~/.bashrc

# Verify installation
rustc --version
cargo --version
```

**Expected output:** You should see version numbers (e.g., `rustc 1.xx.x`)

---

## Step 3: Install ROS2 Jazzy

For Ubuntu 24.04, the correct ROS2 distribution is **Jazzy Jalisco**.

```bash
# Install prerequisites
sudo apt install -y software-properties-common curl gnupg lsb-release

# Ensure Universe repository is enabled
sudo add-apt-repository universe
sudo apt update

# Add ROS2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS2 repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2-latest.list > /dev/null

# Update package list
sudo apt update

# Install ROS2 Jazzy Desktop
sudo apt install -y ros-jazzy-desktop

# Install development tools
sudo apt install -y python3-rosdep python3-colcon-common-extensions ros-dev-tools

# Initialize rosdep (only needed once - may give error if already done, that's OK)
sudo rosdep init 2>/dev/null || echo "rosdep already initialized"
rosdep update
```

**Verify ROS2 installation:**
```bash
source /opt/ros/jazzy/setup.bash
ros2 --help
```

You should see ROS2 help text. If not, try opening a new terminal.

---

## Step 4: Install Required ROS2 Packages

```bash
# Make sure ROS2 is sourced
source /opt/ros/jazzy/setup.bash

# Install rosbridge (required for ROS2 communication with the agent)
sudo apt install -y ros-jazzy-rosbridge-suite

# Install camera and image processing packages
sudo apt install -y ros-jazzy-cv-bridge ros-jazzy-image-transport python3-opencv v4l-utils
```

**Verify packages:**
```bash
# Check if rosbridge is installed
dpkg -l | grep ros-jazzy-rosbridge-suite

# Should show something like: ii  ros-jazzy-rosbridge-suite ...
```

---

## Step 5: Install GStreamer

```bash
sudo apt install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav libgstrtspserver-1.0-dev libges-1.0-dev
```

---

## Step 6: Set Up ROS2 Environment (Make It Permanent)

Add ROS2 to your `.bashrc` so it's automatically available in every terminal:

```bash
# Add to ~/.bashrc
echo "" >> ~/.bashrc
echo "# ROS2 Jazzy Setup" >> ~/.bashrc
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "export ROS_DISTRO=jazzy" >> ~/.bashrc

# Reload your shell
source ~/.bashrc

# Verify
echo $ROS_DISTRO
# Should output: jazzy
```

---

## Step 7: Clone and Build the Modulr Agent

```bash
# Navigate to where you want the project
cd ~
# or
cd ~/Documents

# Clone the repository (replace with your fork URL if you have one)
git clone --recurse-submodules https://github.com/ModulrCloud/modulr-agent
cd modulr-agent

# If you're on a branch like webTest
git checkout webTest  # or your branch name
```

**Build the agent:**
```bash
# Make sure ROS2 is sourced
source /opt/ros/jazzy/setup.bash

# Build (this will take 5-10 minutes the first time)
cargo build
```

**Note:** The build does NOT require server URL or robot ID. Those are only needed when you run the agent later.

**If build succeeds**, you should see:
```
Finished dev [unoptimized + debuginfo] target(s) in ...
```

---

## Step 8: Test Your Camera

```bash
# List available cameras
v4l2-ctl --list-devices
```

You should see your laptop camera listed (e.g., "Integrated Camera" or similar).

**If you get permission errors:**
```bash
sudo usermod -a -G video $USER
# Log out and log back in for this to take effect
```

**Test camera access:**
```bash
v4l2-ctl --device=/dev/video0 --stream-mmap --stream-to=/tmp/test.raw --stream-count=1
# If this completes without errors, your camera works!
```

---

## Step 9: Test the Camera Node

```bash
cd ~/modulr-agent  # or your path

# Make script executable
chmod +x scripts/camera_node.py

# Run the camera node
source /opt/ros/jazzy/setup.bash
python3 scripts/camera_node.py
```

**Expected output:**
```
[INFO] [camera_publisher]: Camera opened successfully. FPS: 30.0
[INFO] [camera_publisher]: Camera node started. Publishing to /camera/image_raw
```

**Press Ctrl+C to stop.**

**In another terminal, verify the topic:**
```bash
source /opt/ros/jazzy/setup.bash

# List topics
ros2 topic list
# Should show: /camera/image_raw

# Check topic info
ros2 topic info /camera/image_raw
# Should show: Type: sensor_msgs/msg/Image
```

---

## Step 10: Test Everything Together (Without Server URL/Robot ID)

You can test the camera and rosbridge connection without the server URL and robot ID:

**Terminal 1 - Start rosbridge:**
```bash
source /opt/ros/jazzy/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

**Expected output:**
```
[INFO] [rosbridge_websocket]: Rosbridge WebSocket server started on port 9090
```

**Terminal 2 - Start camera node:**
```bash
source /opt/ros/jazzy/setup.bash
cd ~/modulr-agent
python3 scripts/camera_node.py
```

**Terminal 3 - Verify topics are available:**
```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list
# Should show: /camera/image_raw

ros2 topic echo /camera/image_raw --once
# Should show image data (will be very long)
```

**This confirms everything is working!** You just can't run the full agent yet without the server URL and robot ID.

---

## Step 11: Get Server URL and Robot ID from Alex

Once you have these values from Alex, you can run the full agent.

---

## Step 12: Run the Complete Agent (After Getting Credentials)

**Terminal 1 - Start rosbridge:**
```bash
source /opt/ros/jazzy/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

**Terminal 2 - Start camera node:**
```bash
source /opt/ros/jazzy/setup.bash
cd ~/modulr-agent
python3 scripts/camera_node.py
```

**Terminal 3 - Run the agent (with your values from Alex):**
```bash
source /opt/ros/jazzy/setup.bash
cd ~/modulr-agent

# First-time setup (replace with values from Alex)
cargo run -- initial-setup --robot-id YOUR_ROBOT_ID --signaling-url wss://YOUR_SERVER_URL:8765

# Then start the agent
cargo run -- start

# Or with verbose logging to see what's happening
cargo run -- -vvv start
```

---

## Troubleshooting

### "ros-jazzy-desktop not found"

Make sure you've:
1. Added the ROS2 repository correctly
2. Run `sudo apt update`
3. Used the correct spelling: **jazzy** (not jade, not jazzy-jalisco)

### "Command 'ros2' not found"

```bash
source /opt/ros/jazzy/setup.bash
```

If this doesn't work, check if ROS2 is installed:
```bash
ls /opt/ros/
# Should show: jazzy
```

### Build fails with missing dependencies

Make sure you've:
1. Installed GStreamer (Step 5)
2. Sourced ROS2 before building: `source /opt/ros/jazzy/setup.bash`

### Camera not found

```bash
# Try different camera indices
python3 scripts/camera_node.py 0
python3 scripts/camera_node.py 1
```

### rosbridge connection fails

Check if rosbridge is running:
```bash
ps aux | grep rosbridge
netstat -tuln | grep 9090
```

---

## Quick Reference Commands (Ubuntu 24.04.3)

```bash
# Source ROS2 (should be in ~/.bashrc automatically)
source /opt/ros/jazzy/setup.bash

# Check ROS2 version
echo $ROS_DISTRO  # Should output: jazzy

# Start rosbridge
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# Start camera node
python3 scripts/camera_node.py

# Build agent
cargo build

# Run agent (after setup)
cargo run -- start
```

---

## Summary Checklist

- [ ] Ubuntu 24.04.3 verified
- [ ] Rust installed (`rustc --version` works)
- [ ] ROS2 Jazzy installed (`ros2 --help` works, `$ROS_DISTRO` = jazzy)
- [ ] rosbridge-suite installed (`ros-jazzy-rosbridge-suite`)
- [ ] cv-bridge and opencv installed
- [ ] GStreamer installed
- [ ] Modulr Agent cloned and built (`cargo build` succeeds)
- [ ] Camera accessible (`v4l2-ctl --list-devices` shows camera)
- [ ] Camera node runs (`python3 scripts/camera_node.py` works)
- [ ] rosbridge runs (`ros2 launch rosbridge_server ...` works)
- [ ] Topics visible (`ros2 topic list` shows `/camera/image_raw`)

Once all checkboxes are done, get the server URL and robot ID from Alex to run the full agent!

