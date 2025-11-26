# Quick Start - Ubuntu 24.04.3

## Key Points

1. **ROS2 Distribution:** Use **Jazzy** (not Jade, not Jazzy Jalisco - just "jazzy")
2. **Server URL/Robot ID:** Only needed when running the agent, NOT for building
3. **You can test everything** without credentials first

---

## Quick Commands

```bash
# 1. Install ROS2 Jazzy
sudo apt update
sudo apt install -y software-properties-common curl gnupg lsb-release
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2-latest.list > /dev/null
sudo apt update
sudo apt install -y ros-jazzy-desktop ros-dev-tools python3-rosdep
sudo rosdep init 2>/dev/null || true
rosdep update

# 2. Install dependencies
source /opt/ros/jazzy/setup.bash
sudo apt install -y ros-jazzy-rosbridge-suite ros-jazzy-cv-bridge python3-opencv v4l-utils
sudo apt install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav libgstrtspserver-1.0-dev libges-1.0-dev

# 3. Set up environment
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "export ROS_DISTRO=jazzy" >> ~/.bashrc
source ~/.bashrc

# 4. Build agent (NO credentials needed)
cd ~/modulr-agent
cargo build

# 5. Test camera
python3 scripts/camera_node.py

# 6. Test rosbridge (in another terminal)
source /opt/ros/jazzy/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

---

## Full Guide

See `scripts/UBUNTU_24_04_SETUP.md` for complete step-by-step instructions.

