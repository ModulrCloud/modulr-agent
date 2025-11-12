# Modulr Robot Agent

This package contains the agent to be installed on any robot connecting to the Modulr infrastructure.

## Building the Package

This package has only been tested on Ubuntu systems. To build the package, you will need to install Rust: https://rustup.rs/

You will also need to install GStreamer:

```bash
sudo apt install -y  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
      gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
      gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
      gstreamer1.0-libav libgstrtspserver-1.0-dev libges-1.0-dev
```

Finally, ROS is required. ROS 1 and ROS 2 are both supported.

## ROS 1

Follow the installation instructions here: https://wiki.ros.org/noetic/Installation/Ubuntu

Then, build the package by running:

```bash
cargo build --features ros1 --no-default-features
```

You can run the package with logging enabled using:

```bash
RUST_LOG=debug cargo run --features ros1 --no-default-features
```

*Alternatively, edit the Cargo.toml file such that ros1 is the default feature, then run without any of the --features or --no-default-features flags.*

## ROS 2

Follow the installation instructions here: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

Note that any distro of ROS 2 should be compatible, but only Jazzy has been tested so far.

Some extra installation is required for Rust-related build tooling:

```bash
sudo apt install -y git libclang-dev python3-pip python3-vcstool
pip install git+https://github.com/colcon/colcon-cargo.git --break-system-packages
pip install git+https://github.com/colcon/colcon-ros-cargo.git --break-system-packages
```

Create a ROS 2 workspace for the package:

```bash
mkdir -p ~/modulr_ws/src
cd ~/modulr_ws
git clone <this repo> src/modulr_agent
vcs import src < src/modulr_agent/ros2rust.repos
# Change the following line depending on your installed version of ROS 2
vcs import src < src/ros2_rust/ros2_rust_jazzy.repos
```

Build the package (first time command):

```bash
cd ~/modulr_ws
colcon build
source install/setup.bash
```

Build the package (subsequent calls):

```bash
cd ~/modulr_ws
colcon build --packages-select modulr_agent
```

Run the package with debug logging:

```bash
RUST_LOG=debug ./install/modulr_agent/bin/modulr_agent
```

*Note that cargo commands should be enabled, including cargo build and cargo run, but these did not work on the first system. It is a TODO to fix this build issue.*
