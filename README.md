# Modulr Robot Agent

This package contains the agent to be installed on any robot connecting to the Modulr infrastructure.

## Cloning the Package

Use the following command to clone the package:

```bash
git clone --recurse-submodules https://github.com/ModulrCloud/modulr-agent

# If already cloned and you need to checkout the submodules:
git submodule update --init --recursive
```

## Building the Package

This package has only been tested on Ubuntu systems. 

To build the package, you will need to install Rust: https://rustup.rs/

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```


You will also need to install GStreamer:

```bash
sudo apt install -y  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
      gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
      gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
      gstreamer1.0-libav libgstrtspserver-1.0-dev libges-1.0-dev
```

Finally, ROS is required. ROS 1 and ROS 2 are both supported. For ROS1, install Noetic as per [these installation instructions](https://wiki.ros.org/noetic/Installation/Ubuntu). For ROS2, we recommend using Kilted as the latest release ([installation instructions](https://docs.ros.org/en/kilted/Installation/Ubuntu-Install-Debs.html)), but earlier distros should also work.

For ROS1: set the flag in `src/commands/start.rs` line 20.

```rust
const ROS1: bool = false;
```

For ROS2: a websocket bridge server is additionally required:

```bash
sudo apt install ros-$ROS_DISTRO-rosbridge-suite
```


Build the package as follows:

```bash
# Replace $ROS_DISTRO with the installed version of ROS
source /opt/ros/$ROS_DISTRO/setup.bash
# For debug mode:
cargo build
# For release mode:
cargo build --release
```

## Running the Agent

### Dependencies

The agent for ROS 2 relies on rosbridge to relay ROS traffic. Run rosbridge as follows:

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

### Commands

For all `cargo run` commands, you can add `--release` for release mode (debug mode is default), and use `--` before passing arguments to ensure that all arguments are passed into the agent, rather than the cargo process. For example, use `cargo run --release -- --help` for the help command.

The following commands are available:

```bash
Usage: modulr_agent [OPTIONS] <COMMAND>

Commands:
  initial-setup  First-time setup. Discovers ROS installation and topics, and initialises token exchange mechanism with Modulr services
  start          Starts the main agent running
  config-path    Prints out the default config path for this application
  help           Print this message or the help of the given subcommand(s)

Options:
  -v...          Set logging verbosity level. 1 = WARN, 2 = INFO, 3 = DEBUG, 4 = TRACE e.g. `agent run -vvvv config-path` sets the verbosity level to TRACE
  -h, --help     Print help
  -V, --version  Print version
```

For first-time setup, use the initial-setup command:

```bash
# Replace ROBOT_ID and SIGNALING_URL with your chosen values
cargo run -- initial-setup --robot-id ROBOT_ID --signaling-url SIGNALING_URL
```

This will perform first-time setup and save required values into a config file. You can override this file path or leave it as the default (`cargo run -- config-path` will get the default file path).

You can then run the agent using the following:

```bash
cargo run -- start
# To enable logging, use the verbosity flags, e.g. for debug logging:
cargo run -- -vvv start
```

## Running Locally (for development)
To control your robot on a LAN, run the Modulr webapp locally following the instructions at https://github.com/ModulrCloud/robot-teleop-webapp.

Additionally, the `modulr-agent/scripts` folder contains a local signaling server script. 
Note that this doesn't expose the server to the internet (without further user configuration), so only robots on the same LAN will be able to access it.
  
Python with virtualenv is recommended to run the server. To install dependencies and create certificates on Ubuntu:

```bash
sudo apt install -y python3-virtualenv
cd scripts
virtualenv venv
source venv/bin/activate
pip install -r requirements.txt
./make_creds.sh
```

After creating the certificates, your OS will need to trust that certificate before you can connect using it. On Windows, this means following the steps [in this guide](https://learn.microsoft.com/en-us/skype-sdk/sdn/articles/installing-the-trusted-root-certificate).

Linux can accomplish the same using these commands:

```bash
sudo cp certs/dev-root-ca.pem /usr/local/share/ca-certificates/dev-root-ca.crt
sudo update-ca-certificates
```

Once the CA has been added, run the signaling server:

```bash
python signaling_server.py
```

From another terminal, confirm that a robot with ID robot1 connects and disconnects:

```bash
python test_signaling_server.py
```

Then you can configure both your robot and the webapp to communicate with the local signaling server.

In the webapp, edit `src/config/signaling.ts` and point it to your server address, for example, ws://192.168.0.200:8765.  No need to rebuild, just relaunch the webapp. 

```bash
  if (window.location.hostname === 'localhost') {
    return 'ws://192.168.0.200:8765';
  }
```

In the robot, generate a local configuration file by passing the robot id and server address, for example:

```bash
# Set the new signaling server URL to a test config
cargo run -- initial-setup --config-override ./local_config.json --robot-id robot1 --signaling-url ws://192.168.0.200:8765
```

Then run the agent with `--allow-skip-cert-check`, for example:
```bash

# Run the agent, skipping the security checks
cargo run -- -vvv start --config-override ./local_config.json --allow-skip-cert-check
```

*Note 1: In a local deployment, ensure all websocket addresses start with ws://*

*Note 2: At present, ROS topics are hard-coded and the webapp supports movement control and video feedback. Ensure your robot is producing images on `/camera/image_raw` and the wheels are controllable on `/cmd/vel`, or alternatively edit `modulr-agent/ros_bridge` as required*



## Running in simulation

If not running on a real robot, you can test the system using a Turtlebot simulation. Install the turtlebot simulator using:

```bash
sudo apt install ros-$ROS_DISTRO-turtlebot3-simulations
```

The simulator can then be run using:

```bash
export TURTLEBOT3_MODEL=waffle_pi
# ROS 1
roslaunch turtlebot3_gazebo turtlebot3_world.launch
# ROS 2
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

The robot should then be controllable using the agent.

*Note The simulation may use Twist or TwistStamped messages for velocity commands, so if movement is not working, double-check the message type.*
