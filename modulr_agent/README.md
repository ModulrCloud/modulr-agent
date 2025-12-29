# Modulr Agent (Binary Crate)

The main binary application for connecting robots to the Modulr infrastructure. This crate handles WebRTC communication, video pipeline management, ROS integration, and robot control.

For installation, build instructions, and general usage, see the [workspace README](../README.md).

## Overview

This crate provides a command-line application that:

1. Connects to a WebRTC signaling server
2. Establishes peer-to-peer video and data channels with a browser client
3. Receives video frames from ROS topics or Zenoh
4. Encodes video using GStreamer and streams via WebRTC
5. Receives movement commands from the browser and forwards to ROS

## Key Components

### Video Sources

The agent supports two video source modes configured via the config file:

#### ROS Video Source
- **ROS 1**: Direct subscription to camera topics
- **ROS 2**: Subscription via rosbridge websocket (requires base64 encoding/decoding)
- **Topic**: `/camera/image_raw` (sensor_msgs/Image)
- **Limitation**: Base64 encoding overhead limits resolution/framerate

#### Zenoh Video Source
- High-performance pub/sub with zero-copy shared memory transport
- Requires [modulr_zenoh_interface](../modulr_zenoh_interface) crate (enabled by default)
- **Advantage**: Eliminates base64 overhead, supports higher resolution/framerate

### Video Pipeline

- Uses GStreamer for H.264 encoding
- Configurable bitrate and quality settings
- Integrates with WebRTC for browser streaming
- Zero-copy frame queuing when using Zenoh

### ROS Bridge

- Bidirectional communication with ROS
- **Outgoing**: Movement commands (geometry_msgs/Twist or TwistStamped) to `/cmd_vel`
- **Incoming**: Camera frames from `/camera/image_raw`
- Supports both ROS 1 and ROS 2 (configure via `ROS1` constant in [src/commands/start.rs:21](src/commands/start.rs#L21))

### WebRTC Link

- WebSocket signaling connection
- Data channels for movement commands
- Media tracks for video streaming
- Configurable certificate verification for local development

## Configuration

### Build Features

- `zenoh` (default) - Enables Zenoh video source support
- Build without Zenoh: `cargo build --no-default-features`

### Runtime Configuration

See the [workspace README](../README.md#configuration) for configuration details.

### ROS Version Selection

Edit [src/commands/start.rs:21](src/commands/start.rs#L21):

```rust
const ROS1: bool = true;  // For ROS 1
// OR
const ROS1: bool = false; // For ROS 2
```

## Commands

### initial-setup

First-time setup to generate configuration:

```bash
modulr_agent initial-setup --robot-id <ROBOT_ID> --signaling-url <URL> --video-source ros

# With custom config path:
modulr_agent initial-setup --config-override ./my_config.json --robot-id robot1 --signaling-url wss://example.com:8765 --video-source ros
```

### start

Start the main agent:

```bash
modulr_agent start

# With custom config:
modulr_agent start --config-override ./my_config.json

# Skip certificate verification (local development only):
modulr_agent start --allow-skip-cert-check

# With debug logging:
modulr_agent -vvv start
```

### config-path

Print the default configuration file path:

```bash
modulr_agent config-path
```

## Module Structure

```
src/
├── commands/
│   ├── config.rs          - Configuration management
│   ├── initial_setup.rs   - First-time setup command
│   └── start.rs           - Main agent startup logic
├── ros_bridge/
│   ├── mod.rs             - ROS bridge trait and common types
│   ├── ros1.rs            - ROS 1 implementation
│   └── ros2.rs            - ROS 2 implementation (via rosbridge)
├── video_pipeline.rs      - GStreamer video encoding pipeline
├── webrtc_link.rs         - WebRTC peer connection management
├── webrtc_message.rs      - WebRTC data channel message types
└── main.rs                - CLI entry point
```

## Integration with Zenoh

When the `zenoh` feature is enabled (default), the agent can use Zenoh for video transport:

1. This crate uses [modulr_zenoh_interface](../modulr_zenoh_interface) to subscribe to Zenoh frames
2. Frames are passed to the video pipeline with zero-copy semantics
3. Zenoh automatically uses shared memory for same-host communication

Other applications on the robot are expected to pass frames into Zenoh to complete the video pipeline.

**Frame format**: `[4 bytes: metadata_len][metadata (MessagePack)][raw frame data]`

See [configure_zenoh_camera_callback](src/commands/start.rs#L43) for integration details.

## Dependencies

### System Dependencies
- GStreamer 1.0 with plugins (base, good, bad, ugly, libav)
- ROS 1 (Noetic) or ROS 2 (Kilted or later)
- For ROS 2: rosbridge-suite package

See the [workspace README](../README.md#prerequisites) for installation instructions.

## Development

### Running with Logging

Use the `-v` flags to control log verbosity:

```bash
modulr_agent -v start      # WARN
modulr_agent -vv start     # INFO
modulr_agent -vvv start    # DEBUG
modulr_agent -vvvv start   # TRACE
```

### Local Testing

See the [workspace README](../README.md#running-locally-for-development) for local development setup instructions.

### Testing with Simulation

See the [workspace README](../README.md#running-in-simulation) for simulation setup instructions.
