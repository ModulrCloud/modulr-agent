# Modulr Zenoh Interface

A Rust library crate providing high-performance video frame transport using Zenoh pub/sub middleware. This crate enables efficient video streaming from ROS2 camera topics via Zenoh.

## Overview

This library provides a `ZenohInterface` that:

1. Subscribes to Zenoh topics containing video frames in ROS2 `sensor_msgs/Image` format (CDR serialized)
2. Deserializes the Image message and extracts the raw pixel data
3. Notifies registered listeners when frames arrive

## Usage

### Basic Example

```rust
use modulr_zenoh_interface::ZenohInterface;
use bytes::Bytes;

#[tokio::main]
async fn main() {
    let interface = ZenohInterface::new();

    // Add a frame listener callback
    interface.add_frame_listener(Box::new(move |data: &Bytes| {
        println!("Received frame: {} bytes", data.len());
        Box::pin(async {})
    })).await;

    // Start listening for frames
    interface.launch().await.expect("Failed to launch");

    // Keep running
    loop {
        tokio::time::sleep(std::time::Duration::from_secs(1)).await;
    }
}
```

## Examples

Two example binaries are provided:

### publish_frames

Publishes test frames (1080p, half black/half white) at 30fps:

```bash
cargo run --example publish_frames
```

This creates a Zenoh session and publishes CDR-serialized `sensor_msgs::Image` messages to the `camera/image_raw` topic.

### listen_frames

Receives frames using the `ZenohInterface` library and prints statistics:

```bash
cargo run --example listen_frames
```

## Configuration

The library uses Zenoh's default configuration with multicast scouting enabled, allowing automatic peer discovery on the local network.

**Default topic:** `camera/image_raw`

## Message Format

Frames are serialized using CDR (Common Data Representation), which is the standard serialization format for ROS2/DDS. The message type is `sensor_msgs::Image`:

```rust
pub struct Image {
    pub header: Header,
    pub height: u32,
    pub width: u32,
    pub encoding: String,  // e.g., "rgb8", "bgr8"
    pub is_bigendian: u8,
    pub step: u32,         // row length in bytes
    pub data: Vec<u8>,     // raw pixel data
}
```

## API Reference

### ZenohInterface

```rust
impl ZenohInterface {
    /// Create a new Zenoh interface
    pub fn new() -> Self;

    /// Launch the interface and start receiving frames
    /// This spawns a background task that listens for messages
    pub async fn launch(&self) -> Result<(), ZenohInterfaceError>;

    /// Add a listener callback for received frames
    /// Callback receives raw pixel data (the `data` field from the Image message)
    pub async fn add_frame_listener(&self, listener: OnFrameReceivedFn);
}
```

### Type Aliases

```rust
/// Callback type for frame reception
pub type OnFrameReceivedFn =
    Box<dyn Fn(&Bytes) -> Pin<Box<dyn Future<Output = ()> + Send + 'static>> + Send + Sync>;
```

### Error Types

```rust
pub enum ZenohInterfaceError {
    SessionOpenFailure(zenoh::Error),
    SubscriptionFailed(zenoh::Error),
}
```

## Development

### Logging

Enable detailed logging:

```bash
RUST_LOG=debug cargo run --example listen_frames
```

Or in your application:

```rust
env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("debug")).init();
```

## License

This crate is part of the Modulr Robot Agent project. See the workspace root for license information.
