//! # Modulr ROS Messages
//!
//! Auto-generated Rust types for ROS message definitions.
//!
//! This crate provides Rust structs for ROS1 and ROS2 message types, generated at build time
//! from ROS message definition files. The generated types implement `serde::Serialize` and
//! `serde::Deserialize` for use with various serialization formats (CDR, JSON, MessagePack, etc.).
//!
//! ## Modules
//!
//! - [`ros1_messages`] - ROS1 message types (std_msgs, sensor_msgs, geometry_msgs, etc.)
//! - [`ros2_messages`] - ROS2 message types (std_msgs, sensor_msgs, builtin_interfaces, etc.)
//!
//! ## Usage
//!
//! ```rust
//! use modulr_ros_messages::ros2_messages::{sensor_msgs, std_msgs, builtin_interfaces};
//!
//! let image = sensor_msgs::Image {
//!     header: std_msgs::Header {
//!         stamp: builtin_interfaces::Time { sec: 0, nanosec: 0 },
//!         frame_id: "camera".to_string(),
//!     },
//!     height: 480,
//!     width: 640,
//!     encoding: "rgb8".to_string(),
//!     is_bigendian: 0,
//!     step: 640 * 3,
//!     data: vec![0u8; 640 * 480 * 3],
//! };
//! ```

/// ROS1 message types generated from standard ROS1 message definitions.
///
/// Includes common message packages like:
/// - `std_msgs` - Standard messages (Header, String, Int32, etc.)
/// - `sensor_msgs` - Sensor messages (Image, CameraInfo, PointCloud2, etc.)
/// - `geometry_msgs` - Geometry messages (Point, Pose, Transform, etc.)
pub mod ros1_messages {
    include!(concat!(env!("OUT_DIR"), "/ros1_messages.rs"));
}

/// ROS2 message types generated from standard ROS2 message definitions.
///
/// Includes common message packages like:
/// - `std_msgs` - Standard messages (Header, String, Int32, etc.)
/// - `sensor_msgs` - Sensor messages (Image, CameraInfo, PointCloud2, etc.)
/// - `builtin_interfaces` - Built-in types (Time, Duration)
/// - `geometry_msgs` - Geometry messages (Point, Pose, Transform, etc.)
pub mod ros2_messages {
    include!(concat!(env!("OUT_DIR"), "/ros2_messages.rs"));
}
