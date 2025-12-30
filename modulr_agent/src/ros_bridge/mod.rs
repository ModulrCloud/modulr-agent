mod ros1_bridge;
mod ros2_bridge;
mod ros_bridge_error;
mod ros_bridge_trait;

use std::pin::Pin;

use bytes::Bytes;

pub type OnCameraImageHdlrFn =
    Box<dyn (FnMut(&Bytes) -> Pin<Box<dyn Future<Output = ()> + Send + 'static>>) + Send + Sync>;

pub use ros_bridge_error::RosBridgeError;

pub use ros_bridge_trait::RosBridge;
pub use ros1_bridge::Ros1Bridge;
pub use ros2_bridge::Ros2Bridge;
