mod ros_bridge_error;
use std::pin::Pin;

use bytes::Bytes;
pub use ros_bridge_error::RosBridgeError;

pub type OnCameraImageHdlrFn =
    Box<dyn (FnMut(&Bytes) -> Pin<Box<dyn Future<Output = ()> + Send + 'static>>) + Send + Sync>;

#[cfg(feature = "ros1")]
mod ros1_bridge;
#[cfg(feature = "ros1")]
pub use ros1_bridge::RosBridge;

#[cfg(feature = "ros2")]
mod ros2_bridge;
#[cfg(feature = "ros2")]
pub use ros2_bridge::RosBridge;

#[cfg(all(feature = "ros1", feature = "ros2"))]
compile_error!("select only one ros feature for RosBridge");
