use crate::{
    commands::config::ImageFormat,
    ros_bridge::{OnCameraImageHdlrFn, RosBridgeError},
    webrtc_message::MovementCommand,
};

use async_trait::async_trait;

#[async_trait]
pub trait RosBridge: Send {
    async fn on_image_frame_received(&mut self, listener: OnCameraImageHdlrFn);
    async fn post_movement_command(
        &mut self,
        command: &MovementCommand,
    ) -> Result<(), RosBridgeError>;
    /// Launch the ROS bridge.
    /// If `image_format` is `Some`, video subscription will be enabled using the specified format.
    /// If `None`, video subscription is disabled.
    async fn launch(&self, image_format: Option<ImageFormat>) -> Result<(), RosBridgeError>;
}
