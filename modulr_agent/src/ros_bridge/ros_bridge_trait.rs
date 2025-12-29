use crate::{
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
    async fn launch(&self, enable_video: bool) -> Result<(), RosBridgeError>;
}
