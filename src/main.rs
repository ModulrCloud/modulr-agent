mod ros_bridge;
mod video_pipeline;
mod webrtc_link;
mod webrtc_message;

use std::sync::Arc;

use anyhow::Result;
use bytes::Bytes;
use log::{debug, info, warn};
use tokio::sync::Mutex;

use crate::ros_bridge::Ros1Bridge;
use crate::ros_bridge::Ros2Bridge;
use crate::ros_bridge::RosBridge;
use crate::video_pipeline::VideoPipeline;
use crate::video_pipeline::VideoPipelineError;
use crate::webrtc_link::WebRtcLink;
use crate::webrtc_link::WebRtcLinkError;
use crate::webrtc_message::WebRtcMessage;

const ROS1: bool = false;

#[tokio::main]
async fn main() -> Result<()> {
    env_logger::init();

    let robot_id = "robot1";
    let signaling_url = "ws://192.168.132.19:8765";
    let webrtc_link = Arc::new(Mutex::new(WebRtcLink::new(robot_id, signaling_url)));
    let pipeline = Arc::new(Mutex::new(VideoPipeline::new()));

    let bridge: Arc<Mutex<dyn RosBridge>> = if ROS1 {
        Arc::new(Mutex::new(Ros1Bridge::new()))
    } else {
        Arc::new(Mutex::new(Ros2Bridge::new()))
    };

    info!("Creating system components and callbacks");

    // Browser -> WebRTC -> ROS
    let bridge_clone = Arc::clone(&bridge);
    webrtc_link
        .lock()
        .await
        .on_webrtc_message(Box::new(move |msg: &WebRtcMessage| {
            let bridge_clone = Arc::clone(&bridge_clone);
            let msg_clone = msg.clone();
            Box::pin(async move {
                match msg_clone {
                    WebRtcMessage::MovementCommand(cmd) => {
                        bridge_clone
                            .lock()
                            .await
                            .post_movement_command(&cmd)
                            .await
                            .expect("Failed to post movement command!");
                    }
                }
            })
        }))
        .await;

    // TODO: only queue frames in pipeline when WebRTC session is established

    // Pipeline -> WebRTC -> Browser
    let webrtc_link_clone = Arc::clone(&webrtc_link);
    pipeline
        .lock()
        .await
        .on_frame_ready(Box::new(move |frame: &Bytes| {
            let webrtc_link_clone = Arc::clone(&webrtc_link_clone);
            let frame_clone = Bytes::copy_from_slice(frame);
            Box::pin(async move {
                let err = webrtc_link_clone
                    .lock()
                    .await
                    .write_frame(frame_clone)
                    .await;
                match err {
                    Err(WebRtcLinkError::IncorrectStateForOperation) => {
                        warn!("Still waiting for WebRTC pipeline to connect.")
                    }
                    Err(_) => {
                        panic!("Failed to write frame!")
                    }
                    _ => (),
                }
            })
        }))
        .await;

    // Robot frame -> Pipeline
    let pipeline_clone = Arc::clone(&pipeline);
    bridge
        .lock()
        .await
        .on_image_frame_received(Box::new(move |data: &Bytes| {
            let data_clone = Bytes::copy_from_slice(data);
            let pipeline_clone = Arc::clone(&pipeline_clone);
            Box::pin(async move {
                pipeline_clone
                    .lock()
                    .await
                    .queue_frame(&data_clone)
                    .await
                    .expect("Failed to write camera frame to pipeline!");
            })
        }))
        .await;

    info!("Starting all tasks running");

    tokio::spawn(async move {
        let mut guard = webrtc_link.lock().await;
        guard.try_connect().await?;
        guard.try_register().await?;
        Ok::<(), WebRtcLinkError>(())
    });
    let pipeline_clone = Arc::clone(&pipeline);
    tokio::spawn(async move {
        pipeline_clone.lock().await.launch().await?;
        debug!("Finished launching video pipeline");
        Ok::<(), VideoPipelineError>(())
    });
    bridge.lock().await.launch().await?;

    let _ = tokio::signal::ctrl_c().await;
    info!("Exit requested, cleaning up...");
    pipeline.lock().await.stop_pipeline().await?;
    Ok(())
}
