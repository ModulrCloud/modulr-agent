use std::path::PathBuf;
use std::sync::Arc;

use anyhow::Result;
use bytes::Bytes;
use clap::Parser;
use log::{debug, error, info, warn};
use tokio::sync::Mutex;

use crate::commands::config::{ImageFormat, VideoSource, read_config};
use crate::ros_bridge::Ros1Bridge;
use crate::ros_bridge::Ros2Bridge;
use crate::ros_bridge::RosBridge;
use crate::video_pipeline::VideoPipeline;
use crate::video_pipeline::VideoPipelineError;
use crate::webrtc_link::WebRtcLink;
use crate::webrtc_link::WebRtcLinkError;
use crate::webrtc_message::WebRtcMessage;

const ROS1: bool = false;

#[derive(Parser, Debug)]
pub struct StartArgs {
    /// Override default config path
    #[arg(short, long, value_name = "FILE")]
    pub config_override: Option<PathBuf>,
    /// Allow not checking certificates on WebRTC link
    #[arg(long, default_value_t = false)]
    allow_skip_cert_check: bool,
}

#[cfg(not(feature = "zenoh"))]
async fn configure_zenoh_camera_callback(
    _pipeline: Arc<Mutex<VideoPipeline>>,
    _webrtc_link: Arc<Mutex<WebRtcLink>>,
    _image_format: ImageFormat,
) -> Result<()> {
    Err(anyhow::anyhow!(
        "Zenoh is not enabled! Recompile application with zenoh feature enabled to use this video source."
    ))
}

#[cfg(feature = "zenoh")]
async fn configure_zenoh_camera_callback(
    pipeline: Arc<Mutex<VideoPipeline>>,
    webrtc_link: Arc<Mutex<WebRtcLink>>,
    image_format: ImageFormat,
) -> Result<()> {
    use modulr_zenoh_interface::ZenohInterface;

    info!("Zenoh video source in use. Setting up callback.");

    // Create Zenoh video source with specified image format
    let zenoh_source = Arc::new(ZenohInterface::new(image_format));

    // Add frame listener
    zenoh_source
        .add_frame_listener(Box::new(move |data: &Bytes| {
            let pipeline_clone = Arc::clone(&pipeline);
            let webrtc_link_clone = Arc::clone(&webrtc_link);
            let data_clone = data.clone();
            Box::pin(async move {
                // Only process frames when WebRTC is connected
                if webrtc_link_clone.lock().await.is_connected().await {
                    pipeline_clone
                        .lock()
                        .await
                        .queue_frame(data_clone)
                        .await
                        .expect("Failed to write camera frame to pipeline!");
                }
            })
        }))
        .await;

    // Start the Zenoh video source
    zenoh_source.launch().await?;

    info!("Zenoh video source started successfully");

    Ok(())
}

async fn configure_ros_camera_callback(
    pipeline: Arc<Mutex<VideoPipeline>>,
    webrtc_link: Arc<Mutex<WebRtcLink>>,
    bridge: Arc<Mutex<dyn RosBridge>>,
) -> Result<()> {
    info!("ROS video source in use. Setting up callback.");
    bridge
        .lock()
        .await
        .on_image_frame_received(Box::new(move |data: &Bytes| {
            let pipeline_clone = Arc::clone(&pipeline);
            let webrtc_link_clone = Arc::clone(&webrtc_link);
            let data_clone = data.clone();
            Box::pin(async move {
                // Only process frames when WebRTC is connected
                if webrtc_link_clone.lock().await.is_connected().await {
                    pipeline_clone
                        .lock()
                        .await
                        .queue_frame(data_clone)
                        .await
                        .expect("Failed to write camera frame to pipeline!");
                }
            })
        }))
        .await;
    Ok(())
}

pub async fn start(args: StartArgs) -> Result<()> {
    let config = read_config(args.config_override)?;

    let robot_id = config.robot_id;
    let signaling_url = config.signaling_url;
    let webrtc_link = Arc::new(Mutex::new(WebRtcLink::new(
        &robot_id,
        &signaling_url,
        args.allow_skip_cert_check,
    )));
    let pipeline = Arc::new(Mutex::new(VideoPipeline::new(config.image_format)));

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

    // Pipeline -> WebRTC -> Browser
    let webrtc_link_clone = Arc::clone(&webrtc_link);
    pipeline
        .lock()
        .await
        .on_frame_ready(Box::new(move |frame: &Bytes| {
            let webrtc_link_clone = Arc::clone(&webrtc_link_clone);
            let frame_clone = frame.clone();
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

    // Robot camera frame -> Pipeline
    // Determine which image format to use for the video source
    let ros_image_format = match config.video_source {
        VideoSource::Ros => Some(config.image_format),
        VideoSource::Zenoh => None, // Zenoh handles its own subscription
    };

    match config.video_source {
        VideoSource::Zenoh => {
            configure_zenoh_camera_callback(
                Arc::clone(&pipeline),
                Arc::clone(&webrtc_link),
                config.image_format,
            )
            .await?
        }
        VideoSource::Ros => {
            configure_ros_camera_callback(
                Arc::clone(&pipeline),
                Arc::clone(&webrtc_link),
                Arc::clone(&bridge),
            )
            .await?
        }
    };

    info!("Starting all tasks running");

    tokio::spawn(async move {
        let mut guard = webrtc_link.lock().await;
        match guard.try_connect().await {
            Ok(x) => Ok(x),
            Err(e) => {
                error!("Error during WebRTC connection: {}", e);
                Err(e)
            }
        }?;
        guard.try_register().await?;
        Ok::<(), WebRtcLinkError>(())
    });
    let pipeline_clone = Arc::clone(&pipeline);
    tokio::spawn(async move {
        pipeline_clone.lock().await.launch().await?;
        debug!("Finished launching video pipeline");
        Ok::<(), VideoPipelineError>(())
    });
    if let Err(e) = bridge.lock().await.launch(ros_image_format).await {
        log::error!("{}", e);
        return Err(e.into());
    };

    let _ = tokio::signal::ctrl_c().await;
    info!("Exit requested, cleaning up...");
    pipeline.lock().await.stop_pipeline().await?;
    Ok(())
}
