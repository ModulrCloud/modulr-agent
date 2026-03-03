use std::path::PathBuf;
use std::sync::Arc;

use anyhow::Result;
use bytes::Bytes;
use clap::Parser;
use log::{debug, error, info, warn};
use tokio::sync::Mutex;

use crate::commands::config::{
    ImageFormat, VideoSource, cancel_navigation, create_location, delete_location, read_config,
    start_navigation, update_location,
};
use crate::ros_bridge::Ros1Bridge;
use crate::ros_bridge::Ros2Bridge;
use crate::ros_bridge::RosBridge;
use crate::video_pipeline::VideoPipeline;
use crate::video_pipeline::VideoPipelineError;
use crate::webrtc_link::WebRtcLink;
use crate::webrtc_link::WebRtcLinkError;
use modulr_webrtc_message::{
    AgentMessage, Location, LocationResponsePayload, MessageEnvelope, NavigationStatus, ToMessage,
};

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

async fn send_webrtc_message(
    webrtc_link: &Arc<Mutex<WebRtcLink>>,
    message: &MessageEnvelope,
    error_context: &str,
) {
    if let Err(e) = webrtc_link
        .lock()
        .await
        .send_data_channel_message(message)
        .await
    {
        error!("Failed to {}: {}", error_context, e);
    }
}

pub async fn start(args: StartArgs) -> Result<()> {
    let config_path = args.config_override.clone();
    let mut config = read_config(args.config_override)?;

    let robot_id = config.core.robot_id.clone();
    let signaling_url = config.core.signaling_url.clone();
    let image_format = config.robot.image_format;
    let video_source = config.robot.video_source.clone();
    let locations: Arc<Mutex<Vec<Location>>> =
        Arc::new(Mutex::new(std::mem::take(&mut config.locations)));
    let navigation_target: Arc<Mutex<Option<String>>> = Arc::new(Mutex::new(None));
    let config = Arc::new(config);

    let webrtc_link = Arc::new(Mutex::new(WebRtcLink::new(
        &robot_id,
        &signaling_url,
        args.allow_skip_cert_check,
    )));
    let pipeline = Arc::new(Mutex::new(VideoPipeline::new(image_format)));

    let bridge: Arc<Mutex<dyn RosBridge>> = if ROS1 {
        Arc::new(Mutex::new(Ros1Bridge::new()))
    } else {
        Arc::new(Mutex::new(Ros2Bridge::new()))
    };

    info!("Creating system components and callbacks");

    // Browser -> WebRTC -> ROS
    let bridge_clone = Arc::clone(&bridge);
    let locations_clone = Arc::clone(&locations);
    let navigation_target_clone = Arc::clone(&navigation_target);
    let webrtc_link_clone_for_msg = Arc::clone(&webrtc_link);
    let config_clone = Arc::clone(&config);
    webrtc_link
        .lock()
        .await
        .on_webrtc_message(Box::new(move |msg: &AgentMessage| {
            let bridge_clone = Arc::clone(&bridge_clone);
            let locations_clone = Arc::clone(&locations_clone);
            let navigation_target_clone = Arc::clone(&navigation_target_clone);
            let webrtc_link_clone = Arc::clone(&webrtc_link_clone_for_msg);
            let config_clone = Arc::clone(&config_clone);
            let config_path_clone = config_path.clone();
            let msg_clone = msg.clone();
            Box::pin(async move {
                match msg_clone {
                    AgentMessage::Movement(cmd) => {
                        if let Err(e) = bridge_clone.lock().await.post_movement_command(&cmd).await
                        {
                            error!("Failed to post movement command: {}", e);
                        }
                    }
                    AgentMessage::LocationList(ref list_payload) => {
                        let locs = locations_clone.lock().await.clone();
                        let response_envelope =
                            AgentMessage::LocationResponse(LocationResponsePayload {
                                correlation_id: list_payload.correlation_id.clone(),
                                operation: "list".to_string(),
                                locations: Some(locs),
                            })
                            .to_message();
                        send_webrtc_message(
                            &webrtc_link_clone,
                            &response_envelope,
                            "send location list response",
                        )
                        .await;
                    }
                    AgentMessage::LocationCreate(location) => {
                        let corr_id = location.correlation_id.clone();
                        let create_result = {
                            let locs = locations_clone.lock().await;
                            create_location(
                                &locs,
                                location,
                                &config_clone,
                                config_path_clone.clone(),
                            )
                        };
                        match create_result {
                            Ok(new_locs) => {
                                *locations_clone.lock().await = new_locs;
                                let response_envelope =
                                    AgentMessage::LocationResponse(LocationResponsePayload {
                                        correlation_id: corr_id.clone(),
                                        operation: "create".to_string(),
                                        locations: None,
                                    })
                                    .to_message();
                                send_webrtc_message(
                                    &webrtc_link_clone,
                                    &response_envelope,
                                    "send location create response",
                                )
                                .await;
                            }
                            Err(ref err) => {
                                let (code, details) = err.to_error_code_and_details("create");
                                let err_envelope = AgentMessage::error(
                                    code,
                                    &err.to_string(),
                                    corr_id.as_deref(),
                                    Some(details),
                                )
                                .to_message();
                                send_webrtc_message(
                                    &webrtc_link_clone,
                                    &err_envelope,
                                    "send location create error",
                                )
                                .await;
                            }
                        }
                    }
                    AgentMessage::LocationUpdate(location) => {
                        let corr_id = location.correlation_id.clone();
                        let update_result = {
                            let locs = locations_clone.lock().await;
                            update_location(
                                &locs,
                                location,
                                &config_clone,
                                config_path_clone.clone(),
                            )
                        };
                        match update_result {
                            Ok(new_locs) => {
                                *locations_clone.lock().await = new_locs;
                                let response_envelope =
                                    AgentMessage::LocationResponse(LocationResponsePayload {
                                        correlation_id: corr_id.clone(),
                                        operation: "update".to_string(),
                                        locations: None,
                                    })
                                    .to_message();
                                send_webrtc_message(
                                    &webrtc_link_clone,
                                    &response_envelope,
                                    "send location update response",
                                )
                                .await;
                            }
                            Err(ref err) => {
                                let (code, details) = err.to_error_code_and_details("update");
                                let err_envelope = AgentMessage::error(
                                    code,
                                    &err.to_string(),
                                    corr_id.as_deref(),
                                    Some(details),
                                )
                                .to_message();
                                send_webrtc_message(
                                    &webrtc_link_clone,
                                    &err_envelope,
                                    "send location update error",
                                )
                                .await;
                            }
                        }
                    }
                    AgentMessage::LocationDelete(payload) => {
                        let corr_id = payload.correlation_id.clone();
                        let delete_result = {
                            let locs = locations_clone.lock().await;
                            delete_location(
                                &locs,
                                &payload.name,
                                &config_clone,
                                config_path_clone.clone(),
                            )
                        };
                        match delete_result {
                            Ok(new_locs) => {
                                *locations_clone.lock().await = new_locs;
                                let response_envelope =
                                    AgentMessage::LocationResponse(LocationResponsePayload {
                                        correlation_id: corr_id.clone(),
                                        operation: "delete".to_string(),
                                        locations: None,
                                    })
                                    .to_message();
                                send_webrtc_message(
                                    &webrtc_link_clone,
                                    &response_envelope,
                                    "send location delete response",
                                )
                                .await;
                            }
                            Err(ref err) => {
                                let (code, details) = err.to_error_code_and_details("delete");
                                let err_envelope = AgentMessage::error(
                                    code,
                                    &err.to_string(),
                                    corr_id.as_deref(),
                                    Some(details),
                                )
                                .to_message();
                                send_webrtc_message(
                                    &webrtc_link_clone,
                                    &err_envelope,
                                    "send location delete error",
                                )
                                .await;
                            }
                        }
                    }
                    AgentMessage::NavigationStart(payload) => {
                        let corr_id = payload.correlation_id.clone();
                        let nav_start_result = {
                            let nav = navigation_target_clone.lock().await;
                            let locs = locations_clone.lock().await;
                            start_navigation(&nav, &locs, &payload.name)
                        };
                        match nav_start_result {
                            Ok((new_nav, location)) => {
                                *navigation_target_clone.lock().await = new_nav;
                                if let Err(e) = bridge_clone
                                    .lock()
                                    .await
                                    .post_navigation_goal(&location)
                                    .await
                                {
                                    error!("Failed to post navigation goal: {}", e);
                                }
                                let response_envelope =
                                    AgentMessage::navigation_response(
                                        NavigationStatus::Started,
                                        &payload.name,
                                        None,
                                        corr_id.as_deref(),
                                    )
                                    .to_message();
                                send_webrtc_message(
                                    &webrtc_link_clone,
                                    &response_envelope,
                                    "send navigation start response",
                                )
                                .await;
                            }
                            Err(ref err) => {
                                let (code, details) =
                                    err.to_error_code_and_details("start");
                                let err_envelope = AgentMessage::error(
                                    code,
                                    &err.to_string(),
                                    corr_id.as_deref(),
                                    Some(details),
                                )
                                .to_message();
                                send_webrtc_message(
                                    &webrtc_link_clone,
                                    &err_envelope,
                                    "send navigation start error",
                                )
                                .await;
                            }
                        }
                    }
                    AgentMessage::NavigationCancel(payload) => {
                        let corr_id = payload.correlation_id.clone();
                        let nav_cancel_result = {
                            let nav = navigation_target_clone.lock().await;
                            cancel_navigation(&nav)
                        };
                        match nav_cancel_result {
                            Ok((new_nav, cancelled_name)) => {
                                *navigation_target_clone.lock().await = new_nav;
                                if let Err(e) =
                                    bridge_clone.lock().await.cancel_navigation().await
                                {
                                    error!("Failed to cancel navigation: {}", e);
                                }
                                let response_envelope =
                                    AgentMessage::navigation_response(
                                        NavigationStatus::Cancelled,
                                        &cancelled_name,
                                        None,
                                        corr_id.as_deref(),
                                    )
                                    .to_message();
                                send_webrtc_message(
                                    &webrtc_link_clone,
                                    &response_envelope,
                                    "send navigation cancel response",
                                )
                                .await;
                            }
                            Err(ref err) => {
                                let (code, details) =
                                    err.to_error_code_and_details("cancel");
                                let err_envelope = AgentMessage::error(
                                    code,
                                    &err.to_string(),
                                    corr_id.as_deref(),
                                    Some(details),
                                )
                                .to_message();
                                send_webrtc_message(
                                    &webrtc_link_clone,
                                    &err_envelope,
                                    "send navigation cancel error",
                                )
                                .await;
                            }
                        }
                    }
                    AgentMessage::LocationResponse(_)
                    | AgentMessage::NavigationResponse(_)
                    | AgentMessage::Ping(_)
                    | AgentMessage::Pong(_)
                    | AgentMessage::Capabilities(_)
                    | AgentMessage::Error(_) => (),
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
    let ros_image_format = match video_source {
        VideoSource::Ros => Some(config.robot.image_format),
        VideoSource::Zenoh => None, // Zenoh handles its own subscription
    };

    match video_source {
        VideoSource::Zenoh => {
            configure_zenoh_camera_callback(
                Arc::clone(&pipeline),
                Arc::clone(&webrtc_link),
                config.robot.image_format,
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
