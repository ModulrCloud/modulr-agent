use std::path::PathBuf;
use std::sync::Arc;
use std::time::{Duration, Instant};

use anyhow::Result;
use bytes::Bytes;
use clap::Parser;
use log::{debug, info, warn};
use tokio::sync::Mutex;
use tokio::time::sleep;

use crate::commands::config::read_config;
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
    /// Use test pattern instead of camera feed (bypasses ROS)
    #[arg(long, default_value_t = false)]
    test_pattern: bool,
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
    
    // Start test pattern generator if requested (bypasses ROS)
    if args.test_pattern {
        info!("Starting test pattern generator (bypassing ROS bridge)");
        let pipeline_test = Arc::clone(&pipeline);
        tokio::spawn(async move {
            generate_test_pattern(pipeline_test).await;
        });
    } else {
        bridge.lock().await.launch().await?;
    }

    let _ = tokio::signal::ctrl_c().await;
    info!("Exit requested, cleaning up...");
    pipeline.lock().await.stop_pipeline().await?;
    Ok(())
}

/// Generate test pattern frames at ~30fps and feed them to the video pipeline
/// This bypasses ROS entirely and tests the video pipeline + WebRTC connection
async fn generate_test_pattern(pipeline: Arc<Mutex<VideoPipeline>>) {
    const WIDTH: usize = 640;
    const HEIGHT: usize = 480;
    const FRAME_SIZE: usize = WIDTH * HEIGHT * 3; // RGB format
    
    let frame_duration = Duration::from_secs(1) / 30; // ~30 fps
    let mut frame_count = 0u64;
    let start_time = Instant::now();
    
    info!("Test pattern generator started - generating {}x{} RGB frames", WIDTH, HEIGHT);
    
    loop {
        let frame_start = Instant::now();
        
        // Generate a test pattern: moving colored gradient
        let mut frame = vec![0u8; FRAME_SIZE];
        let t = start_time.elapsed().as_secs_f32();
        
        for y in 0..HEIGHT {
            for x in 0..WIDTH {
                let idx = (y * WIDTH + x) * 3;
                
                // Create a moving gradient pattern
                let fx = x as f32 / WIDTH as f32;
                let fy = y as f32 / HEIGHT as f32;
                
                // Animated gradient
                let r = ((fx + t * 0.1) * 255.0) as u8 % 255;
                let g = ((fy + t * 0.15) * 255.0) as u8 % 255;
                let b = ((fx + fy + t * 0.2) * 255.0) as u8 % 255;
                
                frame[idx] = r;
                frame[idx + 1] = g;
                frame[idx + 2] = b;
                
                // Add a moving white square
                let square_size = 100;
                let square_x = ((t * 50.0) as usize) % (WIDTH - square_size);
                let square_y = ((t * 30.0) as usize) % (HEIGHT - square_size);
                
                if x >= square_x && x < square_x + square_size &&
                   y >= square_y && y < square_y + square_size {
                    frame[idx] = 255;
                    frame[idx + 1] = 255;
                    frame[idx + 2] = 255;
                }
            }
        }
        
        // Send frame to pipeline
        if let Err(e) = pipeline.lock().await.queue_frame(&frame).await {
            warn!("Failed to queue test frame: {:?}", e);
            break;
        }
        
        frame_count += 1;
        if frame_count % 90 == 0 {
            info!("Generated {} test frames (running for {:.1}s)", 
                  frame_count, start_time.elapsed().as_secs_f32());
        }
        
        // Sleep to maintain ~30fps
        let elapsed = frame_start.elapsed();
        if elapsed < frame_duration {
            sleep(frame_duration - elapsed).await;
        }
    }
}
