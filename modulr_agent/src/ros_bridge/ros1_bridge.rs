use crate::commands::config::ImageFormat;
use crate::ros_bridge::ros_bridge_trait::RosBridge;
use crate::ros_bridge::{OnCameraImageHdlrFn, RosBridgeError};
use modulr_ros_messages::ros1_messages::{
    geometry_msgs::{Twist, Vector3},
    sensor_msgs::{CompressedImage, Image},
};

use std::sync::Arc;

use async_trait::async_trait;
use bytes::Bytes;
use roslibrust::ros1::{NodeHandle, Publisher};
use tokio::sync::Mutex;

#[derive(Clone)]
pub struct Ros1Bridge {
    image_listeners: Arc<Mutex<Vec<OnCameraImageHdlrFn>>>,
    mvmt_pub: Arc<Mutex<Option<Publisher<Twist>>>>,
    node_handle: Arc<Mutex<Option<NodeHandle>>>,
}

impl Ros1Bridge {
    pub fn new() -> Self {
        Self {
            image_listeners: Arc::new(Mutex::new(vec![])),
            mvmt_pub: Arc::new(Mutex::new(None)),
            node_handle: Arc::new(Mutex::new(None)),
        }
    }
}

#[async_trait]
impl RosBridge for Ros1Bridge {
    async fn launch(&self, image_format: Option<ImageFormat>) -> Result<(), super::RosBridgeError> {
        let nh = NodeHandle::new("http://localhost:11311", "modulr_agent")
            .await
            .map_err(|_| RosBridgeError::InitFailure)?;

        let mvmt_pub = nh
            .advertise::<Twist>("/cmd_vel", 1, false)
            .await
            .map_err(|_| RosBridgeError::PublisherCreateFailure)?;

        if let Some(format) = image_format {
            match format {
                ImageFormat::Raw => {
                    log::info!("ROS1 video enabled. Subscribing to `/camera/image_raw`.");
                    let mut image_sub = nh
                        .subscribe::<Image>("/camera/image_raw", 1)
                        .await
                        .map_err(|_| RosBridgeError::SubscriptionCreateFailure)?;
                    let listeners = Arc::clone(&self.image_listeners);
                    tokio::spawn(async move {
                        while let Some(Ok(msg)) = image_sub.next().await {
                            let buffer = Bytes::from(msg.data);
                            for listener in listeners.lock().await.iter_mut() {
                                listener(&buffer).await;
                            }
                        }
                    });
                }
                ImageFormat::Jpeg => {
                    log::info!(
                        "ROS1 video enabled. Subscribing to `/camera/image_raw/compressed`."
                    );
                    let mut image_sub = nh
                        .subscribe::<CompressedImage>("/camera/image_raw/compressed", 1)
                        .await
                        .map_err(|_| RosBridgeError::SubscriptionCreateFailure)?;
                    let listeners = Arc::clone(&self.image_listeners);
                    tokio::spawn(async move {
                        while let Some(Ok(msg)) = image_sub.next().await {
                            let buffer = Bytes::from(msg.data);
                            for listener in listeners.lock().await.iter_mut() {
                                listener(&buffer).await;
                            }
                        }
                    });
                }
            }
        }

        self.node_handle.lock().await.replace(nh);
        self.mvmt_pub.lock().await.replace(mvmt_pub);

        Ok(())
    }

    async fn on_image_frame_received(&mut self, listener: super::OnCameraImageHdlrFn) {
        self.image_listeners.lock().await.push(listener);
    }

    async fn post_movement_command(
        &mut self,
        command: &crate::webrtc_message::MovementCommand,
    ) -> Result<(), super::RosBridgeError> {
        let msg = Twist {
            linear: Vector3 {
                x: command.forward,
                y: 0f64,
                z: 0f64,
            },
            angular: Vector3 {
                x: 0f64,
                y: 0f64,
                z: command.turn,
            },
        };

        if let Some(mvmt_pub) = self.mvmt_pub.lock().await.as_ref() {
            mvmt_pub
                .publish(&msg)
                .await
                .map_err(|_| RosBridgeError::PublishFailed)?;
            Ok(())
        } else {
            Err(RosBridgeError::InvalidStateError)
        }
    }
}
