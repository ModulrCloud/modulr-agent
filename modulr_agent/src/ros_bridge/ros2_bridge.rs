use crate::ros_bridge::{RosBridge, RosBridgeError};
use modulr_ros_messages::ros2_messages::{
    geometry_msgs::{Twist, TwistStamped, Vector3},
    sensor_msgs::Image,
};

use std::sync::Arc;

use async_trait::async_trait;
use bytes::Bytes;
use roslibrust::rosbridge::{ClientHandle, Publisher};
use tokio::sync::Mutex;

use crate::ros_bridge::OnCameraImageHdlrFn;

#[derive(Clone)]
pub struct Ros2Bridge {
    image_listeners: Arc<Mutex<Vec<OnCameraImageHdlrFn>>>,
    mvmt_pub: Arc<Mutex<Option<Publisher<TwistStamped>>>>,
    bridge_handle: Arc<Mutex<Option<ClientHandle>>>,
}

impl Ros2Bridge {
    pub fn new() -> Self {
        Self {
            image_listeners: Arc::new(Mutex::new(vec![])),
            mvmt_pub: Arc::new(Mutex::new(None)),
            bridge_handle: Arc::new(Mutex::new(None)),
        }
    }
}

#[async_trait]
impl RosBridge for Ros2Bridge {
    async fn launch(&self, enable_video: bool) -> Result<(), super::RosBridgeError> {
        let ros = ClientHandle::new("ws://localhost:9090")
            .await
            .map_err(|_| RosBridgeError::InitFailure)?;

        let mvmt_pub = ros
            .advertise::<TwistStamped>("/cmd_vel")
            .await
            .map_err(|_| RosBridgeError::PublisherCreateFailure)?;

        if enable_video {
            log::info!("ROS2 video enabled. Subscribing to `/camera/image_raw`.");
            let image_sub = ros
                .subscribe::<Image>("/camera/image_raw")
                .await
                .map_err(|_| RosBridgeError::SubscriptionCreateFailure)?;
            let listeners = Arc::clone(&self.image_listeners);
            tokio::spawn(async move {
                loop {
                    let msg = image_sub.next().await;
                    let buffer = Bytes::from(msg.data);
                    for listener in listeners.lock().await.iter_mut() {
                        listener(&buffer).await;
                    }
                }
            });
        }

        self.bridge_handle.lock().await.replace(ros);
        self.mvmt_pub.lock().await.replace(mvmt_pub);

        Ok(())
    }

    async fn on_image_frame_received(&mut self, listener: OnCameraImageHdlrFn) {
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
        let stamped = TwistStamped {
            twist: msg,
            ..Default::default()
        };

        if let Some(mvmt_pub) = self.mvmt_pub.lock().await.as_ref() {
            mvmt_pub
                .publish(&stamped)
                .await
                .map_err(|_| RosBridgeError::PublishFailed)?;
            Ok(())
        } else {
            Err(RosBridgeError::InvalidStateError)
        }
    }
}
