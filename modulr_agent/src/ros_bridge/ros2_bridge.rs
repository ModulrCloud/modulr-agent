use crate::commands::config::ImageFormat;
use crate::ros_bridge::{RosBridge, RosBridgeError};
use modulr_ros_messages::ros2_messages::{
    geometry_msgs::{Point, Pose, PoseStamped, Quaternion, Twist, TwistStamped, Vector3},
    sensor_msgs::{CompressedImage, Image},
    std_msgs,
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
    nav_goal_pub: Arc<Mutex<Option<Publisher<PoseStamped>>>>,
    nav_cancel_pub: Arc<Mutex<Option<Publisher<std_msgs::Empty>>>>,
    bridge_handle: Arc<Mutex<Option<ClientHandle>>>,
}

impl Ros2Bridge {
    pub fn new() -> Self {
        Self {
            image_listeners: Arc::new(Mutex::new(vec![])),
            mvmt_pub: Arc::new(Mutex::new(None)),
            nav_goal_pub: Arc::new(Mutex::new(None)),
            nav_cancel_pub: Arc::new(Mutex::new(None)),
            bridge_handle: Arc::new(Mutex::new(None)),
        }
    }
}

#[async_trait]
impl RosBridge for Ros2Bridge {
    async fn launch(&self, image_format: Option<ImageFormat>) -> Result<(), super::RosBridgeError> {
        let ros = ClientHandle::new("ws://localhost:9090")
            .await
            .map_err(|_| RosBridgeError::InitFailure)?;

        let mvmt_pub = ros
            .advertise::<TwistStamped>("/cmd_vel")
            .await
            .map_err(|_| RosBridgeError::PublisherCreateFailure)?;

        let nav_goal_pub = ros
            .advertise::<PoseStamped>("/modulr/nav/goal")
            .await
            .map_err(|_| RosBridgeError::PublisherCreateFailure)?;

        let nav_cancel_pub = ros
            .advertise::<std_msgs::Empty>("/modulr/nav/cancel")
            .await
            .map_err(|_| RosBridgeError::PublisherCreateFailure)?;

        if let Some(format) = image_format {
            match format {
                ImageFormat::Raw => {
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
                ImageFormat::Jpeg => {
                    log::info!(
                        "ROS2 video enabled. Subscribing to `/camera/image_raw/compressed`."
                    );
                    let image_sub = ros
                        .subscribe::<CompressedImage>("/camera/image_raw/compressed")
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
            }
        }

        self.bridge_handle.lock().await.replace(ros);
        self.mvmt_pub.lock().await.replace(mvmt_pub);
        self.nav_goal_pub.lock().await.replace(nav_goal_pub);
        self.nav_cancel_pub.lock().await.replace(nav_cancel_pub);

        Ok(())
    }

    async fn on_image_frame_received(&mut self, listener: OnCameraImageHdlrFn) {
        self.image_listeners.lock().await.push(listener);
    }

    async fn post_movement_command(
        &mut self,
        command: &modulr_webrtc_message::MovementCommand,
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

    async fn post_navigation_goal(
        &mut self,
        location: &modulr_webrtc_message::Location,
    ) -> Result<(), RosBridgeError> {
        let (qx, qy, qz, qw) = match &location.orientation {
            Some(o) => super::conversions::euler_to_quaternion(
                o.yaw.unwrap_or(0.0),
                o.pitch.unwrap_or(0.0),
                o.roll.unwrap_or(0.0),
            ),
            None => (0.0, 0.0, 0.0, 1.0),
        };

        let msg = PoseStamped {
            header: std_msgs::Header {
                frame_id: "map".to_string(),
                ..Default::default()
            },
            pose: Pose {
                position: Point {
                    x: location.position.x,
                    y: location.position.y,
                    z: location.position.z.unwrap_or(0.0),
                },
                orientation: Quaternion {
                    x: qx,
                    y: qy,
                    z: qz,
                    w: qw,
                },
            },
        };

        if let Some(nav_pub) = self.nav_goal_pub.lock().await.as_ref() {
            nav_pub
                .publish(&msg)
                .await
                .map_err(|_| RosBridgeError::PublishFailed)?;
            Ok(())
        } else {
            Err(RosBridgeError::InvalidStateError)
        }
    }

    async fn cancel_navigation(&mut self) -> Result<(), RosBridgeError> {
        if let Some(nav_pub) = self.nav_cancel_pub.lock().await.as_ref() {
            nav_pub
                .publish(&std_msgs::Empty {})
                .await
                .map_err(|_| RosBridgeError::PublishFailed)?;
            Ok(())
        } else {
            Err(RosBridgeError::InvalidStateError)
        }
    }
}
