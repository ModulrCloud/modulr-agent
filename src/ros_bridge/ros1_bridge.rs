use std::sync::{Arc, Mutex};

use bytes::Bytes;
use log::{debug, error};
use rosrust::{Publisher, Subscriber};
use std::pin::Pin;
use tokio::sync::mpsc;

use crate::{
    ros_bridge::{OnCameraImageHdlrFn, RosBridgeError},
    webrtc_message::MovementCommand,
};

pub struct RosBridge {
    _image_sub: Subscriber,
    image_listeners: Arc<Mutex<Vec<RosBridgeOnFrameCallback>>>,
    mvmt_pub: Publisher<rosrust_msg::geometry_msgs::Twist>,
}

impl RosBridge {
    pub fn try_new() -> Result<Self, RosBridgeError> {
        rosrust::init("modulr_agent");

        let image_listeners: Arc<Mutex<Vec<OnCameraImageHdlrFn>>> = Arc::new(Mutex::new(vec![]));

        let (tx, mut rx) = mpsc::channel::<Bytes>(10);

        let listeners_clone = Arc::clone(image_listeners);
        let image_sub = rosrust::subscribe(
            "/camera/image_raw",
            1,
            move |v: rosrust_msg::sensor_msgs::Image| {
                let buffer = Bytes::from(v.data);
                if tx.try_send(buffer).is_err() {
                    error!("Failed to send image buffer to channel!");
                }
            },
        )
        .map_err(|_| RosBridgeError::SubscriptionCreateFailure)?;

        let listeners_clone = Arc::clone(&image_listeners);
        tokio::spawn(async move {
            while let Some(buffer) = rx.recv().await {
                // debug!("Received frame from ROS camera subscription");
                for listener in listeners_clone.lock().unwrap().iter_mut() {
                    tokio::spawn(listener(&buffer));
                }
            }
        });

        let mvmt_pub =
            rosrust::publish("/cmd_vel", 2).map_err(|_| RosBridgeError::PublisherCreateFailure)?;

        Ok(Self {
            _image_sub: image_sub,
            image_listeners,
            mvmt_pub,
        })
    }

    // Register a callback for camera images
    pub fn on_image_frame_received(&mut self, listener: OnCameraImageHdlrFn) {
        if let Ok(mut listeners) = self.image_listeners.try_lock() {
            listeners.push(listener);
        }
    }

    // Post a new movement command
    pub fn post_movement_command(
        &mut self,
        command: &MovementCommand,
    ) -> Result<(), RosBridgeError> {
        debug!("Received movement command: {:?}", command);
        let msg = rosrust_msg::geometry_msgs::Twist {
            linear: rosrust_msg::geometry_msgs::Vector3 {
                x: command.forward,
                y: 0f64,
                z: 0f64,
            },
            angular: rosrust_msg::geometry_msgs::Vector3 {
                x: 0f64,
                y: 0f64,
                z: command.turn,
            },
        };
        self.mvmt_pub
            .send(msg)
            .map_err(|_| RosBridgeError::PublishFailed)?;
        Ok(())
    }

    pub fn spin(&self) {
        std::thread::spawn(|| {
            rosrust::spin();
        });
    }
}
