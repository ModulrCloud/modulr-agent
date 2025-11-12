use std::sync::Arc;

use bytes::Bytes;
use geometry_msgs::msg::{
    Twist as TwistMsg, TwistStamped as TwistStampedMsg, Vector3 as Vector3Msg,
};
use log::error;
use rclrs::*;
use sensor_msgs::msg::Image as ImageMsg;
use tokio::sync::{Mutex, mpsc};

use crate::{
    ros_bridge::{OnCameraImageHdlrFn, RosBridgeError},
    webrtc_message::MovementCommand,
};

pub struct RosBridge {
    executor: Option<Executor>,
    _image_sub: WorkerSubscription<ImageMsg, ()>,
    image_listeners: Arc<Mutex<Vec<OnCameraImageHdlrFn>>>,
    mvmt_pub: Arc<rclrs::PublisherState<TwistStampedMsg>>,
}

type Empty = ();

impl RosBridge {
    pub fn try_new() -> Result<Self, RosBridgeError> {
        let executor = rclrs::Context::default_from_env()
            .map_err(|_| RosBridgeError::InitFailure)?
            .create_basic_executor();

        let node = executor
            .create_node("modulr_agent")
            .map_err(|_| RosBridgeError::NodeCreateFailure)?;

        let image_listeners: Arc<Mutex<Vec<OnCameraImageHdlrFn>>> = Arc::new(Mutex::new(vec![]));

        let (tx, mut rx) = mpsc::channel::<Bytes>(10);

        let worker = node.create_worker::<()>(());
        let image_sub = worker
            .create_subscription("/camera/image_raw", move |_: &mut Empty, msg: ImageMsg| {
                let buffer = Bytes::from(msg.data);
                if tx.try_send(buffer).is_err() {
                    error!("Failed to send image buffer to channel!");
                }
            })
            .map_err(|_| RosBridgeError::SubscriptionCreateFailure)?;

        let listeners_clone = Arc::clone(&image_listeners);
        tokio::spawn(async move {
            while let Some(buffer) = rx.recv().await {
                // debug!("Received frame from ROS camera subscription");
                for listener in listeners_clone.lock().await.iter_mut() {
                    tokio::spawn(listener(&buffer));
                }
            }
        });

        let mvmt_pub = node
            .create_publisher::<TwistStampedMsg>("cmd_vel")
            .map_err(|_| RosBridgeError::PublisherCreateFailure)?;

        Ok(Self {
            executor: Some(executor),
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
        let msg = TwistStampedMsg {
            twist: TwistMsg {
                linear: Vector3Msg {
                    x: command.forward,
                    y: 0f64,
                    z: 0f64,
                },
                angular: Vector3Msg {
                    x: 0f64,
                    y: 0f64,
                    z: command.turn,
                },
            },
            ..Default::default()
        };
        self.mvmt_pub
            .publish(&msg)
            .map_err(|_| RosBridgeError::PublishFailed)?;
        Ok(())
    }

    pub fn spin(&mut self) {
        let exec = match self.executor.take() {
            None => {
                error!("No executor available for spinning!");
                return;
            }
            Some(exec) => exec,
        };

        tokio::spawn(async move {
            exec.spin_async(rclrs::SpinOptions::default()).await;
        });
    }
}
