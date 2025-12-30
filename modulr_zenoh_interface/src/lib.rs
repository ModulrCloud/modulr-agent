//! # Modulr Zenoh Interface
//!
//! High-performance video frame transport using Zenoh pub/sub middleware.
//!
//! This crate provides [`ZenohInterface`], which subscribes to Zenoh topics containing
//! video frames in ROS2 `sensor_msgs/Image` or `sensor_msgs/CompressedImage` format
//! (CDR serialized) and delivers the image data to registered listener callbacks.
//!
//! For examples, see the files in the `examples` folder.

use std::{pin::Pin, sync::Arc};

use bytes::Bytes;
pub use modulr_agent_common::ImageFormat;
use modulr_ros_messages::ros2_messages::sensor_msgs;
use thiserror::Error;
use tokio::sync::Mutex;

/// Callback type for frame reception.
///
/// The callback receives a reference to the raw pixel data (`&Bytes`) extracted from
/// the `sensor_msgs::Image::data` field. The callback must return a pinned boxed future
/// that will be awaited after all listeners have been notified.
///
/// # Example
///
/// ```rust
/// use bytes::Bytes;
/// use std::pin::Pin;
///
/// let listener: modulr_zenoh_interface::OnFrameReceivedFn = Box::new(|data: &Bytes| {
///     println!("Got {} bytes", data.len());
///     Box::pin(async {})
/// });
/// ```
pub type OnFrameReceivedFn =
    Box<dyn (Fn(&Bytes) -> Pin<Box<dyn Future<Output = ()> + Send + 'static>>) + Send + Sync>;

/// Error types for the Zenoh interface.
#[derive(Error, Debug)]
pub enum ZenohInterfaceError {
    /// Failed to open the Zenoh session.
    #[error("Failed to open the Zenoh session: {0}")]
    SessionOpenFailure(zenoh::Error),

    /// Failed to subscribe to the camera topic.
    #[error("Failed to subscribe to camera topic: {0}")]
    SubscriptionFailed(zenoh::Error),
}

/// Interface for receiving video frames over Zenoh.
///
/// `ZenohInterface` subscribes to Zenoh topics for video frames, deserializes
/// incoming messages from CDR format, and delivers the image data to registered
/// listener callbacks.
///
/// # Topics
///
/// - For `ImageFormat::Raw`: subscribes to `camera/image_raw` expecting `sensor_msgs::Image`
/// - For `ImageFormat::Jpeg`: subscribes to `camera/image_raw/compressed` expecting `sensor_msgs::CompressedImage`
///
/// Messages must be CDR-serialized (little-endian).
///
/// # Lifecycle
///
/// 1. Create with [`ZenohInterface::new()`]
/// 2. Register listeners with [`ZenohInterface::add_frame_listener()`]
/// 3. Start receiving with [`ZenohInterface::launch()`]
///
/// The interface spawns a background tokio task that runs until the Zenoh subscription
/// encounters an error.
pub struct ZenohInterface {
    image_listeners: Arc<Mutex<Vec<OnFrameReceivedFn>>>,
    image_format: ImageFormat,
}

impl Default for ZenohInterface {
    fn default() -> Self {
        Self::new(ImageFormat::Raw)
    }
}

impl ZenohInterface {
    /// Creates a new `ZenohInterface` with no registered listeners.
    ///
    /// Call [`add_frame_listener`](Self::add_frame_listener) to register callbacks
    /// before calling [`launch`](Self::launch).
    pub fn new(image_format: ImageFormat) -> Self {
        Self {
            image_listeners: Arc::new(Mutex::new(vec![])),
            image_format,
        }
    }

    /// Launches the Zenoh interface and begins receiving frames.
    ///
    /// This method:
    /// 1. Opens a Zenoh session with default configuration
    /// 2. Subscribes to the appropriate topic based on image format
    /// 3. Spawns a background task that receives and processes frames
    ///
    /// The background task will continue running until the subscription encounters
    /// an error. Each received frame is deserialized from CDR format and the
    /// image data is passed to all registered listeners.
    pub async fn launch(&self) -> Result<(), ZenohInterfaceError> {
        log::info!("Launching Zenoh interface for grabbing camera frames.");
        let session = zenoh::open(zenoh::Config::default())
            .await
            .map_err(ZenohInterfaceError::SessionOpenFailure)?;

        let listeners = Arc::clone(&self.image_listeners);

        match self.image_format {
            ImageFormat::Raw => {
                let subscriber = session
                    .declare_subscriber("camera/image_raw")
                    .await
                    .map_err(ZenohInterfaceError::SubscriptionFailed)?;

                log::info!("Spawning receiver task for raw images...");
                tokio::spawn(async move {
                    // Keep session alive for the lifetime of this task
                    let _session = session;
                    log::info!("Receiver task started, waiting for samples...");
                    loop {
                        log::debug!("Waiting for next sample...");
                        let sample = match subscriber.recv_async().await {
                            Ok(sample) => {
                                log::debug!(
                                    "Received sample with {} bytes",
                                    sample.payload().len()
                                );
                                sample
                            }
                            Err(e) => {
                                log::error!("Failed to receive camera frame: {e}");
                                break;
                            }
                        };
                        // Deserialize directly from the payload slice to avoid an extra copy
                        let payload = sample.payload();
                        let image: sensor_msgs::Image =
                            match cdr::deserialize_from(payload.reader(), cdr::size::Infinite) {
                                Ok(img) => img,
                                Err(e) => {
                                    log::error!("Failed to deserialize Image message: {e}");
                                    continue;
                                }
                            };
                        log::debug!("Deserialized image: {}x{}", image.width, image.height);
                        let buffer = Bytes::from(image.data);
                        let listener_count = listeners.lock().await.len();
                        log::debug!("Notifying {} listeners", listener_count);
                        for listener in listeners.lock().await.iter() {
                            listener(&buffer).await;
                        }
                    }
                });
            }
            ImageFormat::Jpeg => {
                let subscriber = session
                    .declare_subscriber("camera/image_raw/compressed")
                    .await
                    .map_err(ZenohInterfaceError::SubscriptionFailed)?;

                log::info!("Spawning receiver task for compressed images...");
                tokio::spawn(async move {
                    // Keep session alive for the lifetime of this task
                    let _session = session;
                    log::info!("Receiver task started, waiting for samples...");
                    loop {
                        log::debug!("Waiting for next sample...");
                        let sample = match subscriber.recv_async().await {
                            Ok(sample) => {
                                log::debug!(
                                    "Received sample with {} bytes",
                                    sample.payload().len()
                                );
                                sample
                            }
                            Err(e) => {
                                log::error!("Failed to receive camera frame: {e}");
                                break;
                            }
                        };
                        // Deserialize directly from the payload slice to avoid an extra copy
                        let payload = sample.payload();
                        let image: sensor_msgs::CompressedImage =
                            match cdr::deserialize_from(payload.reader(), cdr::size::Infinite) {
                                Ok(img) => img,
                                Err(e) => {
                                    log::error!(
                                        "Failed to deserialize CompressedImage message: {e}"
                                    );
                                    continue;
                                }
                            };
                        log::debug!(
                            "Deserialized compressed image: {} bytes, format: {}",
                            image.data.len(),
                            image.format
                        );
                        let buffer = Bytes::from(image.data);
                        let listener_count = listeners.lock().await.len();
                        log::debug!("Notifying {} listeners", listener_count);
                        for listener in listeners.lock().await.iter() {
                            listener(&buffer).await;
                        }
                    }
                });
            }
        }
        Ok(())
    }

    /// Adds a listener callback for received frames.
    ///
    /// The listener will be called with the raw pixel data (from `sensor_msgs::Image::data`)
    /// each time a frame is received. Multiple listeners can be registered and will all
    /// be called sequentially for each frame.
    ///
    /// Listeners should be registered before calling [`launch`](Self::launch).
    pub async fn add_frame_listener(&self, listener: OnFrameReceivedFn) {
        let mut listeners = self.image_listeners.lock().await;
        listeners.push(listener);
        log::debug!(
            "Frame listener registered, total listeners: {}",
            listeners.len()
        );
    }
}
