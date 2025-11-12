use std::{pin::Pin, sync::Arc};

use bytes::Bytes;
use gstreamer::{
    self as gst, Pipeline,
    glib::object::Cast,
    prelude::{ElementExt, GstBinExt},
};
use gstreamer_app::{self as gst_app, AppSinkCallbacks, AppSrc};
use log::info;
use thiserror::Error;
use tokio::sync::Mutex;

pub type OnFrameReadyHandlerFn =
    Box<dyn (FnMut(&Bytes) -> Pin<Box<dyn Future<Output = ()> + Send + 'static>>) + Send + Sync>;

#[derive(Error, Debug)]
pub enum VideoPipelineError {
    #[error("Failed to initialise GStreamer pipeline")]
    GStreamerInitFailure,
    #[error("Failed to parse GStreamer launch command")]
    GStreamerParseError,
    #[error("Failed to downcast GStreamer pipeline")]
    GStreamerDowncastFailure,
    #[error("Requested pipeline name missing")]
    GStreamerNameMissing,
    #[error("Start pipeline playing failed")]
    GStreamerPipelinePlayFailure,
    #[error("Pushing frame to GStreamer buffer failed")]
    GStreamerBufferPushError,
    #[error("Failed to stop pipeline from playing")]
    GStreamerStopFailure,
    #[error("Invalid state for requested operation")]
    InvalidStateError,
}

pub struct VideoPipeline {
    appsrc: Arc<Mutex<Option<AppSrc>>>,
    frame_listeners: Arc<Mutex<Vec<OnFrameReadyHandlerFn>>>,
    pipeline: Option<Pipeline>,
}

impl VideoPipeline {
    pub fn new() -> Self {
        Self {
            appsrc: Arc::new(Mutex::new(None)),
            frame_listeners: Arc::new(Mutex::new(vec![])),
            pipeline: None,
        }
    }

    pub async fn launch(&mut self) -> Result<(), VideoPipelineError> {
        info!("Initialising GStreamer");
        gst::init().map_err(|_| VideoPipelineError::GStreamerInitFailure)?;

        info!("Launching GStreamer pipeline");
        let pipeline = gst::parse::launch(
            "appsrc name=src ! videoconvert ! x264enc tune=zerolatency key-int-max=30 byte-stream=true ! video/x-h264,stream-format=byte-stream,profile=baseline ! appsink name=sink",
        ).map_err(|_| VideoPipelineError::GStreamerParseError)?;
        let pipeline = pipeline
            .downcast::<gst::Pipeline>()
            .map_err(|_| VideoPipelineError::GStreamerDowncastFailure)?;

        let caps = gst::Caps::builder("video/x-raw")
            .field("format", "RGB")
            .field("width", 640)
            .field("height", 480)
            .field("framerate", gst::Fraction::new(30, 1))
            .build();

        let src = pipeline
            .by_name("src")
            .ok_or(VideoPipelineError::GStreamerNameMissing)?
            .downcast::<gst_app::AppSrc>()
            .map_err(|_| VideoPipelineError::GStreamerDowncastFailure)?;
        src.set_caps(Some(&caps));

        // TODO: experiment with these and callbacks to prevent too much data
        // going in to the pipeline
        // src.set_property("is-live", true);
        // src.set_property("block", true);

        // Store app source for pushing frames later
        self.appsrc.lock().await.replace(src);

        let sink = pipeline
            .by_name("sink")
            .unwrap()
            .downcast::<gst_app::AppSink>()
            .unwrap();
        let sink = Arc::new(sink);

        // Create a channel for tokio to listen to
        let (frame_tx, mut frame_rx) = tokio::sync::mpsc::channel::<Bytes>(10);
        let listeners_clone = Arc::clone(&self.frame_listeners);
        tokio::spawn(async move {
            while let Some(buffer) = frame_rx.recv().await {
                for listener in listeners_clone.lock().await.iter_mut() {
                    listener(&buffer).await;
                }
            }
        });

        sink.set_callbacks(
            AppSinkCallbacks::builder()
                .new_sample(move |sink| {
                    let sample = sink.pull_sample().map_err(|_| gst::FlowError::Error)?;
                    let buffer = sample
                        .buffer()
                        .expect("Error while retrieving buffer from frame!");
                    let map = buffer
                        .map_readable()
                        .expect("Error mapping readable data from frame buffer!");
                    let data = Bytes::from(map.to_vec());
                    frame_tx
                        .blocking_send(data)
                        .expect("Error sending video frame to channel!");
                    Ok(gst::FlowSuccess::Ok)
                })
                .build(),
        );

        pipeline
            .set_state(gst::State::Playing)
            .map_err(|_| VideoPipelineError::GStreamerPipelinePlayFailure)?;

        self.pipeline.replace(pipeline);

        Ok(())
    }

    pub async fn on_frame_ready(&self, listener: OnFrameReadyHandlerFn) {
        self.frame_listeners.lock().await.push(listener);
    }

    pub async fn queue_frame(&self, frame: &[u8]) -> Result<(), VideoPipelineError> {
        let buffer = gst::Buffer::from_slice(frame.to_vec());
        if let Some(src) = self.appsrc.lock().await.as_mut() {
            src.push_buffer(buffer)
                .map_err(|_| VideoPipelineError::GStreamerBufferPushError)?;
        }
        Ok(())
    }

    pub async fn stop_pipeline(&self) -> Result<(), VideoPipelineError> {
        match &self.pipeline {
            Some(pipeline) => {
                pipeline
                    .set_state(gst::State::Null)
                    .map_err(|_| VideoPipelineError::GStreamerStopFailure)?;
                Ok(())
            }
            None => Err(VideoPipelineError::InvalidStateError),
        }
    }
}
