use std::{pin::Pin, sync::Arc};

use bytes::Bytes;
use gstreamer::{
    self as gst, Pipeline,
    glib::object::Cast,
    prelude::{ElementExt, GstBinExt},
};
use gstreamer_app::{self as gst_app, AppSink, AppSrc};
use log::{debug, info};
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
    #[error("Failed to retrieve frame from GStreamer sink")]
    FrameRetrievalError,
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

        info!("Launching pipeline");
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

        // Store app source for pushing frames later
        self.appsrc.lock().await.replace(src);

        let sink = pipeline
            .by_name("sink")
            .unwrap()
            .downcast::<gst_app::AppSink>()
            .unwrap();
        let sink = Arc::new(sink);
        debug!("Starting to listen for frames");
        tokio::spawn(loop_reading_frames(sink, self.frame_listeners.clone()));

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
        if let Some(src) = self.appsrc.lock().await.as_mut() {
            let buffer = gst::Buffer::from_slice(frame.to_vec());
            src.push_buffer(buffer)
                .map_err(|_| VideoPipelineError::GStreamerBufferPushError)?;
            Ok(())
        } else {
            Err(VideoPipelineError::InvalidStateError)
        }
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

async fn loop_reading_frames(
    sink: Arc<AppSink>,
    listeners: Arc<Mutex<Vec<OnFrameReadyHandlerFn>>>,
) -> Result<(), VideoPipelineError> {
    loop {
        if let Ok(sample) = sink.pull_sample() {
            let buffer = sample
                .buffer()
                .ok_or(VideoPipelineError::FrameRetrievalError)?;
            let map = buffer
                .map_readable()
                .map_err(|_| VideoPipelineError::FrameRetrievalError)?;
            let data = Bytes::from(map.to_vec());

            // Call all listeners with new frame
            for listener in listeners.lock().await.iter_mut() {
                listener(&data).await;
            }
        }

        // Prevent busy-waiting
        tokio::time::sleep(tokio::time::Duration::from_millis(1)).await;
    }
}
