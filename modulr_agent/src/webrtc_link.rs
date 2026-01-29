use bytes::Bytes;
use futures_util::stream::{SplitSink, SplitStream};
use futures_util::{SinkExt, StreamExt};
use log::{debug, error, info, warn};
use rustls::client::{ServerCertVerified, ServerCertVerifier};
use rustls::{ClientConfig, RootCertStore};
use serde::{Deserialize, Serialize};
use std::future::Future;
use std::pin::Pin;
use std::sync::Arc;
use std::time::{Duration, Instant};
use thiserror::Error;
use tokio::net::TcpStream;
use tokio::sync::Mutex;
use tokio_tungstenite::{
    Connector, MaybeTlsStream, WebSocketStream, connect_async_tls_with_config,
};
use tungstenite::Message;
use webrtc::api::APIBuilder;
use webrtc::api::media_engine::{MIME_TYPE_H264, MediaEngine};
use webrtc::data_channel::RTCDataChannel;
use webrtc::data_channel::data_channel_message::DataChannelMessage;
use webrtc::ice_transport::ice_candidate::{RTCIceCandidate, RTCIceCandidateInit};
use webrtc::ice_transport::ice_connection_state::RTCIceConnectionState;
use webrtc::ice_transport::ice_server::RTCIceServer;
use webrtc::media::Sample;
use webrtc::peer_connection::RTCPeerConnection;
use webrtc::peer_connection::configuration::RTCConfiguration;
use webrtc::peer_connection::sdp::session_description::RTCSessionDescription;
use webrtc::rtp_transceiver::rtp_codec::RTCRtpCodecCapability;
use webrtc::track::track_local::TrackLocal;
use webrtc::track::track_local::track_local_static_sample::TrackLocalStaticSample;

use crate::webrtc_message::{
    MessageEnvelope,
    parse_message,
    validate_envelope,
    handle_message,
};

type MaybeWebSocketWriter =
    Arc<Mutex<Option<SplitSink<WebSocketStream<MaybeTlsStream<TcpStream>>, Message>>>>;
type MaybeWebSocketReader =
    Arc<Mutex<Option<SplitStream<WebSocketStream<MaybeTlsStream<TcpStream>>>>>>;

pub type OnWebRtcMessageHdlrFn = Box<
    dyn (FnMut(&MessageEnvelope) -> Pin<Box<dyn Future<Output = ()> + Send + 'static>>) + Send + Sync,
>;

fn insecure_verifier() -> Arc<dyn ServerCertVerifier> {
    #[derive(Debug)]
    struct InsecureVerifier;

    impl ServerCertVerifier for InsecureVerifier {
        fn verify_server_cert(
            &self,
            _end_entity: &rustls::Certificate,
            _intermediates: &[rustls::Certificate],
            _server_name: &rustls::ServerName,
            _scts: &mut dyn Iterator<Item = &[u8]>,
            _ocsp_response: &[u8],
            _now: std::time::SystemTime,
        ) -> Result<ServerCertVerified, rustls::Error> {
            Ok(ServerCertVerified::assertion())
        }
    }

    Arc::new(InsecureVerifier)
}

#[derive(Debug, PartialEq, Copy, Clone)]
pub enum WebRtcLinkStatus {
    // Link starts disconnected
    Disconnected,
    // Call try_connect to start connection process
    Unregistered,
    // On connection to signaling server, class automatically registers
    Registered,
    // Browser tries to connect; negotiate link and connect
    Connected,
}

#[derive(Error, Debug)]
pub enum WebRtcLinkError {
    #[error("Connection attempt failed: `{0}`")]
    ConnectionAttemptFailed(tungstenite::Error),
    #[error("Incorrect state for operation")]
    IncorrectStateForOperation,
    #[error("Failed to add default media codecs")]
    AddDefaultCodecFailed,
    #[error("Failed to add video track")]
    AddVideoTrackFailed,
    #[error("Failed to add ICE candidate")]
    AddIceCandidateFailed,
    #[error("Failed to create peer connection")]
    PeerConnectionCreationFailed,
    #[error("Failed to set remote description")]
    RemoteDescriptionFailed,
    #[error("Failed to set local description")]
    LocalDescriptionFailed,
    #[error("Invalid message for string conversion sent")]
    EncodeMessageFailure,
    #[error("Invalid message for string conversion received")]
    ParseMessageFailure,
    #[error("Failed to send message")]
    FailedToSendMessage,
    #[error("Unrecognised message type received")]
    UnrecognisedMessage,
    #[error("Failed to write frame to GStreamer")]
    WriteFrameError,
}

pub struct WebRtcLink {
    status: Arc<Mutex<WebRtcLinkStatus>>,
    local_id: String,
    signaling_url: String,
    allow_skip_cert_check: bool,

    // Can be added to after link creation
    message_listeners: Arc<Mutex<Vec<OnWebRtcMessageHdlrFn>>>,

    // Valid only after try_connect has been called successfully
    ws_write: MaybeWebSocketWriter,
    ws_read: MaybeWebSocketReader,
    peer_connection: Arc<Mutex<Option<RTCPeerConnection>>>,

    // ICE state helpers
    remote_desc_set: Arc<Mutex<bool>>,
    pending_remote_candidates: Arc<Mutex<Vec<RTCIceCandidateInit>>>,

    // Valid only after full connection
    video_track: Arc<Mutex<Option<Arc<TrackLocalStaticSample>>>>,
    data_channel: Arc<Mutex<Option<Arc<RTCDataChannel>>>>,

    // id of the partner
    remote_id: Arc<Mutex<Option<String>>>,

    // Timing for frame rate calculation
    last_frame_time: Arc<Mutex<Option<Instant>>>,
}

impl WebRtcLink {
    pub fn new(id: &str, url: &str, allow_skip_cert_check: bool) -> Self {
        Self {
            status: Arc::new(Mutex::new(WebRtcLinkStatus::Disconnected)),
            local_id: id.to_string(),
            signaling_url: url.to_string(),
            allow_skip_cert_check,
            message_listeners: Arc::new(Mutex::new(vec![])),
            ws_write: Arc::new(Mutex::new(None)),
            ws_read: Arc::new(Mutex::new(None)),
            peer_connection: Arc::new(Mutex::new(None)),
            remote_desc_set: Arc::new(Mutex::new(false)),
            pending_remote_candidates: Arc::new(Mutex::new(Vec::new())),
            video_track: Arc::new(Mutex::new(None)),
            data_channel: Arc::new(Mutex::new(None)),
            remote_id: Arc::new(Mutex::new(None)),
            last_frame_time: Arc::new(Mutex::new(None)),
        }
    }

    pub async fn on_webrtc_message(&mut self, listener: OnWebRtcMessageHdlrFn) {
        self.message_listeners.lock().await.push(listener);
    }

    pub async fn is_connected(&self) -> bool {
        *self.status.lock().await == WebRtcLinkStatus::Connected
    }

    pub async fn try_connect(&mut self) -> Result<(), WebRtcLinkError> {
        info!(
            "WebRtcLink::try_connect: robot_id={}, signaling_url={}",
            self.local_id, self.signaling_url
        );

        let mut config = ClientConfig::builder()
            .with_safe_defaults()
            .with_root_certificates(RootCertStore::empty())
            .with_no_client_auth();

        if self.allow_skip_cert_check {
            warn!("allow_skip_cert_check=true, using insecure TLS verifier");
            config
                .dangerous()
                .set_certificate_verifier(insecure_verifier());
        }

        let connector = Connector::Rustls(Arc::new(config));

        info!("Connecting WebSocket to {}", self.signaling_url);
        let (ws_stream, _) =
            connect_async_tls_with_config(&self.signaling_url, None, false, Some(connector))
                .await
                .map_err(WebRtcLinkError::ConnectionAttemptFailed)?;

        info!("WebSocket connected to signaling server");

        let (ws_write, ws_read) = ws_stream.split();

        debug!("Adding default codecs to WebRTC offer");
        let mut media_engine = MediaEngine::default();
        media_engine
            .register_default_codecs()
            .map_err(|_| WebRtcLinkError::AddDefaultCodecFailed)?;
        let api = APIBuilder::new().with_media_engine(media_engine).build();

        debug!("Adding STUN servers to WebRTC offer");
        let config = RTCConfiguration {
            ice_servers: vec![RTCIceServer {
                urls: vec!["stun:stun.l.google.com:19302".to_string()],
                ..Default::default()
            }],
            ..Default::default()
        };

        debug!("Creating peer connection");
        let peer_connection = api
            .new_peer_connection(config)
            .await
            .map_err(|_| WebRtcLinkError::PeerConnectionCreationFailed)?;

        info!("PeerConnection created for robot_id={}", self.local_id);

        self.ws_write.lock().await.replace(ws_write);
        self.ws_read.lock().await.replace(ws_read);

        // Setup callback methods for peer connection
        let write_clone = Arc::clone(&self.ws_write);
        let remote_id = Arc::clone(&self.remote_id);
        let robot_id = self.local_id.clone();

        peer_connection.on_ice_candidate(Box::new(move |c: Option<RTCIceCandidate>| {
            let write_clone = Arc::clone(&write_clone);
            let remote_id = Arc::clone(&remote_id);
            let robot_id = robot_id.clone();

            Box::pin(async move {
                if let Some(candidate) = c
                    && let Some(ws_write) = &mut *write_clone.lock().await
                {
                    match candidate.to_json() {
                        Ok(json_candidate) => {
                            let maybe_to = remote_id.lock().await.clone();

                            if let Some(to_id) = maybe_to {
                                debug!(
                                    "on_ice_candidate: robot_id={}, to={}, candidate={:?}",
                                    robot_id, to_id, json_candidate
                                );

                                let msg = serde_json::json!({
                                    "type": "candidate",
                                    "from": robot_id,
                                    "to": to_id,
                                    "candidate": json_candidate
                                });

                                if let Err(e) = ws_write
                                    .send(tokio_tungstenite::tungstenite::Message::Text(
                                        serde_json::to_string(&msg).unwrap(),
                                    ))
                                    .await
                                {
                                    warn!("Failed to send ICE candidate to {to_id}: {e}");
                                } else {
                                    debug!("Sent ICE candidate to browser ({to_id})");
                                }
                            } else {
                                warn!(
                                    "ICE candidate generated but remote_id is not set yet; skipping"
                                );
                            }
                        }
                        Err(e) => {
                            error!("Failed to convert ICE candidate to JSON: {e}");
                        }
                    }
                } else {
                    warn!("on_ice_candidate fired with None or no ws_write");
                }
            })
        }));

        // On connection state Connected, update our own connected state
        let status_clone = Arc::clone(&self.status);
        peer_connection.on_ice_connection_state_change(Box::new(move |state| {
            info!("ICE connection state changed: {:?}", state);

            let status_clone = Arc::clone(&status_clone);
            Box::pin(async move {
                if state == RTCIceConnectionState::Connected {
                    info!("ICE state Connected, marking WebRtcLink as Connected");
                    *status_clone.lock().await = WebRtcLinkStatus::Connected;
                }
            })
        }));

        let listeners_clone = Arc::clone(&self.message_listeners);
        let data_channel_clone = Arc::clone(&self.data_channel);
        peer_connection.on_data_channel(Box::new(move |dc| {
            info!("DataChannel opened: {}", dc.label());

            let listeners_clone = Arc::clone(&listeners_clone);
            let data_channel_for_messages = Arc::clone(&data_channel_clone);  // Clone for on_message
            dc.on_message(Box::new(move |msg: DataChannelMessage| {
                debug!("Received data channel message: {:?}", msg);
                let listeners_clone = Arc::clone(&listeners_clone);
                let data_channel_clone = Arc::clone(&data_channel_for_messages);
                Box::pin(async move {
                    // Convert bytes to string
                    let text = match std::str::from_utf8(&msg.data) {
                        Ok(t) => t,
                        Err(e) => {
                            error!("Invalid UTF-8 in data channel message: {}", e);
                            return;
                        }
                    };

                    // Parse into MessageEnvelope
                    let envelope = match parse_message(text) {
                        Ok(env) => env,
                        Err(e) => {
                            error!("Failed to parse message: {}", e);
                            return;
                        }
                    };

                    // Validate envelope structure
                    if let Err(e) = validate_envelope(&envelope) {
                        error!("Invalid message envelope: {}", e);
                        return;
                    }

                    // Handle protocol-level messages (ping, pong, capabilities, errors)
                    if let Some(response) = handle_message(&envelope) {
                        if let Some(dc) = data_channel_clone.lock().await.as_ref() {
                            if let Ok(response_json) = serde_json::to_string(&response) {
                                if let Err(e) = dc.send_text(response_json).await {
                                    error!("Failed to send response: {}", e);
                                }
                            }
                        }
                    }

                    // Forward to application listeners
                    for listener in listeners_clone.lock().await.iter_mut() {
                        tokio::spawn(listener(&envelope));
                    }
                })
            }));
                        
            let data_channel_clone = Arc::clone(&data_channel_clone);
            Box::pin(async move {
                data_channel_clone.lock().await.replace(dc);
            })
        }));

        self.peer_connection.lock().await.replace(peer_connection);
        *self.status.lock().await = WebRtcLinkStatus::Unregistered;
        info!(
            "WebRtcLink::try_connect complete, status=Unregistered, robot_id={}",
            self.local_id
        );

        Ok(())
    }

    pub async fn try_register(&mut self) -> Result<(), WebRtcLinkError> {
        let current_status = *self.status.lock().await;
        info!(
            "WebRtcLink::try_register called, robot_id={}, status={:?}",
            self.local_id, current_status
        );

        if current_status != WebRtcLinkStatus::Unregistered {
            warn!(
                "try_register called in wrong state: {:?} (expected Unregistered)",
                current_status
            );
            return Err(WebRtcLinkError::IncorrectStateForOperation);
        }

        let register_msg = SignalMessage {
            r#type: "register".into(),
            from: Some(self.local_id.clone()),
            to: None,
            sdp: None,
            candidate: None,
        };

        if let Some(ws_write) = &mut *self.ws_write.lock().await {
            let encoded = serde_json::to_string(&register_msg)
                .map_err(|_| WebRtcLinkError::EncodeMessageFailure)?;
            debug!("Sending register message: {encoded}");

            ws_write
                .send(Message::Text(encoded))
                .await
                .map_err(|_| WebRtcLinkError::FailedToSendMessage)?;

            info!("Sent register for robot_id={}", self.local_id);
            *self.status.lock().await = WebRtcLinkStatus::Registered;
        } else {
            error!("try_register called but ws_write is None");
        }

        // Start listening for messages from the server
        self.listen().await;

        Ok(())
    }

    pub async fn write_frame(&self, frame: Bytes) -> Result<(), WebRtcLinkError> {
        if let Some(video) = self.video_track.lock().await.as_ref() {
            // Calculate duration based on actual time between frames
            let now = Instant::now();
            let mut last_time_guard = self.last_frame_time.lock().await;
            let duration = if let Some(last_time) = *last_time_guard {
                now.duration_since(last_time)
            } else {
                // Default to ~30fps for first frame
                Duration::from_millis(33)
            };
            *last_time_guard = Some(now);
            drop(last_time_guard);

            video
                .write_sample(&Sample {
                    data: frame,
                    duration,
                    ..Default::default()
                })
                .await
                .map_err(|_| WebRtcLinkError::WriteFrameError)?;
            Ok(())
        } else {
            Err(WebRtcLinkError::IncorrectStateForOperation)
        }
    }

    async fn listen(&self) {
        let remote_id_clone = Arc::clone(&self.remote_id);

        if self.ws_read.lock().await.is_none() {
            panic!("Listen called before ws_read is valid!");
        }

        let write_clone = Arc::clone(&self.ws_write);
        let read_clone = Arc::clone(&self.ws_read);
        let video_track_clone = Arc::clone(&self.video_track);
        let peer_connection_clone = Arc::clone(&self.peer_connection);
        let remote_desc_set_clone = Arc::clone(&self.remote_desc_set);
        let pending_remote_candidates_clone = Arc::clone(&self.pending_remote_candidates);
        let robot_id = self.local_id.clone();

        info!("Starting listen loop for robot_id={}", robot_id);

        tokio::spawn(async move {
            // Main WS receive loop
            while let Some(Ok(msg)) = read_clone.lock().await.as_mut().unwrap().next().await {
                match msg {
                    Message::Text(txt) => {
                        debug!("WS Text message received: {}", txt);

                        let parsed = serde_json::from_str::<SignalMessage>(&txt);
                        let signal = match parsed {
                            Ok(sig) => sig,
                            Err(e) => {
                                error!("Failed to parse SignalMessage: {e}, raw={}", txt);
                                continue;
                            }
                        };

                        debug!(
                            "SignalMessage parsed: type={}, from={:?}, to={:?}, has_sdp={}, has_candidate={}",
                            signal.r#type,
                            signal.from,
                            signal.to,
                            signal.sdp.is_some(),
                            signal.candidate.is_some()
                        );

                        match signal.r#type.as_str() {
                            "offer" => {
                                info!(
                                    "Processing offer: robot_id={}, from={:?}, to={:?}",
                                    robot_id, signal.from, signal.to
                                );

                                let sdp = match signal.sdp {
                                    Some(s) => s,
                                    None => {
                                        error!("Offer message missing SDP");
                                        return Err(WebRtcLinkError::ParseMessageFailure);
                                    }
                                };

                                debug!("Offer SDP length: {}", sdp.len());

                                let offer = RTCSessionDescription::offer(sdp)
                                    .expect("Could not build SDP from offer");

                                if let Some(from) = &signal.from {
                                    *remote_id_clone.lock().await = Some(from.clone());
                                    info!("Set remote_id to {}", from);
                                } else {
                                    warn!("Offer received without 'from', remote_id not set");
                                }

                                let video_track = add_video_track(&peer_connection_clone).await?;
                                video_track_clone.lock().await.replace(video_track);

                                if let Some(pc) = peer_connection_clone.lock().await.as_mut() {
                                    pc.set_remote_description(offer)
                                        .await
                                        .map_err(|_| WebRtcLinkError::RemoteDescriptionFailed)?;

                                    // Mark remote description as set
                                    {
                                        let mut rd = remote_desc_set_clone.lock().await;
                                        *rd = true;
                                    }

                                    // Flush any queued remote candidates
                                    {
                                        let mut pending =
                                            pending_remote_candidates_clone.lock().await;
                                        if !pending.is_empty() {
                                            info!(
                                                "Flushing {} pending ICE candidates",
                                                pending.len()
                                            );
                                            for cand in pending.drain(..) {
                                                if let Err(e) = pc.add_ice_candidate(cand).await {
                                                    error!(
                                                        "Failed to add queued ICE candidate: {e}"
                                                    );
                                                }
                                            }
                                        }
                                    }

                                    info!("Remote description set, creating answer");

                                    let answer = pc
                                        .create_answer(None)
                                        .await
                                        .map_err(|_| WebRtcLinkError::LocalDescriptionFailed)?;

                                    debug!("Answer SDP length: {}", answer.sdp.len());

                                    let answer_msg = serde_json::json!({
                                        "type": "answer",
                                        "from": robot_id,
                                        "to": signal.from,
                                        "sdp": answer.sdp,
                                    });

                                    pc.set_local_description(answer)
                                        .await
                                        .map_err(|_| WebRtcLinkError::LocalDescriptionFailed)?;

                                    info!("Local description set, sending answer");

                                    if let Some(w) = write_clone.lock().await.as_mut() {
                                        let encoded = serde_json::to_string(&answer_msg)
                                            .map_err(|_| WebRtcLinkError::EncodeMessageFailure)?;
                                        debug!("Sending answer message: {}", encoded);
                                        w.send(Message::Text(encoded))
                                            .await
                                            .map_err(|_| WebRtcLinkError::FailedToSendMessage)?;
                                    } else {
                                        error!("Cannot send answer: ws_write is None");
                                    }
                                } else {
                                    error!("PeerConnection is None in offer handler");
                                }
                            }
                            "candidate" => {
                                info!(
                                    "Processing remote candidate: from={:?}, to={:?}",
                                    signal.from, signal.to
                                );

                                if let Some(candidate_json) = signal.candidate {
                                    debug!("Remote candidate JSON: {}", candidate_json);

                                    let candidate: RTCIceCandidateInit =
                                        serde_json::from_value(candidate_json)
                                            .map_err(|_| WebRtcLinkError::ParseMessageFailure)?;

                                    let remote_desc_is_set = *remote_desc_set_clone.lock().await;

                                    if !remote_desc_is_set {
                                        info!(
                                            "Remote description not set yet, buffering ICE candidate"
                                        );
                                        pending_remote_candidates_clone
                                            .lock()
                                            .await
                                            .push(candidate);
                                    } else if let Some(pc) =
                                        peer_connection_clone.lock().await.as_mut()
                                    {
                                        info!("Adding remote ICE candidate");
                                        pc.add_ice_candidate(candidate)
                                            .await
                                            .map_err(|_| WebRtcLinkError::AddIceCandidateFailed)?;
                                    } else {
                                        error!("PeerConnection is None in candidate handler");
                                    }
                                } else {
                                    warn!("Candidate message with no candidate field");
                                }
                            }
                            other => {
                                warn!(
                                    "Unrecognised message type received: {} (full message: {:?})",
                                    other, signal
                                );
                                return Err(WebRtcLinkError::UnrecognisedMessage);
                            }
                        }
                    }
                    other => {
                        debug!("Non-text WS message received: {:?}", other);
                    }
                }
            }

            info!("WebSocket listen loop ended for robot_id={}", robot_id);
            Ok::<(), WebRtcLinkError>(())
        });
    }
}

#[derive(Serialize, Deserialize, Debug)]
struct SignalMessage {
    r#type: String,
    from: Option<String>,
    to: Option<String>,
    sdp: Option<String>,
    candidate: Option<serde_json::Value>,
}

async fn add_video_track(
    peer_connection: &Arc<Mutex<Option<RTCPeerConnection>>>,
) -> Result<Arc<TrackLocalStaticSample>, WebRtcLinkError> {
    // Need valid peer connection
    if peer_connection.lock().await.is_none() {
        return Err(WebRtcLinkError::IncorrectStateForOperation);
    }
    let codec = RTCRtpCodecCapability {
        mime_type: MIME_TYPE_H264.to_owned(),
        ..Default::default()
    };

    let video_track = TrackLocalStaticSample::new(codec, "video".to_owned(), "camera".to_owned());
    let video_track = Arc::new(video_track);
    peer_connection
        .lock()
        .await
        .as_mut()
        .unwrap()
        .add_track(Arc::clone(&video_track) as Arc<dyn TrackLocal + Send + Sync>)
        .await
        .map_err(|_| WebRtcLinkError::AddVideoTrackFailed)?;

    Ok(video_track)
}