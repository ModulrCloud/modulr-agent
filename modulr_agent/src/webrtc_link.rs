use bytes::Bytes;
use futures_util::stream::{SplitSink, SplitStream};
use futures_util::{SinkExt, StreamExt};
use rustls::ClientConfig;
use rustls::client::{ServerCertVerified, ServerCertVerifier};
use std::future::Future;
use std::pin::Pin;
use std::str::FromStr;
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
    AgentMessage, AnswerPayload, ConnectedPayload, DisconnectedPayload, DisconnectionReason,
    IceCandidatePayload, IceConnectionState, MessageEnvelope, PingPayload, RegisterPayload,
    SignalingMessage, ToMessage,
};

type MaybeWebSocketWriter =
    Arc<Mutex<Option<SplitSink<WebSocketStream<MaybeTlsStream<TcpStream>>, Message>>>>;
type MaybeWebSocketReader =
    Arc<Mutex<Option<SplitStream<WebSocketStream<MaybeTlsStream<TcpStream>>>>>>;

pub type OnWebRtcMessageHdlrFn = Box<
    dyn (FnMut(&AgentMessage) -> Pin<Box<dyn Future<Output = ()> + Send + 'static>>) + Send + Sync,
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

fn build_tls_connector(allow_skip_cert_check: bool) -> Connector {
    let mut root_store = rustls::RootCertStore::empty();

    root_store.add_trust_anchors(webpki_roots::TLS_SERVER_ROOTS.iter().map(|ta| {
        rustls::OwnedTrustAnchor::from_subject_spki_name_constraints(
            ta.subject,
            ta.spki,
            ta.name_constraints,
        )
    }));

    // Load all certificates added to the system manually by the user
    match rustls_native_certs::load_native_certs() {
        Ok(certs) => {
            for cert in certs {
                if let Err(e) = root_store.add(&rustls::Certificate(cert.0)) {
                    log::debug!("Failed to add a native cert to root store: {}", e);
                }
            }
        }
        Err(e) => {
            log::warn!(
                "Failed to load native certs, using webpki-roots only: {}",
                e
            );
        }
    }

    let mut config = ClientConfig::builder()
        .with_safe_defaults()
        .with_root_certificates(root_store)
        .with_no_client_auth();

    if allow_skip_cert_check {
        log::warn!("allow_skip_cert_check=true, using insecure TLS verifier (dev/local only)");
        config
            .dangerous()
            .set_certificate_verifier(insecure_verifier());
    }

    Connector::Rustls(Arc::new(config))
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
    #[error("Failed to send message")]
    FailedToSendMessage,
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
        log::info!(
            "WebRtcLink::try_connect: robot_id={}, signaling_url={}",
            self.local_id,
            self.signaling_url
        );

        let connector = Some(build_tls_connector(self.allow_skip_cert_check));

        log::info!("Connecting WebSocket to {}", self.signaling_url);
        let (ws_stream, _) =
            connect_async_tls_with_config(&self.signaling_url, None, false, connector)
                .await
                .map_err(WebRtcLinkError::ConnectionAttemptFailed)?;

        log::info!("WebSocket connected to signaling server");

        let (ws_write, ws_read) = ws_stream.split();

        log::debug!("Adding default codecs to WebRTC offer");
        let mut media_engine = MediaEngine::default();
        media_engine
            .register_default_codecs()
            .map_err(|_| WebRtcLinkError::AddDefaultCodecFailed)?;
        let api = APIBuilder::new().with_media_engine(media_engine).build();

        log::debug!("Adding STUN servers to WebRTC offer");
        let config = RTCConfiguration {
            ice_servers: vec![RTCIceServer {
                urls: vec!["stun:stun.l.google.com:19302".to_string()],
                ..Default::default()
            }],
            ..Default::default()
        };

        log::debug!("Creating peer connection");
        let peer_connection = api
            .new_peer_connection(config)
            .await
            .map_err(|_| WebRtcLinkError::PeerConnectionCreationFailed)?;

        log::info!("PeerConnection created for robot_id={}", self.local_id);

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
                let Some(candidate) = c else {
                    log::debug!("ICE gathering complete (null candidate)");
                    return;
                };

                let Some(ws_write) = &mut *write_clone.lock().await else {
                    log::warn!("ICE candidate generated but ws_write is None");
                    return;
                };

                match candidate.to_json() {
                    Ok(json_candidate) => {
                        let maybe_remote_id = remote_id.lock().await.clone();

                        if let Some(remote_id) = maybe_remote_id {
                            log::debug!(
                                "on_ice_candidate: robot_id={}, remote_id={}, candidate={:?}",
                                robot_id,
                                remote_id,
                                json_candidate
                            );

                            let Some(sdp_mid) = json_candidate.sdp_mid else {
                                log::warn!("No sdp_mid set in ICE candidate, skipping");
                                return;
                            };

                            let Some(sdp_m_line_index) = json_candidate.sdp_mline_index else {
                                log::warn!("No sdp_mline_index set in ICE candidate, skipping");
                                return;
                            };

                            let msg = SignalingMessage::IceCandidate(IceCandidatePayload {
                                candidate: json_candidate.candidate,
                                connection_id: remote_id.clone(),
                                sdp_mid,
                                sdp_m_line_index,
                                username_fragment: json_candidate.username_fragment,
                            });

                            let encoded = match msg.to_message().to_string() {
                                Ok(encoded) => encoded,
                                Err(e) => {
                                    log::error!("Error occurred while encoding message: {}", e);
                                    return;
                                }
                            };

                            if let Err(e) = ws_write
                                .send(tokio_tungstenite::tungstenite::Message::Text(encoded))
                                .await
                            {
                                log::warn!(
                                    "Failed to send ICE candidate for remote ID {remote_id}: {e}"
                                );
                            } else {
                                log::debug!("Sent ICE candidate for remote ID ({remote_id})");
                            }
                        } else {
                            log::warn!(
                                "ICE candidate generated but remote_id is not set yet; skipping"
                            );
                        }
                    }
                    Err(e) => {
                        log::error!("Failed to convert ICE candidate to JSON: {e}");
                    }
                }
            })
        }));

        // On connection state change, update status and send signaling messages
        let status_clone = Arc::clone(&self.status);
        let write_clone_ice = Arc::clone(&self.ws_write);
        let remote_id_ice = Arc::clone(&self.remote_id);
        peer_connection.on_ice_connection_state_change(Box::new(move |state| {
            log::info!("ICE connection state changed: {:?}", state);

            let status_clone = Arc::clone(&status_clone);
            let write_clone = Arc::clone(&write_clone_ice);
            let remote_id = Arc::clone(&remote_id_ice);

            Box::pin(async move {
                let maybe_remote_id = remote_id.lock().await.clone();

                match state {
                    RTCIceConnectionState::Connected => {
                        log::info!("ICE state Connected, marking WebRtcLink as Connected");
                        *status_clone.lock().await = WebRtcLinkStatus::Connected;

                        if let Some(remote_id) = maybe_remote_id {
                            let msg = SignalingMessage::Connected(ConnectedPayload {
                                connection_id: remote_id,
                                ice_connection_state: IceConnectionState::Connected,
                                data_channel_state: None,
                            });
                            send_signaling_message(&write_clone, msg).await;
                        }
                    }
                    RTCIceConnectionState::Failed => {
                        log::warn!("ICE state Failed");
                        *status_clone.lock().await = WebRtcLinkStatus::Disconnected;

                        if let Some(remote_id) = maybe_remote_id {
                            let msg = SignalingMessage::Disconnected(DisconnectedPayload {
                                connection_id: remote_id,
                                reason: DisconnectionReason::Failed,
                                ice_connection_state: None,
                                details: None,
                            });
                            send_signaling_message(&write_clone, msg).await;
                        }
                    }
                    RTCIceConnectionState::Disconnected => {
                        log::warn!("ICE state Disconnected");
                        *status_clone.lock().await = WebRtcLinkStatus::Disconnected;

                        if let Some(remote_id) = maybe_remote_id {
                            let msg = SignalingMessage::Disconnected(DisconnectedPayload {
                                connection_id: remote_id,
                                reason: DisconnectionReason::Closed,
                                ice_connection_state: None,
                                details: None,
                            });
                            send_signaling_message(&write_clone, msg).await;
                        }
                    }
                    _ => {
                        log::debug!("ICE connection state: {:?} (no action needed)", state);
                    }
                }
            })
        }));

        let listeners_clone = Arc::clone(&self.message_listeners);
        let data_channel_clone = Arc::clone(&self.data_channel);
        peer_connection.on_data_channel(Box::new(move |dc| {
            log::info!("DataChannel opened: {}", dc.label());

            let data_channel_for_open = Arc::clone(&data_channel_clone);
            dc.on_open(Box::new(move || {
                let dc_clone = Arc::clone(&data_channel_for_open);
                Box::pin(async move {
                    if let Ok(json) = AgentMessage::capabilities().to_message().to_string()
                        && let Some(dc) = dc_clone.lock().await.as_ref()
                        && let Err(e) = dc.send_text(json).await
                    {
                        log::error!("Failed to send capabilities: {}", e);
                    }
                })
            }));

            let listeners_clone = Arc::clone(&listeners_clone);
            let data_channel_for_messages = Arc::clone(&data_channel_clone); // Clone for on_message
            dc.on_message(Box::new(move |msg: DataChannelMessage| {
                log::debug!("Received data channel message: {:?}", msg);
                let listeners_clone = Arc::clone(&listeners_clone);
                let data_channel_clone = Arc::clone(&data_channel_for_messages);
                Box::pin(async move {
                    let envelope: MessageEnvelope = match serde_json::from_slice(&msg.data) {
                        Ok(env) => env,
                        Err(e) => {
                            log::error!("Failed to parse message: {}", e);
                            return;
                        }
                    };

                    let agent_message = match AgentMessage::from_message(&envelope) {
                        Ok(msg) => msg,
                        Err(e) => {
                            log::error!("Failed to read message as agent message: {}", e);
                            return;
                        }
                    };

                    match agent_message {
                        AgentMessage::Error(_) | AgentMessage::Pong(_) => {
                            log::warn!(
                                "Unexpected message received from client: {:?}",
                                agent_message
                            )
                        }
                        AgentMessage::Capabilities(ref caps) => {
                            log::info!("Received capabilities from client: {:?}", caps);
                            if let Some(err) = AgentMessage::capabilities_error_response(
                                caps,
                                envelope.correlation_id.as_deref(),
                            ) && let Some(dc) = data_channel_clone.lock().await.as_ref()
                                && let Ok(json) = err.to_message().to_string()
                                && let Err(e) = dc.send_text(json).await
                            {
                                log::error!("Failed to send capabilities error: {}", e);
                            }
                        }
                        AgentMessage::Ping(PingPayload { correlation_id }) => {
                            let pong = AgentMessage::pong(&correlation_id);
                            if let Some(dc) = data_channel_clone.lock().await.as_ref()
                                && let Ok(response_json) = serde_json::to_string(&pong)
                                && let Err(e) = dc.send_text(response_json).await
                            {
                                log::error!("Failed to send response: {}", e);
                            }
                        }
                        AgentMessage::Movement(_) => {
                            for listener in listeners_clone.lock().await.iter_mut() {
                                tokio::spawn(listener(&agent_message));
                            }
                        }
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
        log::info!(
            "WebRtcLink::try_connect complete, status=Unregistered, robot_id={}",
            self.local_id
        );

        Ok(())
    }

    pub async fn try_register(&mut self) -> Result<(), WebRtcLinkError> {
        let current_status = *self.status.lock().await;
        log::info!(
            "WebRtcLink::try_register called, robot_id={}, status={:?}",
            self.local_id,
            current_status
        );

        if current_status != WebRtcLinkStatus::Unregistered {
            log::warn!(
                "try_register called in wrong state: {:?} (expected Unregistered)",
                current_status
            );
            return Err(WebRtcLinkError::IncorrectStateForOperation);
        }

        let register_msg = SignalingMessage::Register(RegisterPayload {
            agent_id: self.local_id.clone(),
            capabilities: None,
            metadata: None,
        });

        if let Some(ws_write) = &mut *self.ws_write.lock().await {
            let encoded = register_msg
                .to_message()
                .to_string()
                .map_err(|_| WebRtcLinkError::EncodeMessageFailure)?;
            log::debug!("Sending register message: {encoded}");

            ws_write
                .send(Message::Text(encoded))
                .await
                .map_err(|_| WebRtcLinkError::FailedToSendMessage)?;

            log::info!("Sent register for robot_id={}", self.local_id);
            *self.status.lock().await = WebRtcLinkStatus::Registered;
        } else {
            log::error!("try_register called but ws_write is None");
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

        log::info!("Starting listen loop for robot_id={}", robot_id);

        tokio::spawn(async move {
            // Main WS receive loop
            while let Some(Ok(msg)) = read_clone.lock().await.as_mut().unwrap().next().await {
                match msg {
                    Message::Text(txt) => {
                        log::debug!("WS Text message received: {}", txt);

                        let envelope = match MessageEnvelope::from_str(&txt) {
                            Ok(env) => env,
                            Err(e) => {
                                log::error!("Failed to parse MessageEnvelope: {e}, raw={}", txt);
                                continue;
                            }
                        };

                        let signaling_msg = match SignalingMessage::from_message(&envelope) {
                            Ok(msg) => msg,
                            Err(e) => {
                                log::error!("Failed to parse SignalingMessage: {e}, raw={}", txt);
                                continue;
                            }
                        };

                        log::debug!("SignalingMessage parsed: {:?}", signaling_msg);

                        match signaling_msg {
                            SignalingMessage::Offer(offer) => {
                                log::info!(
                                    "Processing offer: robot_id={}, connection_id={}",
                                    robot_id,
                                    offer.connection_id
                                );

                                log::debug!("Offer SDP length: {}", offer.sdp.len());

                                let rtc_offer = RTCSessionDescription::offer(offer.sdp.clone())
                                    .expect("Could not build SDP from offer");

                                *remote_id_clone.lock().await = Some(offer.connection_id.clone());
                                log::info!("Set remote_id to {}", offer.connection_id);

                                let video_track = add_video_track(&peer_connection_clone).await?;
                                video_track_clone.lock().await.replace(video_track);

                                if let Some(pc) = peer_connection_clone.lock().await.as_mut() {
                                    pc.set_remote_description(rtc_offer)
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
                                            log::info!(
                                                "Flushing {} pending ICE candidates",
                                                pending.len()
                                            );
                                            for cand in pending.drain(..) {
                                                if let Err(e) = pc.add_ice_candidate(cand).await {
                                                    log::error!(
                                                        "Failed to add queued ICE candidate: {e}"
                                                    );
                                                }
                                            }
                                        }
                                    }

                                    log::info!("Remote description set, creating answer");

                                    let answer = pc
                                        .create_answer(None)
                                        .await
                                        .map_err(|_| WebRtcLinkError::LocalDescriptionFailed)?;

                                    log::debug!("Answer SDP length: {}", answer.sdp.len());

                                    let answer_msg = SignalingMessage::Answer(AnswerPayload {
                                        connection_id: offer.connection_id.clone(),
                                        sdp: answer.sdp.clone(),
                                        sdp_type: "answer".to_string(),
                                    });

                                    pc.set_local_description(answer)
                                        .await
                                        .map_err(|_| WebRtcLinkError::LocalDescriptionFailed)?;

                                    log::info!("Local description set, sending answer");

                                    if let Some(w) = write_clone.lock().await.as_mut() {
                                        let encoded = answer_msg
                                            .to_message()
                                            .to_string()
                                            .map_err(|_| WebRtcLinkError::EncodeMessageFailure)?;
                                        log::debug!("Sending answer message: {}", encoded);
                                        w.send(Message::Text(encoded))
                                            .await
                                            .map_err(|_| WebRtcLinkError::FailedToSendMessage)?;
                                    } else {
                                        log::error!("Cannot send answer: ws_write is None");
                                    }
                                } else {
                                    log::error!("PeerConnection is None in offer handler");
                                }
                            }

                            SignalingMessage::IceCandidate(ice) => {
                                log::info!(
                                    "Processing remote ICE candidate: connection_id={}",
                                    ice.connection_id
                                );

                                log::debug!("Remote candidate: {}", ice.candidate);

                                let candidate = RTCIceCandidateInit {
                                    candidate: ice.candidate,
                                    sdp_mid: Some(ice.sdp_mid),
                                    sdp_mline_index: Some(ice.sdp_m_line_index),
                                    username_fragment: ice.username_fragment,
                                };

                                let remote_desc_is_set = *remote_desc_set_clone.lock().await;

                                if !remote_desc_is_set {
                                    log::info!(
                                        "Remote description not set yet, buffering ICE candidate"
                                    );
                                    pending_remote_candidates_clone.lock().await.push(candidate);
                                } else if let Some(pc) = peer_connection_clone.lock().await.as_mut()
                                {
                                    log::info!("Adding remote ICE candidate");
                                    pc.add_ice_candidate(candidate)
                                        .await
                                        .map_err(|_| WebRtcLinkError::AddIceCandidateFailed)?;
                                } else {
                                    log::error!("PeerConnection is None in candidate handler");
                                }
                            }

                            SignalingMessage::Capabilities(caps) => {
                                log::info!("Received capabilities from server: {:?}", caps);
                            }

                            SignalingMessage::Error(err) => {
                                log::error!("Received error from server: {:?}", err);
                            }

                            SignalingMessage::Connected(connected) => {
                                log::info!("Received connected notification: {:?}", connected);
                            }

                            SignalingMessage::Disconnected(disconnected) => {
                                log::warn!(
                                    "Received disconnected notification: {:?}",
                                    disconnected
                                );
                            }

                            other => {
                                log::warn!("Received unexpected signaling message: {:?}", other);
                            }
                        }
                    }
                    other => {
                        log::debug!("Non-text WS message received: {:?}", other);
                    }
                }
            }

            log::info!("WebSocket listen loop ended for robot_id={}", robot_id);
            Ok::<(), WebRtcLinkError>(())
        });
    }
}

async fn send_signaling_message(ws_write: &MaybeWebSocketWriter, msg: SignalingMessage) {
    if let Some(ws) = &mut *ws_write.lock().await {
        match msg.to_message().to_string() {
            Ok(encoded) => {
                if let Err(e) = ws
                    .send(tokio_tungstenite::tungstenite::Message::Text(encoded))
                    .await
                {
                    log::error!("Failed to send signaling message: {e}");
                }
            }
            Err(e) => log::error!("Failed to encode signaling message: {e}"),
        }
    } else {
        log::warn!("Cannot send signaling message: ws_write is None");
    }
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
