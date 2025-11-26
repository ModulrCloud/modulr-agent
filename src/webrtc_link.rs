use bytes::Bytes;
use futures_util::stream::{SplitSink, SplitStream};
use futures_util::{SinkExt, StreamExt};
use log::{debug, error, info, warn};
use rustls::client::danger::{HandshakeSignatureValid, ServerCertVerified, ServerCertVerifier};
use rustls::pki_types::{CertificateDer, ServerName};
use rustls::{ClientConfig, DigitallySignedStruct, RootCertStore, SignatureScheme};
use serde::{Deserialize, Serialize};
use std::pin::Pin;
use std::sync::Arc;
use std::time::Duration;
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

use crate::webrtc_message::WebRtcMessage;

type MaybeWebSocketWriter =
    Arc<Mutex<Option<SplitSink<WebSocketStream<MaybeTlsStream<TcpStream>>, Message>>>>;
type MaybeWebSocketReader =
    Arc<Mutex<Option<SplitStream<WebSocketStream<MaybeTlsStream<TcpStream>>>>>>;

pub type OnWebRtcMessageHdlrFn = Box<
    dyn (FnMut(&WebRtcMessage) -> Pin<Box<dyn Future<Output = ()> + Send + 'static>>) + Send + Sync,
>;

fn insecure_verifier() -> Arc<dyn ServerCertVerifier> {
    #[derive(Debug)]
    struct InsecureVerifier;

    impl ServerCertVerifier for InsecureVerifier {
        fn verify_server_cert(
            &self,
            _end_entity: &CertificateDer<'_>,
            _intermediates: &[CertificateDer<'_>],
            _server_name: &ServerName<'_>,
            _ocsp: &[u8],
            _now: rustls::pki_types::UnixTime,
        ) -> Result<ServerCertVerified, rustls::Error> {
            Ok(ServerCertVerified::assertion())
        }

        fn verify_tls12_signature(
            &self,
            _message: &[u8],
            _cert: &CertificateDer<'_>,
            _dss: &DigitallySignedStruct,
        ) -> Result<HandshakeSignatureValid, rustls::Error> {
            Ok(HandshakeSignatureValid::assertion())
        }

        fn verify_tls13_signature(
            &self,
            _message: &[u8],
            _cert: &CertificateDer<'_>,
            _dss: &DigitallySignedStruct,
        ) -> Result<HandshakeSignatureValid, rustls::Error> {
            Ok(HandshakeSignatureValid::assertion())
        }

        fn supported_verify_schemes(&self) -> Vec<SignatureScheme> {
            vec![
                SignatureScheme::RSA_PKCS1_SHA256,
                SignatureScheme::RSA_PKCS1_SHA384,
                SignatureScheme::RSA_PKCS1_SHA512,
                SignatureScheme::ECDSA_NISTP256_SHA256,
                SignatureScheme::ECDSA_NISTP384_SHA384,
                SignatureScheme::ECDSA_NISTP521_SHA512,
                SignatureScheme::RSA_PSS_SHA256,
                SignatureScheme::RSA_PSS_SHA384,
                SignatureScheme::RSA_PSS_SHA512,
                SignatureScheme::ED25519,
                SignatureScheme::ED448,
            ]
        }
    }

    Arc::new(InsecureVerifier)
}

#[derive(Debug, PartialEq)]
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
    #[error("Connection attempt failed")]
    ConnectionAttemptFailed,
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
    #[error("Failed to write framte to GStreamer")]
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

    // Valid only after full connection
    video_track: Arc<Mutex<Option<Arc<TrackLocalStaticSample>>>>,
    data_channel: Arc<Mutex<Option<Arc<RTCDataChannel>>>>,
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
            video_track: Arc::new(Mutex::new(None)),
            data_channel: Arc::new(Mutex::new(None)),
        }
    }

    pub async fn on_webrtc_message(&mut self, listener: OnWebRtcMessageHdlrFn) {
        self.message_listeners.lock().await.push(listener);
    }

    pub async fn try_connect(&mut self) -> Result<(), WebRtcLinkError> {
        info!("Connecting to WebRTC server at {}", self.signaling_url);

        let connector: Option<Connector> = if self.allow_skip_cert_check {
            let config = ClientConfig::builder()
                .with_root_certificates(RootCertStore::empty())
                .with_no_client_auth()
                .dangerous()
                .set_certificate_verifier(insecure_verifier());
            Some(Connector::Rustls(Arc::new(config)))
        } else {
            None
        };

        let (ws_stream, _) =
            connect_async_tls_with_config(&self.signaling_url, None, false, connector)
                .await
                .map_err(|_| WebRtcLinkError::ConnectionAttemptFailed)?;
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

        self.ws_write.lock().await.replace(ws_write);
        self.ws_read.lock().await.replace(ws_read);

        // Setup callback methods for peer connection
        let write_clone = Arc::clone(&self.ws_write);
        peer_connection.on_ice_candidate(Box::new(move |c: Option<RTCIceCandidate>| {
            let write_clone = Arc::clone(&write_clone);

            Box::pin(async move {
                if let Some(candidate) = c
                    && let Some(ws_write) = &mut *write_clone.lock().await
                {
                    match candidate.to_json() {
                        Ok(json_candidate) => {
                            let msg = serde_json::json!({
                                "type": "candidate",
                                "from": "robot1",
                                "to": "browser1",
                                "candidate": json_candidate
                            });

                            if let Err(e) = ws_write
                                .send(tokio_tungstenite::tungstenite::Message::Text(
                                    serde_json::to_string(&msg).unwrap(),
                                ))
                                .await
                            {
                                warn!("❌ Failed to send ICE candidate: {e}");
                            } else {
                                debug!("✅ Sent ICE candidate to browser");
                            }
                        }
                        Err(e) => {
                            error!("❌ Failed to convert ICE candidate to JSON: {e}");
                        }
                    }
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
                    *status_clone.lock().await = WebRtcLinkStatus::Connected;
                }
            })
        }));

        let listeners_clone = Arc::clone(&self.message_listeners);
        let data_channel_clone = Arc::clone(&self.data_channel);
        peer_connection.on_data_channel(Box::new(move |dc| {
            println!("DataChannel opened: {}", dc.label());

            // Every message should be decoded, then used to call message listeners
            let listeners_clone = Arc::clone(&listeners_clone);
            dc.on_message(Box::new(move |msg: DataChannelMessage| {
                debug!("Received data channel message: {:?}", msg);
                let listeners_clone = Arc::clone(&listeners_clone);
                Box::pin(async move {
                    if let Ok(msg) = serde_json::from_slice(&msg.data) {
                        for listener in listeners_clone.lock().await.iter_mut() {
                            tokio::spawn(listener(&msg));
                        }
                    } else {
                        error!("Failed to decode message from WebRTC");
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

        Ok(())
    }

    pub async fn try_register(&mut self) -> Result<(), WebRtcLinkError> {
        if *self.status.lock().await != WebRtcLinkStatus::Unregistered {
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
            ws_write
                .send(Message::Text(
                    serde_json::to_string(&register_msg)
                        .map_err(|_| WebRtcLinkError::EncodeMessageFailure)?,
                ))
                .await
                .map_err(|_| WebRtcLinkError::FailedToSendMessage)?;

            // Assume registration was successful, as no feedback from server yet
            *self.status.lock().await = WebRtcLinkStatus::Registered;
        }

        // Start listening for messages from the server
        // This spawns its own tokio task after startup, so awaiting is safe
        self.listen().await;

        Ok(())
    }

    pub async fn write_frame(&self, frame: Bytes) -> Result<(), WebRtcLinkError> {
        if let Some(video) = self.video_track.lock().await.as_ref() {
            video
                .write_sample(&Sample {
                    data: Bytes::copy_from_slice(&frame),
                    duration: Duration::from_millis(33),
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
        // Listen should never be called before self.ws_read is valid
        if self.ws_read.lock().await.is_none() {
            panic!("Listen called before ws_read is valid!");
        }

        let write_clone = Arc::clone(&self.ws_write);
        let read_clone = Arc::clone(&self.ws_read);
        let video_track_clone = Arc::clone(&self.video_track);
        let peer_connection_clone = Arc::clone(&self.peer_connection);
        let robot_id = self.local_id.clone();
        tokio::spawn(async move {
            while let Some(Ok(msg)) = read_clone.lock().await.as_mut().unwrap().next().await {
                if let Message::Text(txt) = msg
                    && let Ok(signal) = serde_json::from_str::<SignalMessage>(&txt)
                {
                    match signal.r#type.as_str() {
                        "offer" => {
                            let sdp = signal.sdp.expect("No sdp found in offer message");
                            let offer = RTCSessionDescription::offer(sdp)
                                .expect("Could not build SDP from offer");

                            let video_track = add_video_track(&peer_connection_clone).await?;
                            video_track_clone.lock().await.replace(video_track);

                            if let Some(pc) = peer_connection_clone.lock().await.as_mut() {
                                pc.set_remote_description(offer)
                                    .await
                                    .map_err(|_| WebRtcLinkError::RemoteDescriptionFailed)?;
                                let answer = pc
                                    .create_answer(None)
                                    .await
                                    .map_err(|_| WebRtcLinkError::LocalDescriptionFailed)?;
                                let answer_msg = serde_json::json!({
                                    "type": "answer",
                                    "from": robot_id,
                                    "to": signal.from,
                                    "sdp": answer.sdp,
                                });
                                pc.set_local_description(answer)
                                    .await
                                    .map_err(|_| WebRtcLinkError::LocalDescriptionFailed)?;

                                if let Some(w) = write_clone.lock().await.as_mut() {
                                    let encoded = serde_json::to_string(&answer_msg)
                                        .map_err(|_| WebRtcLinkError::EncodeMessageFailure)?;
                                    w.send(Message::Text(encoded))
                                        .await
                                        .map_err(|_| WebRtcLinkError::FailedToSendMessage)?;
                                }
                            }
                        }
                        "candidate" => {
                            if let Some(candidate_json) = signal.candidate {
                                let candidate: RTCIceCandidateInit =
                                    serde_json::from_value(candidate_json)
                                        .map_err(|_| WebRtcLinkError::ParseMessageFailure)?;
                                if let Some(pc) = peer_connection_clone.lock().await.as_mut() {
                                    pc.add_ice_candidate(candidate)
                                        .await
                                        .map_err(|_| WebRtcLinkError::AddIceCandidateFailed)?;
                                }
                            }
                        }
                        _ => {
                            return Err(WebRtcLinkError::UnrecognisedMessage);
                        }
                    }
                }
            }

            Ok::<(), WebRtcLinkError>(())
        });
    }
}

#[derive(Serialize, Deserialize)]
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
