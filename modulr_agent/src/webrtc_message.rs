// Modulr Agent JSON Schema Implementation

use chrono::Utc;
use log::{debug, info};
use serde::{Deserialize, Serialize};
use thiserror::Error;
use uuid::Uuid;

pub const PROTOCOL_VERSION: &str = "0.0";
pub const SUPPORTED_VERSIONS: &[&str] = &["0.0"];

#[derive(Serialize, Deserialize, Clone, Debug)]
#[serde(rename_all = "camelCase")]
pub struct MessageEnvelope {
    #[serde(rename = "type")]
    pub message_type: String,
    pub version: String,
    pub id: String,
    pub timestamp: String,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub correlation_id: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub payload: Option<serde_json::Value>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub meta: Option<serde_json::Value>,
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub struct PongPayload {
    pub correlation_id: String,
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub struct MovementPayload {
    pub forward: f64,
    pub turn: f64,
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub struct CapabilitiesPayload {
    pub versions: Vec<String>,
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub struct ErrorPayload {
    pub code: ErrorCode,
    pub message: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub details: Option<serde_json::Value>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub correlation_id: Option<String>,
}

pub trait MessageFields {
    fn name(&self) -> String;
    fn correlation_id(&self) -> Option<String>;
    fn payload(&self) -> Option<serde_json::Value>;
    fn meta(&self) -> Option<serde_json::Value>;
}

pub trait ToMessage {
    fn to_message(&self) -> MessageEnvelope;
}

pub type MovementCommand = MovementPayload; //alias to keep naming convention 

impl<T> ToMessage for T
where
    T: MessageFields,
{
    fn to_message(&self) -> MessageEnvelope {
        MessageEnvelope {
            message_type: self.name(),
            version: PROTOCOL_VERSION.to_string(),
            id: generate_id(),
            timestamp: generate_timestamp(),
            correlation_id: self.correlation_id(),
            payload: self.payload(),
            meta: self.meta(),
        }
    }
}

//messages that flow over the WebRTC data channel.
#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub enum AgentMessage {
    Ping,
    Pong(PongPayload),
    Movement(MovementPayload),
    Capabilities(CapabilitiesPayload),
    Error(ErrorPayload),
}

impl MessageFields for AgentMessage {
    fn name(&self) -> String {
        match self {
            AgentMessage::Ping => "agent.ping",
            AgentMessage::Pong(_) => "agent.pong",
            AgentMessage::Movement(_) => "agent.movement",
            AgentMessage::Capabilities(_) => "agent.capabilities",
            AgentMessage::Error(_) => "agent.error",
        }
        .to_string()
    }

    fn correlation_id(&self) -> Option<String> {
        match self {
            AgentMessage::Pong(p) => Some(p.correlation_id.clone()),
            AgentMessage::Error(e) => e.correlation_id.clone(),
            AgentMessage::Ping | AgentMessage::Movement(_) | AgentMessage::Capabilities(_) => None,
        }
    }

    fn payload(&self) -> Option<serde_json::Value> {
        match self {
            AgentMessage::Ping | AgentMessage::Pong(_) => None,
            AgentMessage::Movement(m) => Some(serde_json::json!({
                "forward": m.forward,
                "turn": m.turn
            })),
            AgentMessage::Capabilities(c) => Some(serde_json::json!({
                "versions": c.versions
            })),
            AgentMessage::Error(e) => {
                let mut payload = serde_json::json!({
                    "code": e.code,
                    "message": e.message
                });
                if let Some(d) = &e.details {
                    payload["details"] = d.clone();
                }
                Some(payload)
            }
        }
    }

    fn meta(&self) -> Option<serde_json::Value> {
        None
    }
}

impl AgentMessage {
    pub fn pong(correlation_id: &str) -> Self {
        AgentMessage::Pong(PongPayload {
            correlation_id: correlation_id.to_string(),
        })
    }

    pub fn capabilities() -> Self {
        AgentMessage::Capabilities(CapabilitiesPayload {
            versions: SUPPORTED_VERSIONS.iter().map(|s| s.to_string()).collect(),
        })
    }

    #[allow(dead_code)]
    pub fn capabilities_with_versions(versions: Vec<String>) -> Self {
        AgentMessage::Capabilities(CapabilitiesPayload { versions })
    }

    pub fn error(
        code: ErrorCode,
        message: &str,
        correlation_id: Option<&str>,
        details: Option<serde_json::Value>,
    ) -> Self {
        AgentMessage::Error(ErrorPayload {
            code,
            message: message.to_string(),
            correlation_id: correlation_id.map(String::from),
            details,
        })
    }

    /// handles all agent.* types: ping, pong, movement, capabilities, error.
    pub fn from_message(msg: &MessageEnvelope) -> Result<Self, MessageParseError> {
        validate_envelope(msg)?;

        match msg.message_type.as_str() {
            "agent.ping" => Ok(AgentMessage::Ping),

            "agent.pong" => {
                let correlation_id =
                    msg.correlation_id
                        .clone()
                        .ok_or(MessageParseError::MissingField {
                            field: "correlationId".to_string(),
                        })?;
                Ok(AgentMessage::Pong(PongPayload { correlation_id }))
            }

            "agent.movement" => {
                let payload = msg
                    .payload
                    .as_ref()
                    .ok_or(MessageParseError::MissingField {
                        field: "payload".to_string(),
                    })?;
                let forward =
                    payload["forward"]
                        .as_f64()
                        .ok_or(MessageParseError::MissingField {
                            field: "forward".to_string(),
                        })?;
                let turn = payload["turn"]
                    .as_f64()
                    .ok_or(MessageParseError::MissingField {
                        field: "turn".to_string(),
                    })?;

                if !(-1.0..=1.0).contains(&forward) {
                    return Err(MessageParseError::OutOfRange {
                        field: "forward".to_string(),
                        value: forward.to_string(),
                        expected: "-1.0 to 1.0".to_string(),
                    });
                }
                if !(-1.0..=1.0).contains(&turn) {
                    return Err(MessageParseError::OutOfRange {
                        field: "turn".to_string(),
                        value: turn.to_string(),
                        expected: "-1.0 to 1.0".to_string(),
                    });
                }

                Ok(AgentMessage::Movement(MovementPayload { forward, turn }))
            }

            "agent.capabilities" => {
                let payload = msg
                    .payload
                    .as_ref()
                    .ok_or(MessageParseError::MissingField {
                        field: "payload".to_string(),
                    })?;
                let versions: Vec<String> = payload["versions"]
                    .as_array()
                    .ok_or(MessageParseError::MissingField {
                        field: "versions".to_string(),
                    })?
                    .iter()
                    .filter_map(|v| v.as_str().map(String::from))
                    .collect();
                Ok(AgentMessage::Capabilities(CapabilitiesPayload { versions }))
            }

            "agent.error" => {
                let payload = msg
                    .payload
                    .as_ref()
                    .ok_or(MessageParseError::MissingField {
                        field: "payload".to_string(),
                    })?;
                let code: ErrorCode =
                    serde_json::from_value(payload["code"].clone()).map_err(|_| {
                        MessageParseError::MissingField {
                            field: "code".to_string(),
                        }
                    })?;
                let message = payload["message"]
                    .as_str()
                    .ok_or(MessageParseError::MissingField {
                        field: "message".to_string(),
                    })?
                    .to_string();
                let details = payload.get("details").cloned();
                let correlation_id = msg.correlation_id.clone();

                Ok(AgentMessage::Error(ErrorPayload {
                    code,
                    message,
                    details,
                    correlation_id,
                }))
            }

            _ => Err(MessageParseError::UnknownMessageType {
                message_type: msg.message_type.clone(),
            }),
        }
    }
}

pub fn validate_envelope(msg: &MessageEnvelope) -> Result<(), MessageParseError> {
    if msg.message_type.is_empty() {
        return Err(MessageParseError::EnvelopeValidation {
            reason: "missing required field: type".to_string(),
        });
    }
    if msg.version.is_empty() {
        return Err(MessageParseError::EnvelopeValidation {
            reason: "missing required field: version".to_string(),
        });
    }
    if msg.id.is_empty() {
        return Err(MessageParseError::EnvelopeValidation {
            reason: "missing required field: id".to_string(),
        });
    }
    if msg.timestamp.is_empty() {
        return Err(MessageParseError::EnvelopeValidation {
            reason: "missing required field: timestamp".to_string(),
        });
    }

    Ok(())
}

pub fn generate_id() -> String {
    Uuid::new_v4().to_string()
}

pub fn generate_timestamp() -> String {
    Utc::now().to_rfc3339()
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum ErrorCode {
    InvalidMessage,
    UnsupportedVersion,
    ValidationFailed,
    InvalidPayload,
    UnsupportedMessageType,
    MovementFailed,
    AgentUnavailable,
    CapabilityMismatch,
    InternalError,
}

#[derive(Error, Debug, Clone, PartialEq)]
pub enum MessageParseError {
    #[allow(dead_code)]
    #[error("JSON parse error: {reason}")]
    JsonParse { reason: String },

    #[error("missing required field: {field}")]
    MissingField { field: String },

    #[error("value out of range: {field} = {value} (expected {expected})")]
    OutOfRange {
        field: String,
        value: String,
        expected: String,
    },

    #[error("unknown message type: {message_type}")]
    UnknownMessageType { message_type: String },

    #[error("envelope validation failed: {reason}")]
    EnvelopeValidation { reason: String },
}

#[allow(dead_code)]
pub fn is_version_supported(version: &str) -> bool {
    SUPPORTED_VERSIONS.contains(&version)
}

pub fn negotiate_version(remote_versions: &[String]) -> Option<String> {
    let mut compatible: Vec<&str> = remote_versions
        .iter()
        .filter(|v| SUPPORTED_VERSIONS.contains(&v.as_str()))
        .map(|v| v.as_str())
        .collect();

    compatible.sort();
    compatible.last().map(|s| s.to_string())
}

#[allow(dead_code)]
pub fn parse_message(json_str: &str) -> Result<MessageEnvelope, MessageParseError> {
    serde_json::from_str(json_str).map_err(|e| MessageParseError::JsonParse {
        reason: e.to_string(),
    })
}

pub fn handle_message(envelope: &MessageEnvelope) -> Option<MessageEnvelope> {
    match AgentMessage::from_message(envelope) {
        Ok(msg) => match msg {
            AgentMessage::Ping => Some(AgentMessage::pong(&envelope.id).to_message()),
            AgentMessage::Movement(payload) => {
                debug!(
                    "Movement command: forward={}, turn={}",
                    payload.forward, payload.turn
                );
                None
            }
            AgentMessage::Capabilities(payload) => {
                info!("Received capabilities: versions={:?}", payload.versions);
                if let Some(version) = negotiate_version(&payload.versions) {
                    info!("Negotiated version: {}", version);
                    Some(AgentMessage::capabilities().to_message())
                } else {
                    Some(
                        AgentMessage::error(
                            ErrorCode::CapabilityMismatch,
                            "No compatible protocol version found",
                            Some(&envelope.id),
                            None,
                        )
                        .to_message(),
                    )
                }
            }
            // Pong and error are outbound only
            _ => None,
        },

        Err(e) => {
            let code = match &e {
                MessageParseError::UnknownMessageType { .. } => ErrorCode::UnsupportedMessageType,
                MessageParseError::OutOfRange { .. } => ErrorCode::ValidationFailed,
                MessageParseError::EnvelopeValidation { .. } => ErrorCode::InvalidMessage,
                MessageParseError::MissingField { .. } | MessageParseError::JsonParse { .. } => {
                    ErrorCode::InvalidPayload
                }
            };
            Some(AgentMessage::error(code, &e.to_string(), Some(&envelope.id), None).to_message())
        }
    }
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct RegisterPayload {
    pub agent_id: String,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub capabilities: Option<serde_json::Value>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub metadata: Option<serde_json::Value>,
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct OfferPayload {
    pub connection_id: String,
    pub sdp: String,
    pub sdp_type: String,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub ice_restart: Option<bool>,
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct AnswerPayload {
    pub connection_id: String,
    pub sdp: String,
    pub sdp_type: String,
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct IceCandidatePayload {
    pub connection_id: String,
    pub candidate: String,
    pub sdp_mid: String,
    pub sdp_m_line_index: u32,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub username_fragment: Option<String>,
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct ConnectedPayload {
    pub connection_id: String,
    pub ice_connection_state: String,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub data_channel_state: Option<String>,
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct DisconnectedPayload {
    pub connection_id: String,
    pub reason: String,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub ice_connection_state: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub details: Option<serde_json::Value>,
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum SignallingErrorCode {
    InvalidMessage,
    UnsupportedVersion,
    ValidationFailed,
    InvalidPayload,
    UnsupportedMessageType,
    ConnectionFailed,
    Unauthorized,
    Forbidden,
    Timeout,
    CapabilityMismatch,
    IceFailed,
    SdpInvalid,
    InternalError,
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub struct SignallingErrorPayload {
    pub code: SignallingErrorCode,
    pub message: String,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub details: Option<serde_json::Value>,
}

#[derive(Clone, Debug)]
pub enum SignallingMessage {
    Register(RegisterPayload),
    Offer(OfferPayload),
    Answer(AnswerPayload),
    IceCandidate(IceCandidatePayload),
    Capabilities(CapabilitiesPayload),
    Connected(ConnectedPayload),
    Disconnected(DisconnectedPayload),
    Error(SignallingErrorPayload),
}

impl SignallingMessage {
    pub fn from_message(msg: &MessageEnvelope) -> Result<Self, MessageParseError> {
        validate_envelope(msg)?;

        match msg.message_type.as_str() {
            "signalling.register" => {
                let payload = msg
                    .payload
                    .as_ref()
                    .ok_or(MessageParseError::MissingField {
                        field: "payload".to_string(),
                    })?;
                let register: RegisterPayload =
                    serde_json::from_value(payload.clone()).map_err(|e| {
                        MessageParseError::JsonParse {
                            reason: e.to_string(),
                        }
                    })?;
                Ok(SignallingMessage::Register(register))
            }

            "signalling.offer" => {
                let payload = msg
                    .payload
                    .as_ref()
                    .ok_or(MessageParseError::MissingField {
                        field: "payload".to_string(),
                    })?;
                let offer: OfferPayload = serde_json::from_value(payload.clone()).map_err(|e| {
                    MessageParseError::JsonParse {
                        reason: e.to_string(),
                    }
                })?;
                Ok(SignallingMessage::Offer(offer))
            }

            "signalling.answer" => {
                let payload = msg
                    .payload
                    .as_ref()
                    .ok_or(MessageParseError::MissingField {
                        field: "payload".to_string(),
                    })?;
                let answer: AnswerPayload =
                    serde_json::from_value(payload.clone()).map_err(|e| {
                        MessageParseError::JsonParse {
                            reason: e.to_string(),
                        }
                    })?;
                Ok(SignallingMessage::Answer(answer))
            }

            "signalling.ice_candidate" => {
                let payload = msg
                    .payload
                    .as_ref()
                    .ok_or(MessageParseError::MissingField {
                        field: "payload".to_string(),
                    })?;
                let candidate: IceCandidatePayload = serde_json::from_value(payload.clone())
                    .map_err(|e| MessageParseError::JsonParse {
                        reason: e.to_string(),
                    })?;
                Ok(SignallingMessage::IceCandidate(candidate))
            }

            "signalling.capabilities" => {
                let payload = msg
                    .payload
                    .as_ref()
                    .ok_or(MessageParseError::MissingField {
                        field: "payload".to_string(),
                    })?;
                let versions: Vec<String> = payload["versions"]
                    .as_array()
                    .ok_or(MessageParseError::MissingField {
                        field: "versions".to_string(),
                    })?
                    .iter()
                    .filter_map(|v| v.as_str().map(String::from))
                    .collect();
                Ok(SignallingMessage::Capabilities(CapabilitiesPayload {
                    versions,
                }))
            }

            "signalling.connected" => {
                let payload = msg
                    .payload
                    .as_ref()
                    .ok_or(MessageParseError::MissingField {
                        field: "payload".to_string(),
                    })?;
                let connected: ConnectedPayload =
                    serde_json::from_value(payload.clone()).map_err(|e| {
                        MessageParseError::JsonParse {
                            reason: e.to_string(),
                        }
                    })?;
                Ok(SignallingMessage::Connected(connected))
            }

            "signalling.disconnected" => {
                let payload = msg
                    .payload
                    .as_ref()
                    .ok_or(MessageParseError::MissingField {
                        field: "payload".to_string(),
                    })?;
                let disconnected: DisconnectedPayload = serde_json::from_value(payload.clone())
                    .map_err(|e| MessageParseError::JsonParse {
                        reason: e.to_string(),
                    })?;
                Ok(SignallingMessage::Disconnected(disconnected))
            }

            "signalling.error" => {
                let payload = msg
                    .payload
                    .as_ref()
                    .ok_or(MessageParseError::MissingField {
                        field: "payload".to_string(),
                    })?;
                let error: SignallingErrorPayload = serde_json::from_value(payload.clone())
                    .map_err(|e| MessageParseError::JsonParse {
                        reason: e.to_string(),
                    })?;
                Ok(SignallingMessage::Error(error))
            }

            _ => Err(MessageParseError::UnknownMessageType {
                message_type: msg.message_type.clone(),
            }),
        }
    }
}

impl MessageFields for SignallingMessage {
    fn name(&self) -> String {
        match self {
            SignallingMessage::Register(_) => "signalling.register",
            SignallingMessage::Offer(_) => "signalling.offer",
            SignallingMessage::Answer(_) => "signalling.answer",
            SignallingMessage::IceCandidate(_) => "signalling.ice_candidate",
            SignallingMessage::Capabilities(_) => "signalling.capabilities",
            SignallingMessage::Connected(_) => "signalling.connected",
            SignallingMessage::Disconnected(_) => "signalling.disconnected",
            SignallingMessage::Error(_) => "signalling.error",
        }
        .to_string()
    }

    fn correlation_id(&self) -> Option<String> {
        None
    }

    fn payload(&self) -> Option<serde_json::Value> {
        match self {
            SignallingMessage::Register(p) => serde_json::to_value(p).ok(),
            SignallingMessage::Offer(p) => serde_json::to_value(p).ok(),
            SignallingMessage::Answer(p) => serde_json::to_value(p).ok(),
            SignallingMessage::IceCandidate(p) => serde_json::to_value(p).ok(),
            SignallingMessage::Capabilities(p) => serde_json::to_value(p).ok(),
            SignallingMessage::Connected(p) => serde_json::to_value(p).ok(),
            SignallingMessage::Disconnected(p) => serde_json::to_value(p).ok(),
            SignallingMessage::Error(p) => serde_json::to_value(p).ok(),
        }
    }

    fn meta(&self) -> Option<serde_json::Value> {
        None
    }
}

impl SignallingMessage {
    pub fn register(agent_id: &str) -> Self {
        SignallingMessage::Register(RegisterPayload {
            agent_id: agent_id.to_string(),
            capabilities: None,
            metadata: None,
        })
    }

    pub fn answer(connection_id: &str, sdp: &str) -> Self {
        SignallingMessage::Answer(AnswerPayload {
            connection_id: connection_id.to_string(),
            sdp: sdp.to_string(),
            sdp_type: "answer".to_string(),
        })
    }

    pub fn ice_candidate(
        connection_id: &str,
        candidate: &str,
        sdp_mid: &str,
        sdp_m_line_index: u32,
    ) -> Self {
        SignallingMessage::IceCandidate(IceCandidatePayload {
            connection_id: connection_id.to_string(),
            candidate: candidate.to_string(),
            sdp_mid: sdp_mid.to_string(),
            sdp_m_line_index,
            username_fragment: None,
        })
    }

    #[allow(dead_code)]
    pub fn capabilities() -> Self {
        SignallingMessage::Capabilities(CapabilitiesPayload {
            versions: vec!["0.0".to_string()],
        })
    }

    #[allow(dead_code)]
    pub fn connected(connection_id: &str, ice_connection_state: &str) -> Self {
        SignallingMessage::Connected(ConnectedPayload {
            connection_id: connection_id.to_string(),
            ice_connection_state: ice_connection_state.to_string(),
            data_channel_state: None,
        })
    }

    #[allow(dead_code)]
    pub fn disconnected(connection_id: &str, reason: &str) -> Self {
        SignallingMessage::Disconnected(DisconnectedPayload {
            connection_id: connection_id.to_string(),
            reason: reason.to_string(),
            ice_connection_state: None,
            details: None,
        })
    }

    #[allow(dead_code)]
    pub fn error(code: SignallingErrorCode, message: &str) -> Self {
        SignallingMessage::Error(SignallingErrorPayload {
            code,
            message: message.to_string(),
            details: None,
        })
    }
}

//Unit Tests
//agent messege tests
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pong_payload() {
        let payload = PongPayload {
            correlation_id: "test-123".to_string(),
        };
        assert_eq!(payload.correlation_id, "test-123");
    }

    #[test]
    fn test_movement_payload() {
        let payload = MovementPayload {
            forward: 0.5,
            turn: -0.3,
        };
        assert_eq!(payload.forward, 0.5);
        assert_eq!(payload.turn, -0.3);
    }

    #[test]
    fn test_capabilities_payload() {
        let payload = CapabilitiesPayload {
            versions: vec!["0.0".to_string(), "0.1".to_string()],
        };
        assert_eq!(payload.versions.len(), 2);
    }

    #[test]
    fn test_error_payload() {
        let payload = ErrorPayload {
            code: ErrorCode::InvalidPayload,
            message: "Test error".to_string(),
            details: None,
            correlation_id: Some("msg-123".to_string()),
        };
        assert_eq!(payload.code, ErrorCode::InvalidPayload);
        assert_eq!(payload.message, "Test error");
    }

    #[test]
    fn test_agent_message_pong() {
        let msg = AgentMessage::pong("ping-123");
        if let AgentMessage::Pong(payload) = msg {
            assert_eq!(payload.correlation_id, "ping-123");
        } else {
            panic!("Expected Pong variant");
        }
    }

    #[test]
    fn test_agent_message_pong_to_message() {
        let envelope = AgentMessage::pong("ping-123").to_message();
        assert_eq!(envelope.message_type, "agent.pong");
        assert_eq!(envelope.correlation_id, Some("ping-123".to_string()));
        assert!(envelope.payload.is_none());
    }

    #[test]
    fn test_agent_message_capabilities() {
        let msg = AgentMessage::capabilities();
        if let AgentMessage::Capabilities(payload) = msg {
            assert!(payload.versions.contains(&"0.0".to_string()));
        } else {
            panic!("Expected Capabilities variant");
        }
    }

    #[test]
    fn test_agent_message_capabilities_to_message() {
        let envelope = AgentMessage::capabilities().to_message();
        assert_eq!(envelope.message_type, "agent.capabilities");
        assert!(envelope.payload.is_some());
    }

    #[test]
    fn test_agent_message_error() {
        let msg = AgentMessage::error(
            ErrorCode::MovementFailed,
            "Robot stuck",
            Some("cmd-123"),
            None,
        );
        if let AgentMessage::Error(payload) = msg {
            assert_eq!(payload.code, ErrorCode::MovementFailed);
            assert_eq!(payload.message, "Robot stuck");
            assert_eq!(payload.correlation_id, Some("cmd-123".to_string()));
        } else {
            panic!("Expected Error variant");
        }
    }

    #[test]
    fn test_agent_message_error_to_message() {
        let envelope =
            AgentMessage::error(ErrorCode::InvalidPayload, "Bad data", Some("msg-456"), None)
                .to_message();
        assert_eq!(envelope.message_type, "agent.error");
        assert_eq!(envelope.correlation_id, Some("msg-456".to_string()));
        assert!(envelope.payload.is_some());
    }

    #[test]
    fn test_agent_message_parse_ping() {
        let envelope = MessageEnvelope {
            message_type: "agent.ping".to_string(),
            version: "0.0".to_string(),
            id: "test-id".to_string(),
            timestamp: "2024-01-01T00:00:00Z".to_string(),
            correlation_id: None,
            payload: None,
            meta: None,
        };
        let msg = AgentMessage::from_message(&envelope).unwrap();
        assert!(matches!(msg, AgentMessage::Ping));
    }

    #[test]
    fn test_agent_message_parse_movement() {
        let envelope = MessageEnvelope {
            message_type: "agent.movement".to_string(),
            version: "0.0".to_string(),
            id: "test-id".to_string(),
            timestamp: "2024-01-01T00:00:00Z".to_string(),
            correlation_id: None,
            payload: Some(serde_json::json!({"forward": 0.5, "turn": -0.3})),
            meta: None,
        };
        let msg = AgentMessage::from_message(&envelope).unwrap();
        if let AgentMessage::Movement(payload) = msg {
            assert_eq!(payload.forward, 0.5);
            assert_eq!(payload.turn, -0.3);
        } else {
            panic!("Expected Movement variant");
        }
    }

    #[test]
    fn test_agent_message_parse_movement_out_of_range() {
        let envelope = MessageEnvelope {
            message_type: "agent.movement".to_string(),
            version: "0.0".to_string(),
            id: "test-id".to_string(),
            timestamp: "2024-01-01T00:00:00Z".to_string(),
            correlation_id: None,
            payload: Some(serde_json::json!({"forward": 1.5, "turn": 0.0})),
            meta: None,
        };
        let result = AgentMessage::from_message(&envelope);
        assert!(result.is_err());
        assert!(matches!(
            result.unwrap_err(),
            MessageParseError::OutOfRange { .. }
        ));
    }

    #[test]
    fn test_agent_message_parse_capabilities() {
        let envelope = MessageEnvelope {
            message_type: "agent.capabilities".to_string(),
            version: "0.0".to_string(),
            id: "test-id".to_string(),
            timestamp: "2024-01-01T00:00:00Z".to_string(),
            correlation_id: None,
            payload: Some(serde_json::json!({"versions": ["0.0", "0.1"]})),
            meta: None,
        };
        let msg = AgentMessage::from_message(&envelope).unwrap();
        if let AgentMessage::Capabilities(payload) = msg {
            assert_eq!(payload.versions.len(), 2);
            assert!(payload.versions.contains(&"0.0".to_string()));
        } else {
            panic!("Expected Capabilities variant");
        }
    }

    #[test]
    fn test_agent_message_parse_unknown() {
        let envelope = MessageEnvelope {
            message_type: "agent.unknown".to_string(),
            version: "0.0".to_string(),
            id: "test-id".to_string(),
            timestamp: "2024-01-01T00:00:00Z".to_string(),
            correlation_id: None,
            payload: None,
            meta: None,
        };
        let result = AgentMessage::from_message(&envelope);
        assert!(result.is_err());
    }

    #[test]
    fn test_handle_ping_returns_pong() {
        let ping = MessageEnvelope {
            message_type: "agent.ping".to_string(),
            version: "0.0".to_string(),
            id: "ping-123".to_string(),
            timestamp: "2024-01-01T00:00:00Z".to_string(),
            correlation_id: None,
            payload: None,
            meta: None,
        };
        let response = handle_message(&ping);
        assert!(response.is_some());
        let pong = response.unwrap();
        assert_eq!(pong.message_type, "agent.pong");
        assert_eq!(pong.correlation_id, Some("ping-123".to_string()));
    }

    #[test]
    fn test_handle_movement_returns_none() {
        let movement = MessageEnvelope {
            message_type: "agent.movement".to_string(),
            version: "0.0".to_string(),
            id: "move-123".to_string(),
            timestamp: "2024-01-01T00:00:00Z".to_string(),
            correlation_id: None,
            payload: Some(serde_json::json!({"forward": 0.5, "turn": 0.3})),
            meta: None,
        };
        let response = handle_message(&movement);
        assert!(response.is_none());
    }

    #[test]
    fn test_handle_capabilities_returns_capabilities() {
        let caps = MessageEnvelope {
            message_type: "agent.capabilities".to_string(),
            version: "0.0".to_string(),
            id: "caps-123".to_string(),
            timestamp: "2024-01-01T00:00:00Z".to_string(),
            correlation_id: None,
            payload: Some(serde_json::json!({"versions": ["0.0"]})),
            meta: None,
        };
        let response = handle_message(&caps);
        assert!(response.is_some());
        let our_caps = response.unwrap();
        assert_eq!(our_caps.message_type, "agent.capabilities");
    }

    #[test]
    fn test_handle_capabilities_no_match() {
        let caps = MessageEnvelope {
            message_type: "agent.capabilities".to_string(),
            version: "0.0".to_string(),
            id: "caps-123".to_string(),
            timestamp: "2024-01-01T00:00:00Z".to_string(),
            correlation_id: None,
            payload: Some(serde_json::json!({"versions": ["9.9"]})),
            meta: None,
        };
        let response = handle_message(&caps);
        assert!(response.is_some());
        let error = response.unwrap();
        assert_eq!(error.message_type, "agent.error");
    }

    #[test]
    fn test_handle_unknown_returns_error() {
        let unknown = MessageEnvelope {
            message_type: "agent.unknown".to_string(),
            version: "0.0".to_string(),
            id: "unknown-123".to_string(),
            timestamp: "2024-01-01T00:00:00Z".to_string(),
            correlation_id: None,
            payload: None,
            meta: None,
        };
        let response = handle_message(&unknown);
        assert!(response.is_some());
        let error = response.unwrap();
        assert_eq!(error.message_type, "agent.error");
    }

    #[test]
    fn test_validate_envelope_valid() {
        let envelope = MessageEnvelope {
            message_type: "agent.ping".to_string(),
            version: "0.0".to_string(),
            id: "test-id".to_string(),
            timestamp: "2024-01-01T00:00:00Z".to_string(),
            correlation_id: None,
            payload: None,
            meta: None,
        };
        assert!(validate_envelope(&envelope).is_ok());
    }

    #[test]
    fn test_validate_envelope_missing_type() {
        let envelope = MessageEnvelope {
            message_type: "".to_string(),
            version: "0.0".to_string(),
            id: "test-id".to_string(),
            timestamp: "2024-01-01T00:00:00Z".to_string(),
            correlation_id: None,
            payload: None,
            meta: None,
        };
        let result = validate_envelope(&envelope);
        assert!(result.is_err());
        assert!(matches!(
            result.unwrap_err(),
            MessageParseError::EnvelopeValidation { .. }
        ));
    }

    #[test]
    fn test_negotiate_version_match() {
        let remote = vec!["0.0".to_string(), "0.1".to_string()];
        let result = negotiate_version(&remote);
        assert_eq!(result, Some("0.0".to_string()));
    }

    #[test]
    fn test_negotiate_version_no_match() {
        let remote = vec!["1.0".to_string(), "2.0".to_string()];
        let result = negotiate_version(&remote);
        assert_eq!(result, None);
    }

    #[test]
    fn test_is_version_supported() {
        assert!(is_version_supported("0.0"));
        assert!(!is_version_supported("1.0"));
    }

    #[test]
    fn test_error_code_serialization() {
        let code = ErrorCode::MovementFailed;
        let json = serde_json::to_string(&code).unwrap();
        assert_eq!(json, "\"MOVEMENT_FAILED\"");
    }

    #[test]
    fn test_error_code_deserialization() {
        let json = "\"VALIDATION_FAILED\"";
        let code: ErrorCode = serde_json::from_str(json).unwrap();
        assert_eq!(code, ErrorCode::ValidationFailed);
    }

    #[test]
    fn test_json_field_names() {
        let pong = AgentMessage::pong("test").to_message();
        let json = serde_json::to_string(&pong).unwrap();
        assert!(json.contains("\"type\""));
        assert!(json.contains("\"correlationId\""));
        assert!(!json.contains("\"message_type\""));
        assert!(!json.contains("\"correlation_id\""));
    }

    #[test]
    fn test_parse_message_valid() {
        let json = r#"{
            "type": "agent.ping",
            "version": "0.0",
            "id": "test-id",
            "timestamp": "2024-01-01T00:00:00Z"
        }"#;
        let result = parse_message(json);
        assert!(result.is_ok());
        assert_eq!(result.unwrap().message_type, "agent.ping");
    }

    #[test]
    fn test_parse_message_invalid() {
        let json = "{ invalid json }";
        let result = parse_message(json);
        assert!(result.is_err());
    }

    // signalling message tests
    #[test]
    fn test_register_payload_serialization() {
        let payload = RegisterPayload {
            agent_id: "robot-001".to_string(),
            capabilities: None,
            metadata: None,
        };
        let json = serde_json::to_string(&payload).unwrap();
        assert!(json.contains("\"agentId\":\"robot-001\""));
        assert!(!json.contains("\"agent_id\"")); // Verify camelCase rename
    }

    #[test]
    fn test_signalling_message_register_to_message() {
        let envelope = SignallingMessage::register("robot-001").to_message();
        assert_eq!(envelope.message_type, "signalling.register");
        assert_eq!(envelope.version, "0.0");
        let payload = envelope.payload.unwrap();
        assert_eq!(payload["agentId"], "robot-001");
    }

    #[test]
    fn test_signalling_message_answer_to_message() {
        let envelope = SignallingMessage::answer("browser-456", "v=0\r\ntest-sdp").to_message();
        assert_eq!(envelope.message_type, "signalling.answer");
        let payload = envelope.payload.unwrap();
        assert_eq!(payload["connectionId"], "browser-456");
        assert_eq!(payload["sdp"], "v=0\r\ntest-sdp");
        assert_eq!(payload["sdpType"], "answer");
    }

    #[test]
    fn test_signalling_message_ice_candidate_to_message() {
        let envelope =
            SignallingMessage::ice_candidate("browser-456", "candidate:1 1 UDP ...", "0", 0)
                .to_message();
        assert_eq!(envelope.message_type, "signalling.ice_candidate");
        let payload = envelope.payload.unwrap();
        assert_eq!(payload["connectionId"], "browser-456");
        assert_eq!(payload["candidate"], "candidate:1 1 UDP ...");
        assert_eq!(payload["sdpMid"], "0");
        assert_eq!(payload["sdpMLineIndex"], 0);
    }

    #[test]
    fn test_signalling_message_connected_to_message() {
        let envelope = SignallingMessage::connected("browser-456", "connected").to_message();
        assert_eq!(envelope.message_type, "signalling.connected");
        let payload = envelope.payload.unwrap();
        assert_eq!(payload["connectionId"], "browser-456");
        assert_eq!(payload["iceConnectionState"], "connected");
    }

    #[test]
    fn test_signalling_message_disconnected_to_message() {
        let envelope = SignallingMessage::disconnected("browser-456", "failed").to_message();
        assert_eq!(envelope.message_type, "signalling.disconnected");
        let payload = envelope.payload.unwrap();
        assert_eq!(payload["connectionId"], "browser-456");
        assert_eq!(payload["reason"], "failed");
    }

    #[test]
    fn test_signalling_message_parse_offer() {
        let envelope = MessageEnvelope {
            message_type: "signalling.offer".to_string(),
            version: "0.0".to_string(),
            id: "test-id".to_string(),
            timestamp: "2024-01-01T00:00:00Z".to_string(),
            correlation_id: None,
            payload: Some(serde_json::json!({
                "connectionId": "browser-123",
                "sdp": "v=0\r\ntest-sdp",
                "sdpType": "offer"
            })),
            meta: None,
        };
        let msg = SignallingMessage::from_message(&envelope).unwrap();
        if let SignallingMessage::Offer(offer) = msg {
            assert_eq!(offer.connection_id, "browser-123");
            assert_eq!(offer.sdp, "v=0\r\ntest-sdp");
            assert_eq!(offer.sdp_type, "offer");
        } else {
            panic!("Expected Offer variant");
        }
    }

    #[test]
    fn test_signalling_message_parse_ice_candidate() {
        let envelope = MessageEnvelope {
            message_type: "signalling.ice_candidate".to_string(),
            version: "0.0".to_string(),
            id: "test-id".to_string(),
            timestamp: "2024-01-01T00:00:00Z".to_string(),
            correlation_id: None,
            payload: Some(serde_json::json!({
                "connectionId": "browser-123",
                "candidate": "candidate:1 1 UDP 2130706431 192.168.1.100 54321 typ host",
                "sdpMid": "0",
                "sdpMLineIndex": 0
            })),
            meta: None,
        };
        let msg = SignallingMessage::from_message(&envelope).unwrap();
        if let SignallingMessage::IceCandidate(ice) = msg {
            assert_eq!(ice.connection_id, "browser-123");
            assert_eq!(ice.sdp_mid, "0");
            assert_eq!(ice.sdp_m_line_index, 0);
        } else {
            panic!("Expected IceCandidate variant");
        }
    }

    #[test]
    fn test_signalling_message_capabilities_to_message() {
        let envelope = SignallingMessage::capabilities().to_message();
        assert_eq!(envelope.message_type, "signalling.capabilities");
        let payload = envelope.payload.unwrap();
        assert!(
            payload["versions"]
                .as_array()
                .unwrap()
                .contains(&serde_json::json!("0.0"))
        );
    }

    #[test]
    fn test_signalling_message_parse_capabilities() {
        let envelope = MessageEnvelope {
            message_type: "signalling.capabilities".to_string(),
            version: "0.0".to_string(),
            id: "test-id".to_string(),
            timestamp: "2024-01-01T00:00:00Z".to_string(),
            correlation_id: None,
            payload: Some(serde_json::json!({"versions": ["0.0", "0.1"]})),
            meta: None,
        };
        let msg = SignallingMessage::from_message(&envelope).unwrap();
        if let SignallingMessage::Capabilities(cap) = msg {
            assert_eq!(cap.versions.len(), 2);
            assert!(cap.versions.contains(&"0.0".to_string()));
        } else {
            panic!("Expected Capabilities variant");
        }
    }

    #[test]
    fn test_signalling_message_parse_unknown_type() {
        let envelope = MessageEnvelope {
            message_type: "signalling.unknown".to_string(),
            version: "0.0".to_string(),
            id: "test-id".to_string(),
            timestamp: "2024-01-01T00:00:00Z".to_string(),
            correlation_id: None,
            payload: None,
            meta: None,
        };
        let result = SignallingMessage::from_message(&envelope);
        assert!(result.is_err());
        assert!(matches!(
            result.unwrap_err(),
            MessageParseError::UnknownMessageType { .. }
        ));
    }

    #[test]
    fn test_signalling_message_parse_missing_payload() {
        let envelope = MessageEnvelope {
            message_type: "signalling.offer".to_string(),
            version: "0.0".to_string(),
            id: "test-id".to_string(),
            timestamp: "2024-01-01T00:00:00Z".to_string(),
            correlation_id: None,
            payload: None,
            meta: None,
        };
        let result = SignallingMessage::from_message(&envelope);
        assert!(result.is_err());
        assert!(matches!(
            result.unwrap_err(),
            MessageParseError::MissingField { .. }
        ));
    }

    #[test]
    fn test_signalling_error_code_serialization() {
        let code = SignallingErrorCode::IceFailed;
        let json = serde_json::to_string(&code).unwrap();
        assert_eq!(json, "\"ICE_FAILED\"");
    }

    #[test]
    fn test_signalling_json_field_names() {
        let envelope =
            SignallingMessage::ice_candidate("browser-456", "candidate:...", "0", 0).to_message();
        let json = serde_json::to_string(&envelope).unwrap();
        assert!(json.contains("\"connectionId\""));
        assert!(json.contains("\"sdpMid\""));
        assert!(json.contains("\"sdpMLineIndex\""));
        assert!(!json.contains("\"connection_id\""));
        assert!(!json.contains("\"sdp_mid\""));
    }
}
