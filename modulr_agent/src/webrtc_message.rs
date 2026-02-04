// Modulr Agent JSON Schema Implementation

use chrono::Utc;
use serde::{Deserialize, Serialize};
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

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub enum AgentMessage {
    Pong(PongPayload),
    Capabilities(CapabilitiesPayload),
    Error(ErrorPayload),
}

impl MessageFields for AgentMessage {
    fn name(&self) -> String {
        match self {
            AgentMessage::Pong(_) => "agent.pong",
            AgentMessage::Capabilities(_) => "agent.capabilities",
            AgentMessage::Error(_) => "agent.error",
        }
        .to_string()
    }

    fn correlation_id(&self) -> Option<String> {
        match self {
            AgentMessage::Pong(p) => Some(p.correlation_id.clone()),
            AgentMessage::Error(e) => e.correlation_id.clone(),
            AgentMessage::Capabilities(_) => None,
        }
    }

    fn payload(&self) -> Option<serde_json::Value> {
        match self {
            AgentMessage::Pong(_) => None,
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
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub enum SignalingMessage {
    Ping,
    Movement(MovementPayload),
    Capabilities(CapabilitiesPayload),
}

impl SignalingMessage {
    pub fn from_message(msg: &MessageEnvelope) -> Result<Self, String> {
        validate_envelope(msg)?;

        match msg.message_type.as_str() {
            "agent.ping" => Ok(SignalingMessage::Ping),

            "agent.movement" => {
                let payload = msg.payload.as_ref().ok_or("movement requires payload")?;
                let forward = payload["forward"]
                    .as_f64()
                    .ok_or("movement requires forward field")?;
                let turn = payload["turn"]
                    .as_f64()
                    .ok_or("movement requires turn field")?;

                if !(-1.0..=1.0).contains(&forward) {
                    return Err(format!("forward out of range: {}", forward));
                }
                if !(-1.0..=1.0).contains(&turn) {
                    return Err(format!("turn out of range: {}", turn));
                }

                Ok(SignalingMessage::Movement(MovementPayload {
                    forward,
                    turn,
                }))
            }

            "agent.capabilities" => {
                let payload = msg
                    .payload
                    .as_ref()
                    .ok_or("capabilities requires payload")?;
                let versions: Vec<String> = payload["versions"]
                    .as_array()
                    .ok_or("capabilities requires versions array")?
                    .iter()
                    .filter_map(|v| v.as_str().map(String::from))
                    .collect();

                Ok(SignalingMessage::Capabilities(CapabilitiesPayload {
                    versions,
                }))
            }

            _ => Err(format!("unknown message type: {}", msg.message_type)),
        }
    }
}

pub fn validate_envelope(msg: &MessageEnvelope) -> Result<(), String> {
    if msg.message_type.is_empty() {
        return Err("Missing required field: type".to_string());
    }
    if msg.version.is_empty() {
        return Err("Missing required field: version".to_string());
    }
    if msg.id.is_empty() {
        return Err("Missing required field: id".to_string());
    }
    if msg.timestamp.is_empty() {
        return Err("Missing required field: timestamp".to_string());
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

pub fn parse_message(json_str: &str) -> Result<MessageEnvelope, String> {
    serde_json::from_str(json_str).map_err(|e| format!("Failed to parse message: {}", e))
}

pub fn handle_message(envelope: &MessageEnvelope) -> Option<MessageEnvelope> {
    match SignalingMessage::from_message(envelope) {
        Ok(msg) => match msg {
            SignalingMessage::Ping => Some(AgentMessage::pong(&envelope.id).to_message()),
            SignalingMessage::Movement(payload) => {
                println!(
                    "Movement command: forward={}, turn={}",
                    payload.forward, payload.turn
                );
                None
            }
            SignalingMessage::Capabilities(payload) => {
                println!("Received capabilities: versions={:?}", payload.versions);
                if let Some(version) = negotiate_version(&payload.versions) {
                    println!("Negotiated version: {}", version);
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
        },
        Err(e) => Some(
            AgentMessage::error(ErrorCode::InvalidPayload, &e, Some(&envelope.id), None)
                .to_message(),
        ),
    }
}

//Unit Tests
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
    fn test_signaling_message_parse_ping() {
        let envelope = MessageEnvelope {
            message_type: "agent.ping".to_string(),
            version: "0.0".to_string(),
            id: "test-id".to_string(),
            timestamp: "2024-01-01T00:00:00Z".to_string(),
            correlation_id: None,
            payload: None,
            meta: None,
        };
        let msg = SignalingMessage::from_message(&envelope).unwrap();
        assert!(matches!(msg, SignalingMessage::Ping));
    }

    #[test]
    fn test_signaling_message_parse_movement() {
        let envelope = MessageEnvelope {
            message_type: "agent.movement".to_string(),
            version: "0.0".to_string(),
            id: "test-id".to_string(),
            timestamp: "2024-01-01T00:00:00Z".to_string(),
            correlation_id: None,
            payload: Some(serde_json::json!({"forward": 0.5, "turn": -0.3})),
            meta: None,
        };
        let msg = SignalingMessage::from_message(&envelope).unwrap();
        if let SignalingMessage::Movement(payload) = msg {
            assert_eq!(payload.forward, 0.5);
            assert_eq!(payload.turn, -0.3);
        } else {
            panic!("Expected Movement variant");
        }
    }

    #[test]
    fn test_signaling_message_parse_movement_out_of_range() {
        let envelope = MessageEnvelope {
            message_type: "agent.movement".to_string(),
            version: "0.0".to_string(),
            id: "test-id".to_string(),
            timestamp: "2024-01-01T00:00:00Z".to_string(),
            correlation_id: None,
            payload: Some(serde_json::json!({"forward": 1.5, "turn": 0.0})),
            meta: None,
        };
        let result = SignalingMessage::from_message(&envelope);
        assert!(result.is_err());
        assert!(result.unwrap_err().contains("out of range"));
    }

    #[test]
    fn test_signaling_message_parse_capabilities() {
        let envelope = MessageEnvelope {
            message_type: "agent.capabilities".to_string(),
            version: "0.0".to_string(),
            id: "test-id".to_string(),
            timestamp: "2024-01-01T00:00:00Z".to_string(),
            correlation_id: None,
            payload: Some(serde_json::json!({"versions": ["0.0", "0.1"]})),
            meta: None,
        };
        let msg = SignalingMessage::from_message(&envelope).unwrap();
        if let SignalingMessage::Capabilities(payload) = msg {
            assert_eq!(payload.versions.len(), 2);
            assert!(payload.versions.contains(&"0.0".to_string()));
        } else {
            panic!("Expected Capabilities variant");
        }
    }

    #[test]
    fn test_signaling_message_parse_unknown() {
        let envelope = MessageEnvelope {
            message_type: "agent.unknown".to_string(),
            version: "0.0".to_string(),
            id: "test-id".to_string(),
            timestamp: "2024-01-01T00:00:00Z".to_string(),
            correlation_id: None,
            payload: None,
            meta: None,
        };
        let result = SignalingMessage::from_message(&envelope);
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
        assert!(result.unwrap_err().contains("type"));
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
}
