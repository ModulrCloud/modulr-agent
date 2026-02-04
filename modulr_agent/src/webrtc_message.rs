// Modular agent JSON Schema implementation

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

pub trait MessageFields {
    fn name(&self) -> String;
    fn correlation_id(&self) -> Option<String>;
    fn payload(&self) -> Option<serde_json::Value>;
    fn meta(&self) -> Option<serde_json::Value>;
}

pub trait ToMessage {
    fn to_message(&self) -> MessageEnvelope;
}

impl<T> ToMessage for T
where
    T: MessageFields,
{
    fn to_message(&self) -> MessageEnvelope {
        MessageEnvelope {
            message_type: self.name().to_string(),
            version: PROTOCOL_VERSION.to_string(),
            id: generate_id(),
            timestamp: generate_timestamp(),
            correlation_id: self.correlation_id(),
            payload: self.payload(),
            meta: self.meta(),
        }
    }
}
#[derive(Serialize, Deserialize, Clone, Debug)]
pub enum AgentMessage {
    Ping,
    Pong {
        correlation_id: String,
    },
    Movement {
        forward: f64,
        turn: f64,
    },
    Error {
        correlation_id: Option<String>,
        code: ErrorCode,
        message: String,
        details: Option<serde_json::Value>,
    },
    Capabilities {
        versions: Vec<String>,
    },
}

impl MessageFields for AgentMessage {
    fn name(&self) -> String {
        match self {
            AgentMessage::Ping => "agent.ping",
            AgentMessage::Pong { .. } => "agent.pong",
            AgentMessage::Movement { .. } => "agent.movement",
            AgentMessage::Error { .. } => "agent.error",
            AgentMessage::Capabilities { .. } => "agent.capabilities",
        }
        .to_string()
    }

    fn correlation_id(&self) -> Option<String> {
        match self {
            AgentMessage::Pong { correlation_id } => Some(correlation_id.clone()),
            AgentMessage::Error { correlation_id, .. } => correlation_id.clone(),
            _ => None,
        }
    }

    fn payload(&self) -> Option<serde_json::Value> {
        match self {
            AgentMessage::Ping => None,
            AgentMessage::Pong { .. } => None,
            AgentMessage::Movement { forward, turn } => Some(serde_json::json!({
                "forward": forward,
                "turn": turn
            })),
            AgentMessage::Error {
                code,
                message,
                details,
                ..
            } => {
                let mut payload = serde_json::json!({
                    "code": code,
                    "message": message
                });
                if let Some(d) = details {
                    payload["details"] = d.clone();
                }
                Some(payload)
            }
            AgentMessage::Capabilities { versions } => {
                Some(serde_json::json!({ "versions": versions }))
            }
        }
    }

    fn meta(&self) -> Option<serde_json::Value> {
        None
    }
}

impl AgentMessage {
    #[allow(dead_code)] //pass test -- will need for actual integration
    pub fn ping() -> Self {
        AgentMessage::Ping
    }

    pub fn pong(correlation_id: &str) -> Self {
        AgentMessage::Pong {
            correlation_id: correlation_id.to_string(),
        }
    }

    #[allow(dead_code)] //pass test -- will need for actual integration
    pub fn movement(forward: f64, turn: f64) -> Result<Self, String> {
        if !(-1.0..=1.0).contains(&forward) {
            return Err(format!(
                "forward must be between -1.0 and 1.0, got {}",
                forward
            ));
        }
        if !(-1.0..=1.0).contains(&turn) {
            return Err(format!("turn must be between -1.0 and 1.0, got {}", turn));
        }
        Ok(AgentMessage::Movement { forward, turn })
    }

    pub fn error(
        code: ErrorCode,
        message: &str,
        correlation_id: Option<&str>,
        details: Option<serde_json::Value>,
    ) -> Self {
        AgentMessage::Error {
            correlation_id: correlation_id.map(String::from),
            code,
            message: message.to_string(),
            details,
        }
    }

    pub fn capabilities() -> Self {
        AgentMessage::Capabilities {
            versions: SUPPORTED_VERSIONS.iter().map(|s| s.to_string()).collect(),
        }
    }

    #[allow(dead_code)] //not needed due to only one version currently
    pub fn capabilities_with_versions(versions: Vec<String>) -> Self {
        AgentMessage::Capabilities { versions }
    }

    pub fn from_message(msg: &MessageEnvelope) -> Result<Self, String> {
        Self::validate_envelope(msg)?;

        match msg.message_type.as_str() {
            "agent.ping" => Ok(AgentMessage::Ping),

            "agent.pong" => {
                let correlation_id = msg
                    .correlation_id
                    .clone()
                    .ok_or("pong requires correlation_id")?;
                Ok(AgentMessage::Pong { correlation_id })
            }

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

                Ok(AgentMessage::Movement { forward, turn })
            }

            "agent.error" => {
                let payload = msg.payload.as_ref().ok_or("error requires payload")?;
                let code: ErrorCode = serde_json::from_value(payload["code"].clone())
                    .map_err(|e| format!("invalid error code: {}", e))?;
                let message = payload["message"]
                    .as_str()
                    .ok_or("error requires message field")?
                    .to_string();
                let details = payload.get("details").cloned();

                Ok(AgentMessage::Error {
                    correlation_id: msg.correlation_id.clone(),
                    code,
                    message,
                    details,
                })
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

                Ok(AgentMessage::Capabilities { versions })
            }

            _ => Err(format!("unknown message type: {}", msg.message_type)),
        }
    }

    fn validate_envelope(msg: &MessageEnvelope) -> Result<(), String> {
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

        let version_parts: Vec<&str> = msg.version.split('.').collect();
        if version_parts.len() != 2 {
            return Err("Invalid version format: expected MAJOR.MINOR".to_string());
        }
        for part in version_parts {
            if part.parse::<u32>().is_err() {
                return Err("Invalid version format: parts must be numeric".to_string());
            }
        }

        Ok(())
    }
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct MovementCommand {
    pub forward: f64,
    pub turn: f64,
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
    match AgentMessage::from_message(envelope) {
        Ok(agent_msg) => match agent_msg {
            AgentMessage::Ping => Some(AgentMessage::pong(&envelope.id).to_message()),
            AgentMessage::Pong { correlation_id } => {
                println!("Received pong for message: {}", correlation_id);
                None
            }
            AgentMessage::Movement { forward, turn } => {
                println!("Movement command: forward={}, turn={}", forward, turn);
                None
            }
            AgentMessage::Error { code, message, .. } => {
                println!("Received error: code={:?}, message={}", code, message);
                None
            }
            AgentMessage::Capabilities { versions } => {
                println!("Received capabilities: versions={:?}", versions);
                if let Some(version) = negotiate_version(&versions) {
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

//unit tests -- not ready, need to go through and verify validity of each test
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_create_ping() {
        let ping = AgentMessage::ping().to_message();
        assert_eq!(ping.message_type, "agent.ping");
        assert_eq!(ping.version, PROTOCOL_VERSION);
        assert!(!ping.id.is_empty());
        assert!(!ping.timestamp.is_empty());
        assert!(ping.payload.is_none());
    }

    #[test]
    fn test_create_pong() {
        let pong = AgentMessage::pong("original-ping-id").to_message();
        assert_eq!(pong.message_type, "agent.pong");
        assert_eq!(pong.version, PROTOCOL_VERSION);
        assert_eq!(pong.correlation_id, Some("original-ping-id".to_string()));
        assert!(pong.payload.is_none());
    }

    #[test]
    fn test_create_movement() {
        let movement = AgentMessage::movement(0.5, -0.3).unwrap().to_message();
        assert_eq!(movement.message_type, "agent.movement");
        assert!(movement.payload.is_some());
    }

    #[test]
    fn test_create_movement_boundary_values() {
        assert!(AgentMessage::movement(1.0, 1.0).is_ok());
        assert!(AgentMessage::movement(-1.0, -1.0).is_ok());
        assert!(AgentMessage::movement(0.0, 0.0).is_ok());
    }

    #[test]
    fn test_create_movement_out_of_range() {
        assert!(AgentMessage::movement(1.5, 0.0).is_err());
        assert!(AgentMessage::movement(0.0, -1.5).is_err());
    }

    #[test]
    fn test_create_error() {
        let error = AgentMessage::error(
            ErrorCode::MovementFailed,
            "Robot hit obstacle",
            Some("msg-123"),
            None,
        )
        .to_message();
        assert_eq!(error.message_type, "agent.error");
        assert_eq!(error.correlation_id, Some("msg-123".to_string()));
    }

    #[test]
    fn test_create_capabilities() {
        let caps = AgentMessage::capabilities().to_message();
        assert_eq!(caps.message_type, "agent.capabilities");
        assert!(caps.payload.is_some());
    }

    #[test]
    fn test_ping_roundtrip() {
        let original = AgentMessage::ping().to_message();
        let json = serde_json::to_string(&original).unwrap();
        let parsed: MessageEnvelope = serde_json::from_str(&json).unwrap();

        assert_eq!(original.message_type, parsed.message_type);
        assert_eq!(original.id, parsed.id);
        assert_eq!(original.version, parsed.version);
    }

    #[test]
    fn test_movement_roundtrip() {
        let original = AgentMessage::movement(0.75, -0.25).unwrap().to_message();
        let json = serde_json::to_string(&original).unwrap();
        let parsed: MessageEnvelope = serde_json::from_str(&json).unwrap();

        assert_eq!(original.message_type, parsed.message_type);
        assert_eq!(original.id, parsed.id);

        let orig_msg = AgentMessage::from_message(&original).unwrap();
        let parsed_msg = AgentMessage::from_message(&parsed).unwrap();

        if let (
            AgentMessage::Movement {
                forward: f1,
                turn: t1,
            },
            AgentMessage::Movement {
                forward: f2,
                turn: t2,
            },
        ) = (orig_msg, parsed_msg)
        {
            assert_eq!(f1, f2);
            assert_eq!(t1, t2);
        } else {
            panic!("Expected Movement variants");
        }
    }

    #[test]
    fn test_error_roundtrip() {
        let original =
            AgentMessage::error(ErrorCode::ValidationFailed, "Invalid input", None, None)
                .to_message();
        let json = serde_json::to_string(&original).unwrap();
        let parsed: MessageEnvelope = serde_json::from_str(&json).unwrap();

        assert_eq!(original.message_type, parsed.message_type);
        assert_eq!(original.id, parsed.id);
    }

    #[test]
    fn test_json_field_names() {
        let ping = AgentMessage::ping().to_message();
        let json = serde_json::to_string(&ping).unwrap();

        assert!(json.contains("\"type\""));
        assert!(!json.contains("\"message_type\""));

        let pong = AgentMessage::pong("test").to_message();
        let pong_json = serde_json::to_string(&pong).unwrap();
        assert!(pong_json.contains("\"correlationId\""));
        assert!(!pong_json.contains("\"correlation_id\""));
    }

    #[test]
    fn test_extract_movement_command() {
        let msg = AgentMessage::movement(0.6, -0.2).unwrap().to_message();
        let parsed = AgentMessage::from_message(&msg).unwrap();

        if let AgentMessage::Movement { forward, turn } = parsed {
            assert_eq!(forward, 0.6);
            assert_eq!(turn, -0.2);
        } else {
            panic!("Expected Movement variant");
        }
    }

    #[test]
    fn test_extract_error_payload() {
        let msg =
            AgentMessage::error(ErrorCode::MovementFailed, "Test error", None, None).to_message();
        let parsed = AgentMessage::from_message(&msg).unwrap();

        if let AgentMessage::Error { code, message, .. } = parsed {
            assert_eq!(code, ErrorCode::MovementFailed);
            assert_eq!(message, "Test error");
        } else {
            panic!("Expected Error variant");
        }
    }

    #[test]
    fn test_extract_capabilities_payload() {
        let msg = AgentMessage::capabilities().to_message();
        let parsed = AgentMessage::from_message(&msg).unwrap();

        if let AgentMessage::Capabilities { versions } = parsed {
            assert!(!versions.is_empty());
            assert!(versions.contains(&"0.0".to_string()));
        } else {
            panic!("Expected Capabilities variant");
        }
    }
    #[test]
    fn test_parse_ping_from_json() {
        let json = r#"{
            "type": "agent.ping",
            "version": "0.0",
            "id": "test-id",
            "timestamp": "2024-01-01T00:00:00Z"
        }"#;

        let msg = parse_message(json).unwrap();
        assert_eq!(msg.message_type, "agent.ping");
    }

    #[test]
    fn test_parse_movement_from_json() {
        let json = r#"{
            "type": "agent.movement",
            "version": "0.0",
            "id": "test-id",
            "timestamp": "2024-01-01T00:00:00Z",
            "payload": {
                "forward": 0.5,
                "turn": -0.3
            }
        }"#;

        let msg = parse_message(json).unwrap();
        assert_eq!(msg.message_type, "agent.movement");

        let parsed = AgentMessage::from_message(&msg).unwrap();
        if let AgentMessage::Movement { forward, turn } = parsed {
            assert_eq!(forward, 0.5);
            assert_eq!(turn, -0.3);
        } else {
            panic!("Expected Movement variant");
        }
    }

    #[test]
    fn test_parse_invalid_json() {
        let json = r#"{ invalid json }"#;
        let result = parse_message(json);
        assert!(result.is_err());
    }

    #[test]
    fn test_validate_envelope_valid() {
        let msg = AgentMessage::ping().to_message();
        assert!(AgentMessage::from_message(&msg).is_ok());
    }

    #[test]
    fn test_validate_envelope_missing_type() {
        let msg = MessageEnvelope {
            message_type: "".to_string(),
            version: "0.0".to_string(),
            id: "123".to_string(),
            timestamp: "2024-01-01T00:00:00Z".to_string(),
            correlation_id: None,
            payload: None,
            meta: None,
        };

        let result = AgentMessage::from_message(&msg);
        assert!(result.is_err());
        assert!(result.unwrap_err().contains("type"));
    }

    #[test]
    fn test_validate_envelope_bad_version() {
        let msg = MessageEnvelope {
            message_type: "agent.ping".to_string(),
            version: "0.0.2".to_string(),
            id: "123".to_string(),
            timestamp: "2024-01-01T00:00:00Z".to_string(),
            correlation_id: None,
            payload: None,
            meta: None,
        };

        let result = AgentMessage::from_message(&msg);
        assert!(result.is_err());
        assert!(result.unwrap_err().contains("version"));
    }

    #[test]
    fn test_version_format_matches_schema() {
        let ping = AgentMessage::ping().to_message();
        let version = &ping.version;
        let parts: Vec<&str> = version.split('.').collect();

        assert_eq!(parts.len(), 2, "Version must be MAJOR.MINOR format");
        assert!(
            parts[0].parse::<u32>().is_ok(),
            "MAJOR version must be numeric"
        );
        assert!(
            parts[1].parse::<u32>().is_ok(),
            "MINOR version must be numeric"
        );
    }

    #[test]
    fn test_is_version_supported() {
        assert!(is_version_supported("0.0"));
        assert!(!is_version_supported("0.1"));
        assert!(!is_version_supported("0.5"));
        assert!(!is_version_supported("1.0"));
    }

    #[test]
    fn test_negotiate_version() {
        let remote = vec!["0.0".to_string(), "0.2".to_string()];
        let negotiated = negotiate_version(&remote);
        assert_eq!(negotiated, Some("0.0".to_string()));
    }

    #[test]
    fn test_negotiate_version_no_match() {
        let remote = vec!["0.5".to_string(), "1.0".to_string()];
        let negotiated = negotiate_version(&remote);
        assert_eq!(negotiated, None);
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
    fn test_handle_ping_returns_pong() {
        let ping = AgentMessage::ping().to_message();
        let response = handle_message(&ping);

        assert!(response.is_some());
        let pong = response.unwrap();
        assert_eq!(pong.message_type, "agent.pong");
        assert_eq!(pong.correlation_id, Some(ping.id.clone()));
    }

    #[test]
    fn test_handle_pong_returns_none() {
        let pong = AgentMessage::pong("test-id").to_message();
        let response = handle_message(&pong);
        assert!(response.is_none());
    }

    #[test]
    fn test_handle_movement_valid() {
        let movement = AgentMessage::movement(0.5, 0.3).unwrap().to_message();
        let response = handle_message(&movement);
        assert!(response.is_none());
    }

    #[test]
    fn test_handle_unknown_message_type() {
        let mut msg = AgentMessage::ping().to_message();
        msg.message_type = "agent.unknown".to_string();
        let response = handle_message(&msg);

        assert!(response.is_some());
        let error = response.unwrap();
        assert_eq!(error.message_type, "agent.error");
    }

    #[test]
    fn test_from_message_all_variants() {
        let ping = AgentMessage::ping().to_message();
        assert!(matches!(
            AgentMessage::from_message(&ping),
            Ok(AgentMessage::Ping)
        ));

        let pong = AgentMessage::pong("test").to_message();
        assert!(matches!(
            AgentMessage::from_message(&pong),
            Ok(AgentMessage::Pong { .. })
        ));

        let movement = AgentMessage::movement(0.5, 0.5).unwrap().to_message();
        assert!(matches!(
            AgentMessage::from_message(&movement),
            Ok(AgentMessage::Movement { .. })
        ));

        let error = AgentMessage::error(ErrorCode::InternalError, "test", None, None).to_message();
        assert!(matches!(
            AgentMessage::from_message(&error),
            Ok(AgentMessage::Error { .. })
        ));

        let caps = AgentMessage::capabilities().to_message();
        assert!(matches!(
            AgentMessage::from_message(&caps),
            Ok(AgentMessage::Capabilities { .. })
        ));
    }
}
