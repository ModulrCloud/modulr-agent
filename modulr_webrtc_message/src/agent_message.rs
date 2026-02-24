use serde::{Deserialize, Serialize};

use crate::common::{
    CapabilitiesErrorCode, Location, MessageEnvelope, MessageEnvelopeError, MessageFields,
};
use crate::{SUPPORTED_VERSIONS, validate_capabilities};

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
    LocationNotFound,
    LocationAlreadyExists,
    LocationStorageFull,
    LocationNameInvalid,
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub struct PingPayload {
    pub correlation_id: String,
}
pub type PongPayload = PingPayload;

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub struct MovementPayload {
    pub forward: f64,
    pub turn: f64,
}
pub type MovementCommand = MovementPayload;

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub struct LocationDeletePayload {
    pub name: String,
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct LocationResponsePayload {
    pub operation: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub locations: Option<Vec<Location>>,
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub struct CapabilitiesPayload {
    pub versions: Vec<String>,
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub struct ErrorPayload {
    #[serde(skip)]
    correlation_id: Option<String>,
    code: ErrorCode,
    message: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    details: Option<serde_json::Value>,
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub enum AgentMessage {
    Capabilities(CapabilitiesPayload),
    Error(ErrorPayload),
    LocationCreate(Location),
    LocationDelete(LocationDeletePayload),
    LocationList,
    LocationResponse(LocationResponsePayload),
    LocationUpdate(Location),
    Movement(MovementPayload),
    Ping(PingPayload),
    Pong(PongPayload),
}

impl MessageFields for AgentMessage {
    fn name(&self) -> String {
        match self {
            AgentMessage::Capabilities(_) => "agent.capabilities",
            AgentMessage::Error(_) => "agent.error",
            AgentMessage::LocationCreate(_) => "agent.location.create",
            AgentMessage::LocationDelete(_) => "agent.location.delete",
            AgentMessage::LocationList => "agent.location.list",
            AgentMessage::LocationResponse(_) => "agent.location.response",
            AgentMessage::LocationUpdate(_) => "agent.location.update",
            AgentMessage::Movement(_) => "agent.movement",
            AgentMessage::Ping(_) => "agent.ping",
            AgentMessage::Pong(_) => "agent.pong",
        }
        .to_string()
    }

    fn correlation_id(&self) -> Option<String> {
        match self {
            AgentMessage::Error(e) => e.correlation_id.clone(),
            AgentMessage::Ping(p) | AgentMessage::Pong(p) => Some(p.correlation_id.clone()),
            AgentMessage::Capabilities(_) => None,
            AgentMessage::LocationCreate(_) => None,
            AgentMessage::LocationDelete(_) => None,
            AgentMessage::LocationList => None,
            AgentMessage::LocationResponse(_) => None,
            AgentMessage::LocationUpdate(_) => None,
            AgentMessage::Movement(_) => None,
        }
    }

    fn payload(&self) -> Option<serde_json::Value> {
        match self {
            AgentMessage::Capabilities(c) => serde_json::to_value(c).ok(),
            AgentMessage::Error(e) => serde_json::to_value(e).ok(),
            AgentMessage::LocationCreate(l) => serde_json::to_value(l).ok(),
            AgentMessage::LocationDelete(l) => serde_json::to_value(l).ok(),
            AgentMessage::LocationList => None,
            AgentMessage::LocationResponse(r) => serde_json::to_value(r).ok(),
            AgentMessage::LocationUpdate(l) => serde_json::to_value(l).ok(),
            AgentMessage::Movement(cmd) => serde_json::to_value(cmd).ok(),
            AgentMessage::Ping(_) => None,
            AgentMessage::Pong(_) => None,
        }
    }

    fn meta(&self) -> Option<serde_json::Value> {
        None
    }
}

impl AgentMessage {
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

    pub fn error_from_envelope_error(
        err: &MessageEnvelopeError,
        correlation_id: Option<&str>,
    ) -> Self {
        let (code, message) = match err {
            MessageEnvelopeError::JsonParse { reason } => (
                ErrorCode::InvalidMessage,
                format!("JSON parse error: {reason}"),
            ),
            MessageEnvelopeError::JsonEncode { reason } => (
                ErrorCode::InternalError,
                format!("JSON encode error: {reason}"),
            ),
            MessageEnvelopeError::MissingFields { fields } => (
                ErrorCode::InvalidPayload,
                format!("Missing required fields: {fields:?}"),
            ),
            MessageEnvelopeError::OutOfRange {
                field,
                value,
                expected,
            } => (
                ErrorCode::ValidationFailed,
                format!("Value out of range: {field} = {value} (expected {expected})"),
            ),
            MessageEnvelopeError::UnknownMessageType { message_type } => (
                ErrorCode::UnsupportedMessageType,
                format!("Unknown message type: {message_type}"),
            ),
            MessageEnvelopeError::EnvelopeValidation { reason } => (
                ErrorCode::InvalidMessage,
                format!("Envelope validation failed: {reason}"),
            ),
        };
        AgentMessage::error(code, &message, correlation_id, None)
    }

    #[allow(dead_code)]
    pub fn movement_with_payload(payload: &MovementPayload) -> Self {
        AgentMessage::Movement(payload.clone())
    }

    #[allow(dead_code)]
    pub fn movement(forward: f64, turn: f64) -> Self {
        AgentMessage::Movement(MovementPayload { forward, turn })
    }

    #[allow(dead_code)]
    pub fn ping(correlation_id: &str) -> Self {
        AgentMessage::Ping(PingPayload {
            correlation_id: correlation_id.to_string(),
        })
    }

    pub fn pong(correlation_id: &str) -> Self {
        AgentMessage::Pong(PongPayload {
            correlation_id: correlation_id.to_string(),
        })
    }

    pub fn parse_error_response(reason: &str, correlation_id: Option<&str>) -> Self {
        AgentMessage::error(ErrorCode::InvalidMessage, reason, correlation_id, None)
    }

    pub fn capabilities_error_response(
        caps: &CapabilitiesPayload,
        correlation_id: Option<&str>,
    ) -> Option<Self> {
        let version_strs: Vec<&str> = caps.versions.iter().map(|s| s.as_str()).collect();
        match validate_capabilities(&version_strs) {
            Ok(_) => {
                log::info!("All capabilities supported by agent.");
                None
            }
            Err(err) => {
                let code = match err.code {
                    CapabilitiesErrorCode::UnsupportedVersion => ErrorCode::UnsupportedVersion,
                    CapabilitiesErrorCode::CapabilityMismatch => ErrorCode::CapabilityMismatch,
                };
                log::warn!("Error while comparing capabilities: {}", &err.message);
                Some(AgentMessage::error(
                    code,
                    &err.message,
                    correlation_id,
                    Some(err.details),
                ))
            }
        }
    }

    pub fn from_message(msg: &MessageEnvelope) -> Result<Self, MessageEnvelopeError> {
        msg.validate()?;

        match msg.message_type.as_str() {
            "agent.capabilities" => {
                let payload = msg
                    .payload
                    .as_ref()
                    .ok_or(MessageEnvelopeError::MissingFields {
                        fields: vec!["payload".to_string()],
                    })?;
                let versions: Vec<String> = payload["versions"]
                    .as_array()
                    .ok_or(MessageEnvelopeError::MissingFields {
                        fields: vec!["versions".to_string()],
                    })?
                    .iter()
                    .filter_map(|v| v.as_str().map(String::from))
                    .collect();
                Ok(AgentMessage::Capabilities(CapabilitiesPayload { versions }))
            }

            "agent.error" => {
                let mut missing_fields = vec![];

                let payload = msg
                    .payload
                    .as_ref()
                    .ok_or(MessageEnvelopeError::MissingFields {
                        fields: vec!["payload".to_string()],
                    })?;

                let code: ErrorCode = serde_json::from_value(payload["code"].clone())
                    .unwrap_or_else(|_| {
                        missing_fields.push("code".to_string());
                        ErrorCode::InvalidMessage
                    });

                let message = payload["message"]
                    .as_str()
                    .unwrap_or_else(|| {
                        missing_fields.push("message".to_string());
                        ""
                    })
                    .to_string();

                if !missing_fields.is_empty() {
                    return Err(MessageEnvelopeError::MissingFields {
                        fields: missing_fields,
                    });
                }

                let details = payload.get("details").cloned();
                let correlation_id = msg.correlation_id.clone();

                Ok(AgentMessage::Error(ErrorPayload {
                    code,
                    message,
                    details,
                    correlation_id,
                }))
            }

            "agent.movement" => {
                let payload = msg
                    .payload
                    .as_ref()
                    .ok_or(MessageEnvelopeError::MissingFields {
                        fields: vec!["payload".to_string()],
                    })?;
                let movement: MovementPayload =
                    serde_json::from_value(payload.clone()).map_err(|e| {
                        MessageEnvelopeError::JsonParse {
                            reason: e.to_string(),
                        }
                    })?;
                if !(-1.0..=1.0).contains(&movement.forward) {
                    return Err(MessageEnvelopeError::OutOfRange {
                        field: "forward".to_string(),
                        value: movement.forward.to_string(),
                        expected: "-1.0 to 1.0".to_string(),
                    });
                }
                if !(-1.0..=1.0).contains(&movement.turn) {
                    return Err(MessageEnvelopeError::OutOfRange {
                        field: "turn".to_string(),
                        value: movement.turn.to_string(),
                        expected: "-1.0 to 1.0".to_string(),
                    });
                }
                Ok(AgentMessage::Movement(movement))
            }

            "agent.ping" => {
                let correlation_id =
                    msg.correlation_id
                        .clone()
                        .ok_or(MessageEnvelopeError::MissingFields {
                            fields: vec!["correlationId".to_string()],
                        })?;
                Ok(AgentMessage::Ping(PingPayload { correlation_id }))
            }

            "agent.pong" => {
                let correlation_id =
                    msg.correlation_id
                        .clone()
                        .ok_or(MessageEnvelopeError::MissingFields {
                            fields: vec!["correlationId".to_string()],
                        })?;
                Ok(AgentMessage::Pong(PongPayload { correlation_id }))
            }

            "agent.location.create" | "agent.location.update" => {
                let payload = msg
                    .payload
                    .as_ref()
                    .ok_or(MessageEnvelopeError::MissingFields {
                        fields: vec!["payload".to_string()],
                    })?;
                let location: Location = serde_json::from_value(payload.clone()).map_err(|e| {
                    MessageEnvelopeError::JsonParse {
                        reason: e.to_string(),
                    }
                })?;
                location
                    .validate()
                    .map_err(|e| MessageEnvelopeError::JsonParse {
                        reason: e.to_string(),
                    })?;
                if msg.message_type == "agent.location.create" {
                    Ok(AgentMessage::LocationCreate(location))
                } else {
                    Ok(AgentMessage::LocationUpdate(location))
                }
            }

            "agent.location.list" => Ok(AgentMessage::LocationList),

            "agent.location.delete" => {
                let payload = msg
                    .payload
                    .as_ref()
                    .ok_or(MessageEnvelopeError::MissingFields {
                        fields: vec!["payload".to_string()],
                    })?;
                let del: LocationDeletePayload =
                    serde_json::from_value(payload.clone()).map_err(|e| {
                        MessageEnvelopeError::JsonParse {
                            reason: e.to_string(),
                        }
                    })?;
                Ok(AgentMessage::LocationDelete(del))
            }

            "agent.location.response" => {
                let payload = msg
                    .payload
                    .as_ref()
                    .ok_or(MessageEnvelopeError::MissingFields {
                        fields: vec!["payload".to_string()],
                    })?;
                let resp: LocationResponsePayload = serde_json::from_value(payload.clone())
                    .map_err(|e| MessageEnvelopeError::JsonParse {
                        reason: e.to_string(),
                    })?;
                Ok(AgentMessage::LocationResponse(resp))
            }

            _ => Err(MessageEnvelopeError::UnknownMessageType {
                message_type: msg.message_type.clone(),
            }),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ToMessage;
    use serde_json::json;
    use std::str::FromStr;

    fn make_envelope(
        message_type: &str,
        correlation_id: Option<&str>,
        payload: Option<serde_json::Value>,
    ) -> MessageEnvelope {
        MessageEnvelope {
            message_type: message_type.to_string(),
            version: "0.0".to_string(),
            id: "test-id".to_string(),
            timestamp: "2024-01-01T00:00:00Z".to_string(),
            correlation_id: correlation_id.map(String::from),
            payload,
            meta: None,
        }
    }

    // --- Serialisation tests ---

    #[test]
    fn test_base_envelope_has_all_fields() {
        let json = AgentMessage::pong("test").to_message().to_string().unwrap();
        assert!(json.contains("\"type\""));
        assert!(json.contains("\"correlationId\""));
        assert!(!json.contains("\"message_type\""));
        assert!(!json.contains("\"correlation_id\""));
    }

    #[test]
    fn test_capabilities_serialises_correctly() {
        let envelope = AgentMessage::capabilities().to_message();
        assert_eq!(envelope.message_type, "agent.capabilities");
        assert!(envelope.payload.is_some());
    }

    #[test]
    fn test_error_code_serialisation() {
        let code = ErrorCode::InvalidMessage;
        let json = serde_json::to_string(&code).unwrap();
        assert_eq!(json, "\"INVALID_MESSAGE\"");
    }

    #[test]
    fn test_error_code_deserialisation() {
        let json = "\"VALIDATION_FAILED\"";
        let code: ErrorCode = serde_json::from_str(json).unwrap();
        assert_eq!(code, ErrorCode::ValidationFailed);
    }

    #[test]
    fn test_error_serialises_correctly() {
        let envelope =
            AgentMessage::error(ErrorCode::InvalidPayload, "Bad data", Some("msg-456"), None)
                .to_message();
        assert_eq!(envelope.message_type, "agent.error");
        assert_eq!(envelope.correlation_id, Some("msg-456".to_string()));
        assert!(envelope.payload.is_some());
    }

    #[test]
    fn test_movement_serialises_correctly() {
        let envelope = AgentMessage::movement(1.0, -0.5).to_message();
        assert_eq!(envelope.message_type, "agent.movement");
        let payload = envelope.payload.unwrap();
        assert_eq!(payload["forward"], 1.0);
        assert_eq!(payload["turn"], -0.5);
    }

    #[test]
    fn test_ping_serialises_correctly() {
        let envelope = AgentMessage::ping("ping-123").to_message();
        assert_eq!(envelope.message_type, "agent.ping");
        assert_eq!(envelope.correlation_id, Some("ping-123".to_string()));
        assert!(envelope.payload.is_none());
    }

    #[test]
    fn test_pong_serialises_correctly() {
        let envelope = AgentMessage::pong("ping-123").to_message();
        assert_eq!(envelope.message_type, "agent.pong");
        assert_eq!(envelope.correlation_id, Some("ping-123".to_string()));
        assert!(envelope.payload.is_none());
    }

    // --- Parsing (from_message) tests ---

    #[test]
    fn test_parse_capabilities() {
        let envelope = make_envelope(
            "agent.capabilities",
            None,
            Some(json!({ "versions": ["0.0", "1.0"] })),
        );
        let msg = AgentMessage::from_message(&envelope).unwrap();
        match msg {
            AgentMessage::Capabilities(c) => {
                let payload = serde_json::to_value(&c).unwrap();
                assert_eq!(payload["versions"], json!(["0.0", "1.0"]));
            }
            other => panic!("Expected Capabilities, got {:?}", other),
        }
    }

    #[test]
    fn test_parse_capabilities_missing_payload_returns_error() {
        let envelope = make_envelope("agent.capabilities", None, None);
        let err = AgentMessage::from_message(&envelope).unwrap_err();
        assert!(matches!(err, MessageEnvelopeError::MissingFields { .. }));
    }

    #[test]
    fn test_parse_capabilities_missing_versions_returns_error() {
        let envelope = make_envelope("agent.capabilities", None, Some(json!({})));
        let err = AgentMessage::from_message(&envelope).unwrap_err();
        assert!(matches!(err, MessageEnvelopeError::MissingFields { .. }));
    }

    #[test]
    fn test_parse_error_message() {
        let envelope = make_envelope(
            "agent.error",
            Some("corr-1"),
            Some(json!({
                "code": "VALIDATION_FAILED",
                "message": "bad input",
                "details": { "field": "name" }
            })),
        );
        let msg = AgentMessage::from_message(&envelope).unwrap();
        match msg {
            AgentMessage::Error(e) => {
                assert_eq!(e.code, ErrorCode::ValidationFailed);
                assert_eq!(e.message, "bad input");
                assert_eq!(e.correlation_id, Some("corr-1".to_string()));
                assert!(e.details.is_some());
                assert_eq!(e.details.unwrap()["field"], "name");
            }
            other => panic!("Expected Error, got {:?}", other),
        }
    }

    #[test]
    fn test_parse_error_missing_payload_returns_error() {
        let envelope = make_envelope("agent.error", None, None);
        let err = AgentMessage::from_message(&envelope).unwrap_err();
        assert!(matches!(err, MessageEnvelopeError::MissingFields { .. }));
    }

    #[test]
    fn test_parse_error_missing_code_returns_error() {
        let envelope = make_envelope("agent.error", None, Some(json!({ "message": "bad input" })));
        let err = AgentMessage::from_message(&envelope).unwrap_err();
        assert!(matches!(err, MessageEnvelopeError::MissingFields { .. }));
    }

    #[test]
    fn test_parse_error_missing_message_returns_error() {
        let envelope = make_envelope(
            "agent.error",
            None,
            Some(json!({ "code": "INTERNAL_ERROR" })),
        );
        let err = AgentMessage::from_message(&envelope).unwrap_err();
        assert!(matches!(err, MessageEnvelopeError::MissingFields { .. }));
    }

    #[test]
    fn test_parse_error_without_details() {
        let envelope = make_envelope(
            "agent.error",
            None,
            Some(json!({
                "code": "INTERNAL_ERROR",
                "message": "something went wrong"
            })),
        );
        let msg = AgentMessage::from_message(&envelope).unwrap();
        match msg {
            AgentMessage::Error(e) => {
                assert_eq!(e.code, ErrorCode::InternalError);
                assert!(e.details.is_none());
                assert_eq!(e.correlation_id, None);
            }
            other => panic!("Expected Error, got {:?}", other),
        }
    }

    #[test]
    fn test_parse_movement() {
        let envelope = make_envelope(
            "agent.movement",
            None,
            Some(json!({ "forward": 0.75, "turn": -0.3 })),
        );
        let msg = AgentMessage::from_message(&envelope).unwrap();
        match msg {
            AgentMessage::Movement(m) => {
                assert_eq!(m.forward, 0.75);
                assert_eq!(m.turn, -0.3);
            }
            other => panic!("Expected Movement, got {:?}", other),
        }
    }

    #[test]
    fn test_parse_movement_missing_payload_returns_error() {
        let envelope = make_envelope("agent.movement", None, None);
        let err = AgentMessage::from_message(&envelope).unwrap_err();
        assert!(matches!(err, MessageEnvelopeError::MissingFields { .. }));
    }

    #[test]
    fn test_parse_movement_invalid_payload_returns_error() {
        let envelope = make_envelope(
            "agent.movement",
            None,
            Some(json!({ "forward": "not_a_number" })),
        );
        let err = AgentMessage::from_message(&envelope).unwrap_err();
        assert!(matches!(err, MessageEnvelopeError::JsonParse { .. }));
    }

    #[test]
    fn test_parse_movement_forward_out_of_range_returns_error() {
        let envelope = make_envelope(
            "agent.movement",
            None,
            Some(json!({ "forward": 1.5, "turn": 0.0 })),
        );
        let err = AgentMessage::from_message(&envelope).unwrap_err();
        match err {
            MessageEnvelopeError::OutOfRange {
                field, expected, ..
            } => {
                assert_eq!(field, "forward");
                assert_eq!(expected, "-1.0 to 1.0");
            }
            other => panic!("Expected OutOfRange, got {:?}", other),
        }
    }

    #[test]
    fn test_parse_movement_turn_out_of_range_returns_error() {
        let envelope = make_envelope(
            "agent.movement",
            None,
            Some(json!({ "forward": 0.0, "turn": -1.1 })),
        );
        let err = AgentMessage::from_message(&envelope).unwrap_err();
        match err {
            MessageEnvelopeError::OutOfRange {
                field, expected, ..
            } => {
                assert_eq!(field, "turn");
                assert_eq!(expected, "-1.0 to 1.0");
            }
            other => panic!("Expected OutOfRange, got {:?}", other),
        }
    }

    #[test]
    fn test_parse_movement_boundary_values() {
        // Exactly -1.0 and 1.0 should be accepted
        let envelope = make_envelope(
            "agent.movement",
            None,
            Some(json!({ "forward": -1.0, "turn": 1.0 })),
        );
        let msg = AgentMessage::from_message(&envelope).unwrap();
        match msg {
            AgentMessage::Movement(m) => {
                assert_eq!(m.forward, -1.0);
                assert_eq!(m.turn, 1.0);
            }
            other => panic!("Expected Movement, got {:?}", other),
        }
    }

    #[test]
    fn test_parse_ping() {
        let envelope = make_envelope("agent.ping", Some("ping-abc"), None);
        let msg = AgentMessage::from_message(&envelope).unwrap();
        match msg {
            AgentMessage::Ping(p) => {
                assert_eq!(p.correlation_id, "ping-abc");
            }
            other => panic!("Expected Ping, got {:?}", other),
        }
    }

    #[test]
    fn test_parse_ping_missing_correlation_id_returns_error() {
        let envelope = make_envelope("agent.ping", None, None);
        let err = AgentMessage::from_message(&envelope).unwrap_err();
        assert!(matches!(err, MessageEnvelopeError::MissingFields { .. }));
    }

    #[test]
    fn test_parse_pong() {
        let envelope = make_envelope("agent.pong", Some("pong-xyz"), None);
        let msg = AgentMessage::from_message(&envelope).unwrap();
        match msg {
            AgentMessage::Pong(p) => {
                assert_eq!(p.correlation_id, "pong-xyz");
            }
            other => panic!("Expected Pong, got {:?}", other),
        }
    }

    #[test]
    fn test_parse_pong_missing_correlation_id_returns_error() {
        let envelope = make_envelope("agent.pong", None, None);
        let err = AgentMessage::from_message(&envelope).unwrap_err();
        assert!(matches!(err, MessageEnvelopeError::MissingFields { .. }));
    }

    #[test]
    fn test_parse_unknown_message_type_returns_error() {
        let envelope = make_envelope("agent.unknown", None, None);
        let err = AgentMessage::from_message(&envelope).unwrap_err();
        assert!(matches!(
            err,
            MessageEnvelopeError::UnknownMessageType { .. }
        ));
    }

    // --- Round-trip tests ---

    #[test]
    fn test_capabilities_round_trip() {
        let original = AgentMessage::capabilities();
        let json_str = original.to_message().to_string().unwrap();
        let envelope = MessageEnvelope::from_str(&json_str).unwrap();
        let parsed = AgentMessage::from_message(&envelope).unwrap();
        assert!(matches!(parsed, AgentMessage::Capabilities(_)));
    }

    #[test]
    fn test_error_round_trip() {
        let original = AgentMessage::error(
            ErrorCode::InvalidPayload,
            "Bad data",
            Some("corr-99"),
            Some(json!({ "hint": "check field X" })),
        );
        let json_str = original.to_message().to_string().unwrap();
        let envelope = MessageEnvelope::from_str(&json_str).unwrap();
        let parsed = AgentMessage::from_message(&envelope).unwrap();
        match parsed {
            AgentMessage::Error(e) => {
                assert_eq!(e.code, ErrorCode::InvalidPayload);
                assert_eq!(e.message, "Bad data");
                assert_eq!(e.correlation_id, Some("corr-99".to_string()));
            }
            other => panic!("Expected Error, got {:?}", other),
        }
    }

    #[test]
    fn test_movement_round_trip() {
        let original = AgentMessage::movement(1.0, -0.5);
        let json_str = original.to_message().to_string().unwrap();
        let envelope = MessageEnvelope::from_str(&json_str).unwrap();
        let parsed = AgentMessage::from_message(&envelope).unwrap();
        match parsed {
            AgentMessage::Movement(m) => {
                assert_eq!(m.forward, 1.0);
                assert_eq!(m.turn, -0.5);
            }
            other => panic!("Expected Movement, got {:?}", other),
        }
    }

    #[test]
    fn test_ping_round_trip() {
        let original = AgentMessage::ping("rt-ping");
        let json_str = original.to_message().to_string().unwrap();
        let envelope = MessageEnvelope::from_str(&json_str).unwrap();
        let parsed = AgentMessage::from_message(&envelope).unwrap();
        match parsed {
            AgentMessage::Ping(p) => assert_eq!(p.correlation_id, "rt-ping"),
            other => panic!("Expected Ping, got {:?}", other),
        }
    }

    #[test]
    fn test_pong_round_trip() {
        let original = AgentMessage::pong("rt-pong");
        let json_str = original.to_message().to_string().unwrap();
        let envelope = MessageEnvelope::from_str(&json_str).unwrap();
        let parsed = AgentMessage::from_message(&envelope).unwrap();
        match parsed {
            AgentMessage::Pong(p) => assert_eq!(p.correlation_id, "rt-pong"),
            other => panic!("Expected Pong, got {:?}", other),
        }
    }

    // --- Location parsing tests ---

    #[test]
    fn test_parse_location_create() {
        let envelope = make_envelope(
            "agent.location.create",
            None,
            Some(json!({
                "name": "Warehouse Loading Dock",
                "position": { "x": 12.5, "y": 8.3 },
                "orientation": { "yaw": 1.57 }
            })),
        );
        let msg = AgentMessage::from_message(&envelope).unwrap();
        match msg {
            AgentMessage::LocationCreate(l) => {
                assert_eq!(l.name, "Warehouse Loading Dock");
                assert_eq!(l.position.x, 12.5);
                assert_eq!(l.position.y, 8.3);
                assert!(l.position.z.is_none());
                assert_eq!(l.orientation.unwrap().yaw, Some(1.57));
            }
            other => panic!("Expected LocationCreate, got {:?}", other),
        }
    }

    #[test]
    fn test_parse_location_create_missing_payload_returns_error() {
        let envelope = make_envelope("agent.location.create", None, None);
        let err = AgentMessage::from_message(&envelope).unwrap_err();
        assert!(matches!(err, MessageEnvelopeError::MissingFields { .. }));
    }

    #[test]
    fn test_parse_location_list() {
        let envelope = make_envelope("agent.location.list", None, None);
        let msg = AgentMessage::from_message(&envelope).unwrap();
        assert!(matches!(msg, AgentMessage::LocationList));
    }

    #[test]
    fn test_parse_location_update() {
        let envelope = make_envelope(
            "agent.location.update",
            None,
            Some(json!({
                "name": "Warehouse Loading Dock",
                "position": { "x": 12.8, "y": 8.5 }
            })),
        );
        let msg = AgentMessage::from_message(&envelope).unwrap();
        match msg {
            AgentMessage::LocationUpdate(l) => {
                assert_eq!(l.name, "Warehouse Loading Dock");
                assert_eq!(l.position.x, 12.8);
            }
            other => panic!("Expected LocationUpdate, got {:?}", other),
        }
    }

    #[test]
    fn test_parse_location_delete() {
        let envelope = make_envelope(
            "agent.location.delete",
            None,
            Some(json!({ "name": "Old Warehouse Location" })),
        );
        let msg = AgentMessage::from_message(&envelope).unwrap();
        match msg {
            AgentMessage::LocationDelete(d) => {
                assert_eq!(d.name, "Old Warehouse Location");
            }
            other => panic!("Expected LocationDelete, got {:?}", other),
        }
    }

    #[test]
    fn test_parse_location_delete_missing_payload_returns_error() {
        let envelope = make_envelope("agent.location.delete", None, None);
        let err = AgentMessage::from_message(&envelope).unwrap_err();
        assert!(matches!(err, MessageEnvelopeError::MissingFields { .. }));
    }

    #[test]
    fn test_parse_location_response() {
        let envelope = make_envelope(
            "agent.location.response",
            Some("req-123"),
            Some(json!({
                "operation": "list",
                "locations": [
                    { "name": "Home", "position": { "x": 0.0, "y": 0.0 } }
                ]
            })),
        );
        let msg = AgentMessage::from_message(&envelope).unwrap();
        match msg {
            AgentMessage::LocationResponse(r) => {
                assert_eq!(r.operation, "list");
                let locs = r.locations.unwrap();
                assert_eq!(locs.len(), 1);
                assert_eq!(locs[0].name, "Home");
            }
            other => panic!("Expected LocationResponse, got {:?}", other),
        }
    }

    #[test]
    fn test_location_error_codes_serialise() {
        assert_eq!(
            serde_json::to_string(&ErrorCode::LocationNotFound).unwrap(),
            "\"LOCATION_NOT_FOUND\""
        );
        assert_eq!(
            serde_json::to_string(&ErrorCode::LocationAlreadyExists).unwrap(),
            "\"LOCATION_ALREADY_EXISTS\""
        );
        assert_eq!(
            serde_json::to_string(&ErrorCode::LocationNameInvalid).unwrap(),
            "\"LOCATION_NAME_INVALID\""
        );
    }

    #[test]
    fn test_parse_location_response_spec_example() {
        // Matches response.example.json: two locations, one with full orientation
        // and z, one with only x/y — also validates optional field handling
        let envelope = make_envelope(
            "agent.location.response",
            Some("c3d4e5f6-a7b8-9c0d-1e2f-3a4b5c6d7e8f"),
            Some(json!({
                "operation": "list",
                "locations": [
                    {
                        "name": "Warehouse Loading Dock",
                        "position": { "x": 12.5, "y": 8.3, "z": 0.0 },
                        "orientation": { "yaw": 1.60, "pitch": 0.50, "roll": 0.24 }
                    },
                    {
                        "name": "Assembly Station 1",
                        "position": { "x": 5.2, "y": 10.8 }
                    }
                ]
            })),
        );
        let msg = AgentMessage::from_message(&envelope).unwrap();
        match msg {
            AgentMessage::LocationResponse(r) => {
                assert_eq!(r.operation, "list");
                let locs = r.locations.unwrap();
                assert_eq!(locs.len(), 2);
                let dock = &locs[0];
                assert_eq!(dock.name, "Warehouse Loading Dock");
                assert_eq!(dock.position.z, Some(0.0));
                let orient = dock.orientation.as_ref().unwrap();
                assert_eq!(orient.yaw, Some(1.60));
                assert_eq!(orient.pitch, Some(0.50));
                assert_eq!(orient.roll, Some(0.24));
                let station = &locs[1];
                assert_eq!(station.name, "Assembly Station 1");
                assert!(station.position.z.is_none());
                assert!(station.orientation.is_none());
            }
            other => panic!("Expected LocationResponse, got {:?}", other),
        }
    }

    #[test]
    fn test_parse_location_list_empty_payload() {
        // Matches list.example.json which sends payload: {}
        let envelope = make_envelope("agent.location.list", None, Some(json!({})));
        let msg = AgentMessage::from_message(&envelope).unwrap();
        assert!(matches!(msg, AgentMessage::LocationList));
    }
}
