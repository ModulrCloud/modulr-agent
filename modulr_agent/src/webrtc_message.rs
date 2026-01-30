// Modular agent JSON Shema implementation

use serde::{Deserialize, Serialize};


//Constants
pub const PROTOCOL_VERSION: &str = "0.0";
pub const MSG_TYPE_PING: &str = "agent.ping";
pub const MSG_TYPE_PONG: &str = "agent.pong";
pub const MSG_TYPE_MOVEMENT: &str = "agent.movement";
pub const MSG_TYPE_ERROR: &str = "agent.error";
pub const MSG_TYPE_CAPABILITIES: &str = "agent.capabilities";


pub const SUPPORTED_VERSIONS: &[&str] = &["0.0"];

// Common envelope for all protocol messages
#[derive(Serialize, Deserialize, Clone, Debug)]
#[serde(rename_all = "camelCase")]
pub struct MessageEnvelope {

    //Required
    #[serde(rename = "type")]
    pub message_type: String,
    pub version: String,
    pub id: String,
    pub timestamp: String,

    //Optional
    #[serde(skip_serializing_if = "Option::is_none")]
    pub correlation_id: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub payload: Option<serde_json::Value>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub meta: Option<serde_json::Value>,
}



// Generate a simple unique ID -- what is the standard approach we are supposed to be using here 
pub fn generate_id() -> String {
    use std::time::{SystemTime, UNIX_EPOCH};
    let duration = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default();
    format!("msg-{}-{}", duration.as_secs(), duration.subsec_nanos())
}

// Generate RFC 3339 timestamp
pub fn generate_timestamp() -> String {
    use chrono::Utc;
    Utc::now().to_rfc3339()
}

pub fn create_ping() -> MessageEnvelope {
    MessageEnvelope {
        message_type: MSG_TYPE_PING.to_string(),
        version: PROTOCOL_VERSION.to_string(),
        id: generate_id(),
        timestamp: generate_timestamp(),
        correlation_id: None,
        payload: None,
        meta: None,
    }
}


pub fn create_pong(ping_id: &str) -> MessageEnvelope {
    MessageEnvelope {
        message_type: MSG_TYPE_PONG.to_string(),
        version: PROTOCOL_VERSION.to_string(),
        id: generate_id(),
        timestamp: generate_timestamp(),
        correlation_id: Some(ping_id.to_string()), 
        payload: None,
        meta: None,
    }
}


// Movement payload 
#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct MovementCommand {
    pub forward: f64,
    pub turn: f64,
}

// Validate that movement payload values are within valid range
pub fn validate_movement_command(payload: &MovementCommand) -> Result<(), String> {
    if payload.forward < -1.0 || payload.forward > 1.0 {
        return Err(format!(
            "forward must be between -1.0 and 1.0, got {}",
            payload.forward
        ));
    }
    if payload.turn < -1.0 || payload.turn > 1.0 {
        return Err(format!(
            "turn must be between -1.0 and 1.0, got {}",
            payload.turn
        ));
    }
    Ok(())
}

// Create agent.movement message with the given velocities
pub fn create_movement(forward: f64, turn: f64) -> Result<MessageEnvelope, String> {
    let payload = MovementCommand { forward, turn };
    validate_movement_command(&payload)?;

    Ok(MessageEnvelope {
        message_type: MSG_TYPE_MOVEMENT.to_string(),
        version: PROTOCOL_VERSION.to_string(),
        id: generate_id(),
        timestamp: generate_timestamp(),
        correlation_id: None,
        payload: Some(serde_json::to_value(payload).unwrap()),
        meta: None,
    })
}

// Extract and validate movement payload from an envelope
pub fn extract_movement_command(envelope: &MessageEnvelope) -> Result<MovementCommand, String> {
    if envelope.message_type != MSG_TYPE_MOVEMENT {
        return Err(format!(
            "Expected {} message, got {}",
            MSG_TYPE_MOVEMENT, envelope.message_type
        ));
    }

    let payload = envelope
        .payload
        .as_ref()
        .ok_or("Movement message requires payload")?;

    let movement: MovementCommand = serde_json::from_value(payload.clone())
        .map_err(|e| format!("Invalid movement payload: {}", e))?;

    validate_movement_command(&movement)?;

    Ok(movement)
}


// Allowed error messeges
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

// Error payload 
#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct ErrorPayload {
    pub code: ErrorCode,
    pub message: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub details: Option<serde_json::Value>,
}

//agent.error message
pub fn create_error(
    code: ErrorCode,
    message: &str,
    correlation_id: Option<&str>,
    details: Option<serde_json::Value>,
) -> MessageEnvelope {
    MessageEnvelope {
        message_type: MSG_TYPE_ERROR.to_string(),
        version: PROTOCOL_VERSION.to_string(),
        id: generate_id(),
        timestamp: generate_timestamp(),
        correlation_id: correlation_id.map(String::from),
        payload: Some(
            serde_json::to_value(ErrorPayload {
                code,
                message: message.to_string(),
                details,
            })
            .unwrap(),
        ),
        meta: None,
    }
}

// Extract error payload from an envelope
pub fn extract_error_payload(envelope: &MessageEnvelope) -> Result<ErrorPayload, String> {
    if envelope.message_type != MSG_TYPE_ERROR {
        return Err(format!(
            "Expected {} message, got {}",
            MSG_TYPE_ERROR, envelope.message_type
        ));
    }

    let payload = envelope
        .payload
        .as_ref()
        .ok_or("Error message requires payload")?;

    serde_json::from_value(payload.clone())
        .map_err(|e| format!("Invalid error payload: {}", e))
}


// Capabilities payload 
#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct CapabilitiesPayload {
    pub versions: Vec<String>,
}

// Creates an agent.capabilities message 
pub fn create_capabilities() -> MessageEnvelope {
    MessageEnvelope {
        message_type: MSG_TYPE_CAPABILITIES.to_string(),
        version: PROTOCOL_VERSION.to_string(),
        id: generate_id(),
        timestamp: generate_timestamp(),
        correlation_id: None,
        payload: Some(
            serde_json::to_value(CapabilitiesPayload {
                versions: SUPPORTED_VERSIONS.iter().map(|s| s.to_string()).collect(),
            })
            .unwrap(),
        ),
        meta: None,
    }
}

// Extract capabilities payload from an envelope
pub fn extract_capabilities_payload(
    envelope: &MessageEnvelope,
) -> Result<CapabilitiesPayload, String> {
    if envelope.message_type != MSG_TYPE_CAPABILITIES {
        return Err(format!(
            "Expected {} message, got {}",
            MSG_TYPE_CAPABILITIES, envelope.message_type
        ));
    }

    let payload = envelope
        .payload
        .as_ref()
        .ok_or("Capabilities message requires payload")?;

    serde_json::from_value(payload.clone())
        .map_err(|e| format!("Invalid capabilities payload: {}", e))
}

// Check if a version is supported 
pub fn is_version_supported(version: &str) -> bool {
    SUPPORTED_VERSIONS.contains(&version)
}

// Find the highest compatible version between local and remote capabilities
pub fn negotiate_version(remote_versions: &[String]) -> Option<String> {
    let mut compatible: Vec<&str> = remote_versions
        .iter()
        .filter(|v| SUPPORTED_VERSIONS.contains(&v.as_str()))
        .map(|v| v.as_str())
        .collect();

    compatible.sort();
    compatible.last().map(|s| s.to_string())
}


// Parse incoming JSON into a MessageEnvelope
pub fn parse_message(json_str: &str) -> Result<MessageEnvelope, String> {
    serde_json::from_str(json_str).map_err(|e| format!("Failed to parse message: {}", e))
}

// Validate that an envelope has all required fields and checks required fields are not empty
pub fn validate_envelope(envelope: &MessageEnvelope) -> Result<(), String> {
    if envelope.message_type.is_empty() {
        return Err("Missing required field: type".to_string());
    }
    if envelope.version.is_empty() {
        return Err("Missing required field: version".to_string());
    }
    if envelope.id.is_empty() {
        return Err("Missing required field: id".to_string());
    }
    if envelope.timestamp.is_empty() {
        return Err("Missing required field: timestamp".to_string());
    }

    let version_parts: Vec<&str> = envelope.version.split('.').collect();
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

// Route a message based on its type and return appropriate response
pub fn handle_message(envelope: &MessageEnvelope) -> Option<MessageEnvelope> {
    match envelope.message_type.as_str() {
        MSG_TYPE_PING => {
            Some(create_pong(&envelope.id))
        }
        MSG_TYPE_PONG => {
            println!("Received pong for message: {:?}", envelope.correlation_id);
            None
        }
        MSG_TYPE_MOVEMENT => {
            match extract_movement_command(envelope) {
                Ok(movement) => {
                    println!(
                        "Movement command: forward={}, turn={}", 
                        movement.forward, movement.turn
                    );
                    None
                }
                Err(e) => {
                    Some(create_error(
                        ErrorCode::InvalidPayload,
                        &e,
                        Some(&envelope.id),
                        None,
                    ))
                }
            }
        }
        MSG_TYPE_ERROR => {
            match extract_error_payload(envelope) {
                Ok(error) => {
                    println!(
                        "Received error: code={:?}, message={}",
                        error.code, error.message
                    );
                }
                Err(e) => {
                    println!("Failed to parse error message: {}", e);
                }
            }
            None
        }
        MSG_TYPE_CAPABILITIES => {
            match extract_capabilities_payload(envelope) {
                Ok(caps) => {
                    println!("Received capabilities: versions={:?}", caps.versions);
                    if let Some(version) = negotiate_version(&caps.versions) {
                        println!("Negotiated version: {}", version);
                    } else {
                        return Some(create_error(
                            ErrorCode::CapabilityMismatch,
                            "No compatible protocol version found",
                            Some(&envelope.id),
                            None,
                        ));
                    }
                }
                Err(e) => {
                    return Some(create_error(
                        ErrorCode::InvalidPayload,
                        &e,
                        Some(&envelope.id),
                        None,
                    ));
                }
            }
            Some(create_capabilities())
        }
        _ => {
            Some(create_error(
                ErrorCode::UnsupportedMessageType,
                &format!("Unknown message type: {}", envelope.message_type),
                Some(&envelope.id),
                None,
            ))
        }
    }
}







//unit tests 
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_create_ping() {
        let ping = create_ping();
        assert_eq!(ping.message_type, MSG_TYPE_PING);
        assert_eq!(ping.version, PROTOCOL_VERSION);
        assert!(!ping.id.is_empty());
        assert!(!ping.timestamp.is_empty());
        assert!(ping.payload.is_none());
    }

    #[test]
    fn test_create_pong() {
        let pong = create_pong("original-ping-id");
        assert_eq!(pong.message_type, MSG_TYPE_PONG);
        assert_eq!(pong.version, PROTOCOL_VERSION);
        assert_eq!(pong.correlation_id, Some("original-ping-id".to_string()));
        assert!(pong.payload.is_none());
    }

    #[test]
    fn test_create_movement() {
        let movement = create_movement(0.5, -0.3).unwrap();
        assert_eq!(movement.message_type, MSG_TYPE_MOVEMENT);
        assert!(movement.payload.is_some());
    }

    #[test]
    fn test_create_movement_boundary_values() {
        assert!(create_movement(1.0, 1.0).is_ok());
        assert!(create_movement(-1.0, -1.0).is_ok());
        assert!(create_movement(0.0, 0.0).is_ok());
    }

    #[test]
    fn test_create_error() {
        let error = create_error(
            ErrorCode::MovementFailed,
            "Robot hit obstacle",
            Some("msg-123"),
            None,0.1
        );
        assert_eq!(error.message_type, MSG_TYPE_ERROR);
        assert_eq!(error.correlation_id, Some("msg-123".to_string()));
    }

    #[test]
    fn test_create_capabilities() {
        let caps = create_capabilities();
        assert_eq!(caps.message_type, MSG_TYPE_CAPABILITIES);
        assert!(caps.payload.is_some());
    }

    #[test]
    fn test_ping_roundtrip() {
        let original = create_ping();
        let json = serde_json::to_string(&original).unwrap();
        let parsed: MessageEnvelope = serde_json::from_str(&json).unwrap();
        
        assert_eq!(original.message_type, parsed.message_type);
        assert_eq!(original.id, parsed.id);
        assert_eq!(original.version, parsed.version);
    }

    #[test]
    fn test_movement_roundtrip() {
        let original = create_movement(0.75, -0.25).unwrap();
        let json = serde_json::to_string(&original).unwrap();
        let parsed: MessageEnvelope = serde_json::from_str(&json).unwrap();
        
        assert_eq!(original.message_type, parsed.message_type);
        assert_eq!(original.id, parsed.id);
        
        let orig_payload = extract_movement_command(&original).unwrap();
        let parsed_payload = extract_movement_command(&parsed).unwrap();
        assert_eq!(orig_payload.forward, parsed_payload.forward);
        assert_eq!(orig_payload.turn, parsed_payload.turn);
    }

    #[test]
    fn test_error_roundtrip() {
        let original = create_error(
            ErrorCode::ValidationFailed,
            "Invalid input",
            None,
            None,
        );
        let json = serde_json::to_string(&original).unwrap();
        let parsed: MessageEnvelope = serde_json::from_str(&json).unwrap();
        
        assert_eq!(original.message_type, parsed.message_type);
        assert_eq!(original.id, parsed.id);
    }

    #[test]
    fn test_json_field_names() {
        let ping = create_ping();
        let json = serde_json::to_string(&ping).unwrap();
        
        assert!(json.contains("\"type\""));
        assert!(!json.contains("\"message_type\""));
        
        let pong = create_pong("test");
        let pong_json = serde_json::to_string(&pong).unwrap();
        assert!(pong_json.contains("\"correlationId\""));
        assert!(!pong_json.contains("\"correlation_id\""));
    }


    #[test]
    fn test_extract_movement_command() {
        let msg = create_movement(0.6, -0.2).unwrap();
        let payload = extract_movement_command(&msg).unwrap();
        
        assert_eq!(payload.forward, 0.6);
        assert_eq!(payload.turn, -0.2);
    }

    #[test]
    fn test_extract_error_payload() {
        let msg = create_error(
            ErrorCode::MovementFailed,
            "Test error",
            None,
            None,
        );
        let payload = extract_error_payload(&msg).unwrap();
        
        assert_eq!(payload.code, ErrorCode::MovementFailed);
        assert_eq!(payload.message, "Test error");
    }

    #[test]
    fn test_extract_capabilities_payload() {
        let msg = create_capabilities();
        let payload = extract_capabilities_payload(&msg).unwrap();
        
        assert!(!payload.versions.is_empty());
        assert!(payload.versions.contains(&"0.0".to_string()));
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
        assert_eq!(msg.message_type, MSG_TYPE_PING);
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
        assert_eq!(msg.message_type, MSG_TYPE_MOVEMENT);
        
        let payload = extract_movement_command(&msg).unwrap();
        assert_eq!(payload.forward, 0.5);
        assert_eq!(payload.turn, -0.3);
    }

    #[test]
    fn test_parse_invalid_json() {
        let json = r#"{ invalid json }"#;
        let result = parse_message(json);
        assert!(result.is_err());
    }


    #[test]
    fn test_validate_envelope_valid() {
        let msg = create_ping();
        assert!(validate_envelope(&msg).is_ok());
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
        
        let result = validate_envelope(&msg);
        assert!(result.is_err());
        assert!(result.unwrap_err().contains("type"));
    }

    #[test]
    fn test_validate_envelope_bad_version() {
        let msg = MessageEnvelope {
            message_type: MSG_TYPE_PING.to_string(),
            version: "0.0.2".to_string(),
            id: "123".to_string(),
            timestamp: "2024-01-01T00:00:00Z".to_string(),
            correlation_id: None,
            payload: None,
            meta: None,
        };
        
        let result = validate_envelope(&msg);
        assert!(result.is_err());
        assert!(result.unwrap_err().contains("version"));
    }

    #[test]
    fn test_version_format_matches_schema() {
        let ping = create_ping();
        let version = &ping.version;
        let parts: Vec<&str> = version.split('.').collect();
        
        assert_eq!(parts.len(), 2, "Version must be MAJOR.MINOR format");
        assert!(parts[0].parse::<u32>().is_ok(), "MAJOR version must be numeric");
        assert!(parts[1].parse::<u32>().is_ok(), "MINOR version must be numeric");
    }


    #[test]
    fn test_is_version_supported() {
        assert!(is_version_supported("0.0"));
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
        let ping = create_ping();
        let response = handle_message(&ping);
        
        assert!(response.is_some());
        let pong = response.unwrap();
        assert_eq!(pong.message_type, MSG_TYPE_PONG);
        assert_eq!(pong.correlation_id, Some(ping.id.clone()));
    }

    #[test]
    fn test_handle_pong_returns_none() {
        let pong = create_pong("test-id");
        let response = handle_message(&pong);
        assert!(response.is_none());
    }

    #[test]
    fn test_handle_movement_valid() {
        let movement = create_movement(0.5, 0.3).unwrap();
        let response = handle_message(&movement);
        assert!(response.is_none());
    }

    #[test]
    fn test_handle_unknown_message_type() {
        let mut msg = create_ping();
        msg.message_type = "agent.unknown".to_string();
        let response = handle_message(&msg);

        assert!(response.is_some());
        let error = response.unwrap();
        assert_eq!(error.message_type, MSG_TYPE_ERROR);
    }
}