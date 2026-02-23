use serde::{Deserialize, Serialize};
use std::str::FromStr;

use crate::webrtc_message::common::MessageEnvelopeError;

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

impl FromStr for MessageEnvelope {
    type Err = MessageEnvelopeError;
    fn from_str(json_str: &str) -> Result<Self, Self::Err> {
        serde_json::from_str(json_str).map_err(|e| MessageEnvelopeError::JsonParse {
            reason: e.to_string(),
        })
    }
}

impl MessageEnvelope {
    pub fn validate(&self) -> Result<(), MessageEnvelopeError> {
        if self.message_type.is_empty() {
            return Err(MessageEnvelopeError::EnvelopeValidation {
                reason: "missing required field: type".to_string(),
            });
        }
        if self.version.is_empty() {
            return Err(MessageEnvelopeError::EnvelopeValidation {
                reason: "missing required field: version".to_string(),
            });
        }
        if self.id.is_empty() {
            return Err(MessageEnvelopeError::EnvelopeValidation {
                reason: "missing required field: id".to_string(),
            });
        }
        if self.timestamp.is_empty() {
            return Err(MessageEnvelopeError::EnvelopeValidation {
                reason: "missing required field: timestamp".to_string(),
            });
        }

        Ok(())
    }

    pub fn to_string(&self) -> Result<String, MessageEnvelopeError> {
        serde_json::to_string(self).map_err(|e| MessageEnvelopeError::JsonEncode {
            reason: e.to_string(),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_valid_envelope_validates() {
        let envelope = MessageEnvelope {
            message_type: "fake.type".to_string(),
            version: "0.0".to_string(),
            id: "test-id".to_string(),
            timestamp: "2024-01-01T00:00:00Z".to_string(),
            correlation_id: None,
            payload: None,
            meta: None,
        };
        assert!(envelope.validate().is_ok());
    }

    #[test]
    fn test_missing_type_envelope_doesnt_validate() {
        let envelope = MessageEnvelope {
            message_type: "".to_string(),
            version: "0.0".to_string(),
            id: "test-id".to_string(),
            timestamp: "2024-01-01T00:00:00Z".to_string(),
            correlation_id: None,
            payload: None,
            meta: None,
        };
        let result = envelope.validate();
        assert!(result.is_err());
        assert!(matches!(
            result.unwrap_err(),
            MessageEnvelopeError::EnvelopeValidation { .. }
        ));
    }

    #[test]
    fn test_valid_message_parses() {
        let json = r#"{
            "type": "agent.ping",
            "version": "0.0",
            "id": "test-id",
            "timestamp": "2024-01-01T00:00:00Z"
        }"#;
        let result = MessageEnvelope::from_str(json);
        assert!(result.is_ok());
        assert_eq!(result.unwrap().message_type, "agent.ping");
    }

    #[test]
    fn test_invalid_message_doesnt_parse() {
        let json = "{ invalid json }";
        let result = MessageEnvelope::from_str(json);
        assert!(result.is_err());
    }
}
