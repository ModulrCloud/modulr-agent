use serde::{Deserialize, Serialize};

use crate::common::{MessageEnvelope, MessageEnvelopeError, MessageFields};

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct AnswerPayload {
    pub connection_id: String,
    pub sdp: String,
    pub sdp_type: String,
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct CapabilitiesPayload {
    versions: Vec<String>,
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
#[serde(rename_all = "camelCase")]
pub enum IceConnectionState {
    Connected,
    Completed,
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
#[serde(rename_all = "camelCase")]
pub enum DataChannelState {
    Open,
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct ConnectedPayload {
    pub connection_id: String,
    pub ice_connection_state: IceConnectionState,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub data_channel_state: Option<DataChannelState>,
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
#[serde(rename_all = "camelCase")]
pub enum DisconnectionReason {
    Closed,
    Failed,
    Timeout,
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct DisconnectedPayload {
    pub connection_id: String,
    pub reason: DisconnectionReason,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub ice_connection_state: Option<IceConnectionState>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub details: Option<serde_json::Value>,
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum ErrorCode {
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
#[serde(rename_all = "camelCase")]
pub struct ErrorPayload {
    pub code: ErrorCode,
    pub message: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub details: Option<serde_json::Value>,
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct IceCandidatePayload {
    pub candidate: String,
    pub connection_id: String,
    pub sdp_mid: String,
    pub sdp_m_line_index: u16,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub username_fragment: Option<String>,
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
pub struct RobotCapabilities {
    video_codecs: Vec<String>,
    audio_codecs: Vec<String>,
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct RegisterPayload {
    pub agent_id: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub capabilities: Option<RobotCapabilities>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub metadata: Option<serde_json::Value>,
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct PkiChallengePayload {
    pub challenge: String,
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct PkiResponsePayload {
    #[serde(skip)]
    pub correlation_id: String,
    pub signature: String,
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct PkiVerifiedPayload {
    #[serde(skip)]
    pub correlation_id: String,
    pub agent_id: String,
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub enum SignalingMessage {
    Answer(AnswerPayload),
    Capabilities(CapabilitiesPayload),
    Connected(ConnectedPayload),
    Disconnected(DisconnectedPayload),
    Error(ErrorPayload),
    IceCandidate(IceCandidatePayload),
    Offer(OfferPayload),
    Register(RegisterPayload),
    PkiChallenge(PkiChallengePayload),
    PkiResponse(PkiResponsePayload),
    PkiVerified(PkiVerifiedPayload),
}

impl MessageFields for SignalingMessage {
    fn name(&self) -> String {
        match self {
            SignalingMessage::Answer(_) => "signalling.answer",
            SignalingMessage::Capabilities(_) => "signalling.capabilities",
            SignalingMessage::Connected(_) => "signalling.connected",
            SignalingMessage::Disconnected(_) => "signalling.disconnected",
            SignalingMessage::Error(_) => "signalling.error",
            SignalingMessage::IceCandidate(_) => "signalling.ice_candidate",
            SignalingMessage::Offer(_) => "signalling.offer",
            SignalingMessage::Register(_) => "signalling.register",
            SignalingMessage::PkiChallenge(_) => "signalling.pki_challenge",
            SignalingMessage::PkiResponse(_) => "signalling.pki_response",
            SignalingMessage::PkiVerified(_) => "signalling.pki_verified",
        }
        .to_string()
    }

    fn correlation_id(&self) -> Option<String> {
        match self {
            SignalingMessage::PkiResponse(response) => Some(response.correlation_id.to_string()),
            SignalingMessage::PkiVerified(response) => Some(response.correlation_id.to_string()),
            _ => None,
        }
    }

    fn payload(&self) -> Option<serde_json::Value> {
        match self {
            SignalingMessage::Answer(payload) => serde_json::to_value(payload).ok(),
            SignalingMessage::Capabilities(payload) => serde_json::to_value(payload).ok(),
            SignalingMessage::Connected(payload) => serde_json::to_value(payload).ok(),
            SignalingMessage::Disconnected(payload) => serde_json::to_value(payload).ok(),
            SignalingMessage::Error(payload) => serde_json::to_value(payload).ok(),
            SignalingMessage::IceCandidate(payload) => serde_json::to_value(payload).ok(),
            SignalingMessage::Offer(payload) => serde_json::to_value(payload).ok(),
            SignalingMessage::Register(payload) => serde_json::to_value(payload).ok(),
            SignalingMessage::PkiChallenge(payload) => serde_json::to_value(payload).ok(),
            SignalingMessage::PkiResponse(payload) => serde_json::to_value(payload).ok(),
            SignalingMessage::PkiVerified(payload) => serde_json::to_value(payload).ok(),
        }
    }

    fn meta(&self) -> Option<serde_json::Value> {
        None
    }
}

impl SignalingMessage {
    pub fn from_message(msg: &MessageEnvelope) -> Result<Self, MessageEnvelopeError> {
        msg.validate()?;

        match msg.message_type.as_str() {
            "signalling.answer" => {
                let payload = msg
                    .payload
                    .as_ref()
                    .ok_or(MessageEnvelopeError::MissingFields {
                        fields: vec!["payload".to_string()],
                    })?;

                let ans: AnswerPayload = serde_json::from_value(payload.clone()).map_err(|e| {
                    MessageEnvelopeError::JsonParse {
                        reason: e.to_string(),
                    }
                })?;

                if ans.sdp_type != "answer" {
                    return Err(MessageEnvelopeError::EnvelopeValidation {
                        reason: "sdpType field must be 'answer'".to_string(),
                    });
                }

                Ok(SignalingMessage::Answer(ans))
            }

            "signalling.capabilities" => {
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
                Ok(SignalingMessage::Capabilities(CapabilitiesPayload {
                    versions,
                }))
            }

            "signalling.connected" => {
                let payload = msg
                    .payload
                    .as_ref()
                    .ok_or(MessageEnvelopeError::MissingFields {
                        fields: vec!["payload".to_string()],
                    })?;

                let connected_payload: ConnectedPayload = serde_json::from_value(payload.clone())
                    .map_err(|e| {
                    MessageEnvelopeError::JsonParse {
                        reason: e.to_string(),
                    }
                })?;
                Ok(SignalingMessage::Connected(connected_payload))
            }

            "signalling.disconnected" => {
                let payload = msg
                    .payload
                    .as_ref()
                    .ok_or(MessageEnvelopeError::MissingFields {
                        fields: vec!["payload".to_string()],
                    })?;

                let disconnected_payload: DisconnectedPayload =
                    serde_json::from_value(payload.clone()).map_err(|e| {
                        MessageEnvelopeError::JsonParse {
                            reason: e.to_string(),
                        }
                    })?;
                Ok(SignalingMessage::Disconnected(disconnected_payload))
            }

            "signalling.error" => {
                let payload = msg
                    .payload
                    .as_ref()
                    .ok_or(MessageEnvelopeError::MissingFields {
                        fields: vec!["payload".to_string()],
                    })?;
                let error_payload: ErrorPayload =
                    serde_json::from_value(payload.clone()).map_err(|e| {
                        MessageEnvelopeError::JsonParse {
                            reason: e.to_string(),
                        }
                    })?;
                Ok(SignalingMessage::Error(error_payload))
            }

            "signalling.ice_candidate" => {
                let payload = msg
                    .payload
                    .as_ref()
                    .ok_or(MessageEnvelopeError::MissingFields {
                        fields: vec!["payload".to_string()],
                    })?;
                let ice: IceCandidatePayload =
                    serde_json::from_value(payload.clone()).map_err(|e| {
                        MessageEnvelopeError::JsonParse {
                            reason: e.to_string(),
                        }
                    })?;
                Ok(SignalingMessage::IceCandidate(ice))
            }

            "signalling.offer" => {
                let payload = msg
                    .payload
                    .as_ref()
                    .ok_or(MessageEnvelopeError::MissingFields {
                        fields: vec!["payload".to_string()],
                    })?;
                let offer: OfferPayload = serde_json::from_value(payload.clone()).map_err(|e| {
                    MessageEnvelopeError::JsonParse {
                        reason: e.to_string(),
                    }
                })?;
                Ok(SignalingMessage::Offer(offer))
            }

            "signalling.register" => {
                let payload = msg
                    .payload
                    .as_ref()
                    .ok_or(MessageEnvelopeError::MissingFields {
                        fields: vec!["payload".to_string()],
                    })?;
                let register_payload: RegisterPayload = serde_json::from_value(payload.clone())
                    .map_err(|e| MessageEnvelopeError::JsonParse {
                        reason: e.to_string(),
                    })?;
                Ok(SignalingMessage::Register(register_payload))
            }

            "signalling.pki_challenge" => {
                let payload = msg
                    .payload
                    .as_ref()
                    .ok_or(MessageEnvelopeError::MissingFields {
                        fields: vec!["payload".to_string()],
                    })?;
                let challenge_payload: PkiChallengePayload =
                    serde_json::from_value(payload.clone()).map_err(|e| {
                        MessageEnvelopeError::JsonParse {
                            reason: e.to_string(),
                        }
                    })?;
                Ok(SignalingMessage::PkiChallenge(challenge_payload))
            }

            "signalling.pki_response" => {
                let mut missing_fields = vec![];
                if msg.correlation_id.is_none() {
                    missing_fields.push("correlationId".to_string());
                }
                if msg.payload.is_none() {
                    missing_fields.push("payload".to_string());
                }
                if !missing_fields.is_empty() {
                    return Err(MessageEnvelopeError::MissingFields {
                        fields: missing_fields,
                    });
                }

                let correlation_id = msg
                    .correlation_id
                    .as_ref()
                    .expect("Failed to retrieve correlation ID despite checking field is present!");
                let payload = msg
                    .payload
                    .as_ref()
                    .expect("Failed to retrieve payload despite checking field is present!");

                let signature =
                    payload["signature"]
                        .as_str()
                        .ok_or(MessageEnvelopeError::MissingFields {
                            fields: vec!["signature".to_string()],
                        })?;

                let response_payload = PkiResponsePayload {
                    correlation_id: correlation_id.clone(),
                    signature: signature.to_string(),
                };

                Ok(SignalingMessage::PkiResponse(response_payload))
            }

            "signalling.pki_verified" => {
                let mut missing_fields = vec![];
                if msg.correlation_id.is_none() {
                    missing_fields.push("correlationId".to_string());
                }
                if msg.payload.is_none() {
                    missing_fields.push("payload".to_string());
                }
                if !missing_fields.is_empty() {
                    return Err(MessageEnvelopeError::MissingFields {
                        fields: missing_fields,
                    });
                }

                let correlation_id = msg
                    .correlation_id
                    .as_ref()
                    .expect("Failed to retrieve correlation ID despite checking field is present!");
                let payload = msg
                    .payload
                    .as_ref()
                    .expect("Failed to retrieve payload despite checking field is present!");

                let mut verified_payload: PkiVerifiedPayload =
                    serde_json::from_value(payload.clone()).map_err(|e| {
                        MessageEnvelopeError::JsonParse {
                            reason: e.to_string(),
                        }
                    })?;
                verified_payload.correlation_id = correlation_id.clone();

                Ok(SignalingMessage::PkiVerified(verified_payload))
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

    fn make_envelope(message_type: &str, payload: Option<serde_json::Value>) -> MessageEnvelope {
        MessageEnvelope {
            message_type: message_type.to_string(),
            version: "0.0".to_string(),
            id: "test-id".to_string(),
            timestamp: "2024-01-01T00:00:00Z".to_string(),
            correlation_id: None,
            payload,
            meta: None,
        }
    }

    // --- Serialisation tests ---

    #[test]
    fn test_offer_serialises_correctly() {
        let msg = SignalingMessage::Offer(OfferPayload {
            connection_id: "conn-1".to_string(),
            sdp: "v=0...".to_string(),
            sdp_type: "offer".to_string(),
            ice_restart: Some(true),
        });
        let envelope = msg.to_message();
        assert_eq!(envelope.message_type, "signalling.offer");
        let payload = envelope.payload.unwrap();
        assert_eq!(payload["sdpType"], "offer");
        assert_eq!(payload["iceRestart"], true);
    }

    #[test]
    fn test_register_serialises_correctly() {
        let msg = SignalingMessage::Register(RegisterPayload {
            agent_id: "agent-1".to_string(),
            capabilities: None,
            metadata: None,
        });
        let envelope = msg.to_message();
        assert_eq!(envelope.message_type, "signalling.register");
        let payload = envelope.payload.unwrap();
        assert_eq!(payload["agentId"], "agent-1");
    }

    #[test]
    fn test_answer_serialises_correctly() {
        let msg = SignalingMessage::Answer(AnswerPayload {
            connection_id: "conn-1".to_string(),
            sdp: "v=0...".to_string(),
            sdp_type: "answer".to_string(),
        });
        let envelope = msg.to_message();
        assert_eq!(envelope.message_type, "signalling.answer");
        assert!(envelope.correlation_id.is_none());
        let payload = envelope.payload.unwrap();
        assert_eq!(payload["connectionId"], "conn-1");
        assert_eq!(payload["sdp"], "v=0...");
        assert_eq!(payload["sdpType"], "answer");
    }

    #[test]
    fn test_capabilities_serialises_correctly() {
        let msg = SignalingMessage::Capabilities(CapabilitiesPayload {
            versions: vec!["0.0".to_string()],
        });
        let envelope = msg.to_message();
        assert_eq!(envelope.message_type, "signalling.capabilities");
        let payload = envelope.payload.unwrap();
        assert_eq!(payload["versions"], json!(["0.0"]));
    }

    #[test]
    fn test_connected_serialises_correctly() {
        let msg = SignalingMessage::Connected(ConnectedPayload {
            connection_id: "conn-1".to_string(),
            ice_connection_state: IceConnectionState::Connected,
            data_channel_state: Some(DataChannelState::Open),
        });
        let envelope = msg.to_message();
        assert_eq!(envelope.message_type, "signalling.connected");
        let payload = envelope.payload.unwrap();
        assert_eq!(payload["connectionId"], "conn-1");
        assert_eq!(payload["iceConnectionState"], "connected");
        assert_eq!(payload["dataChannelState"], "open");
    }

    #[test]
    fn test_connected_omits_optional_data_channel_state() {
        let msg = SignalingMessage::Connected(ConnectedPayload {
            connection_id: "conn-1".to_string(),
            ice_connection_state: IceConnectionState::Completed,
            data_channel_state: None,
        });
        let payload = msg.to_message().payload.unwrap();
        assert!(payload.get("dataChannelState").is_none());
    }

    #[test]
    fn test_disconnected_serialises_correctly() {
        let msg = SignalingMessage::Disconnected(DisconnectedPayload {
            connection_id: "conn-1".to_string(),
            reason: DisconnectionReason::Failed,
            ice_connection_state: Some(IceConnectionState::Connected),
            details: None,
        });
        let envelope = msg.to_message();
        assert_eq!(envelope.message_type, "signalling.disconnected");
        let payload = envelope.payload.unwrap();
        assert_eq!(payload["connectionId"], "conn-1");
        assert_eq!(payload["reason"], "failed");
    }

    #[test]
    fn test_error_serialises_correctly() {
        let msg = SignalingMessage::Error(ErrorPayload {
            code: ErrorCode::SdpInvalid,
            message: "bad sdp".to_string(),
            details: None,
        });
        let envelope = msg.to_message();
        assert_eq!(envelope.message_type, "signalling.error");
        let payload = envelope.payload.unwrap();
        assert_eq!(payload["code"], "SDP_INVALID");
        assert_eq!(payload["message"], "bad sdp");
    }

    #[test]
    fn test_error_code_serialisation() {
        let code = ErrorCode::ConnectionFailed;
        let json = serde_json::to_string(&code).unwrap();
        assert_eq!(json, "\"CONNECTION_FAILED\"");
    }

    #[test]
    fn test_error_code_deserialisation() {
        let json = "\"ICE_FAILED\"";
        let code: ErrorCode = serde_json::from_str(json).unwrap();
        assert_eq!(code, ErrorCode::IceFailed);
    }

    #[test]
    fn test_ice_candidate_serialises_correctly() {
        let msg = SignalingMessage::IceCandidate(IceCandidatePayload {
            candidate: "candidate:1 ...".to_string(),
            connection_id: "conn-1".to_string(),
            sdp_mid: "0".to_string(),
            sdp_m_line_index: 0,
            username_fragment: None,
        });
        let envelope = msg.to_message();
        assert_eq!(envelope.message_type, "signalling.ice_candidate");
        let payload = envelope.payload.unwrap();
        assert_eq!(payload["candidate"], "candidate:1 ...");
        assert_eq!(payload["connectionId"], "conn-1");
        assert_eq!(payload["sdpMid"], "0");
        assert_eq!(payload["sdpMLineIndex"], 0);
        assert!(payload.get("usernameFragment").is_none());
    }

    #[test]
    fn test_ice_candidate_payload_json_key_names() {
        // Verify payload serializes with standard WebRTC field names per schema
        let ice = IceCandidatePayload {
            candidate: "candidate:1 1 udp 2130706431 192.168.1.1 12345 typ host".to_string(),
            connection_id: "conn-1".to_string(),
            sdp_mid: "0".to_string(),
            sdp_m_line_index: 0,
            username_fragment: Some("frag".to_string()),
        };
        let json_value = serde_json::to_value(&ice).unwrap();
        let obj = json_value.as_object().unwrap();

        assert!(obj.contains_key("candidate"), "missing 'candidate' key");
        assert!(
            obj.contains_key("connectionId"),
            "missing 'connectionId' key"
        );
        assert!(obj.contains_key("sdpMid"), "missing 'sdpMid' key");
        assert!(
            obj.contains_key("sdpMLineIndex"),
            "missing 'sdpMLineIndex' key (got keys: {:?})",
            obj.keys().collect::<Vec<_>>()
        );
        assert!(
            obj.contains_key("usernameFragment"),
            "missing 'usernameFragment' key"
        );

        // candidate must be a string, not a nested object
        assert!(obj["candidate"].is_string(), "candidate must be a string");
        assert_eq!(obj["sdpMid"], "0");
        assert_eq!(obj["sdpMLineIndex"], 0);
    }

    #[test]
    fn test_ice_candidate_full_round_trip() {
        // Simulate: create candidate → serialize to JSON string → parse back
        let original = SignalingMessage::IceCandidate(IceCandidatePayload {
            candidate: "candidate:1 1 udp 2130706431 192.168.1.1 12345 typ host".to_string(),
            connection_id: "conn-1".to_string(),
            sdp_mid: "0".to_string(),
            sdp_m_line_index: 0,
            username_fragment: Some("frag".to_string()),
        });

        // Serialize to JSON string (same path as send_signaling_message)
        let json_str = original.to_message().to_string().unwrap();

        // Deserialize back (same path as listen loop)
        let envelope = MessageEnvelope::from_str(&json_str).unwrap();
        let parsed = SignalingMessage::from_message(&envelope).unwrap();

        match parsed {
            SignalingMessage::IceCandidate(ice) => {
                assert_eq!(ice.connection_id, "conn-1");
                assert_eq!(
                    ice.candidate,
                    "candidate:1 1 udp 2130706431 192.168.1.1 12345 typ host"
                );
                assert_eq!(ice.sdp_mid, "0");
                assert_eq!(ice.sdp_m_line_index, 0);
                assert_eq!(ice.username_fragment, Some("frag".to_string()));
            }
            other => panic!("Expected IceCandidate, got {:?}", other),
        }
    }

    #[test]
    fn test_ice_candidate_round_trip_empty_sdp_mid() {
        // webrtc-rs often returns sdp_mid: Some("") — verify this round-trips correctly
        let original = SignalingMessage::IceCandidate(IceCandidatePayload {
            candidate: "candidate:1 1 udp 2130706431 192.168.1.1 12345 typ host".to_string(),
            connection_id: "conn-1".to_string(),
            sdp_mid: "".to_string(),
            sdp_m_line_index: 0,
            username_fragment: None,
        });

        let json_str = original.to_message().to_string().unwrap();
        let envelope = MessageEnvelope::from_str(&json_str).unwrap();
        let parsed = SignalingMessage::from_message(&envelope).unwrap();

        match parsed {
            SignalingMessage::IceCandidate(ice) => {
                assert_eq!(ice.sdp_mid, "");
                assert_eq!(ice.username_fragment, None);
            }
            other => panic!("Expected IceCandidate, got {:?}", other),
        }
    }

    #[test]
    fn test_answer_full_round_trip() {
        let original = SignalingMessage::Answer(AnswerPayload {
            connection_id: "conn-1".to_string(),
            sdp: "v=0\r\no=- 12345 2 IN IP4 127.0.0.1\r\n".to_string(),
            sdp_type: "answer".to_string(),
        });

        let json_str = original.to_message().to_string().unwrap();
        let envelope = MessageEnvelope::from_str(&json_str).unwrap();
        let parsed = SignalingMessage::from_message(&envelope).unwrap();

        match parsed {
            SignalingMessage::Answer(ans) => {
                assert_eq!(ans.connection_id, "conn-1");
                assert_eq!(ans.sdp, "v=0\r\no=- 12345 2 IN IP4 127.0.0.1\r\n");
                assert_eq!(ans.sdp_type, "answer");
            }
            other => panic!("Expected Answer, got {:?}", other),
        }
    }

    #[test]
    fn test_base_envelope_has_all_fields() {
        let msg = SignalingMessage::Register(RegisterPayload {
            agent_id: "agent-1".to_string(),
            capabilities: None,
            metadata: None,
        });
        let json = msg.to_message().to_string().unwrap();
        assert!(json.contains("\"type\""));
        assert!(!json.contains("\"message_type\""));
        assert!(!json.contains("\"correlation_id\""));
    }

    // --- Deserialisation (from_message) tests ---

    #[test]
    fn test_answer_parses_correctly() {
        let envelope = make_envelope(
            "signalling.answer",
            Some(json!({
                "connectionId": "conn-1",
                "sdp": "v=0...",
                "sdpType": "answer"
            })),
        );
        let msg = SignalingMessage::from_message(&envelope).unwrap();
        match msg {
            SignalingMessage::Answer(a) => {
                assert_eq!(a.connection_id, "conn-1");
                assert_eq!(a.sdp, "v=0...");
                assert_eq!(a.sdp_type, "answer");
            }
            other => panic!("Expected Answer, got {:?}", other),
        }
    }

    #[test]
    fn test_answer_rejects_wrong_sdp_type() {
        let envelope = make_envelope(
            "signalling.answer",
            Some(json!({
                "connectionId": "conn-1",
                "sdp": "v=0...",
                "sdpType": "offer"
            })),
        );
        let err = SignalingMessage::from_message(&envelope).unwrap_err();
        assert!(matches!(
            err,
            MessageEnvelopeError::EnvelopeValidation { .. }
        ));
    }

    #[test]
    fn test_answer_missing_payload() {
        let envelope = make_envelope("signalling.answer", None);
        let err = SignalingMessage::from_message(&envelope).unwrap_err();
        assert!(matches!(err, MessageEnvelopeError::MissingFields { .. }));
    }

    #[test]
    fn test_answer_missing_fields() {
        let envelope = make_envelope("signalling.answer", Some(json!({ "sdpType": "answer" })));
        let err = SignalingMessage::from_message(&envelope).unwrap_err();
        assert!(matches!(err, MessageEnvelopeError::JsonParse { .. }));
    }

    #[test]
    fn test_capabilities_parses_correctly() {
        let envelope = make_envelope(
            "signalling.capabilities",
            Some(json!({ "versions": ["0.0", "1.0"] })),
        );
        let msg = SignalingMessage::from_message(&envelope).unwrap();
        match msg {
            SignalingMessage::Capabilities(c) => {
                let payload = serde_json::to_value(&c).unwrap();
                assert_eq!(payload["versions"], json!(["0.0", "1.0"]));
            }
            other => panic!("Expected Capabilities, got {:?}", other),
        }
    }

    #[test]
    fn test_capabilities_missing_payload() {
        let envelope = make_envelope("signalling.capabilities", None);
        let err = SignalingMessage::from_message(&envelope).unwrap_err();
        assert!(matches!(err, MessageEnvelopeError::MissingFields { .. }));
    }

    #[test]
    fn test_capabilities_missing_versions() {
        let envelope = make_envelope("signalling.capabilities", Some(json!({})));
        let err = SignalingMessage::from_message(&envelope).unwrap_err();
        assert!(matches!(err, MessageEnvelopeError::MissingFields { .. }));
    }

    #[test]
    fn test_connected_parses_correctly() {
        let envelope = make_envelope(
            "signalling.connected",
            Some(json!({
                "connectionId": "conn-1",
                "iceConnectionState": "connected",
                "dataChannelState": "open"
            })),
        );
        let msg = SignalingMessage::from_message(&envelope).unwrap();
        match msg {
            SignalingMessage::Connected(c) => {
                assert_eq!(c.connection_id, "conn-1");
                assert_eq!(c.ice_connection_state, IceConnectionState::Connected);
                assert_eq!(c.data_channel_state, Some(DataChannelState::Open));
            }
            other => panic!("Expected Connected, got {:?}", other),
        }
    }

    #[test]
    fn test_connected_without_data_channel_state() {
        let envelope = make_envelope(
            "signalling.connected",
            Some(json!({
                "connectionId": "conn-1",
                "iceConnectionState": "completed"
            })),
        );
        let msg = SignalingMessage::from_message(&envelope).unwrap();
        match msg {
            SignalingMessage::Connected(c) => {
                assert_eq!(c.ice_connection_state, IceConnectionState::Completed);
                assert_eq!(c.data_channel_state, None);
            }
            other => panic!("Expected Connected, got {:?}", other),
        }
    }

    #[test]
    fn test_connected_missing_payload() {
        let envelope = make_envelope("signalling.connected", None);
        let err = SignalingMessage::from_message(&envelope).unwrap_err();
        assert!(matches!(err, MessageEnvelopeError::MissingFields { .. }));
    }

    #[test]
    fn test_connected_invalid_payload() {
        let envelope = make_envelope("signalling.connected", Some(json!({ "wrong": "fields" })));
        let err = SignalingMessage::from_message(&envelope).unwrap_err();
        assert!(matches!(err, MessageEnvelopeError::JsonParse { .. }));
    }

    #[test]
    fn test_disconnected_parses_correctly() {
        let envelope = make_envelope(
            "signalling.disconnected",
            Some(json!({
                "connectionId": "conn-1",
                "reason": "failed",
                "iceConnectionState": "connected",
                "details": { "info": "timeout" }
            })),
        );
        let msg = SignalingMessage::from_message(&envelope).unwrap();
        match msg {
            SignalingMessage::Disconnected(d) => {
                assert_eq!(d.connection_id, "conn-1");
                assert_eq!(d.reason, DisconnectionReason::Failed);
                assert_eq!(d.ice_connection_state, Some(IceConnectionState::Connected));
                assert_eq!(d.details.unwrap()["info"], "timeout");
            }
            other => panic!("Expected Disconnected, got {:?}", other),
        }
    }

    #[test]
    fn test_disconnected_missing_payload() {
        let envelope = make_envelope("signalling.disconnected", None);
        let err = SignalingMessage::from_message(&envelope).unwrap_err();
        assert!(matches!(err, MessageEnvelopeError::MissingFields { .. }));
    }

    #[test]
    fn test_disconnected_invalid_reason_returns_error() {
        let envelope = make_envelope(
            "signalling.disconnected",
            Some(json!({
                "connectionId": "conn-1",
                "reason": "invalid_reason"
            })),
        );
        let err = SignalingMessage::from_message(&envelope).unwrap_err();
        assert!(matches!(err, MessageEnvelopeError::JsonParse { .. }));
    }

    #[test]
    fn test_error_parses_correctly() {
        let envelope = make_envelope(
            "signalling.error",
            Some(json!({
                "code": "ICE_FAILED",
                "message": "ice negotiation failed",
                "details": { "candidate": "none" }
            })),
        );
        let msg = SignalingMessage::from_message(&envelope).unwrap();
        match msg {
            SignalingMessage::Error(e) => {
                let payload = serde_json::to_value(&e).unwrap();
                assert_eq!(payload["code"], "ICE_FAILED");
                assert_eq!(payload["message"], "ice negotiation failed");
                assert_eq!(payload["details"]["candidate"], "none");
            }
            other => panic!("Expected Error, got {:?}", other),
        }
    }

    #[test]
    fn test_error_missing_payload() {
        let envelope = make_envelope("signalling.error", None);
        let err = SignalingMessage::from_message(&envelope).unwrap_err();
        assert!(matches!(err, MessageEnvelopeError::MissingFields { .. }));
    }

    #[test]
    fn test_error_missing_code() {
        let envelope = make_envelope("signalling.error", Some(json!({ "message": "something" })));
        let err = SignalingMessage::from_message(&envelope).unwrap_err();
        assert!(matches!(err, MessageEnvelopeError::JsonParse { .. }));
    }

    #[test]
    fn test_ice_candidate_parses_correctly() {
        let envelope = make_envelope(
            "signalling.ice_candidate",
            Some(json!({
                "candidate": "candidate:1 1 udp 2130706431 192.168.1.1 12345 typ host",
                "connectionId": "conn-1",
                "sdpMid": "0",
                "sdpMLineIndex": 0,
                "usernameFragment": "frag"
            })),
        );
        let msg = SignalingMessage::from_message(&envelope).unwrap();
        match msg {
            SignalingMessage::IceCandidate(ice) => {
                assert_eq!(
                    ice.candidate,
                    "candidate:1 1 udp 2130706431 192.168.1.1 12345 typ host"
                );
                assert_eq!(ice.connection_id, "conn-1");
                assert_eq!(ice.sdp_mid, "0");
                assert_eq!(ice.sdp_m_line_index, 0);
                assert_eq!(ice.username_fragment, Some("frag".to_string()));
            }
            other => panic!("Expected IceCandidate, got {:?}", other),
        }
    }

    #[test]
    fn test_ice_candidate_missing_payload_returns_error() {
        let envelope = make_envelope("signalling.ice_candidate", None);
        let err = SignalingMessage::from_message(&envelope).unwrap_err();
        assert!(matches!(err, MessageEnvelopeError::MissingFields { .. }));
    }

    #[test]
    fn test_offer_parses_correctly() {
        let envelope = make_envelope(
            "signalling.offer",
            Some(json!({
                "connectionId": "conn-1",
                "sdp": "v=0\r\no=- session\r\n",
                "sdpType": "offer",
                "iceRestart": true
            })),
        );
        let msg = SignalingMessage::from_message(&envelope).unwrap();
        match msg {
            SignalingMessage::Offer(o) => {
                assert_eq!(o.connection_id, "conn-1");
                assert_eq!(o.sdp, "v=0\r\no=- session\r\n");
                assert_eq!(o.sdp_type, "offer");
                assert_eq!(o.ice_restart, Some(true));
            }
            other => panic!("Expected Offer, got {:?}", other),
        }
    }

    #[test]
    fn test_offer_missing_payload_returns_error() {
        let envelope = make_envelope("signalling.offer", None);
        let err = SignalingMessage::from_message(&envelope).unwrap_err();
        assert!(matches!(err, MessageEnvelopeError::MissingFields { .. }));
    }

    #[test]
    fn test_offer_missing_sdp_returns_error() {
        let envelope = make_envelope(
            "signalling.offer",
            Some(json!({
                "connectionId": "conn-1",
                "sdpType": "offer"
            })),
        );
        let err = SignalingMessage::from_message(&envelope).unwrap_err();
        assert!(matches!(err, MessageEnvelopeError::JsonParse { .. }));
    }

    #[test]
    fn test_register_parses_correctly() {
        let envelope = make_envelope(
            "signalling.register",
            Some(json!({
                "agentId": "agent-1",
                "metadata": { "version": "1.0" }
            })),
        );
        let msg = SignalingMessage::from_message(&envelope).unwrap();
        match msg {
            SignalingMessage::Register(r) => {
                assert_eq!(r.agent_id, "agent-1");
                assert_eq!(r.capabilities, None);
                assert_eq!(r.metadata.unwrap()["version"], "1.0");
            }
            other => panic!("Expected Register, got {:?}", other),
        }
    }

    #[test]
    fn test_register_missing_payload_returns_error() {
        let envelope = make_envelope("signalling.register", None);
        let err = SignalingMessage::from_message(&envelope).unwrap_err();
        assert!(matches!(err, MessageEnvelopeError::MissingFields { .. }));
    }

    #[test]
    fn test_register_missing_agent_id_returns_error() {
        let envelope = make_envelope("signalling.register", Some(json!({})));
        let err = SignalingMessage::from_message(&envelope).unwrap_err();
        assert!(matches!(err, MessageEnvelopeError::JsonParse { .. }));
    }

    #[test]
    fn test_unknown_message_type_returns_error() {
        let envelope = make_envelope("signalling.unknown", None);
        let err = SignalingMessage::from_message(&envelope).unwrap_err();
        assert!(matches!(
            err,
            MessageEnvelopeError::UnknownMessageType { .. }
        ));
    }

    // --- PkiChallenge tests ---

    #[test]
    fn test_pki_challenge_serialises_correctly() {
        let msg = SignalingMessage::PkiChallenge(PkiChallengePayload {
            challenge: "abc123".to_string(),
        });
        let envelope = msg.to_message();
        assert_eq!(envelope.message_type, "signalling.pki_challenge");
        assert!(envelope.correlation_id.is_none());
        let payload = envelope.payload.unwrap();
        assert_eq!(payload["challenge"], "abc123");
    }

    #[test]
    fn test_pki_challenge_parses_correctly() {
        let envelope = make_envelope(
            "signalling.pki_challenge",
            Some(json!({ "challenge": "server-nonce-xyz" })),
        );
        let msg = SignalingMessage::from_message(&envelope).unwrap();
        match msg {
            SignalingMessage::PkiChallenge(c) => {
                assert_eq!(c.challenge, "server-nonce-xyz");
            }
            other => panic!("Expected PkiChallenge, got {:?}", other),
        }
    }

    #[test]
    fn test_pki_challenge_missing_payload_returns_error() {
        let envelope = make_envelope("signalling.pki_challenge", None);
        let err = SignalingMessage::from_message(&envelope).unwrap_err();
        assert!(matches!(err, MessageEnvelopeError::MissingFields { .. }));
    }

    #[test]
    fn test_pki_challenge_missing_challenge_returns_error() {
        let envelope = make_envelope("signalling.pki_challenge", Some(json!({})));
        let err = SignalingMessage::from_message(&envelope).unwrap_err();
        assert!(matches!(err, MessageEnvelopeError::JsonParse { .. }));
    }

    // --- PkiResponse tests ---

    #[test]
    fn test_pki_response_serialises_correctly() {
        let msg = SignalingMessage::PkiResponse(PkiResponsePayload {
            correlation_id: "challenge-id-1".to_string(),
            signature: "c2lnbmVk".to_string(),
        });
        let envelope = msg.to_message();
        assert_eq!(envelope.message_type, "signalling.pki_response");
        assert_eq!(envelope.correlation_id.unwrap(), "challenge-id-1");
        let payload = envelope.payload.unwrap();
        assert_eq!(payload["signature"], "c2lnbmVk");
        // correlation_id should NOT appear in the payload (serde skip)
        assert!(payload.get("correlationId").is_none());
    }

    #[test]
    fn test_pki_response_parses_correctly() {
        let mut envelope = make_envelope(
            "signalling.pki_response",
            Some(json!({ "signature": "c2lnbmVk" })),
        );
        envelope.correlation_id = Some("challenge-id-1".to_string());
        let msg = SignalingMessage::from_message(&envelope).unwrap();
        match msg {
            SignalingMessage::PkiResponse(r) => {
                assert_eq!(r.correlation_id, "challenge-id-1");
                assert_eq!(r.signature, "c2lnbmVk");
            }
            other => panic!("Expected PkiResponse, got {:?}", other),
        }
    }

    #[test]
    fn test_pki_response_missing_correlation_id_returns_error() {
        let envelope = make_envelope(
            "signalling.pki_response",
            Some(json!({ "signature": "c2lnbmVk" })),
        );
        let err = SignalingMessage::from_message(&envelope).unwrap_err();
        match err {
            MessageEnvelopeError::MissingFields { fields } => {
                assert!(fields.contains(&"correlationId".to_string()));
            }
            other => panic!("Expected MissingFields, got {:?}", other),
        }
    }

    #[test]
    fn test_pki_response_missing_payload_returns_error() {
        let mut envelope = make_envelope("signalling.pki_response", None);
        envelope.correlation_id = Some("challenge-id-1".to_string());
        let err = SignalingMessage::from_message(&envelope).unwrap_err();
        match err {
            MessageEnvelopeError::MissingFields { fields } => {
                assert!(fields.contains(&"payload".to_string()));
            }
            other => panic!("Expected MissingFields, got {:?}", other),
        }
    }

    #[test]
    fn test_pki_response_missing_both_returns_error_with_both_fields() {
        let envelope = make_envelope("signalling.pki_response", None);
        let err = SignalingMessage::from_message(&envelope).unwrap_err();
        match err {
            MessageEnvelopeError::MissingFields { fields } => {
                assert!(fields.contains(&"correlationId".to_string()));
                assert!(fields.contains(&"payload".to_string()));
            }
            other => panic!("Expected MissingFields, got {:?}", other),
        }
    }

    #[test]
    fn test_pki_response_missing_signature_returns_error() {
        let mut envelope =
            make_envelope("signalling.pki_response", Some(json!({ "other": "field" })));
        envelope.correlation_id = Some("challenge-id-1".to_string());
        let err = SignalingMessage::from_message(&envelope).unwrap_err();
        assert!(matches!(err, MessageEnvelopeError::MissingFields { .. }));
    }

    // --- PkiVerified tests ---

    #[test]
    fn test_pki_verified_serialises_correctly() {
        let msg = SignalingMessage::PkiVerified(PkiVerifiedPayload {
            correlation_id: "challenge-id-1".to_string(),
            agent_id: "agent-42".to_string(),
        });
        let envelope = msg.to_message();
        assert_eq!(envelope.message_type, "signalling.pki_verified");
        assert_eq!(envelope.correlation_id.unwrap(), "challenge-id-1");
        let payload = envelope.payload.unwrap();
        assert_eq!(payload["agentId"], "agent-42");
        // correlation_id should NOT appear in the payload (serde skip)
        assert!(payload.get("correlationId").is_none());
    }

    #[test]
    fn test_pki_verified_parses_correctly() {
        let mut envelope = make_envelope(
            "signalling.pki_verified",
            Some(json!({ "agentId": "agent-42" })),
        );
        envelope.correlation_id = Some("challenge-id-1".to_string());
        let msg = SignalingMessage::from_message(&envelope).unwrap();
        match msg {
            SignalingMessage::PkiVerified(v) => {
                assert_eq!(v.correlation_id, "challenge-id-1");
                assert_eq!(v.agent_id, "agent-42");
            }
            other => panic!("Expected PkiVerified, got {:?}", other),
        }
    }

    #[test]
    fn test_pki_verified_missing_correlation_id_returns_error() {
        let envelope = make_envelope(
            "signalling.pki_verified",
            Some(json!({ "agentId": "agent-42" })),
        );
        let err = SignalingMessage::from_message(&envelope).unwrap_err();
        match err {
            MessageEnvelopeError::MissingFields { fields } => {
                assert!(fields.contains(&"correlationId".to_string()));
            }
            other => panic!("Expected MissingFields, got {:?}", other),
        }
    }

    #[test]
    fn test_pki_verified_missing_payload_returns_error() {
        let mut envelope = make_envelope("signalling.pki_verified", None);
        envelope.correlation_id = Some("challenge-id-1".to_string());
        let err = SignalingMessage::from_message(&envelope).unwrap_err();
        match err {
            MessageEnvelopeError::MissingFields { fields } => {
                assert!(fields.contains(&"payload".to_string()));
            }
            other => panic!("Expected MissingFields, got {:?}", other),
        }
    }

    #[test]
    fn test_pki_verified_missing_both_returns_error_with_both_fields() {
        let envelope = make_envelope("signalling.pki_verified", None);
        let err = SignalingMessage::from_message(&envelope).unwrap_err();
        match err {
            MessageEnvelopeError::MissingFields { fields } => {
                assert!(fields.contains(&"correlationId".to_string()));
                assert!(fields.contains(&"payload".to_string()));
            }
            other => panic!("Expected MissingFields, got {:?}", other),
        }
    }

    #[test]
    fn test_pki_verified_missing_agent_id_returns_error() {
        let mut envelope =
            make_envelope("signalling.pki_verified", Some(json!({ "other": "field" })));
        envelope.correlation_id = Some("challenge-id-1".to_string());
        let err = SignalingMessage::from_message(&envelope).unwrap_err();
        assert!(matches!(err, MessageEnvelopeError::JsonParse { .. }));
    }

    // --- Round-trip tests ---

    #[test]
    fn test_pki_verified_round_trip() {
        let original = SignalingMessage::PkiVerified(PkiVerifiedPayload {
            correlation_id: "challenge-id-1".to_string(),
            agent_id: "agent-42".to_string(),
        });
        let json_str = original.to_message().to_string().unwrap();
        let envelope = MessageEnvelope::from_str(&json_str).unwrap();
        let parsed = SignalingMessage::from_message(&envelope).unwrap();
        match parsed {
            SignalingMessage::PkiVerified(v) => {
                assert_eq!(v.correlation_id, "challenge-id-1");
                assert_eq!(v.agent_id, "agent-42");
            }
            other => panic!("Expected PkiVerified, got {:?}", other),
        }
    }

    #[test]
    fn test_pki_challenge_round_trip() {
        let original = SignalingMessage::PkiChallenge(PkiChallengePayload {
            challenge: "server-nonce-xyz".to_string(),
        });
        let json_str = original.to_message().to_string().unwrap();
        let envelope = MessageEnvelope::from_str(&json_str).unwrap();
        let parsed = SignalingMessage::from_message(&envelope).unwrap();
        match parsed {
            SignalingMessage::PkiChallenge(c) => {
                assert_eq!(c.challenge, "server-nonce-xyz");
            }
            other => panic!("Expected PkiChallenge, got {:?}", other),
        }
    }

    #[test]
    fn test_pki_response_round_trip() {
        let original = SignalingMessage::PkiResponse(PkiResponsePayload {
            correlation_id: "challenge-id-1".to_string(),
            signature: "c2lnbmVk".to_string(),
        });
        let json_str = original.to_message().to_string().unwrap();
        let envelope = MessageEnvelope::from_str(&json_str).unwrap();
        let parsed = SignalingMessage::from_message(&envelope).unwrap();
        match parsed {
            SignalingMessage::PkiResponse(r) => {
                assert_eq!(r.correlation_id, "challenge-id-1");
                assert_eq!(r.signature, "c2lnbmVk");
            }
            other => panic!("Expected PkiResponse, got {:?}", other),
        }
    }

    #[test]
    fn test_register_round_trip() {
        let original = SignalingMessage::Register(RegisterPayload {
            agent_id: "agent-1".to_string(),
            capabilities: None,
            metadata: Some(json!({ "key": "value" })),
        });
        let json_str = original.to_message().to_string().unwrap();
        let envelope = MessageEnvelope::from_str(&json_str).unwrap();
        let parsed = SignalingMessage::from_message(&envelope).unwrap();
        match parsed {
            SignalingMessage::Register(r) => {
                assert_eq!(r.agent_id, "agent-1");
                assert_eq!(r.metadata.unwrap()["key"], "value");
            }
            other => panic!("Expected Register, got {:?}", other),
        }
    }

    #[test]
    fn test_offer_round_trip() {
        let original = SignalingMessage::Offer(OfferPayload {
            connection_id: "conn-1".to_string(),
            sdp: "v=0\r\no=- 12345\r\n".to_string(),
            sdp_type: "offer".to_string(),
            ice_restart: Some(true),
        });
        let json_str = original.to_message().to_string().unwrap();
        let envelope = MessageEnvelope::from_str(&json_str).unwrap();
        let parsed = SignalingMessage::from_message(&envelope).unwrap();
        match parsed {
            SignalingMessage::Offer(o) => {
                assert_eq!(o.connection_id, "conn-1");
                assert_eq!(o.sdp, "v=0\r\no=- 12345\r\n");
                assert_eq!(o.sdp_type, "offer");
                assert_eq!(o.ice_restart, Some(true));
            }
            other => panic!("Expected Offer, got {:?}", other),
        }
    }

    #[test]
    fn test_connected_round_trip() {
        let original = SignalingMessage::Connected(ConnectedPayload {
            connection_id: "conn-1".to_string(),
            ice_connection_state: IceConnectionState::Connected,
            data_channel_state: Some(DataChannelState::Open),
        });
        let json_str = original.to_message().to_string().unwrap();
        let envelope = MessageEnvelope::from_str(&json_str).unwrap();
        let parsed = SignalingMessage::from_message(&envelope).unwrap();
        match parsed {
            SignalingMessage::Connected(c) => {
                assert_eq!(c.connection_id, "conn-1");
                assert_eq!(c.ice_connection_state, IceConnectionState::Connected);
                assert_eq!(c.data_channel_state, Some(DataChannelState::Open));
            }
            other => panic!("Expected Connected, got {:?}", other),
        }
    }

    #[test]
    fn test_disconnected_round_trip() {
        let original = SignalingMessage::Disconnected(DisconnectedPayload {
            connection_id: "conn-1".to_string(),
            reason: DisconnectionReason::Timeout,
            ice_connection_state: None,
            details: Some(json!({ "elapsed": 30 })),
        });
        let json_str = original.to_message().to_string().unwrap();
        let envelope = MessageEnvelope::from_str(&json_str).unwrap();
        let parsed = SignalingMessage::from_message(&envelope).unwrap();
        match parsed {
            SignalingMessage::Disconnected(d) => {
                assert_eq!(d.connection_id, "conn-1");
                assert_eq!(d.reason, DisconnectionReason::Timeout);
                assert_eq!(d.ice_connection_state, None);
                assert_eq!(d.details.unwrap()["elapsed"], 30);
            }
            other => panic!("Expected Disconnected, got {:?}", other),
        }
    }
}
