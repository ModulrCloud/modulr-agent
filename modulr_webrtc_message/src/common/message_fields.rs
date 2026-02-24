use chrono::Utc;
use uuid::Uuid;

use crate::{MessageEnvelope, PROTOCOL_VERSION};

pub fn generate_id() -> String {
    Uuid::new_v4().to_string()
}

pub fn generate_timestamp() -> String {
    Utc::now().to_rfc3339()
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
