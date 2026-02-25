use thiserror::Error;

#[derive(Error, Debug, Clone, PartialEq)]
pub enum MessageEnvelopeError {
    #[error("JSON parse error: {reason}")]
    JsonParse { reason: String },

    #[error("JSON encode error: {reason}")]
    JsonEncode { reason: String },

    #[error("missing required fields: {fields:?}")]
    MissingFields { fields: Vec<String> },

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

    #[error("location validation failed: {reason}")]
    LocationValidation { reason: String },
}
