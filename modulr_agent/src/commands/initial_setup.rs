use std::path::PathBuf;

use clap::Parser;
use serde::Serialize;
use thiserror::Error;

use crate::commands::config::{
    AgentConfig, CoreConfig, ImageFormat, RobotConfig, VideoSource, write_config,
};
use crate::commands::keys;

#[derive(Error, Debug)]
pub enum SetupError {
    #[error("enrollment request failed")]
    EnrollmentRequestFailed {
        #[source]
        source: reqwest::Error,
    },

    #[error("enrollment rejected by server (HTTP {status}): {body}")]
    EnrollmentRejected {
        status: reqwest::StatusCode,
        body: String,
    },
}

#[derive(Serialize)]
#[serde(rename_all = "camelCase")]
struct EnrollmentRequest {
    robot_id: String,
    enrollment_token: String,
    public_key: String,
}

#[derive(Parser, Debug)]
pub struct InitialSetupArgs {
    /// Robot ID to use for connection with Modulr services
    #[arg(short, long)]
    robot_id: String,
    /// Signaling URL for establishing WebRTC link
    #[arg(short, long)]
    signaling_url: String,
    /// Enrollment URL to register public key
    #[arg(long)]
    enrollment_url: String,
    /// Enrollment token to register public key
    #[arg(long)]
    enrollment_token: String,
    /// Determine source of video frames (defaults to Ros)
    #[arg(long)]
    video_source: Option<VideoSource>,
    /// Format of incoming image frames (raw BGR or JPEG)
    #[arg(long, default_value = "raw")]
    image_format: ImageFormat,
    /// Override default config path
    #[arg(short, long, value_name = "FILE")]
    config_override: Option<PathBuf>,
}

pub async fn initial_setup(args: InitialSetupArgs) -> anyhow::Result<()> {
    let key_path = keys::resolve_key_path(&args.config_override)?;
    let public_key_b64 = keys::generate_or_load_keypair(&key_path)?;

    enroll_public_key(
        &args.enrollment_url,
        &args.robot_id,
        &args.enrollment_token,
        &public_key_b64,
    )
    .await?;

    let config = AgentConfig {
        core: CoreConfig {
            robot_id: args.robot_id,
            signaling_url: args.signaling_url,
        },
        robot: RobotConfig {
            video_source: args.video_source.unwrap_or_default(),
            image_format: args.image_format,
        },
    };
    write_config(&config, args.config_override)
}

async fn enroll_public_key(
    enrollment_url: &str,
    robot_id: &str,
    enrollment_token: &str,
    public_key: &str,
) -> Result<(), SetupError> {
    log::info!("Enrolling public key with {}", enrollment_url);

    let body = EnrollmentRequest {
        robot_id: robot_id.to_string(),
        enrollment_token: enrollment_token.to_string(),
        public_key: public_key.to_string(),
    };

    let response = reqwest::Client::new()
        .post(enrollment_url)
        .json(&body)
        .send()
        .await
        .map_err(|source| SetupError::EnrollmentRequestFailed { source })?;

    let status = response.status();
    if !status.is_success() {
        let body = response.text().await.unwrap_or_default();
        return Err(SetupError::EnrollmentRejected { status, body });
    }

    log::info!("Enrollment successful");
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_enrollment_request_serializes_correctly() {
        let body = EnrollmentRequest {
            robot_id: "robot-123".to_string(),
            enrollment_token: "token-abc".to_string(),
            public_key: "key-xyz".to_string(),
        };
        let json = serde_json::to_value(&body).unwrap();
        assert_eq!(json["robotId"], "robot-123");
        assert_eq!(json["enrollmentToken"], "token-abc");
        assert_eq!(json["publicKey"], "key-xyz");
    }
}
