use anyhow::Result;
use clap::Parser;
use std::path::PathBuf;

use crate::commands::config::{AgentConfig, ImageFormat, VideoSource, write_config};

#[derive(Parser, Debug)]
pub struct InitialSetupArgs {
    /// Robot ID to use for connection with Modulr services
    #[arg(short, long)]
    robot_id: String,
    /// Signaling URL for establishing WebRTC link
    #[arg(short, long)]
    signaling_url: String,
    /// Determine source of video frames
    #[arg(long)]
    video_source: VideoSource,
    /// Format of incoming image frames (raw BGR or JPEG)
    #[arg(long, default_value = "raw")]
    image_format: ImageFormat,
    /// Override default config path
    #[arg(short, long, value_name = "FILE")]
    config_override: Option<PathBuf>,
}

pub async fn initial_setup(args: InitialSetupArgs) -> Result<()> {
    let config = AgentConfig {
        robot_id: args.robot_id,
        signaling_url: args.signaling_url,
        video_source: args.video_source,
        image_format: args.image_format,
    };
    write_config(&config, args.config_override)
}
