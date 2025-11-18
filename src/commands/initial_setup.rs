use anyhow::Result;
use clap::Parser;
use std::path::PathBuf;

use crate::commands::config::{AgentConfig, write_config};

#[derive(Parser, Debug)]
pub struct InitialSetupArgs {
    /// Robot ID to use for connection with Modulr services
    #[arg(short, long)]
    robot_id: String,
    /// Signaling URL for establishing WebRTC link
    #[arg(short, long)]
    signaling_url: String,
    /// Override default config path
    #[arg(short, long, value_name = "FILE")]
    config_override: Option<PathBuf>,
}

pub async fn initial_setup(args: InitialSetupArgs) -> Result<()> {
    let config = AgentConfig {
        robot_id: args.robot_id,
        signaling_url: args.signaling_url,
    };
    write_config(&config, args.config_override)
}
