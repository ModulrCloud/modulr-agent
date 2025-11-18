mod commands;
mod ros_bridge;
mod video_pipeline;
mod webrtc_link;
mod webrtc_message;

use anyhow::Result;
use clap::{Parser, Subcommand};

use crate::commands::{InitialSetupArgs, StartArgs, config, initial_setup, start};

#[derive(Parser)]
#[command(version, about, long_about = None)]
struct Cli {
    /// Set logging verbosity level. 1 = WARN, 2 = INFO, 3 = DEBUG, 4 = TRACE
    /// e.g. `agent run -vvvv config-path` sets the verbosity level to TRACE.
    #[arg(short, action = clap::ArgAction::Count)]
    verbose: u8,

    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
pub enum Commands {
    /// First-time setup. Discovers ROS installation and topics, and initialises
    /// token exchange mechanism with Modulr services.
    InitialSetup(InitialSetupArgs),
    /// Starts the main agent running
    Start(StartArgs),
    /// Prints out the default config path for this application.
    ConfigPath,
}

#[tokio::main]
async fn main() -> Result<()> {
    env_logger::Builder::from_env(
        env_logger::Env::default().default_filter_or("modulr_agent=trace"),
    )
    .init();
    let cli = Cli::parse();

    match cli.verbose {
        1 => log::set_max_level(log::LevelFilter::Warn),
        2 => log::set_max_level(log::LevelFilter::Info),
        3 => log::set_max_level(log::LevelFilter::Debug),
        4.. => log::set_max_level(log::LevelFilter::Trace),
        // Assume environment variable used to set max level
        _ => log::set_max_level(log::LevelFilter::Trace),
    };

    let result = match cli.command {
        Commands::Start(args) => {
            log::debug!("Requested a start with arguments: {:?}", args);
            start(args).await
        }
        Commands::InitialSetup(args) => {
            log::debug!("Requested initial setup with arguments: {:?}", args);
            let result = initial_setup(args).await;
            log::info!("Initial setup completed successfully.");
            result
        }
        Commands::ConfigPath => {
            log::info!("Requested config path");
            println!(
                "{}",
                config::get_default_path()
                    .ok_or_else(|| anyhow::anyhow!("Could not find default path!"))?
                    .display()
            );
            Ok(())
        }
    };

    if let Err(ref error) = result {
        log::error!("{}", error);
    }

    result
}
