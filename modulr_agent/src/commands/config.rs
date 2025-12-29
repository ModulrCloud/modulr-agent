use std::path::PathBuf;

use anyhow::Result;
use clap::{Parser, ValueEnum};
use serde::{Deserialize, Serialize};

#[derive(Parser, Debug, Clone, Serialize, Deserialize, PartialEq, ValueEnum)]
pub enum VideoSource {
    Ros,
    Zenoh,
}

#[derive(Debug, Serialize, Deserialize, PartialEq)]
pub struct AgentConfig {
    pub robot_id: String,
    pub signaling_url: String,
    pub video_source: VideoSource,
}

pub fn get_default_path() -> Option<PathBuf> {
    dirs::config_dir().map(|mut path| {
        path.push("modulr_agent");
        path.push("config.json");
        path
    })
}

pub fn read_config(override_path: Option<PathBuf>) -> Result<AgentConfig> {
    let config_path = override_path.or(get_default_path()).ok_or(anyhow::anyhow!(
        "No configuration file provided and default file cannot be found!"
    ))?;
    let config = std::fs::read_to_string(&config_path).map_err(|_| {
        anyhow::anyhow!(
            "Unable to read config from file path: {}",
            &config_path.display()
        )
    })?;
    serde_json::from_str::<AgentConfig>(&config)
        .map_err(|_| anyhow::anyhow!("Unable to deserialize configuration from file!"))
}

pub fn write_config(config: &AgentConfig, override_path: Option<PathBuf>) -> Result<()> {
    let config_path = override_path.or(get_default_path()).ok_or(anyhow::anyhow!(
        "No configuration file provided and default file path cannot be built!"
    ))?;
    std::fs::write(
        &config_path,
        serde_json::to_string_pretty(config)
            .map_err(|_| anyhow::anyhow!("Unable to serialize configuration for writing!"))?,
    )
    .map_err(|_| {
        anyhow::anyhow!(
            "Unable to write config to file path: {}",
            &config_path.display()
        )
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use tempfile::NamedTempFile;

    #[test]
    fn test_path_produced() {
        let path = get_default_path().unwrap();
        println!("Default path: {}", path.display());
        assert!(path.ends_with("modulr_agent/config.json"));
    }

    #[test]
    fn test_read_config_roundtrip() {
        let config = AgentConfig {
            robot_id: "my_robot_id".to_string(),
            signaling_url: "my_signaling_url".to_string(),
            video_source: VideoSource::Ros,
        };
        let serialized = serde_json::to_string_pretty(&config).unwrap();
        println!("Serialized config: {}", serialized);
        let temp_file = NamedTempFile::new().unwrap();
        std::fs::write(temp_file.path(), &serialized).unwrap();

        let deserialized = read_config(Some(temp_file.into_temp_path().to_path_buf())).unwrap();

        assert_eq!(deserialized, config);
    }

    #[test]
    fn test_write_config_roundtrip() {
        let config = AgentConfig {
            robot_id: "my_robot_id".to_string(),
            signaling_url: "my_signaling_url".to_string(),
            video_source: VideoSource::Ros,
        };
        let temp_file = NamedTempFile::new().unwrap();
        write_config(&config, Some(temp_file.path().to_path_buf())).unwrap();
        let deserialized = read_config(Some(temp_file.path().to_path_buf())).unwrap();
        assert_eq!(deserialized, config);
    }
}
