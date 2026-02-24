use std::path::PathBuf;

use anyhow::Result;
use clap::{Parser, ValueEnum};
use serde::{Deserialize, Serialize};

pub use modulr_agent_common::ImageFormat;

#[derive(Parser, Default, Debug, Clone, Serialize, Deserialize, PartialEq, ValueEnum)]
pub enum VideoSource {
    #[default]
    Ros,
    Zenoh,
}

#[derive(Debug, Serialize, Deserialize, PartialEq)]
pub struct CoreConfig {
    pub robot_id: String,
    pub signaling_url: String,
}

#[derive(Debug, Default, Serialize, Deserialize, PartialEq)]
pub struct RobotConfig {
    #[serde(default)]
    pub video_source: VideoSource,
    #[serde(default)]
    pub image_format: ImageFormat,
}

#[derive(Debug, Serialize, Deserialize, PartialEq)]
pub struct AgentConfig {
    pub core: CoreConfig,
    #[serde(default)]
    pub robot: RobotConfig,
}

pub fn get_default_path() -> Option<PathBuf> {
    dirs::config_dir().map(|mut path| {
        path.push("modulr_agent");
        path.push("config.json");
        path
    })
}

pub fn get_default_key_path() -> Option<PathBuf> {
    dirs::config_dir().map(|mut path| {
        path.push("modulr_agent");
        path.push("agent_key");
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
    if let Some(parent) = config_path.parent()
        && !parent.exists()
    {
        std::fs::create_dir_all(parent)?;
    }
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
            core: CoreConfig {
                robot_id: "my_robot_id".to_string(),
                signaling_url: "my_signaling_url".to_string(),
            },
            robot: RobotConfig {
                video_source: VideoSource::Ros,
                image_format: ImageFormat::Raw,
            },
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
            core: CoreConfig {
                robot_id: "my_robot_id".to_string(),
                signaling_url: "my_signaling_url".to_string(),
            },
            robot: RobotConfig {
                video_source: VideoSource::Ros,
                image_format: ImageFormat::Jpeg,
            },
        };
        let temp_file = NamedTempFile::new().unwrap();
        write_config(&config, Some(temp_file.path().to_path_buf())).unwrap();
        let deserialized = read_config(Some(temp_file.path().to_path_buf())).unwrap();
        assert_eq!(deserialized, config);
    }

    #[test]
    fn test_write_config_creates_directory_if_not_exists() {
        let config = AgentConfig {
            core: CoreConfig {
                robot_id: "my_robot_id".to_string(),
                signaling_url: "my_signaling_url".to_string(),
            },
            robot: RobotConfig {
                video_source: VideoSource::Ros,
                image_format: ImageFormat::Jpeg,
            },
        };
        let temp_dir = tempfile::tempdir().unwrap();
        let config_path = temp_dir.path().join("fake_dir").join("config.json");

        // Parent directory should not exist yet
        assert!(!config_path.parent().unwrap().exists());

        write_config(&config, Some(config_path.clone())).unwrap();

        // Directory should now exist
        assert!(config_path.parent().unwrap().exists());
    }

    #[test]
    fn test_read_config_with_missing_robot_uses_defaults() {
        // Config file with only core config, missing robot section
        let partial_json = r#"{
            "core": {
                "robot_id": "test_robot",
                "signaling_url": "ws://localhost:8080"
            }
        }"#;

        let temp_file = NamedTempFile::new().unwrap();
        std::fs::write(temp_file.path(), partial_json).unwrap();

        let result = read_config(Some(temp_file.path().to_path_buf()));
        assert!(
            result.is_ok(),
            "read_config should use defaults for missing robot config, but failed: {:?}",
            result.err()
        );

        let config = result.unwrap();
        assert_eq!(config.core.robot_id, "test_robot");
        assert_eq!(config.core.signaling_url, "ws://localhost:8080");
        assert_eq!(config.robot, RobotConfig::default());
    }

    #[test]
    fn test_read_config_missing_core_fails() {
        // Config file missing required core section should fail
        let partial_json = r#"{
            "robot": {
                "video_source": "Ros",
                "image_format": "Raw"
            }
        }"#;

        let temp_file = NamedTempFile::new().unwrap();
        std::fs::write(temp_file.path(), partial_json).unwrap();

        let result = read_config(Some(temp_file.path().to_path_buf()));
        assert!(
            result.is_err(),
            "read_config should fail when core config is missing"
        );
    }
}
