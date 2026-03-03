use std::path::PathBuf;

use anyhow::Result;
use clap::{Parser, ValueEnum};
use modulr_webrtc_message::{Location, LocationValidationError};
use serde::{Deserialize, Serialize};

pub use modulr_agent_common::ImageFormat;

#[derive(Debug, thiserror::Error)]
pub enum ConfigWriteError {
    #[error("no config path provided and default path cannot be built")]
    NoConfigPath,
    #[error("failed to create config directory: {0}")]
    CreateDirFailed(#[source] std::io::Error),
    #[error("failed to serialize configuration: {0}")]
    SerializeFailed(#[source] serde_json::Error),
    #[error("failed to write config file: {0}")]
    WriteFailed(#[source] std::io::Error),
}

#[derive(Debug, thiserror::Error)]
pub enum LocationError {
    #[error(transparent)]
    NameInvalid(#[from] LocationValidationError),
    #[error("location '{0}' already exists")]
    AlreadyExists(String),
    #[error("location '{0}' not found")]
    NotFound(String),
    #[error("failed to persist location: {0}")]
    PersistFailed(#[from] ConfigWriteError),
}

#[derive(Serialize)]
#[serde(rename_all = "camelCase")]
pub struct LocationErrorDetails {
    pub operation: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub requested_name: Option<String>,
}

impl LocationError {
    pub fn to_error_code_and_details(
        &self,
        operation: &str,
    ) -> (modulr_webrtc_message::AgentErrorCode, serde_json::Value) {
        let (code, details) = match self {
            LocationError::NameInvalid(e) => {
                let requested_name = match e {
                    LocationValidationError::NameWhitespace(name) => Some(name.clone()),
                    LocationValidationError::NameEmpty => None,
                };
                (
                    modulr_webrtc_message::AgentErrorCode::LocationNameInvalid,
                    LocationErrorDetails {
                        operation: operation.to_string(),
                        requested_name,
                    },
                )
            }
            LocationError::AlreadyExists(name) => (
                modulr_webrtc_message::AgentErrorCode::LocationAlreadyExists,
                LocationErrorDetails {
                    operation: operation.to_string(),
                    requested_name: Some(name.clone()),
                },
            ),
            LocationError::NotFound(name) => (
                modulr_webrtc_message::AgentErrorCode::LocationNotFound,
                LocationErrorDetails {
                    operation: operation.to_string(),
                    requested_name: Some(name.clone()),
                },
            ),
            LocationError::PersistFailed(_) => (
                modulr_webrtc_message::AgentErrorCode::InternalError,
                LocationErrorDetails {
                    operation: operation.to_string(),
                    requested_name: None,
                },
            ),
        };
        (
            code,
            serde_json::to_value(details)
                .expect("LocationErrorDetails serialization is infallible"),
        )
    }
}

#[derive(Debug, thiserror::Error)]
pub enum NavigationError {
    #[error("navigation already active to '{0}'")]
    AlreadyActive(String),
    #[error("no navigation is active")]
    NotActive,
    #[error("location '{0}' not found")]
    LocationNotFound(String),
}

#[derive(Serialize)]
#[serde(rename_all = "camelCase")]
pub struct NavigationErrorDetails {
    pub operation: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub requested_name: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub current_target: Option<String>,
}

impl NavigationError {
    pub fn to_error_code_and_details(
        &self,
        operation: &str,
    ) -> (modulr_webrtc_message::AgentErrorCode, serde_json::Value) {
        let (code, details) = match self {
            NavigationError::AlreadyActive(name) => (
                modulr_webrtc_message::AgentErrorCode::NavigationAlreadyActive,
                NavigationErrorDetails {
                    operation: operation.to_string(),
                    requested_name: None,
                    current_target: Some(name.clone()),
                },
            ),
            NavigationError::NotActive => (
                modulr_webrtc_message::AgentErrorCode::NavigationNotActive,
                NavigationErrorDetails {
                    operation: operation.to_string(),
                    requested_name: None,
                    current_target: None,
                },
            ),
            NavigationError::LocationNotFound(name) => (
                modulr_webrtc_message::AgentErrorCode::LocationNotFound,
                NavigationErrorDetails {
                    operation: operation.to_string(),
                    requested_name: Some(name.clone()),
                    current_target: None,
                },
            ),
        };
        (
            code,
            serde_json::to_value(details)
                .expect("NavigationErrorDetails serialization is infallible"),
        )
    }
}

pub fn start_navigation(
    navigation_target: &Option<String>,
    locations: &[Location],
    name: &str,
) -> Result<(Option<String>, Location), NavigationError> {
    if let Some(current) = navigation_target.as_deref() {
        return Err(NavigationError::AlreadyActive(current.to_string()));
    }
    let location = locations
        .iter()
        .find(|l| l.name == name)
        .cloned()
        .ok_or_else(|| NavigationError::LocationNotFound(name.to_string()))?;
    Ok((Some(name.to_string()), location))
}

pub fn cancel_navigation(
    navigation_target: &Option<String>,
) -> Result<(Option<String>, String), NavigationError> {
    navigation_target
        .as_deref()
        .map(|name| (None, name.to_string()))
        .ok_or(NavigationError::NotActive)
}

#[derive(Parser, Default, Debug, Clone, Serialize, Deserialize, PartialEq, ValueEnum)]
pub enum VideoSource {
    #[default]
    Ros,
    Zenoh,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct CoreConfig {
    pub robot_id: String,
    pub signaling_url: String,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize, PartialEq)]
pub struct RobotConfig {
    #[serde(default)]
    pub video_source: VideoSource,
    #[serde(default)]
    pub image_format: ImageFormat,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct AgentConfig {
    pub core: CoreConfig,
    #[serde(default)]
    pub robot: RobotConfig,
    #[serde(default)]
    pub locations: Vec<Location>,
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

pub fn write_config(
    config: &AgentConfig,
    override_path: Option<PathBuf>,
) -> Result<(), ConfigWriteError> {
    let config_path = override_path
        .or(get_default_path())
        .ok_or(ConfigWriteError::NoConfigPath)?;
    if let Some(parent) = config_path.parent()
        && !parent.exists()
    {
        std::fs::create_dir_all(parent).map_err(ConfigWriteError::CreateDirFailed)?;
    }
    std::fs::write(
        &config_path,
        serde_json::to_string_pretty(config).map_err(ConfigWriteError::SerializeFailed)?,
    )
    .map_err(ConfigWriteError::WriteFailed)
}

fn persist_locations(
    locations: &[Location],
    config: &AgentConfig,
    config_path: Option<PathBuf>,
) -> std::result::Result<(), LocationError> {
    let mut updated = config.clone();
    updated.locations = locations.to_vec();
    write_config(&updated, config_path).map_err(LocationError::PersistFailed)
}

pub fn create_location(
    locations: &[Location],
    location: Location,
    config: &AgentConfig,
    config_path: Option<PathBuf>,
) -> std::result::Result<Vec<Location>, LocationError> {
    location.validate()?;
    if locations.iter().any(|l| l.name == location.name) {
        return Err(LocationError::AlreadyExists(location.name));
    }
    let mut new_locations = locations.to_vec();
    new_locations.push(location);
    persist_locations(&new_locations, config, config_path)?;
    Ok(new_locations)
}

pub fn update_location(
    locations: &[Location],
    location: Location,
    config: &AgentConfig,
    config_path: Option<PathBuf>,
) -> std::result::Result<Vec<Location>, LocationError> {
    location.validate()?;
    let pos = locations
        .iter()
        .position(|l| l.name == location.name)
        .ok_or_else(|| LocationError::NotFound(location.name.clone()))?;
    let mut new_locations = locations.to_vec();
    new_locations[pos] = location;
    persist_locations(&new_locations, config, config_path)?;
    Ok(new_locations)
}

pub fn delete_location(
    locations: &[Location],
    name: &str,
    config: &AgentConfig,
    config_path: Option<PathBuf>,
) -> std::result::Result<Vec<Location>, LocationError> {
    let pos = locations
        .iter()
        .position(|l| l.name == name)
        .ok_or_else(|| LocationError::NotFound(name.to_string()))?;
    let mut new_locations = locations.to_vec();
    new_locations.remove(pos);
    persist_locations(&new_locations, config, config_path)?;
    Ok(new_locations)
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
            locations: vec![],
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
            locations: vec![],
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
            locations: vec![],
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

    fn make_config() -> AgentConfig {
        AgentConfig {
            core: CoreConfig {
                robot_id: "test".to_string(),
                signaling_url: "ws://localhost".to_string(),
            },
            robot: RobotConfig::default(),
            locations: vec![],
        }
    }

    fn make_location(name: &str) -> Location {
        use modulr_webrtc_message::Position;
        Location {
            correlation_id: None,
            name: name.to_string(),
            position: Position {
                x: 0.0,
                y: 0.0,
                z: None,
            },
            orientation: None,
            metadata: None,
        }
    }

    #[test]
    fn test_create_location_rejects_empty_name() {
        let err = create_location(&[], make_location(""), &make_config(), None).unwrap_err();
        assert!(matches!(
            err,
            LocationError::NameInvalid(LocationValidationError::NameEmpty)
        ));
    }

    #[test]
    fn test_create_location_rejects_whitespace_name() {
        let err =
            create_location(&[], make_location(" Dock A "), &make_config(), None).unwrap_err();
        assert!(matches!(
            err,
            LocationError::NameInvalid(LocationValidationError::NameWhitespace(ref s)) if s == " Dock A "
        ));
    }

    #[test]
    fn test_update_location_rejects_empty_name() {
        let err = update_location(&[], make_location(""), &make_config(), None).unwrap_err();
        assert!(matches!(
            err,
            LocationError::NameInvalid(LocationValidationError::NameEmpty)
        ));
    }

    #[test]
    fn test_update_location_rejects_whitespace_name() {
        let err =
            update_location(&[], make_location(" Dock A "), &make_config(), None).unwrap_err();
        assert!(matches!(
            err,
            LocationError::NameInvalid(LocationValidationError::NameWhitespace(ref s)) if s == " Dock A "
        ));
    }
}
