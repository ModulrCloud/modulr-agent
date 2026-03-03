use std::path::{Path, PathBuf};

use base64::Engine as _;
use base64::engine::general_purpose::STANDARD_NO_PAD;
use ed25519_dalek::SigningKey;
use rand::rngs::OsRng;
use thiserror::Error;

#[cfg(unix)]
use std::os::unix::fs::PermissionsExt;

use crate::commands::config::get_default_key_path;

pub const KEY_NAME: &str = "agent_key";

#[derive(Error, Debug)]
pub enum KeyError {
    #[error("config override path has no parent directory")]
    NoParentDirectory,

    #[error("unable to determine default key path")]
    NoDefaultKeyPath,

    #[error("failed to read key file at {path}")]
    KeyReadFailed {
        path: PathBuf,
        #[source]
        source: std::io::Error,
    },

    #[error("key file at {path} contains invalid base64")]
    InvalidBase64 {
        path: PathBuf,
        #[source]
        source: base64::DecodeError,
    },

    #[error("key file at {path} has wrong length (expected 32 bytes, got {actual})")]
    InvalidKeyLength { path: PathBuf, actual: usize },

    #[error("failed to create key directory {path}")]
    CreateDirFailed {
        path: PathBuf,
        #[source]
        source: std::io::Error,
    },

    #[error("failed to write key file {path}")]
    KeyWriteFailed {
        path: PathBuf,
        #[source]
        source: std::io::Error,
    },

    #[error("failed to set permissions on {path}")]
    SetPermissionsFailed {
        path: PathBuf,
        #[source]
        source: std::io::Error,
    },
}

pub fn resolve_key_path(config_override: &Option<PathBuf>) -> Result<PathBuf, KeyError> {
    match config_override {
        Some(config_path) => {
            let parent = config_path.parent().ok_or(KeyError::NoParentDirectory)?;
            Ok(parent.join(KEY_NAME))
        }
        None => get_default_key_path().ok_or(KeyError::NoDefaultKeyPath),
    }
}

pub fn load_signing_key(config_override: &Option<PathBuf>) -> Result<SigningKey, KeyError> {
    let key_path = resolve_key_path(config_override)?;

    let key_b64 = std::fs::read_to_string(&key_path).map_err(|source| KeyError::KeyReadFailed {
        path: key_path.to_path_buf(),
        source,
    })?;

    let key_bytes =
        STANDARD_NO_PAD
            .decode(key_b64.trim())
            .map_err(|source| KeyError::InvalidBase64 {
                path: key_path.to_path_buf(),
                source,
            })?;

    let secret_bytes: [u8; 32] =
        key_bytes
            .try_into()
            .map_err(|bytes: Vec<u8>| KeyError::InvalidKeyLength {
                path: key_path.to_path_buf(),
                actual: bytes.len(),
            })?;

    Ok(SigningKey::from_bytes(&secret_bytes))
}

/// Generates a new keypair or loads an existing one.
/// Returns the base64-encoded public key.
pub fn generate_or_load_keypair(key_path: &Path) -> Result<String, KeyError> {
    if key_path.exists() {
        load_existing_keypair(key_path)
    } else {
        generate_new_keypair(key_path)
    }
}

fn load_existing_keypair(key_path: &Path) -> Result<String, KeyError> {
    log::info!(
        "Key file already exists at {}, loading existing key",
        key_path.display()
    );

    let key_b64 = std::fs::read_to_string(key_path).map_err(|source| KeyError::KeyReadFailed {
        path: key_path.to_path_buf(),
        source,
    })?;

    let key_bytes =
        STANDARD_NO_PAD
            .decode(key_b64.trim())
            .map_err(|source| KeyError::InvalidBase64 {
                path: key_path.to_path_buf(),
                source,
            })?;

    let secret_bytes: [u8; 32] =
        key_bytes
            .try_into()
            .map_err(|bytes: Vec<u8>| KeyError::InvalidKeyLength {
                path: key_path.to_path_buf(),
                actual: bytes.len(),
            })?;

    let signing_key = SigningKey::from_bytes(&secret_bytes);
    let public_key = signing_key.verifying_key();
    let public_key_b64 = STANDARD_NO_PAD.encode(public_key.as_bytes());

    // Ensure public key file exists and matches
    let pub_path = key_path.with_extension("pub");
    let needs_pub_write = if pub_path.exists() {
        let existing = std::fs::read_to_string(&pub_path).unwrap_or_default();
        existing.trim() != public_key_b64
    } else {
        true
    };
    if needs_pub_write {
        std::fs::write(&pub_path, &public_key_b64).map_err(|source| KeyError::KeyWriteFailed {
            path: pub_path.clone(),
            source,
        })?;
    }

    log::info!("Loaded existing ED25519 keypair.");
    Ok(public_key_b64)
}

fn generate_new_keypair(key_path: &Path) -> Result<String, KeyError> {
    log::info!("No existing key file found. Generating new ED25519 keypair.");

    let signing_key = SigningKey::generate(&mut OsRng);
    let public_key = signing_key.verifying_key();

    let private_key_b64 = STANDARD_NO_PAD.encode(signing_key.to_bytes());
    let public_key_b64 = STANDARD_NO_PAD.encode(public_key.as_bytes());

    if let Some(parent) = key_path.parent() {
        std::fs::create_dir_all(parent).map_err(|source| KeyError::CreateDirFailed {
            path: parent.to_path_buf(),
            source,
        })?;
    }

    std::fs::write(key_path, &private_key_b64).map_err(|source| KeyError::KeyWriteFailed {
        path: key_path.to_path_buf(),
        source,
    })?;

    #[cfg(unix)]
    {
        let permissions = std::fs::Permissions::from_mode(0o600);
        std::fs::set_permissions(key_path, permissions).map_err(|source| {
            KeyError::SetPermissionsFailed {
                path: key_path.to_path_buf(),
                source,
            }
        })?;
    }

    let pub_path = key_path.with_extension("pub");
    std::fs::write(&pub_path, &public_key_b64).map_err(|source| KeyError::KeyWriteFailed {
        path: pub_path.clone(),
        source,
    })?;

    log::info!("Generated new ED25519 keypair at {}", key_path.display());
    Ok(public_key_b64)
}

#[cfg(test)]
mod tests {
    use super::*;
    use ed25519_dalek::Signer;
    use tempfile::TempDir;

    #[test]
    fn test_resolve_key_path_with_override() {
        let tmp_dir = TempDir::new().unwrap();
        let override_path = tmp_dir.path().join("config.json");
        let key_path = resolve_key_path(&Some(override_path)).unwrap();
        assert_eq!(key_path, tmp_dir.path().join(KEY_NAME));
    }

    #[test]
    fn test_resolve_key_path_default() {
        let key_path = resolve_key_path(&None).unwrap();
        assert!(key_path.ends_with(format!("modulr_agent/{}", KEY_NAME)));
    }

    #[test]
    fn test_load_signing_key_valid() {
        let tmp_dir = TempDir::new().unwrap();
        let key_path = tmp_dir.path().join(KEY_NAME);

        // Generate and write a valid key
        let original_key = SigningKey::generate(&mut rand::rngs::OsRng);
        let key_b64 = STANDARD_NO_PAD.encode(original_key.to_bytes());
        std::fs::write(&key_path, &key_b64).unwrap();

        // Create a config path in the same directory
        let config_path = tmp_dir.path().join("config.json");
        let loaded = load_signing_key(&Some(config_path)).unwrap();

        // Verify it produces the same signatures
        let test_data = b"test message";
        assert_eq!(
            original_key.sign(test_data).to_bytes(),
            loaded.sign(test_data).to_bytes()
        );
    }

    #[test]
    fn test_load_signing_key_missing_file_returns_error() {
        let tmp_dir = TempDir::new().unwrap();
        let config_path = tmp_dir.path().join("config.json");
        let result = load_signing_key(&Some(config_path));
        assert!(matches!(
            result.unwrap_err(),
            KeyError::KeyReadFailed { .. }
        ));
    }

    #[test]
    fn test_load_signing_key_invalid_base64_returns_error() {
        let tmp_dir = TempDir::new().unwrap();
        let key_path = tmp_dir.path().join(KEY_NAME);
        std::fs::write(&key_path, "not-valid-base64!!!").unwrap();

        let config_path = tmp_dir.path().join("config.json");
        let result = load_signing_key(&Some(config_path));
        assert!(matches!(
            result.unwrap_err(),
            KeyError::InvalidBase64 { .. }
        ));
    }

    #[test]
    fn test_load_signing_key_wrong_length_returns_error() {
        let tmp_dir = TempDir::new().unwrap();
        let key_path = tmp_dir.path().join(KEY_NAME);
        let short_key = STANDARD_NO_PAD.encode([0u8; 16]);
        std::fs::write(&key_path, &short_key).unwrap();

        let config_path = tmp_dir.path().join("config.json");
        let result = load_signing_key(&Some(config_path));
        assert!(matches!(
            result.unwrap_err(),
            KeyError::InvalidKeyLength { actual: 16, .. }
        ));
    }

    #[test]
    fn test_generate_new_keypair() {
        let tmp_dir = TempDir::new().unwrap();
        let key_path = tmp_dir.path().join(KEY_NAME);

        let public_key_b64 = generate_or_load_keypair(&key_path).unwrap();

        // Both key files should exist
        assert!(key_path.exists());
        assert!(key_path.with_extension("pub").exists());

        // Returned public key should match file
        let pub_file = std::fs::read_to_string(key_path.with_extension("pub")).unwrap();
        assert_eq!(public_key_b64, pub_file);

        // Private key should be valid base64 of 32 bytes
        let private_b64 = std::fs::read_to_string(&key_path).unwrap();
        let private_bytes = STANDARD_NO_PAD.decode(private_b64.trim()).unwrap();
        assert_eq!(private_bytes.len(), 32);

        // Public key should be valid base64 of 32 bytes
        let public_bytes = STANDARD_NO_PAD.decode(public_key_b64.trim()).unwrap();
        assert_eq!(public_bytes.len(), 32);

        #[cfg(unix)]
        {
            let metadata = std::fs::metadata(&key_path).unwrap();
            assert_eq!(metadata.permissions().mode() & 0o777, 0o600);
        }
    }

    #[test]
    fn test_load_existing_keypair() {
        let tmp_dir = TempDir::new().unwrap();
        let key_path = tmp_dir.path().join(KEY_NAME);

        let public_key_1 = generate_or_load_keypair(&key_path).unwrap();

        // Load again -- should produce the same public key
        let public_key_2 = generate_or_load_keypair(&key_path).unwrap();

        assert_eq!(public_key_1, public_key_2);
    }
}
