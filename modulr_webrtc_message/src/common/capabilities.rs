pub const PROTOCOL_VERSION: &str = "0.3";
pub const SUPPORTED_VERSIONS: &[&str] = &["0.0", "0.3"];

#[derive(Debug, Clone, PartialEq)]
pub enum CapabilitiesErrorCode {
    UnsupportedVersion,
    CapabilityMismatch,
}

#[derive(Debug, Clone, PartialEq)]
pub struct CapabilitiesError {
    pub code: CapabilitiesErrorCode,
    pub message: String,
    pub details: serde_json::Value,
}

/// Parses a "major.minor" version string into (major, minor).
fn parse_version(version: &str) -> Option<(u32, u32)> {
    let mut parts = version.split('.');
    let major = parts.next()?.parse().ok()?;
    let minor = parts.next()?.parse().ok()?;
    Some((major, minor))
}

/// Checks whether a version is within the bounds of a protocol version.
/// A version is within bounds if its major is not above the protocol's,
/// and if its major equals the protocol's, its minor is not above the protocol's.
/// Unparseable versions are out of bounds.
fn is_version_within_bounds(version: &str, protocol_version: &str) -> bool {
    let Some((our_major, our_minor)) = parse_version(protocol_version) else {
        return false;
    };
    let Some((their_major, their_minor)) = parse_version(version) else {
        return false;
    };
    if their_major > our_major {
        return false;
    }
    if their_major == our_major && their_minor > our_minor {
        return false;
    }
    true
}

/// Validates received capability versions against a given protocol version
/// and set of supported versions.
///
/// Each version is checked in two stages:
/// 1. Is it within bounds of protocol_version? If not → UnsupportedVersion
/// 2. Is it in supported_versions? If not → CapabilityMismatch
///
/// UnsupportedVersion takes priority. If there are both out-of-bounds and
/// in-bounds-but-unsupported versions, UnsupportedVersion is returned.
/// If all are within bounds but some aren't in supported_versions,
/// CapabilityMismatch is returned.
fn validate_capabilities_against(
    versions: &[&str],
    protocol_version: &str,
    supported_versions: &[&str],
) -> Result<(), CapabilitiesError> {
    let mut unsupported_versions: Vec<&str> = Vec::new();
    let mut mismatched_versions: Vec<&str> = Vec::new();

    for &v in versions {
        if !is_version_within_bounds(v, protocol_version) {
            unsupported_versions.push(v);
        } else if !supported_versions.contains(&v) {
            mismatched_versions.push(v);
        }
    }

    if !unsupported_versions.is_empty() {
        return Err(CapabilitiesError {
            code: CapabilitiesErrorCode::UnsupportedVersion,
            message: format!("Unsupported versions: {:?}", unsupported_versions),
            details: serde_json::json!({
                "unsupported": unsupported_versions,
                "protocolVersion": protocol_version,
            }),
        });
    }

    if !mismatched_versions.is_empty() {
        return Err(CapabilitiesError {
            code: CapabilitiesErrorCode::CapabilityMismatch,
            message: format!(
                "Capability mismatch for versions: {:?}",
                mismatched_versions
            ),
            details: serde_json::json!({
                "mismatched": mismatched_versions,
                "received": versions,
                "supported": supported_versions,
            }),
        });
    }

    Ok(())
}

/// Validates received capability versions against the agent's
/// PROTOCOL_VERSION and SUPPORTED_VERSIONS.
pub fn validate_capabilities(versions: &[&str]) -> Result<(), CapabilitiesError> {
    validate_capabilities_against(versions, PROTOCOL_VERSION, SUPPORTED_VERSIONS)
}

#[cfg(test)]
mod tests {
    use super::*;

    // --- is_version_within_bounds tests ---

    #[test]
    fn test_exact_match_is_within_bounds() {
        assert!(is_version_within_bounds("1.2", "1.2"));
    }

    #[test]
    fn test_lower_version_is_within_bounds() {
        assert!(is_version_within_bounds("0.0", "1.2"));
        assert!(is_version_within_bounds("1.0", "1.2"));
        assert!(is_version_within_bounds("1.1", "1.2"));
    }

    #[test]
    fn test_higher_major_is_out_of_bounds() {
        assert!(!is_version_within_bounds("2.0", "1.2"));
    }

    #[test]
    fn test_higher_minor_is_out_of_bounds() {
        assert!(!is_version_within_bounds("1.3", "1.2"));
    }

    #[test]
    fn test_invalid_version_is_out_of_bounds() {
        assert!(!is_version_within_bounds("abc", "1.0"));
        assert!(!is_version_within_bounds("", "1.0"));
        assert!(!is_version_within_bounds("1", "1.0"));
    }

    #[test]
    fn test_invalid_protocol_version_is_out_of_bounds() {
        assert!(!is_version_within_bounds("1.0", "abc"));
    }

    // --- validate_capabilities_against tests ---

    #[test]
    fn test_validate_all_supported() {
        assert!(validate_capabilities_against(&["1.0", "1.1"], "1.2", &["1.0", "1.1"]).is_ok());
    }

    #[test]
    fn test_validate_empty_returns_ok() {
        assert!(validate_capabilities_against(&[], "1.0", &["1.0"]).is_ok());
    }

    #[test]
    fn test_validate_higher_version_returns_unsupported() {
        let err =
            validate_capabilities_against(&["2.0"], "1.2", &["1.0", "1.1", "1.2"]).unwrap_err();
        assert_eq!(err.code, CapabilitiesErrorCode::UnsupportedVersion);
        let unsupported = err.details["unsupported"].as_array().unwrap();
        assert_eq!(unsupported, &[serde_json::json!("2.0")]);
    }

    #[test]
    fn test_validate_mixed_with_too_high_returns_unsupported() {
        // UnsupportedVersion takes priority over CapabilityMismatch
        let err =
            validate_capabilities_against(&["1.0", "99.0"], "1.2", &["1.0", "1.2"]).unwrap_err();
        assert_eq!(err.code, CapabilitiesErrorCode::UnsupportedVersion);
        let unsupported = err.details["unsupported"].as_array().unwrap();
        assert_eq!(unsupported, &[serde_json::json!("99.0")]);
    }

    #[test]
    fn test_validate_higher_minor_returns_unsupported() {
        let err = validate_capabilities_against(&["1.3"], "1.2", &["1.0", "1.2"]).unwrap_err();
        assert_eq!(err.code, CapabilitiesErrorCode::UnsupportedVersion);
    }

    #[test]
    fn test_validate_error_details_list_all_unsupported() {
        let err =
            validate_capabilities_against(&["2.0", "3.0"], "1.2", &["1.0", "1.2"]).unwrap_err();
        assert_eq!(err.code, CapabilitiesErrorCode::UnsupportedVersion);
        let unsupported = err.details["unsupported"].as_array().unwrap();
        assert_eq!(unsupported.len(), 2);
        assert!(unsupported.contains(&serde_json::json!("2.0")));
        assert!(unsupported.contains(&serde_json::json!("3.0")));
        assert_eq!(err.details["protocolVersion"], "1.2");
    }

    #[test]
    fn test_within_bounds_but_not_supported_returns_capability_mismatch() {
        // 1.1 is within bounds of protocol 1.2, but not in supported list
        let err = validate_capabilities_against(&["1.1"], "1.2", &["1.0", "1.2"]).unwrap_err();
        assert_eq!(err.code, CapabilitiesErrorCode::CapabilityMismatch);
        let mismatched = err.details["mismatched"].as_array().unwrap();
        assert_eq!(mismatched, &[serde_json::json!("1.1")]);
        let supported = err.details["supported"].as_array().unwrap();
        assert_eq!(
            supported,
            &[serde_json::json!("1.0"), serde_json::json!("1.2")]
        );
    }

    #[test]
    fn test_unsupported_takes_priority_over_mismatch() {
        // 1.1 is mismatched, 2.0 is unsupported — should get UnsupportedVersion
        let err =
            validate_capabilities_against(&["1.1", "2.0"], "1.2", &["1.0", "1.2"]).unwrap_err();
        assert_eq!(err.code, CapabilitiesErrorCode::UnsupportedVersion);
    }
}
