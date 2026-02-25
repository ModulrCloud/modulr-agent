use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub struct Position {
    pub x: f64,
    pub y: f64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub z: Option<f64>,
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub struct Orientation {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub yaw: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub pitch: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub roll: Option<f64>,
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub struct Location {
    pub name: String,
    pub position: Position,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub orientation: Option<Orientation>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub metadata: Option<serde_json::Value>,
}

impl Location {
    pub fn validate(&self) -> Result<(), &'static str> {
        if self.name.is_empty() {
            return Err("location name must not be empty");
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    #[test]
    fn test_location_serialises_optional_fields_omitted() {
        let loc = Location {
            name: "Dock A".to_string(),
            position: Position {
                x: 1.0,
                y: 2.0,
                z: None,
            },
            orientation: None,
            metadata: None,
        };
        let val = serde_json::to_value(&loc).unwrap();
        assert!(!val.as_object().unwrap().contains_key("z"));
        assert!(!val.as_object().unwrap().contains_key("orientation"));
        assert!(!val.as_object().unwrap().contains_key("metadata"));
    }

    #[test]
    fn test_location_round_trip_with_all_fields() {
        let json = json!({
            "name": "Warehouse",
            "position": { "x": 3.0, "y": 4.0, "z": 1.5 },
            "orientation": { "yaw": 1.57 },
            "metadata": { "zone": "B" }
        });
        let loc: Location = serde_json::from_value(json).unwrap();
        assert_eq!(loc.name, "Warehouse");
        assert_eq!(loc.position.z, Some(1.5));
        assert_eq!(loc.orientation.unwrap().yaw, Some(1.57));
    }

    #[test]
    fn test_validate_passes_for_valid_location() {
        let loc = Location {
            name: "Home".to_string(),
            position: Position {
                x: 0.0,
                y: 0.0,
                z: None,
            },
            orientation: None,
            metadata: None,
        };
        assert!(loc.validate().is_ok());
    }

    #[test]
    fn test_validate_fails_for_empty_name() {
        let loc = Location {
            name: "".to_string(),
            position: Position {
                x: 0.0,
                y: 0.0,
                z: None,
            },
            orientation: None,
            metadata: None,
        };
        assert!(loc.validate().is_err());
    }

}
