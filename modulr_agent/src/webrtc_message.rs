use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct MovementCommand {
    pub forward: f64,
    pub turn: f64,
}

#[derive(Serialize, Deserialize, Clone, Debug)]
#[serde(tag = "type", content = "params")]
pub enum WebRtcMessage {
    MovementCommand(MovementCommand),
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_mvmt_command() {
        let mvmt = WebRtcMessage::MovementCommand(MovementCommand {
            forward: 3.0,
            turn: 5.0,
        });
        let decoded = serde_json::from_str(&serde_json::to_string(&mvmt).unwrap()).unwrap();
        match decoded {
            WebRtcMessage::MovementCommand(cmd) => {
                assert_eq!(cmd.forward, 3.0);
                assert_eq!(cmd.turn, 5.0);
            }
        }
    }
}
