use thiserror::Error;

#[derive(Error, Debug)]
pub enum RosBridgeError {
    #[error("Failed to initialise ROS context")]
    InitFailure,
    #[error("Failed to create ROS node")]
    NodeCreateFailure,
    #[error("Failed to create ROS publisher")]
    PublisherCreateFailure,
    #[error("Failed to create ROS subscription")]
    SubscriptionCreateFailure,
    #[error("Failed to publish message")]
    PublishFailed,
}
