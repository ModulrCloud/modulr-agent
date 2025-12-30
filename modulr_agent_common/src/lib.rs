//! # Modulr Agent Common
//!
//! Shared types and utilities for the modulr agent ecosystem.

use clap::ValueEnum;
use serde::{Deserialize, Serialize};

/// Format of image frames received from video sources.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, ValueEnum, Default)]
pub enum ImageFormat {
    /// Raw BGR pixel data (sensor_msgs/Image)
    #[default]
    Raw,
    /// JPEG compressed data (sensor_msgs/CompressedImage)
    Jpeg,
}
