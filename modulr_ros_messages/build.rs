use std::path::PathBuf;

fn generate_messages(
    paths: Vec<PathBuf>,
    out_file: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    let (source, dependent_paths) =
        roslibrust::codegen::find_and_generate_ros_messages_without_ros_package_path(paths)?;
    let out_dir = std::env::var_os("OUT_DIR").unwrap();
    let dest_path = std::path::Path::new(&out_dir).join(out_file);
    std::fs::write(dest_path, source.to_string())?;

    for path in &dependent_paths {
        // Skip files causing over-eager rebuilds as per https://github.com/RosLibRust/roslibrust/issues/292
        if path.ends_with("Duration.msg")
            || path.ends_with("Time.msg")
            || path.ends_with("ServiceEventInfo.msg")
        {
            continue;
        }
        println!("cargo:rerun-if-changed={}", path.display());
    }

    Ok(())
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ros1_paths = vec!["./assets/ros1_common_interfaces".into()];

    let ros2_paths = vec![
        "./assets/ros2_common_interfaces".into(),
        "./assets/ros2_required_msgs/rcl_interfaces/builtin_interfaces".into(),
    ];

    generate_messages(ros1_paths, "ros1_messages.rs")?;
    generate_messages(ros2_paths, "ros2_messages.rs")?;

    Ok(())
}
