use modulr_ros_messages::ros2_messages::{builtin_interfaces, sensor_msgs, std_msgs};

#[tokio::main]
async fn main() {
    let session = zenoh::open(zenoh::Config::default()).await.unwrap();

    let publisher = session.declare_publisher("camera/image_raw").await.unwrap();

    // 1080p RGB image (1920x1080, 3 bytes per pixel)
    let width = 1920u32;
    let height = 1080u32;
    let bytes_per_pixel = 3;
    let step = width * bytes_per_pixel;
    // Half black (top), half white (bottom)
    let mut data = vec![0u8; (height * step) as usize];
    let half_height = height / 2;

    for y in half_height..height {
        let row_start = (y * step) as usize;
        let row_end = row_start + step as usize;
        data[row_start..row_end].fill(255);
    }

    let image = sensor_msgs::Image {
        header: std_msgs::Header {
            stamp: builtin_interfaces::Time { sec: 0, nanosec: 0 },
            frame_id: "camera".to_string(),
        },
        height,
        width,
        encoding: "bgr8".to_string(),
        is_bigendian: 0,
        step,
        data,
    };

    // Serialize the image once since it doesn't change
    let payload: Vec<u8> = cdr::serialize::<_, _, cdr::CdrLe>(&image, cdr::size::Infinite).unwrap();

    // 30 FPS
    let mut interval = tokio::time::interval(std::time::Duration::from_millis(33));

    let mut frame_count = 0u64;
    loop {
        publisher.put(&payload).await.unwrap();

        frame_count += 1;
        if frame_count.is_multiple_of(30) {
            println!("Published {} frames", frame_count);
        }
        interval.tick().await;
    }
}
