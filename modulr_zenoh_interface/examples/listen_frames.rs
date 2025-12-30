use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};

use modulr_zenoh_interface::ZenohInterface;

#[tokio::main]
async fn main() {
    // Enable logging to see library debug/error messages
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("debug")).init();

    let interface = ZenohInterface::new();

    let frame_count = Arc::new(AtomicU64::new(0));
    let frame_count_clone = Arc::clone(&frame_count);

    interface
        .add_frame_listener(Box::new(move |data| {
            let count = frame_count_clone.fetch_add(1, Ordering::Relaxed) + 1;
            let size = data.len();
            println!("Received frame {count}: {size} bytes");
            Box::pin(async {})
        }))
        .await;

    println!("Launching interface...");
    interface
        .launch()
        .await
        .expect("Failed to launch interface");
    println!("Interface launched, waiting for frames...");

    // Keep the program running
    loop {
        tokio::time::sleep(std::time::Duration::from_secs(1)).await;
    }
}
