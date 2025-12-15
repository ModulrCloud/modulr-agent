FROM ghcr.io/cross-rs/aarch64-unknown-linux-gnu:main

RUN dpkg --add-architecture arm64 && \
    apt update && \
    apt install -y libgstreamer1.0-dev:arm64 libgstreamer-plugins-base1.0-dev:arm64 libssl-dev:arm64 && \
    rm -rf /var/lib/apt/lists/*
