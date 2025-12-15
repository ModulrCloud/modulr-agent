FROM ghcr.io/cross-rs/x86_64-unknown-linux-gnu:main

RUN apt update && \
    apt install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libssl-dev && \
    rm -rf /var/lib/apt/lists/*
