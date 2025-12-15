FROM ghcr.io/cross-rs/x86_64-unknown-linux-gnu:main

RUN apt update && \
    apt remove -y --allow-remove-essential gcc-* && \
    apt install -y clang libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libssl-dev && \
    rm -rf /var/lib/apt/lists/*
