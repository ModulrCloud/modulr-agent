FROM ghcr.io/cross-rs/x86_64-unknown-linux-gnu:main

RUN apt-get update && \
    apt-get install -y clang libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libssl-dev && \
    rm -rf /var/lib/apt/lists/*

# Use clang instead of gcc
ENV CC=clang
ENV CXX=clang++
