FROM ghcr.io/cross-rs/x86_64-unknown-linux-gnu:main

RUN apt-get update && \
    apt-get install -y clang libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libssl-dev curl && \
    rm -rf /var/lib/apt/lists/*

# Install rustup and add clippy/rustfmt for linting
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --default-toolchain stable
ENV PATH="/root/.cargo/bin:${PATH}"
RUN rustup default stable && rustup component add clippy rustfmt

# Use clang instead of gcc
ENV CC=clang
ENV CXX=clang++
