# Use the official Cross ARM64 base image
FROM ghcr.io/cross-rs/aarch64-unknown-linux-gnu:main

# Add ARM64 architecture and install target libraries
RUN dpkg --add-architecture arm64 && \
    apt update && \
    apt install -y --no-install-recommends \
        libssl-dev \
        pkg-config \
        libssl-dev:arm64 \
        libgstreamer1.0-dev:arm64 \
        libgstreamer-plugins-base1.0-dev:arm64 \
        g++-aarch64-linux-gnu && \
    rm -rf /var/lib/apt/lists/*

# Host OpenSSL
ENV OPENSSL_DIR=/usr
ENV OPENSSL_LIB_DIR=/usr/lib/x86_64-linux-gnu
ENV OPENSSL_INCLUDE_DIR=/usr/include

# Target OpenSSL
ENV OPENSSL_TARGET_DIR=/usr/aarch64-linux-gnu
ENV PKG_CONFIG_ALLOW_CROSS=1
ENV PKG_CONFIG_PATH=/usr/lib/aarch64-linux-gnu/pkgconfig
