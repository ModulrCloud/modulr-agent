# Use the official Cross ARM64 base image
FROM ghcr.io/cross-rs/armv7-unknown-linux-gnueabihf:main

# Add armv7 architecture and install target libraries
RUN dpkg --add-architecture armhf && \
    apt update && \
    apt install -y --no-install-recommends \
        libssl-dev \
        pkg-config \
        libssl-dev:armhf \
        libgstreamer1.0-dev:armhf \
        libgstreamer-plugins-base1.0-dev:armhf \
        g++-arm-linux-gnueabihf && \
    rm -rf /var/lib/apt/lists/*

# Host OpenSSL
ENV OPENSSL_DIR=/usr
ENV OPENSSL_LIB_DIR=/usr/lib/x86_64-linux-gnu
ENV OPENSSL_INCLUDE_DIR=/usr/include

# Target OpenSSL
ENV OPENSSL_TARGET_DIR=/usr/armhf-linux-gnu
ENV PKG_CONFIG_ALLOW_CROSS=1
ENV PKG_CONFIG_PATH=/usr/lib/arm-linux-gnueabihf/pkgconfig
