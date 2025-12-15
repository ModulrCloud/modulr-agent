FROM ghcr.io/cross-rs/armv7-unknown-linux-gnueabihf:latest

RUN dpkg --add-architecture armhf && \
    apt update && \
    apt install -y libgstreamer1.0-dev:armhf libgstreamer-plugins-base1.0-dev:armhf libssl-dev:armhf pkg-config:armhf && \
    rm -rf /var/lib/apt/lists/*

ENV CARGO_TARGET_ARMV7_UNKNOWN_LINUX_GNUEABIHF_LINKER=arm-linux-gnueabihf-gcc \
    CC_armv7_unknown_linux_gnueabihf=arm-linux-gnueabihf-gcc \
    CXX_armv7_unknown_linux_gnueabihf=arm-linux-gnueabihf-g++ \
    QEMU_LD_PREFIX=/usr/arm-linux-gnueabihf \
    RUST_TEST_THREADS=1 \
    PKG_CONFIG_PATH=/usr/lib/arm-linux-gnueabihf/pkgconfig
