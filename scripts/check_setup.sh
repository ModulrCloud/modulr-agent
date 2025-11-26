#!/bin/bash

# Quick setup status checker for Modulr Agent
# This script checks what's installed and what's missing

echo "=========================================="
echo "Modulr Agent Setup Status Check"
echo "=========================================="
echo ""

# Check Rust
echo -n "Checking Rust... "
if command -v rustc &> /dev/null; then
    RUST_VERSION=$(rustc --version)
    echo "✓ INSTALLED ($RUST_VERSION)"
else
    echo "✗ NOT INSTALLED"
    echo "  Install with: curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh"
fi

# Check Cargo
echo -n "Checking Cargo... "
if command -v cargo &> /dev/null; then
    CARGO_VERSION=$(cargo --version)
    echo "✓ INSTALLED ($CARGO_VERSION)"
else
    echo "✗ NOT INSTALLED"
fi

# Check ROS2
echo -n "Checking ROS2... "
if [ -f /opt/ros/humble/setup.bash ]; then
    echo "✓ INSTALLED (Humble)"
    ROS_DISTRO="humble"
elif [ -f /opt/ros/jade/setup.bash ]; then
    echo "✓ INSTALLED (Jade)"
    ROS_DISTRO="jade"
elif [ -f /opt/ros/foxy/setup.bash ]; then
    echo "✓ INSTALLED (Foxy)"
    ROS_DISTRO="foxy"
else
    echo "✗ NOT INSTALLED"
    echo "  Install with: ./scripts/setup_ros2_ubuntu.sh"
    ROS_DISTRO=""
fi

# Check rosbridge
if [ -n "$ROS_DISTRO" ]; then
    echo -n "Checking rosbridge-suite... "
    if dpkg -l | grep -q "ros-$ROS_DISTRO-rosbridge-suite"; then
        echo "✓ INSTALLED"
    else
        echo "✗ NOT INSTALLED"
        echo "  Install with: sudo apt install ros-$ROS_DISTRO-rosbridge-suite"
    fi
fi

# Check GStreamer
echo -n "Checking GStreamer... "
if pkg-config --exists gstreamer-1.0; then
    GST_VERSION=$(pkg-config --modversion gstreamer-1.0)
    echo "✓ INSTALLED ($GST_VERSION)"
else
    echo "✗ NOT INSTALLED"
    echo "  Install with: sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav libgstrtspserver-1.0-dev libges-1.0-dev"
fi

# Check OpenCV
echo -n "Checking OpenCV (Python)... "
if python3 -c "import cv2" 2>/dev/null; then
    CV_VERSION=$(python3 -c "import cv2; print(cv2.__version__)" 2>/dev/null)
    echo "✓ INSTALLED ($CV_VERSION)"
else
    echo "✗ NOT INSTALLED"
    echo "  Install with: sudo apt install python3-opencv"
fi

# Check cv_bridge
if [ -n "$ROS_DISTRO" ]; then
    echo -n "Checking cv_bridge... "
    if dpkg -l | grep -q "ros-$ROS_DISTRO-cv-bridge"; then
        echo "✓ INSTALLED"
    else
        echo "✗ NOT INSTALLED"
        echo "  Install with: sudo apt install ros-$ROS_DISTRO-cv-bridge"
    fi
fi

# Check camera
echo -n "Checking camera access... "
if command -v v4l2-ctl &> /dev/null; then
    if v4l2-ctl --list-devices 2>/dev/null | grep -q "/dev/video"; then
        CAMERA_COUNT=$(v4l2-ctl --list-devices 2>/dev/null | grep -c "/dev/video" || echo "0")
        echo "✓ AVAILABLE ($CAMERA_COUNT device(s) found)"
    else
        echo "⚠ NO CAMERAS FOUND"
    fi
else
    echo "⚠ v4l2-utils NOT INSTALLED"
    echo "  Install with: sudo apt install v4l-utils"
fi

# Check if modulr-agent is built
echo -n "Checking Modulr Agent build... "
if [ -f "target/debug/modulr_agent" ] || [ -f "target/release/modulr_agent" ]; then
    echo "✓ BUILT"
else
    echo "✗ NOT BUILT"
    echo "  Build with: cargo build"
fi

# Check camera node script
echo -n "Checking camera node script... "
if [ -f "scripts/camera_node.py" ]; then
    if [ -x "scripts/camera_node.py" ]; then
        echo "✓ EXISTS AND EXECUTABLE"
    else
        echo "⚠ EXISTS BUT NOT EXECUTABLE"
        echo "  Make executable with: chmod +x scripts/camera_node.py"
    fi
else
    echo "✗ NOT FOUND"
fi

echo ""
echo "=========================================="
echo "Summary"
echo "=========================================="
echo ""
echo "To see detailed setup instructions, check:"
echo "  scripts/FRESH_SETUP_GUIDE.md"
echo ""
echo "Quick setup commands:"
echo "  1. Install Rust: curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh"
echo "  2. Install ROS2: ./scripts/setup_ros2_ubuntu.sh"
echo "  3. Build agent: cargo build"
echo "  4. Test camera: python3 scripts/camera_node.py"
echo ""

