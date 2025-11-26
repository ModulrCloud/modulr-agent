#!/bin/bash

# Script to start camera simulation for Modulr Agent
# This starts rosbridge and the camera node

set -e

# Source ROS2
if [ -z "$ROS_DISTRO" ]; then
    if [ -f /opt/ros/humble/setup.bash ]; then
        source /opt/ros/humble/setup.bash
        export ROS_DISTRO=humble
    elif [ -f /opt/ros/jazzy/setup.bash ]; then
        source /opt/ros/jazzy/setup.bash
        export ROS_DISTRO=jazzy
    elif [ -f /opt/ros/jade/setup.bash ]; then
        # Legacy check for jade (shouldn't exist, but just in case)
        source /opt/ros/jade/setup.bash
        export ROS_DISTRO=jade
    else
        echo "Error: ROS2 not found. Please install ROS2 first."
        echo "Run: ./scripts/setup_ros2_ubuntu.sh"
        exit 1
    fi
fi

echo "=========================================="
echo "Starting Camera Simulation for Modulr Agent"
echo "=========================================="
echo "ROS Distribution: $ROS_DISTRO"
echo ""

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# Check if camera node exists
CAMERA_NODE="$SCRIPT_DIR/camera_node.py"
if [ ! -f "$CAMERA_NODE" ]; then
    echo "Error: Camera node not found at $CAMERA_NODE"
    exit 1
fi

# Make camera node executable
chmod +x "$CAMERA_NODE"

# Parse camera index argument
CAMERA_INDEX=${1:-0}
echo "Using camera index: $CAMERA_INDEX"
echo ""

# Function to cleanup on exit
cleanup() {
    echo ""
    echo "Shutting down..."
    kill $ROSBRIDGE_PID 2>/dev/null || true
    kill $CAMERA_PID 2>/dev/null || true
    wait $ROSBRIDGE_PID 2>/dev/null || true
    wait $CAMERA_PID 2>/dev/null || true
    echo "Done."
    exit 0
}

trap cleanup SIGINT SIGTERM

# Start rosbridge in background
echo "Starting rosbridge server..."
ros2 launch rosbridge_server rosbridge_websocket_launch.xml > /tmp/rosbridge.log 2>&1 &
ROSBRIDGE_PID=$!
sleep 2

# Check if rosbridge started successfully
if ! kill -0 $ROSBRIDGE_PID 2>/dev/null; then
    echo "Error: Failed to start rosbridge. Check /tmp/rosbridge.log for details."
    exit 1
fi
echo "rosbridge started (PID: $ROSBRIDGE_PID)"
echo ""

# Start camera node in background
echo "Starting camera node..."
cd "$PROJECT_ROOT"
python3 "$CAMERA_NODE" "$CAMERA_INDEX" > /tmp/camera_node.log 2>&1 &
CAMERA_PID=$!
sleep 2

# Check if camera node started successfully
if ! kill -0 $CAMERA_PID 2>/dev/null; then
    echo "Error: Failed to start camera node. Check /tmp/camera_node.log for details."
    kill $ROSBRIDGE_PID 2>/dev/null || true
    exit 1
fi
echo "Camera node started (PID: $CAMERA_PID)"
echo ""

echo "=========================================="
echo "Camera simulation is running!"
echo "=========================================="
echo ""
echo "Services running:"
echo "  - rosbridge: PID $ROSBRIDGE_PID (logs: /tmp/rosbridge.log)"
echo "  - camera node: PID $CAMERA_PID (logs: /tmp/camera_node.log)"
echo ""
echo "Topics available:"
ros2 topic list | grep -E "(camera|cmd_vel)" || echo "  (waiting for topics...)"
echo ""
echo "Press Ctrl+C to stop all services"
echo ""

# Wait for user interrupt
wait

