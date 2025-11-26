#!/bin/bash
# Check if /camera/image_raw topic is available and has publishers

echo "Checking ROS2 topic availability..."
echo ""

source /opt/ros/jazzy/setup.bash

echo "1. Checking if topic exists:"
ros2 topic list | grep -q "/camera/image_raw" && echo "   ✅ Topic /camera/image_raw exists" || echo "   ❌ Topic /camera/image_raw NOT found"

echo ""
echo "2. Getting topic info:"
ros2 topic info /camera/image_raw 2>/dev/null || echo "   ❌ Cannot get topic info (topic may not exist)"

echo ""
echo "3. Checking if topic has publishers:"
PUB_COUNT=$(ros2 topic info /camera/image_raw 2>/dev/null | grep -c "Publisher count: 1" || echo "0")
if [ "$PUB_COUNT" -gt 0 ]; then
    echo "   ✅ Topic has active publishers"
else
    echo "   ❌ Topic has NO active publishers"
fi

echo ""
echo "4. Checking message rate:"
echo "   (This will run for 3 seconds, press Ctrl+C to stop early)"
timeout 3 ros2 topic hz /camera/image_raw 2>/dev/null || echo "   ❌ Cannot measure topic rate (no messages?)"

echo ""
echo "Done!"

