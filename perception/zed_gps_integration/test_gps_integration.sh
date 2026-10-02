#!/bin/bash
# ZED + GPS Integration Test Script
# Run this script once you're OUTSIDE with clear sky view

echo "=========================================="
echo "  ZED + GPS Integration Test"
echo "=========================================="
echo ""

# Kill any existing processes
echo "🧹 Cleaning up existing processes..."
pkill -9 -f ros
sleep 3

# Launch GPS
echo "📡 Launching GPS driver..."
(source /opt/ros/humble/setup.bash && \
 source ~/workspaces/ros2-ublox-zedf9p/install/setup.bash && \
 ros2 launch ~/workspaces/rover/install/gnss_launch/share/gnss_launch/launch/gnss.launch.py > /tmp/gps_integration_test.log 2>&1 &)
sleep 15
echo "   GPS driver started (waiting for configuration...)"

# Launch ZED
echo "📷 Launching ZED camera..."
(cd ~/workspaces/rover && \
 source /opt/ros/humble/setup.bash && \
 source install/setup.bash && \
 ros2 launch zed2i_launch zed2i_driver.launch.py > /tmp/zed_integration_test.log 2>&1 &)
sleep 25
echo "   ZED camera started (initializing...)"

# Launch map server
echo "🗺️  Launching map server..."
(cd ~/workspaces/rover && \
 source /opt/ros/humble/setup.bash && \
 source install/setup.bash && \
 ~/workspaces/rover/install/zed_gps_integration/bin/map_server > /tmp/map_server_integration_test.log 2>&1 &)
sleep 5
echo "   Map server started"

# Copy web map if needed
if [ ! -f ~/zed_map_data/index.html ]; then
    cp ~/workspaces/rover/install/zed_gps_integration/share/zed_gps_integration/web_map/index.html ~/zed_map_data/
fi

# Launch web server
echo "🌐 Launching web server..."
(cd ~/zed_map_data && python3 -m http.server 8000 > /tmp/webserver_integration_test.log 2>&1 &)
sleep 2

# Open browser
echo "🚀 Opening web browser..."
DISPLAY=:0 xdg-open http://localhost:8000/ 2>/dev/null &

echo ""
echo "=========================================="
echo "  ✅ All Systems Launched!"
echo "=========================================="
echo ""
echo "📍 Check the browser for live map"
echo "🌍 It may take 30-60 seconds for GPS to get a fix"
echo ""
echo "Quick status check:"
echo "-------------------"
sleep 2

source /opt/ros/humble/setup.bash
echo "ROS Nodes running:"
ros2 node list 2>/dev/null | grep -E "(gnss|zed|map)" | head -10

echo ""
echo "GPS Status:"
timeout 2 ros2 topic echo /gps/fix --once 2>/dev/null | grep -E "(status|latitude|longitude)" | head -3

echo ""
echo "=========================================="
echo "Logs available at:"
echo "  GPS: /tmp/gps_integration_test.log"
echo "  ZED: /tmp/zed_integration_test.log"
echo "  Map: /tmp/map_server_integration_test.log"
echo "=========================================="
echo ""
echo "🎯 Next: Walk around and watch the map!"

