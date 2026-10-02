#!/bin/bash
# Check ZED GPS Integration system status

echo "========================================="
echo "  ZED GPS Integration - System Check"
echo "========================================="
echo ""

# Check if ROS is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ ROS not sourced"
    echo "   Run: source ~/workspaces/rover/install/setup.bash"
    exit 1
else
    echo "✅ ROS sourced: $ROS_DISTRO"
fi

# Check GPS topic
echo ""
echo "Checking GPS topic..."
if ros2 topic info /gps/fix &> /dev/null; then
    echo "✅ GPS topic exists: /gps/fix"
    
    # Check if publishing
    RATE=$(timeout 2 ros2 topic hz /gps/fix 2>&1 | grep "average rate" | awk '{print $3}')
    if [ -n "$RATE" ]; then
        echo "✅ GPS publishing at ${RATE} Hz"
    else
        echo "⚠️  GPS topic exists but not publishing"
    fi
else
    echo "❌ GPS topic not found"
    echo "   Start GPS: ros2 launch gnss_launch gnss.launch.py"
fi

# Check fusion node
echo ""
echo "Checking fusion node..."
if ros2 node list | grep -q "zed_gnss_fusion"; then
    echo "✅ Fusion node running"
    
    # Check topics
    if ros2 topic info /zed_gnss_fusion/geo_pose &> /dev/null; then
        echo "✅ Fusion topics available"
        
        # Check if publishing
        FUSED_RATE=$(timeout 2 ros2 topic hz /zed_gnss_fusion/geo_pose 2>&1 | grep "average rate" | awk '{print $3}')
        if [ -n "$FUSED_RATE" ]; then
            echo "✅ Fused pose publishing at ${FUSED_RATE} Hz"
            echo "   🎉 CALIBRATION COMPLETE!"
        else
            echo "⏳ Fusion running but geo_pose not publishing yet"
            echo "   Still calibrating... Move camera to complete"
        fi
    fi
else
    echo "❌ Fusion node not running"
    echo "   Start: ros2 launch zed_gps_integration zed_gnss_fusion.launch.py"
fi

# Check map server
echo ""
echo "Checking map server..."
if ros2 node list | grep -q "map_server"; then
    echo "✅ Map server running"
else
    echo "❌ Map server not running"
fi

# Check map data files
echo ""
echo "Checking map data files..."
if [ -f ~/zed_map_data/data.txt ]; then
    AGE=$(( $(date +%s) - $(stat -c %Y ~/zed_map_data/data.txt) ))
    if [ $AGE -lt 5 ]; then
        echo "✅ Map data fresh (${AGE}s ago)"
    else
        echo "⚠️  Map data stale (${AGE}s ago)"
    fi
    
    cat ~/zed_map_data/data.txt
else
    echo "❌ No map data yet"
    echo "   Wait for calibration to complete"
fi

# Check if web server might be running
echo ""
echo "Checking web server..."
if lsof -i:8000 &> /dev/null; then
    echo "✅ Web server running on port 8000"
    echo "   View map: http://localhost:8000/"
else
    echo "❌ Web server not running"
    echo "   Start: cd ~/zed_map_data && python3 -m http.server 8000"
fi

echo ""
echo "========================================="
echo "System check complete"
echo "========================================="

