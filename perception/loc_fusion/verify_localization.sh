#!/bin/bash
# Verify GPS+ZED localization is working

echo "🔍 Localization System Verification"
echo "="*70
echo ""

source /opt/ros/humble/setup.bash 2>/dev/null

# Check required nodes
echo "📊 Checking Nodes..."
REQUIRED_NODES=(
    "ekf_local"
    "ekf_global"
    "navsat_transform"
    "gnss_driver"
)

MISSING=()
for NODE in "${REQUIRED_NODES[@]}"; do
    if ros2 node list 2>/dev/null | grep -q "$NODE"; then
        echo "  ✓ $NODE"
    else
        echo "  ✗ $NODE (missing)"
        MISSING+=("$NODE")
    fi
done

if [ ${#MISSING[@]} -gt 0 ]; then
    echo ""
    echo "❌ Some nodes not running. Start with:"
    echo "   ros2 launch loc_fusion loc_fusion.launch.py"
    exit 1
fi

echo ""
echo "📡 Checking Topics..."

# Check TF transforms
echo "🔗 TF Transforms:"
if timeout 2 ros2 run tf2_ros tf2_echo map odom 2>/dev/null | grep -q "At time"; then
    echo "  ✓ map → odom"
else
    echo "  ⚠  map → odom (not yet available)"
fi

if timeout 2 ros2 run tf2_ros tf2_echo odom base_link 2>/dev/null | grep -q "At time"; then
    echo "  ✓ odom → base_link"
else
    echo "  ✗ odom → base_link (check ekf_local)"
fi

echo ""
echo "🛰️  GPS Status:"
GPS_STATUS=$(timeout 2 ros2 topic echo /gps/fix --once 2>/dev/null | grep "status:" | head -1 | awk '{print $2}')

case $GPS_STATUS in
    2)
        echo "  ✅ RTK FIX (cm accuracy)"
        ;;
    1)
        echo "  🟡 GPS FIX (meter accuracy)"
        ;;
    *)
        echo "  ⚠️  No GPS fix (go outside)"
        ;;
esac

echo ""
echo "📍 Odometry Topics:"

# Check local odometry
if timeout 1 ros2 topic hz /odometry/filtered 2>&1 | grep -q "average rate"; then
    RATE=$(timeout 1 ros2 topic hz /odometry/filtered 2>&1 | grep "average rate" | awk '{print $3}')
    echo "  ✓ /odometry/filtered: ${RATE}Hz"
else
    echo "  ✗ /odometry/filtered (not publishing)"
fi

# Check GPS odometry
if timeout 1 ros2 topic hz /odom/gps 2>&1 | grep -q "average rate"; then
    RATE=$(timeout 1 ros2 topic hz /odom/gps 2>&1 | grep "average rate" | awk '{print $3}')
    echo "  ✓ /odom/gps: ${RATE}Hz"
else
    echo "  ⚠  /odom/gps (waiting for GPS fix)"
fi

# Check global odometry
if timeout 1 ros2 topic hz /odometry/global 2>&1 | grep -q "average rate"; then
    RATE=$(timeout 1 ros2 topic hz /odometry/global 2>&1 | grep "average rate" | awk '{print $3}')
    echo "  ✓ /odometry/global: ${RATE}Hz"
else
    echo "  ⚠  /odometry/global (waiting for GPS)"
fi

echo ""
echo "="*70
echo "📋 SUMMARY"
echo "="*70

if [ ${#MISSING[@]} -eq 0 ] && [ "$GPS_STATUS" != "" ]; then
    echo "✅ Localization system operational!"
    echo ""
    echo "TF Chain: map → odom → base_link"
    echo "  - map frame: GPS-anchored global reference"
    echo "  - odom frame: Smooth VIO from ZED"
    echo "  - base_link: Robot center"
    echo ""
    echo "Ready for Nav2 waypoint navigation!"
else
    echo "⚠️  System needs attention"
    echo ""
    if [ "$GPS_STATUS" = "" ]; then
        echo "- Go outside for GPS fix"
    fi
    echo "- Wait 30-60 seconds for all systems to initialize"
fi

echo ""

