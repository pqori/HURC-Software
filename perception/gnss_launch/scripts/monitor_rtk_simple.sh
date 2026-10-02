#!/bin/bash
# Simple RTK Monitor - just shows the key fields

source /opt/ros/jazzy/setup.bash 2>/dev/null

echo "🛰️  RTK Monitor (Press Ctrl+C to stop)"
echo "========================================================================"
echo ""
echo "Status codes: 0=NO_FIX | 1=GPS_FIX | 2=RTK_FIX 🎯"
echo ""

ros2 topic echo /gps/fix | grep -E "(status:|latitude:|longitude:)" --line-buffered | \
while IFS= read -r line; do
    if [[ $line == *"status:"* ]]; then
        status_line="$line"
        # Check if this is the nested status (has leading spaces)
        if [[ $line == *"  status:"* ]]; then
            STATUS=$(echo "$line" | awk '{print $2}')
            case $STATUS in
                2) echo "🎯 RTK FIX! | $status_line";;
                1) echo "🟡 GPS FIX  | $status_line";;
                0) echo "⚪ NO FIX   | $status_line";;
                *) echo "❓ UNKNOWN | $status_line";;
            esac
        fi
    elif [[ $line == *"latitude:"* ]]; then
        echo "   📍 $line"
    elif [[ $line == *"longitude:"* ]]; then
        echo "   📍 $line"
        echo "---"
    fi
done

