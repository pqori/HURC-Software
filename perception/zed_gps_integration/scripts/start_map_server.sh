#!/bin/bash
# Start web server for ZED map visualization

MAP_DIR=~/zed_map_data
PORT=8000

echo "========================================="
echo "  ZED GNSS Map Server"
echo "========================================="
echo ""
echo "Starting HTTP server on port $PORT..."
echo "Map directory: $MAP_DIR"
echo ""
echo "Open browser to: http://localhost:$PORT/"
echo ""
echo "Press Ctrl+C to stop"
echo ""

cd $MAP_DIR
python3 -m http.server $PORT

