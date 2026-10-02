# ZED GPS Integration

**Global localization system combining ZED camera Visual-Inertial Odometry (VIO) with GPS for centimeter-accurate positioning.**

---

## 📋 Table of Contents

- [What This Module Does](#what-this-module-does)
- [System Architecture](#system-architecture)
- [Quick Start - GPS Visualization](#quick-start---gps-visualization)
- [Full ZED + GPS Fusion](#full-zed--gps-fusion)
- [Components](#components)
- [Topics](#topics)
- [Parameters](#parameters)
- [Troubleshooting](#troubleshooting)

---

## What This Module Does

### Two Main Functions:

1. **GPS Visualization** (Simple - Works Now!) 🗺️
   - Subscribe to GPS topic (`/gps/fix`)
   - Display real-time position on interactive web map
   - Shows GPS status (NO_FIX → SINGLE → RTK)
   - No ZED camera required

2. **ZED + GPS Fusion** (Advanced - Requires ZED Camera) 🎯
   - Fuses ZED camera VIO with GPS using ZED SDK Fusion API
   - Provides centimeter-accurate global localization
   - Best of both: GPS global position + ZED local precision
   - Outputs fused odometry and geo-poses

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│  INPUT: GPS Receiver                                        │
│  ┌────────────────┐                                         │
│  │ /gps/fix       │  NavSatFix (lat, lon, status, sats)    │
│  └────────────────┘                                         │
└─────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────┐
│  PROCESSING: Two Parallel Paths                             │
│                                                              │
│  Path 1: GPS ONLY (Visualization)                           │
│  ┌──────────────────────────────────────────────┐           │
│  │  map_server.py                               │           │
│  │  • Subscribes to /gps/fix                    │           │
│  │  • Writes to ~/zed_map_data/raw_data.txt     │           │
│  │  • Format: lat,lon,timestamp,status          │           │
│  └──────────────────────────────────────────────┘           │
│                                                              │
│  Path 2: ZED FUSION (Advanced)                              │
│  ┌──────────────────────────────────────────────┐           │
│  │  zed_gnss_fusion.py                          │           │
│  │  • ZED camera provides VIO (visual odometry) │           │
│  │  • Fuses VIO + GPS using ZED SDK             │           │
│  │  • Publishes fused odometry                  │           │
│  │  • Writes to ~/zed_map_data/data.txt         │           │
│  └──────────────────────────────────────────────┘           │
└─────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────┐
│  OUTPUT: Web Map Visualization                              │
│  ┌────────────────────────────────────────────┐             │
│  │  web_map/index.html (Leaflet.js)           │             │
│  │  • Polls data files every 1 second         │             │
│  │  • Displays position on OpenStreetMap      │             │
│  │  • Shows GPS status, satellites, accuracy  │             │
│  │  • Color-coded markers by fix quality      │             │
│  └────────────────────────────────────────────┘             │
│                                                              │
│  Access: http://localhost:8000/                             │
└─────────────────────────────────────────────────────────────┘
```

---

## Quick Start - GPS Visualization

**Just want to see GPS on a map? Start here!**

### Prerequisites
- ✅ GPS publishing to `/gps/fix`
- ❌ ZED camera NOT required

### Launch (3 Commands)

```bash
# Terminal 1: Start GPS (if not already running)
cd ~/workspaces/rover/src/perception/gnss_launch/
./launch_gps.sh

# Terminal 2: Start map server
source /opt/ros/humble/setup.bash
source ~/workspaces/rover/install/setup.bash
ros2 run zed_gps_integration map_server

# Terminal 3: Start web server & open browser
cd ~/zed_map_data
python3 -m http.server 8000 &
firefox http://localhost:8000/
```

### One-Liner Launch

```bash
source /opt/ros/humble/setup.bash && \
source ~/workspaces/rover/install/setup.bash && \
ros2 launch zed_gps_integration zed_gnss_fusion.launch.py \
  launch_gnss:=true enable_map:=true
```

Then open: `http://localhost:8000/`

### What You'll See

![Map Interface](https://via.placeholder.com/800x400/3498db/ffffff?text=Interactive+GPS+Map)

- 🟡 **Yellow marker** at current GPS position
- 📊 **Real-time updates** every second
- 📍 **Status panel** (top-right):
  - Latitude & Longitude
  - Altitude
  - GPS fix status (NO_FIX, SINGLE, DGPS, RTK)
  - Number of satellites
- 🗺️ **Interactive controls**:
  - Zoom in/out
  - Pan/drag
  - Switch between street and satellite view
- 🛤️ **Trail** showing path history

### GPS Status Colors

| Status | Color | Accuracy | Meaning |
|--------|-------|----------|---------|
| **NO_FIX** | 🔴 Red | N/A | No satellites (indoors) |
| **SINGLE** | 🟡 Yellow | ~2-5m | Basic GPS (outdoor) |
| **DGPS** | 🟢 Green | ~1-3m | Differential GPS |
| **RTK** | 🔵 Blue | **~1-2cm** | **RTK FIX (best!)** ✨ |

---

## Full ZED + GPS Fusion

**For advanced users with ZED camera - get centimeter-level localization!**

### Prerequisites
- ✅ ZED camera connected
- ✅ ZED SDK installed
- ✅ GPS publishing to `/gps/fix`
- ✅ Both outdoors with clear sky view

### How It Works

1. **ZED Camera** provides:
   - Visual odometry (tracks camera movement)
   - Inertial measurements (IMU)
   - High-frequency, smooth local position (~90 Hz)

2. **GPS** provides:
   - Absolute global position (lat/lon)
   - Lower frequency (~1 Hz)
   - Can drift or have outliers

3. **ZED SDK Fusion** combines both:
   - Uses GPS to anchor VIO in global coordinates
   - Uses VIO to smooth GPS between updates
   - Detects and rejects GPS outliers
   - Outputs: Fused odometry with global position + VIO precision

### Launch Full System

```bash
source /opt/ros/humble/setup.bash
source ~/workspaces/rover/install/setup.bash

# Launch everything (GPS + ZED + Map)
ros2 launch zed_gps_integration zed_gnss_fusion.launch.py \
  launch_gnss:=true \
  enable_map:=true \
  camera_sn:=0

# OR launch without GPS (if already running)
ros2 launch zed_gps_integration zed_gnss_fusion.launch.py \
  launch_gnss:=false \
  enable_map:=true
```

### Calibration Process

The fusion requires calibration (automatic):

1. **Wait for GPS fix** (check: `ros2 topic echo /gps/fix`)
2. **Move camera**: 
   - Walk forward 10-15 meters
   - Turn 30-45 degrees (left or right)
   - Do this 2-3 times
3. **Watch for message**: `"Calibration complete!"`
4. **Result**: Now publishing fused odometry with cm-level accuracy

### Monitor Fusion Status

```bash
# Watch fusion diagnostics
ros2 topic echo /zed_gnss_fusion/diagnostics

# Monitor fused odometry
ros2 topic echo /zed_gnss_fusion/fused_odom

# Check geographic pose
ros2 topic echo /zed_gnss_fusion/geo_pose

# View calibration status
ros2 topic echo /zed_gnss_fusion/calibration_status
```

---

## Components

### 1. `map_server.py`

**Purpose:** Bridge ROS topics to web-accessible files

**Subscribes to:**
- `/gps/fix` (raw GPS)
- `/zed_gnss_fusion/geo_pose` (fused position)

**Writes to:**
- `~/zed_map_data/raw_data.txt` (raw GPS)
- `~/zed_map_data/data.txt` (fused position)

**Output format:**
```
# raw_data.txt
42.3626,-71.1264,1763158035027,2.245,2.245,3.521,SINGLE

# data.txt  
42.3626,-71.1264,1763158035027
```

### 2. `zed_gnss_fusion.py`

**Purpose:** Main fusion node using ZED SDK

**Inputs:**
- ZED camera frames (internal)
- `/gps/fix` (external GPS)

**Outputs:**
- `~/fused_odom` - Fused odometry (local frame)
- `~/geo_pose` - Fused pose (geographic coords)
- `~/fused_path` - Path history
- `~/calibration_status` - Fusion calibration state
- `~/diagnostics` - System health

### 3. `web_map/index.html`

**Purpose:** Interactive web visualization

**Features:**
- Leaflet.js map with OpenStreetMap/Satellite layers
- JavaScript polls data files every 1 second
- Real-time marker updates
- Status panel with GPS info
- Legend for fix types
- Responsive design

---

## Topics

### Subscribed

| Topic | Type | Description |
|-------|------|-------------|
| `/gps/fix` | `sensor_msgs/NavSatFix` | GPS position from receiver |
| `/zed_gnss_fusion/geo_pose` | `geographic_msgs/GeoPoseStamped` | Fused global pose |

### Published

| Topic | Type | Description |
|-------|------|-------------|
| `/zed_gnss_fusion/fused_odom` | `nav_msgs/Odometry` | Fused local odometry |
| `/zed_gnss_fusion/geo_pose` | `geographic_msgs/GeoPoseStamped` | Fused geographic pose |
| `/zed_gnss_fusion/fused_path` | `nav_msgs/Path` | Path history |
| `/zed_gnss_fusion/calibration_status` | `std_msgs/String` | Calibration state |
| `/zed_gnss_fusion/diagnostics` | `diagnostic_msgs/DiagnosticArray` | System health |

---

## Parameters

### map_server

| Parameter | Default | Description |
|-----------|---------|-------------|
| `map_data_dir` | `~/zed_map_data` | Output directory for web files |
| `raw_gnss_topic` | `/gps/fix` | GPS input topic |
| `fused_topic` | `/zed_gnss_fusion/geo_pose` | Fused pose topic |

### zed_gnss_fusion

| Parameter | Default | Description |
|-----------|---------|-------------|
| `camera_sn` | `0` | ZED serial number (0=auto) |
| `camera_resolution` | `HD1080` | ZED resolution |
| `gnss_topic` | `/gps/fix` | GPS input topic |
| `target_yaw_uncertainty` | `0.1` | Calibration yaw threshold |
| `target_translation_uncertainty` | `0.15` | Calibration position threshold |
| `publish_tf` | `true` | Publish TF transforms |
| `publish_path` | `true` | Publish path history |

### Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `launch_gnss` | `false` | Launch GPS driver |
| `enable_map` | `true` | Enable map server |
| `camera_sn` | `0` | ZED serial number |
| `gnss_topic` | `/gps/fix` | GPS topic |

---

## Troubleshooting

### Map shows (0, 0) position

**Problem:** GPS has no fix

**Solution:**
- Go outside with clear sky view
- Wait 30-60 seconds for satellite lock
- Check: `ros2 topic echo /gps/fix --once`

### Map not updating

**Problem:** map_server not receiving data

**Solution:**
```bash
pkill map_server
source /opt/ros/humble/setup.bash
source ~/workspaces/rover/install/setup.bash
ros2 run zed_gps_integration map_server
```

### ZED fusion fails to calibrate

**Problem:** Not enough movement or no GPS fix

**Solution:**
- Ensure GPS has fix (status: 0 or 2)
- Move camera more (15m forward + 45° turn)
- Check GPS topic: `ros2 topic hz /gps/fix` (should be ~1 Hz)

### Web page won't load

**Problem:** Web server not running

**Solution:**
```bash
cd ~/zed_map_data
python3 -m http.server 8000
```

Then open: `http://localhost:8000/`

### "ZED camera not found" error

**Problem:** Camera disconnected or already in use

**Solution:**
```bash
# Check camera
lsusb | grep -i zed

# Kill conflicting processes
pkill -f zed

# Restart system
```

---

## File Locations

```
~/zed_map_data/
├── index.html          # Web map interface (copied from package)
├── raw_data.txt        # GPS data (updates every second)
└── data.txt            # Fused data (when ZED fusion active)
```

---

## Development

### Build Package

```bash
cd ~/workspaces/rover
colcon build --packages-select zed_gps_integration
source install/setup.bash
```

### Test Map Server Only

```bash
# Start GPS first
cd ~/workspaces/rover/src/perception/gnss_launch/
./launch_gps.sh

# Test map server
ros2 run zed_gps_integration map_server

# Check output
cat ~/zed_map_data/raw_data.txt
```

### Test Full Fusion

```bash
# Use test script
cd ~/workspaces/rover/src/perception/zed_gps_integration/
./test_gps_integration.sh
```

---

## Summary

### GPS Visualization (Simple)
- ✅ Works with GPS only (no ZED needed)
- ✅ Shows position on web map
- ✅ Real-time status updates
- ✅ Ready to use now!

### ZED + GPS Fusion (Advanced)  
- ✅ Requires ZED camera + GPS
- ✅ Centimeter-level accuracy
- ✅ Smooth, high-frequency output
- ✅ Automatic calibration

**Start with visualization, upgrade to fusion when needed!** 🚀

