# GNSS Launch Package

GPS driver and RTK system for ZED-F9P GNSS module with centimeter-level accuracy.

---

## 🎯 What This Package Does

**Launches GPS driver** for u-blox ZED-F9P in rover mode  
**Monitors GPS health** with signal quality and satellite tracking  
**Validates GPS data** for sensor fusion readiness  
**Supports RTK corrections** for 1-2cm positioning accuracy

---

## 🚀 Quick Start

### Standard GPS Mode (Basic Navigation)

```bash
cd ~/workspaces/rover/src/perception/gnss_launch/
./launch_gps.sh
```

**Publishes:**
- `/gps/fix` - GPS position (sensor_msgs/NavSatFix)
- `/gps/velocity` - GPS velocity
- `/gnss/health_status` - Signal quality status
- `/gnss/ready_for_fusion` - Ready for robot_localization

**Accuracy:** 2-5 meters (standard GPS)

---

### RTK Mode (Centimeter Accuracy)

For **1-2cm accuracy**, you need a base station + rover setup.

**📖 Complete RTK Setup Guide:**
```bash
# See detailed instructions:
cat ~/workspaces/rover/src/perception/gnss_launch/scripts/README.md

# Or quick reference:
cat ~/RTK_QUICK_REFERENCE.md
```

**Quick Summary:**
1. Configure base GPS (one-time)
2. Configure rover GPS (one-time)
3. Run base station on laptop with PyGPSClient
4. Run rover GPS + RTCM injector on Jetson

**Result:** Status upgrades from 0 → 2 (RTK FIX) with 1-2cm accuracy! 🎯

---

## 📡 Topics Published

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/gps/fix` | sensor_msgs/NavSatFix | 1 Hz | GPS position (lat/lon/alt) |
| `/gps/velocity` | geometry_msgs/TwistWithCovarianceStamped | 1 Hz | GPS velocity |
| `/gnss/health_status` | std_msgs/String | 0.2 Hz | "excellent", "good", "fair", "poor", "no_fix" |
| `/gnss/signal_quality` | std_msgs/Float32 | 0.2 Hz | Quality score 0.0-1.0 |
| `/gnss/satellite_count` | std_msgs/Int32 | 0.2 Hz | Number of satellites |
| `/gnss/ready_for_fusion` | std_msgs/Bool | 0.2 Hz | Ready for robot_localization fusion |

---

## 🔧 Configuration

**GPS configured for:**
- **Rover mode** (tmode3=0, not base station)
- **Automotive model** (optimized for ground vehicles)
- **1Hz navigation rate**
- **38400 baud serial**
- **RTK-ready** (receives RTCM corrections)

**To change settings:**
```bash
# Edit launch file parameters
nano ~/workspaces/rover/src/perception/gnss_launch/launch/gnss.launch.py

# Rebuild after changes
cd ~/workspaces/rover
colcon build --packages-select gnss_launch
```

---

## 🗺️ GPS Visualization (Live Web Map)

### View Real-Time GPS Position on Interactive Map

**Quick Start:**
```bash
# Terminal 1: Start GPS driver (if not already running)
cd ~/workspaces/rover/src/perception/gnss_launch/
./launch_gps.sh

# Terminal 2: Start map server
source /opt/ros/humble/setup.bash
source ~/workspaces/rover/install/setup.bash
python3 ~/workspaces/rover/install/zed_gps_integration/lib/python3.10/site-packages/zed_gps_integration/map_server.py &

# Terminal 3: Start web server
cd ~/zed_map_data
python3 -m http.server 8000 &

# Open browser
firefox http://localhost:8000/ &
```

**What You'll See:**
- 🟡 Yellow marker at current GPS position
- 📊 Real-time position updates every second
- 📍 Latitude, longitude, altitude display
- 🛰️ Satellite count and fix status
- 🗺️ Interactive map (zoom, pan, switch layers)

**GPS Status on Map:**
- **NO_FIX**: No GPS signal (red, indoors)
- **SINGLE**: Basic GPS (yellow, ~2-5m accuracy)
- **DGPS**: Differential GPS (green, ~1-3m)
- **RTK**: RTK FIX (blue, ~1-2cm accuracy!) 🎯

**One-Liner Launch (Everything Together):**
```bash
source /opt/ros/humble/setup.bash && \
source ~/workspaces/rover/install/setup.bash && \
ros2 launch zed_gps_integration zed_gnss_fusion.launch.py \
  launch_gnss:=true enable_map:=true
```

Then open: `http://localhost:8000/`

---

## 🔍 Checking GPS Status

### Quick Status Check

**ROS2 Method (for integration):**
```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /gps/fix --once | grep "status:"
# status: 0 = Basic GPS
# status: 2 = RTK FIX! (1-2cm accuracy)
```

**Script Method (quick field check):**
```bash
cd ~/workspaces/rover/src/perception/gnss_launch/scripts/
./monitor_rtk_simple.sh
# Shows: [18:05:30] ✅ RTK FIX | Sats: 16 | Lat: 42.362...
```

**📖 For detailed comparison of ROS vs Scripts methods:**
```bash
cat ~/workspaces/rover/src/perception/gnss_launch/scripts/README.md
# See "How to Check GPS Status" section
```

---

## 🧭 Integration with Other Systems

### With robot_localization (Sensor Fusion)

```bash
# Full localization stack (GPS + ZED + EKFs)
ros2 launch loc_fusion loc_fusion.launch.py
```

Fuses GPS with ZED camera odometry for drift-free navigation.

### With ZED GPS Integration (Live Map)

```bash
# Launch GPS + ZED with web visualization
ros2 launch zed_gps_integration zed_gnss_fusion.launch.py
```

View real-time position on interactive map in browser.

### With Nav2 (Autonomous Navigation)

GPS provides:
- Global position for waypoint navigation
- `/gps/fix` converted to map frame via navsat_transform
- Enables autonomous navigation to lat/lon coordinates

---

## 🛠️ Troubleshooting

### No GPS fix indoors
**Normal!** GPS needs outdoor sky view. Health monitor shows "no_signal" or "no_fix".

### Permission denied on /dev/ttyACM0
```bash
sudo usermod -a -G dialout $USER
# Then logout and login
```

### GPS driver fails to start
```bash
# Stop all GPS processes
pkill -9 -f "gnss|ublox"
sleep 3

# Restart
./launch_gps.sh
```

### ROS topics not appearing
```bash
# Restart ROS daemon
source /opt/ros/humble/setup.bash
ros2 daemon stop && sleep 2 && ros2 daemon start

# Check topics
ros2 topic list | grep gps
```

---

## 📁 Package Structure

```
gnss_launch/
├── launch/
│   ├── gnss.launch.py          # Main GPS driver launch
│   ├── gnss_health_monitor.py  # Health monitoring node
│   └── gnss_validator.py       # Signal validation node
├── scripts/
│   ├── configure_base_station.py  # Configure base GPS
│   ├── configure_rover.py         # Configure rover GPS
│   ├── monitor_survey.py          # Monitor base survey
│   ├── rtcm_injector.py           # RTCM correction injector
│   ├── monitor_rtk_simple.sh      # RTK status monitor
│   └── README.md                  # ⭐ RTK setup guide
├── launch_gps.sh               # Quick launch script
├── CMakeLists.txt
├── package.xml
└── README.md                   # This file
```

---

## 📚 Documentation

| Document | Description |
|----------|-------------|
| `scripts/README.md` | **Complete RTK setup guide** (start here for RTK) |
| `/home/rover/RTK_GPS_ANALYSIS_REPORT.md` | Full system analysis and troubleshooting |
| `/home/rover/RTK_QUICK_REFERENCE.md` | Quick reference card for field use |

---

## ⚙️ Requirements

**Hardware:**
- u-blox ZED-F9P GPS receiver
- GPS antenna with clear sky view
- USB connection to `/dev/ttyACM0`

**Software:**
- ROS 2 Humble
- ublox_gps driver (installed at `~/workspaces/ros2-ublox-zedf9p`)
- Python 3.10+

**For RTK:**
- Second ZED-F9P as base station
- Laptop with PyGPSClient
- Network connection between base and rover

---

## 🎯 GPS Status Reference

| Status | Name | Accuracy | Meaning |
|--------|------|----------|---------|
| **-1** | NO_FIX | N/A | No satellites (go outside) |
| **0** | GPS | 2-20m | Basic GPS |
| **1** | DGPS | 1-5m | Differential GPS |
| **2** | **RTK FIX** | **1-2cm** | **✅ Centimeter accuracy!** |
| **3** | RTK FLOAT | 10-50cm | RTK converging |

---

## 🚀 Next Steps

### For Basic GPS:
```bash
./launch_gps.sh
# Done! GPS is publishing to /gps/fix
```

### For RTK (Centimeter Accuracy):
```bash
# Read the RTK setup guide:
cat scripts/README.md

# Follow 3-step process:
# 1. Configure GPS units (one-time)
# 2. Start base station (laptop)
# 3. Start rover + RTCM injector (Jetson)
```

### For Sensor Fusion:
```bash
# Combine GPS with ZED camera odometry
ros2 launch loc_fusion loc_fusion.launch.py
```

---

**Questions?** See `scripts/README.md` for detailed RTK instructions or `/home/rover/RTK_GPS_ANALYSIS_REPORT.md` for comprehensive documentation.

**Ready for centimeter-level accuracy?** Follow the RTK guide in `scripts/README.md`! 🎯
