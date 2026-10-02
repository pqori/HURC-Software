# GPS RTK Setup - Simple Guide

**Hardware:** 2× u-blox ZED-F9P GPS Units  
**Result:** Centimeter-level RTK positioning (1-2cm accuracy)

---

## 🎯 Quick Start (Complete Setup in 3 Steps)

### Step 1: Configure GPS Units (One-Time Setup)

**On Laptop (Base Station GPS):**
```bash
python3 configure_base_station.py /dev/cu.usbmodem1201
# ✅ Sets TMODE to 1 (Survey-In)
# ✅ Enables RTCM messages
# ✅ Saves to flash (persists forever)
```

**On Jetson (Rover GPS):**
```bash
cd ~/workspaces/rover/src/perception/gnss_launch/scripts/
python3 configure_rover.py /dev/ttyACM0
# ✅ Sets TMODE to 0 (Rover mode)
# ✅ Enables RTCM input
# ✅ Saves to flash (persists forever)
```

**⚠️ Do this ONCE per GPS unit - configuration is permanent!**

---

### Step 2: Start Base Station (Every Field Session)

**On Laptop - Terminal 1:**
```bash
# Install PyGPSClient (first time only)
pip3 install pygpsclient pyserial

# Launch PyGPSClient
pygpsclient
```

**In PyGPSClient GUI:**
1. **Connect:** Select port `/dev/cu.usbmodem1201`, baud `38400`
2. **Click:** USB/UART button (green USB icon)
3. **Enable:** Socket Server / NTRIP Caster checkbox
4. **Set Mode:** `SOCKET SERVER`
5. **Set Port:** `50012`
6. **Enable:** RTCM protocol checkbox
7. **Take GPS outside** with clear sky view
8. **Wait 5-10 minutes** for survey to complete

**Monitor Survey Progress (Optional):**
```bash
# On Laptop - Terminal 2 (separate terminal)
python3 monitor_survey.py /dev/cu.usbmodem1201 38400 15

# Shows live progress:
# Duration: 03:45 / 05:00 [███████░░] 75.0%
# Accuracy: 1.245m / 1.0m target
# When both targets met → Survey complete!
```

**Get Laptop IP Address:**
```bash
ifconfig | grep "inet " | grep -v 127.0.0.1
# Example: 10.250.80.45
# Write this down - you'll need it for rover setup
```

---

### Step 3: Start Rover (Every Field Session)

**On Jetson - Terminal 1:**
```bash
cd ~/workspaces/rover/src/perception/gnss_launch/
./launch_gps.sh

# Wait for: "U-Blox configured successfully"
```

**On Jetson - Terminal 2:**
```bash
cd ~/workspaces/rover/src/perception/gnss_launch/scripts/
python3 rtcm_injector.py <LAPTOP_IP> 50012 /dev/ttyACM0 38400

# Replace <LAPTOP_IP> with actual IP (e.g., 10.250.80.45)

# Should show:
# ✅ INJECTING RTCM CORRECTIONS
# [16:34:26] 📊 3519 bytes | 81 RTCM msgs | 575.3 B/s
```

**On Jetson - Terminal 3 (Monitor Status):**
```bash
# Check current GPS status
source /opt/ros/humble/setup.bash
ros2 topic echo /gps/fix --once | grep "status:"

# Status values:
#   status: 0  = Basic GPS (waiting for RTK)
#   status: 1  = DGPS (good)
#   status: 2  = RTK FIX! ✅ (1-2cm accuracy achieved!)
```

**Or use continuous monitor:**
```bash
cd ~/workspaces/rover/src/perception/gnss_launch/scripts/
./monitor_rtk_simple.sh
```

---

## 🔍 How to Check GPS Status

### Method 1: Using ROS2 (Recommended for Integration)

**These commands work with the ROS2 system and can be used in your own nodes:**

```bash
# Source ROS2 (required for all ROS commands)
source /opt/ros/humble/setup.bash

# Quick status check
ros2 topic echo /gps/fix --once | grep "status:"
# Output: status: 0  (Basic GPS)
# Output: status: 2  (RTK FIX! ✅)

# Full GPS data
ros2 topic echo /gps/fix --once

# Watch status live (updates continuously)
watch -n 1 'ros2 topic echo /gps/fix --once 2>/dev/null | grep "status:"'

# Check GPS data rate
ros2 topic hz /gps/fix
# Should show: average rate: 1.000 (1 Hz)

# List all GPS topics
ros2 topic list | grep gps

# Check all ROS nodes
ros2 node list
```

**What you'll see:**
```bash
# Basic GPS (waiting for RTK)
status:
  status: 0
latitude: 42.3626778
longitude: -71.1264578
position_covariance: [11.17, ...]  # ~11m accuracy

# RTK FIX achieved! 🎉
status:
  status: 2
latitude: 42.3626778
longitude: -71.1264578
position_covariance: [0.01, ...]  # ~1cm accuracy!
```

---

### Method 2: Using Helper Scripts (Quick Field Checks)

**These scripts provide formatted output without needing to source ROS:**

```bash
cd ~/workspaces/rover/src/perception/gnss_launch/scripts/

# Quick RTK status monitor (prettified output)
./monitor_rtk_simple.sh
# Shows: [18:05:30] ✅ RTK FIX | Sats: 16 | Lat: 42.362... | Lon: -71.126...

# Monitor base station survey (run on laptop)
python3 monitor_survey.py /dev/cu.usbmodem1201 38400 15
# Shows: Duration: 03:45 / 05:00 [███████░░] 75.0%
#        Accuracy: 1.245m / 1.0m target
```

---

### Method Comparison

| Feature | ROS2 Commands | Helper Scripts |
|---------|---------------|----------------|
| **Use Case** | Integration, custom nodes, debugging | Quick field checks, monitoring |
| **Output Format** | ROS message format (detailed) | Human-readable (prettified) |
| **Setup Required** | `source /opt/ros/humble/setup.bash` | None (run directly) |
| **Real-time Updates** | `watch` or `ros2 topic echo` | Built-in live updates |
| **Accessible From** | ROS2 nodes, terminal | Terminal only |
| **Best For** | Development, integration, automation | Field operations, quick checks |

---

### When to Use Each Method

**Use ROS2 Commands when:**
- ✅ Writing a ROS2 node that needs GPS data
- ✅ Integrating with robot_localization or Nav2
- ✅ Debugging ROS2 topic connectivity
- ✅ Checking message types and rates
- ✅ Need raw, unformatted data

**Use Helper Scripts when:**
- ✅ Quick status check in the field
- ✅ Monitoring survey progress on laptop
- ✅ Don't want to source ROS workspaces
- ✅ Need pretty, formatted output
- ✅ Demonstrating to others

---

### Example: Monitoring RTK Status

**ROS2 Method:**
```bash
source /opt/ros/humble/setup.bash

# Continuous monitoring
while true; do
  echo "=== $(date +%H:%M:%S) ==="
  ros2 topic echo /gps/fix --once 2>/dev/null | grep -E "status:|latitude:|covariance\[0\]" | head -4
  echo ""
  sleep 2
done
```

**Script Method:**
```bash
cd ~/workspaces/rover/src/perception/gnss_launch/scripts/
./monitor_rtk_simple.sh
# Just runs - no sourcing needed!
```

**Both methods show the same data, just formatted differently.**

---

## ⏱️ Expected Timeline

| Time | Event | What Happens |
|------|-------|--------------|
| **0:00** | Start base GPS outside | GPS getting satellite lock |
| **0:30** | Base has GPS fix | Survey-In begins automatically |
| **5:00** | Survey duration complete | Still measuring accuracy |
| **8:00** | Accuracy < 1.0m | Survey complete! TMODE 1→2 |
| **8:30** | Start rover + RTCM | Rover gets basic GPS fix |
| **10:00** | Rover receives corrections | Status changes 0→2 (RTK FIX!) |

**Total: ~10-15 minutes from start to RTK FIX** 🎯

---

## 📊 Status Reference

### GPS Fix Status

| Value | Name | Accuracy | Meaning |
|-------|------|----------|---------|
| **-1** | NO_FIX | N/A | No satellites (go outside) |
| **0** | GPS | 2-20m | Basic GPS, waiting for corrections |
| **1** | DGPS | 1-5m | Differential GPS, good quality |
| **2** | **RTK FIX** | **1-2cm** | **✅ Success! Centimeter accuracy** |
| **3** | RTK FLOAT | 10-50cm | Almost RTK, still converging |

### Base Station TMODE

| Value | Name | Status |
|-------|------|--------|
| **0** | Disabled | Rover mode (mobile) |
| **1** | Survey-In | Measuring position (5-10 min) |
| **2** | Fixed | Survey complete, ready for RTK! |

---

## 💻 Using GPS Data in Your ROS2 Nodes

### Python Example

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

class GPSSubscriber(Node):
    def __init__(self):
        super().__init__('gps_subscriber')
        self.subscription = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.gps_callback,
            10)
        
    def gps_callback(self, msg):
        status = msg.status.status
        lat = msg.latitude
        lon = msg.longitude
        
        if status == 2:
            self.get_logger().info(f'RTK FIX! Position: {lat:.7f}, {lon:.7f}')
        elif status == 0:
            self.get_logger().info(f'Basic GPS: {lat:.7f}, {lon:.7f}')
        else:
            self.get_logger().info(f'Status {status}: {lat:.7f}, {lon:.7f}')

def main():
    rclpy.init()
    node = GPSSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### C++ Example

```cpp
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

class GPSSubscriber : public rclcpp::Node {
public:
    GPSSubscriber() : Node("gps_subscriber") {
        subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/gps/fix", 10,
            std::bind(&GPSSubscriber::gps_callback, this, std::placeholders::_1));
    }

private:
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        auto status = msg->status.status;
        auto lat = msg->latitude;
        auto lon = msg->longitude;
        
        if (status == 2) {
            RCLCPP_INFO(this->get_logger(), "RTK FIX! Position: %.7f, %.7f", lat, lon);
        }
    }
    
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GPSSubscriber>());
    rclcpp::shutdown();
    return 0;
}
```

### Launch File Example

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Your GPS node
        Node(
            package='your_package',
            executable='your_gps_node',
            name='gps_processor',
            parameters=[{
                'gps_topic': '/gps/fix',
                'rtk_threshold': 2  # Only use when status == 2
            }]
        )
    ])
```

---

## 🗺️ Visualizing GPS on Live Map

### See Your Rover's Position in Real-Time

Once GPS is running and publishing to `/gps/fix`, you can visualize the rover's position on an interactive web map.

**Quick Start:**
```bash
# 1. Make sure GPS is running (Terminal 1)
cd ~/workspaces/rover/src/perception/gnss_launch/
./launch_gps.sh

# 2. Start map server (Terminal 2)
source /opt/ros/humble/setup.bash
source ~/workspaces/rover/install/setup.bash
ros2 run zed_gps_integration map_server

# 3. Start web server (Terminal 3)
cd ~/zed_map_data
python3 -m http.server 8000

# 4. Open in browser
firefox http://localhost:8000/
```

**What You See on the Map:**
- 🟡 **Yellow marker**: Current GPS position
- 📊 **Real-time updates**: Position updates every second
- 📍 **Status panel**: Shows lat/lon, altitude, satellites, fix status
- 🗺️ **Interactive map**: Zoom, pan, switch between street/satellite view
- 🛤️ **Trail**: Path history showing where you've traveled

**GPS Fix Status Colors:**
- **Red**: NO_FIX (no satellites, indoors)
- **Yellow**: SINGLE (basic GPS, ~2-5m accuracy)
- **Green**: DGPS (differential GPS, ~1-3m)
- **Blue**: RTK (RTK FIX, ~1-2cm accuracy!) 🎯

**Example - Watch RTK Upgrade:**
```bash
# Monitor GPS status while map is running
watch -n 1 'cat ~/zed_map_data/raw_data.txt'

# You'll see status change:
# NO_FIX → SINGLE → RTK
# Accuracy improves: 5m → 1cm!
```

**Files Location:**
- Map HTML: `~/zed_map_data/index.html`
- GPS data: `~/zed_map_data/raw_data.txt` (updates every second)
- Map server: `zed_gps_integration` package

---

## 🔧 Scripts in This Directory

| Script | Purpose | When to Use |
|--------|---------|-------------|
| `configure_base_station.py` | Configure GPS for base station | Once per GPS unit |
| `configure_rover.py` | Configure GPS for rover | Once per GPS unit |
| `monitor_survey.py` | Monitor base survey progress | On laptop during survey |
| `rtcm_injector.py` | Stream corrections to rover | On Jetson, always running |
| `monitor_rtk_simple.sh` | Quick RTK status check | On Jetson for debugging |

---

## 🐛 Troubleshooting

### Problem: Rover stuck at status 0

**Cause:** Base station survey not complete yet

**Solution:**
```bash
# On laptop, check TMODE:
# In PyGPSClient → UBX Config → CFG_TMODE_MODE
# Should be 2 (Fixed), not 1 (Survey-In)
# If still 1, wait longer (needs ~8-12 minutes)
```

---

### Problem: RTCM injector shows 0 messages

**Cause:** Base station not streaming RTCM yet

**Check:**
1. PyGPSClient Socket Server is enabled
2. Port is set to 50012
3. RTCM protocol checkbox is checked
4. Base has GPS fix (TMODE must be 1 or 2)

**Test connection:**
```bash
# On Jetson
nc -zv <LAPTOP_IP> 50012
# Should show: Connection succeeded
```

---

### Problem: GPS driver won't start

**Error:** `Failed to poll MonVER` or `Failed to read GNSS config`

**Solution:**
```bash
# On Jetson - stop everything and restart clean
pkill -9 -f "gnss|ublox|rtcm"
sleep 3

# Restart GPS driver
cd ~/workspaces/rover/src/perception/gnss_launch/
./launch_gps.sh

# Wait 10 seconds, then restart RTCM injector
cd scripts/
python3 rtcm_injector.py <LAPTOP_IP> 50012 /dev/ttyACM0 38400
```

---

### Problem: ROS topics not appearing

**Solution:**
```bash
# Restart ROS daemon
source /opt/ros/humble/setup.bash
ros2 daemon stop
sleep 2
ros2 daemon start

# Check topics
ros2 topic list | grep gps
```

---

### Problem: Network connection refused

**Cause:** Firewall blocking or wrong IP

**Solution:**
```bash
# On laptop, verify IP:
ifconfig | grep "inet " | grep -v 127.0.0.1

# From Jetson, test connection:
ping <LAPTOP_IP>
nc -zv <LAPTOP_IP> 50012

# On Mac: System Preferences → Security & Privacy → Firewall
# Allow incoming connections for Python/PyGPSClient
```

---

### Problem: Survey never completes (stuck at high accuracy)

**Cause:** Poor satellite visibility or moving antenna

**Solution:**
1. Ensure antenna has **completely clear sky view**
2. Check antenna is **completely stationary** (don't touch!)
3. Wait longer (some environments take 15-20 minutes)
4. Check satellite count in PyGPSClient (need 8+ satellites)
5. Try different location with better sky visibility

---

## 🎯 Success Criteria

**You know RTK is working when:**

✅ **Base Station:**
- PyGPSClient shows TMODE = 2 (Fixed)
- Socket Server shows "1 client connected"
- RTCM messages visible in console

✅ **Rover:**
- `rtcm_injector.py` shows message counts > 0
- `ros2 topic echo /gps/fix` shows `status: 2`
- Position covariance < 0.01 (very high precision)

✅ **Real-World Test:**
- Place rover GPS antenna at a marked spot
- Move it 10 meters away
- Return to marked spot
- GPS coordinates should match within **1-2 centimeters!** 🎉

---

## 📝 Quick Commands Reference

### Laptop (Base Station)
```bash
# Find GPS port
ls /dev/cu.usbmodem*

# Configure base (once)
python3 configure_base_station.py /dev/cu.usbmodem1201

# Get IP address
ifconfig | grep "inet " | grep -v 127.0.0.1

# Monitor survey progress
python3 monitor_survey.py /dev/cu.usbmodem1201 38400 15
```

### Jetson (Rover)
```bash
# Configure rover GPS (once)
cd ~/workspaces/rover/src/perception/gnss_launch/scripts/
python3 configure_rover.py /dev/ttyACM0

# Start GPS driver
cd ~/workspaces/rover/src/perception/gnss_launch/
./launch_gps.sh

# Start RTCM injector (replace <IP> with laptop IP)
cd scripts/
python3 rtcm_injector.py 10.250.80.45 50012 /dev/ttyACM0 38400

# Check GPS status
source /opt/ros/humble/setup.bash
ros2 topic echo /gps/fix --once | grep "status:"

# Continuous monitoring
./monitor_rtk_simple.sh
```

### Debug Commands
```bash
# Jetson: Kill all GPS processes
pkill -9 -f "gnss|ublox|rtcm"

# Jetson: Restart ROS daemon
source /opt/ros/humble/setup.bash
ros2 daemon stop && sleep 2 && ros2 daemon start

# Jetson: Check GPS is connected
ls -l /dev/ttyACM0

# Jetson: Test network to base
nc -zv <LAPTOP_IP> 50012

# Jetson: Check running processes
ps aux | grep -E "ublox|rtcm"
```

---

## 📚 Additional Documentation

- **Full Analysis Report:** `/home/rover/RTK_GPS_ANALYSIS_REPORT.md`
- **Quick Reference Card:** `/home/rover/RTK_QUICK_REFERENCE.md`
- **Integration with ZED:** `/home/rover/workspaces/rover/src/perception/zed_gps_integration/`
- **Sensor Fusion:** `/home/rover/workspaces/rover/src/perception/loc_fusion/`

---

## 💡 Pro Tips

1. **Label Your Hardware:** Put tape on each GPS marking "BASE" or "ROVER"

2. **Save Base IP:** Write laptop IP on a sticky note for field use

3. **Test Network First:** Before going to field, verify `nc -zv <IP> 50012` works

4. **Survey Patience:** First survey takes time, but TMODE config persists forever

5. **Keep Base Stationary:** Even small movements reset survey accuracy

6. **Check Satellite Count:** Need 8+ satellites for good RTK (12+ ideal)

7. **Base Distance:** Keep base within 10km of rover for best accuracy

8. **Battery Life:** Base station can run on laptop battery for hours

---

## 🎉 What You Get with RTK FIX

**Before RTK (Standard GPS):**
- Accuracy: 2-5 meters
- Suitable for: General navigation, approximate locations

**After RTK (With Corrections):**
- Accuracy: **1-2 centimeters**
- Suitable for: Precision agriculture, autonomous docking, lane following, surveying

**Real-World Example:**
- Park rover at exact spot
- Drive around obstacle course
- Return to start
- Rover will stop within **1cm of original position!**

---

**Questions? Issues? Check the troubleshooting section above or refer to the full analysis report.**

**Ready to get centimeter-level accuracy? Follow the 3 steps above!** 🚀
