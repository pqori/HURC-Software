### Perception module

#### **`zed2i_launch/` ― the sensor gateway**

This package is nothing but a **launch wrapper and a YAML** for Stereolabs’ `zed_camera` node. The YAML (`params/zed2i.yaml`) tells the ZED 2i SDK to run 720 p @ 30 Hz, publish depth in **ULTRA** mode, enable the on-board IMU and visual-inertial odometry, and stream a registered point cloud. The launch script (`launch/zed2i_driver.launch.py`) starts that node and nothing else. As soon as it is up your ROS graph gains:

* `/zed2i/left|right/image_rect_color` – rectified stereo images
* `/zed2i/depth/depth_registered` – per-pixel depth
* `/zed2i/imu/data` – 200 Hz fused IMU
* `/zed2i/odom` – 6-DoF visual-inertial Odometry
* `/zed2i/point_cloud/cloud_registered` – live XYZRGBA cloud

Every other perception component consumes these topics instead of talking to the camera directly, so if you ever swap the ZED for another depth sensor you only touch **this** folder.

---

#### **`gnss_launch/` ― RTK GPS for centimeter-level positioning**

`gnss_launch` manages the **u-blox ZED-F9P GPS receiver** with full RTK (Real-Time Kinematic) support for **1-2cm accuracy**. Unlike a basic launch wrapper, this package provides:

* **GPS driver** – u-blox ROS2 driver configured for RTK rover mode
* **Health monitoring** – Validates satellite count, HDOP, and fix quality
* **RTK setup scripts** – Configure base station + rover for differential GPS
* **Web visualization** – Real-time GPS position on interactive map (via `zed_gps_integration`)

**Quick Start:**
```bash
cd ~/workspaces/rover/src/perception/gnss_launch/
./launch_gps.sh  # Starts driver + health monitor
```

The node publishes `sensor_msgs/NavSatFix` on `/gps/fix` with status indicating fix quality:
- `status: 0` = SINGLE (basic GPS, ~2-5m)
- `status: 2` = RTK_FIX (~1-2cm) ✨

For RTK setup (base station + corrections), see: `gnss_launch/scripts/README.md`

---

#### **`zed_gps_integration/` ― GPS visualization & ZED+GNSS fusion**

This package provides **two independent capabilities** for working with GPS data:

**1. GPS Visualization (Simple - Works Now!)** 🗺️
* Real-time GPS position on interactive web map
* Subscribe to `/gps/fix` and display on OpenStreetMap/Satellite view
* Color-coded by fix quality (RED=no fix, YELLOW=GPS, BLUE=RTK)
* **No ZED camera required** – works with GPS only!

**Quick Visualization:**
```bash
source ~/workspaces/rover/install/setup.bash
ros2 launch zed_gps_integration zed_gnss_fusion.launch.py \
  launch_gnss:=true enable_map:=true
# Open: http://localhost:8000/
```

**2. ZED + GNSS Fusion (Advanced)** 🎯
* Fuses ZED camera VIO with GPS using ZED SDK Fusion API
* Combines best of both: GPS global position + ZED local precision
* Outputs fused odometry with centimeter-level global accuracy
* Requires ZED camera + GPS with good fix

**Architecture:**
```
GPS (/gps/fix) ──┬──► map_server ──► Web visualization
                  │
                  └──► zed_gnss_fusion ──► Fused odometry
                           ▲
                     ZED Camera VIO
```

The `map_server` node bridges ROS topics to file format (polling by JavaScript), while `zed_gnss_fusion` uses ZED SDK's internal fusion module to merge visual-inertial odometry with GPS corrections. See `zed_gps_integration/README.md` for full details.

---

#### **`loc_fusion/` ― anchoring odometry to GPS**

This package hosts the **localisation backbone**. Three standard `robot_localization` nodes are launched in `launch/loc_fusion.launch.py`:

| Node                    | Inputs                             | Output / TF                                 |
| ----------------------- | ---------------------------------- | ------------------------------------------- |
| `ekf_local`             | ZED odom + IMU                     | `/odometry/filtered`, **odom → base\_link** |
| `navsat_transform_node` | `/gnss/fix` + `/odometry/filtered` | `/odom/gps`                                 |
| `ekf_global`            | `/odometry/filtered` + `/odom/gps` | `/odometry/global`, **map → odom**          |

The single YAML (`config/ekf.yaml`) defines both EKF instances, while `navsat_transform.yaml` sets the GPS projection rules (zero altitude, declination, etc.). Because this layer also **includes** the ZED and GNSS launch files, an upstream package only needs to include `loc_fusion` to obtain a rock-solid TF tree:

```
map ─► odom (GPS-corrected) ─► base_link (smooth VIO)
```

Everything downstream—SLAM, Nav2, MoveIt—relies on that tree for consistent geometry.

---

#### **`pointcloud_tools/` ― 2-D products from 3-D data**

SLAM provides a global grid but local planners often need faster, planar information. `pointcloud_tools` offers two pure-Python nodes:

* **`depth_to_scan_node.py`** slices the front half of the ZED cloud, bins points by angle, and publishes a **synthetic LaserScan** on `/scan`. Nav2’s local costmap or RViz’s lidar display can consume it without any custom plugins.
* **`grid_builder_node.py`** transforms an incoming cloud (default `/rtabmap/cloud_map`) into a rolling **OccupancyGrid** centred on the robot (`/local_map`). Cells with z > 0.15 m become **occupied** (100), ground-level cells become **free** (0), unobserved stay −1.

Execution and ROS registration are handled by `setup.py`, and default parameters (FOV, z-slice, grid size) live in `config/pointcloud_params.yaml`. Launching `launch/pointcloud_tools.launch.py` spins up both nodes; you can then feed `/scan` to Nav2’s obstacle layer and `/local_map` to the gap-guidance module.

---

#### **`slam_launch/` ― the one-button perception stack**

`slam_launch` glues everything:

1. **Camera** – includes `zed2i_launch`.
2. **Localisation** – includes `loc_fusion`.
3. **Global mapping** – starts **RTAB-Map** with the tuning file `config/rtabmap_params.yaml`. RGB and depth topics are remapped from the ZED namespace; odometry comes from `/odometry/filtered`.

RTAB-Map outputs a continually-optimised 3-D cloud (`/rtabmap/cloud_map`) and a latched 2-D grid on `/map`. Thus, after

```bash
ros2 launch slam_launch slam.launch.py
```

the ROS graph contains **every** perception artefact required by autonomy:

* TF: `map → odom → base_link`
* Local odom (`/odometry/filtered`) and global odom (`/odometry/global`)
* Global grid `/map` (Nav2 StaticLayer)
* Live local grid `/local_map` + LaserScan `/scan` (Nav2 obstacle layer)
* Raw point cloud for gap steering (`/zed2i/point_cloud/cloud_registered`)

Higher-level folders—`global_nav`, `local_nav`, `mission_control`—only need to **include this single launch description** to inherit a fully-functional, sensor-agnostic perception stack. Swapping hardware (another GNSS, a different depth camera) or changing mapping parameters now affects exactly one sub-package, keeping the rest of the repository isolated and easy to maintain.
