#!/usr/bin/env python3
"""
standalone_locfusion.py

Purpose
-------
A **2D standalone localization fusion** that:
  1) Opens a ZED camera/SVO, enables positional tracking, and retrieves the
     **OpenGL-ready point cloud** (XYZRGBA) each frame.
  2) Converts the point cloud to NumPy, builds an **interleaved VBO buffer**
     for your own OpenGL renderer (no viewer included here).
  3) Runs a lightweight **2D localization fusion** (EKF) that fuses ZED VIO
     (x, y, yaw) with ZED IMU (gyro + accel) to output a smooth pose in `odom`.
  4) (Optional) Exports a quick **2D occupancy slice** from the point cloud for
     downstream navigation testing.

Notes
-----
- ZED default frame is RIGHT_HANDED_Y_UP (X right, Y up, Z forward).
- This script **converts everything to ROS ENU** for Nav2-style axes:
  x = forward, y = left, z = up; yaw about +Z (CCW).
"""

import argparse
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np
import cv2
import pyzed.sl as sl

try:
    import ogl_viewer.viewer as gl
    HAS_GL_VIEWER = False
except ImportError:
    print("⚠️ OpenGL viewer not available. Install PyOpenGL and ZED samples.")
    HAS_GL_VIEWER = False

def zed_right_handed_y_up_to_ros_enu(points_xyz: np.ndarray) -> np.ndarray:
    """Convert points from ZED RIGHT_HANDED_Y_UP (X right, Y up, Z forward)
    to ROS ENU (X forward, Y left, Z up).

    Mapping:
      ZED -> ROS ENU
      Xr (right)   ->  Yenu (left is negative): Yenu = -Xr
      Yr (up)      ->  Zenu
      Zr (forward) ->  Xenu
    """
    x = points_xyz[:, 0]
    y = points_xyz[:, 1]
    z = points_xyz[:, 2]
    xenu = -z
    yenu = -x
    zenu = y
    return np.stack([xenu, yenu, zenu], axis=-1)


def rot2d(theta: float) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s], [s, c]], dtype=np.float32)


@dataclass
class EKFConfig:
    dt_min: float = 1e-3
    q_acc: float = 1.5      # accel process noise (m/s^2)
    q_gyro: float = 0.1     # gyro process noise (rad/s)
    r_pos: float = 0.05     # ZED VIO pos noise (m)
    r_yaw: float = 0.02     # ZED VIO yaw noise (rad)


class LocFusionEKF2D:
    """State: [x, y, vx, vy, yaw, yaw_rate] (all ENU)"""
    def __init__(self, cfg: EKFConfig):
        self.cfg = cfg
        self.x = np.zeros((6, 1), dtype=np.float32)
        self.P = np.eye(6, dtype=np.float32) * 1e-2

    def predict(self, dt: float, ax_body: float, ay_body: float, gyro_z: float):
        dt = max(dt, self.cfg.dt_min)

        x, y, vx, vy, yaw, yaw_rate = self.x.flatten()

        # Rotate body accel to world frame using current yaw (ENU)
        R = rot2d(yaw)
        acc_world = R @ np.array([[ax_body], [ay_body]], dtype=np.float32)
        ax_w, ay_w = acc_world.flatten()

        # State prediction
        x_pred = x + vx * dt + 0.5 * ax_w * dt * dt
        y_pred = y + vy * dt + 0.5 * ay_w * dt * dt
        vx_pred = vx + ax_w * dt
        vy_pred = vy + ay_w * dt
        yaw_rate_pred = gyro_z  # use gyro as input
        yaw_pred = yaw + yaw_rate_pred * dt

        self.x = np.array([[x_pred], [y_pred], [vx_pred], [vy_pred], [yaw_pred], [yaw_rate_pred]], dtype=np.float32)

        # Linearized F
        F = np.eye(6, dtype=np.float32)
        F[0, 2] = dt
        F[1, 3] = dt
        F[4, 5] = dt

        # Process noise Q
        q = self.cfg
        Q = np.zeros((6, 6), dtype=np.float32)
        Q[0, 0] = (0.5 * q.q_acc * dt * dt) ** 2
        Q[1, 1] = (0.5 * q.q_acc * dt * dt) ** 2
        Q[2, 2] = (q.q_acc * dt) ** 2
        Q[3, 3] = (q.q_acc * dt) ** 2
        Q[4, 4] = (q.q_gyro * dt) ** 2
        Q[5, 5] = (q.q_gyro) ** 2

        self.P = F @ self.P @ F.T + Q

    def update_with_zed_vio(self, x_meas: float, y_meas: float, yaw_meas: float):
        # Measurement model: z = Hx + v, with z=[x,y,yaw]
        H = np.zeros((3, 6), dtype=np.float32)
        H[0, 0] = 1.0  # x
        H[1, 1] = 1.0  # y
        H[2, 4] = 1.0  # yaw

        z = np.array([[x_meas], [y_meas], [yaw_meas]], dtype=np.float32)

        R = np.diag([self.cfg.r_pos ** 2, self.cfg.r_pos ** 2, self.cfg.r_yaw ** 2]).astype(np.float32)
        y_res = z - (H @ self.x)
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y_res
        self.P = (np.eye(6, dtype=np.float32) - K @ H) @ self.P

    def get_pose(self) -> Tuple[float, float, float]:
        x, y, _, _, yaw, _ = self.x.flatten()
        # Wrap yaw to [-pi, pi]
        yaw = (yaw + np.pi) % (2 * np.pi) - np.pi
        return float(x), float(y), float(yaw)



def mat_to_numpy_xyzrgba(pc_mat: sl.Mat) -> Tuple[np.ndarray, np.ndarray]:
    """Return (points Nx3 float32 in meters, colors Nx3 uint8)."""
    data = pc_mat.get_data() 
    xyz = data[..., :3].reshape(-1, 3).copy()
    rgba_f32 = data[..., 3].reshape(-1).copy()
    rgba_u32 = rgba_f32.view(np.uint32)
    r = ((rgba_u32 >> 16) & 255).astype(np.uint8)
    g = ((rgba_u32 >> 8) & 255).astype(np.uint8)
    b = (rgba_u32 & 255).astype(np.uint8)
    rgb = np.stack([r, g, b], axis=-1)

    # mask invalid
    valid = np.isfinite(xyz).all(axis=1)
    return xyz[valid], rgb[valid]


def build_interleaved_vbo(points_xyz: np.ndarray, colors_rgb: np.ndarray) -> np.ndarray:
    """Build an OpenGL-ready **interleaved** float32 buffer [x,y,z,r,g,b] in [0..1]."""
    if len(points_xyz) == 0:
        return np.zeros((0, 6), dtype=np.float32)
    rgb01 = colors_rgb.astype(np.float32) / 255.0
    vbo = np.concatenate([points_xyz.astype(np.float32), rgb01], axis=1)
    return vbo.reshape(-1).astype(np.float32)


def save_ply(path: str, points_xyz: np.ndarray, colors_rgb: Optional[np.ndarray] = None):
    n = points_xyz.shape[0]
    has_color = colors_rgb is not None and colors_rgb.shape[0] == n
    with open(path, 'w') as f:
        f.write('ply\nformat ascii 1.0\n')
        f.write(f'element vertex {n}\n')
        f.write('property float x\nproperty float y\nproperty float z\n')
        if has_color:
            f.write('property uchar red\nproperty uchar green\nproperty uchar blue\n')
        f.write('end_header\n')
        if has_color:
            for (x, y, z), (r, g, b) in zip(points_xyz, colors_rgb):
                f.write(f'{x:.6f} {y:.6f} {z:.6f} {int(r)} {int(g)} {int(b)}\n')
        else:
            for (x, y, z) in points_xyz:
                f.write(f'{x:.6f} {y:.6f} {z:.6f}\n')


def occupancy_slice(points_xyz: np.ndarray, res: float, z_min: float, z_max: float,
                    pad: float = 1.0) -> Tuple[np.ndarray, Tuple[float, float, float, float]]:
    """Make a simple 2D occupancy slice from 3D points. Returns (occ_img, bounds).
    - res: cell size (m)
    - z_min/z_max: keep points within this height band
    - bounds: (xmin, xmax, ymin, ymax)
    """
    if len(points_xyz) == 0:
        return np.zeros((1, 1), dtype=np.uint8), (0, 0, 0, 0)

    z = points_xyz[:, 2]
    sel = (z >= z_min) & (z <= z_max)
    pts = points_xyz[sel][:, :2]  # XY only
    if len(pts) == 0:
        return np.zeros((1, 1), dtype=np.uint8), (0, 0, 0, 0)

    xmin, ymin = pts.min(axis=0) - pad
    xmax, ymax = pts.max(axis=0) + pad
    W = max(1, int(np.ceil((xmax - xmin) / res)))
    H = max(1, int(np.ceil((ymax - ymin) / res)))
    img = np.zeros((H, W), dtype=np.uint8)

    # rasterize
    ix = ((pts[:, 0] - xmin) / res).astype(np.int32)
    iy = ((pts[:, 1] - ymin) / res).astype(np.int32)
    valid = (ix >= 0) & (ix < W) & (iy >= 0) & (iy < H)
    img[iy[valid], ix[valid]] = 255

    # optional morphology to fill gaps
    kernel = np.ones((3, 3), np.uint8)
    img = cv2.dilate(img, kernel, iterations=1)
    return img, (float(xmin), float(xmax), float(ymin), float(ymax))


def main():
    ap = argparse.ArgumentParser(description="ZED point cloud → OpenGL buffer + 2D EKF loc fusion (ENU)")
    ap.add_argument('--svo', type=str, default=None, help='Path to SVO file')
    ap.add_argument('--resolution', type=str, default='HD720', choices=['HD2K', 'HD1080', 'HD720', 'VGA'])
    ap.add_argument('--depth-max', type=float, default=20.0, help='Max depth range (m)')
    ap.add_argument('--frames', type=int, default=0, help='If >0, process this many frames then exit (0 = run forever)')

    ap.add_argument('--save-ply', type=str, help='Write one PLY of the point cloud and exit after frames')
    ap.add_argument('--save-vbo', type=str, help='Write raw float32 interleaved VBO [x y z r g b] to .bin')
    ap.add_argument('--save-occ', type=str, help='Write occupancy PNG (with --z-min/--z-max/--occ-res)')
    ap.add_argument('--occ-res', type=float, default=0.05, help='Occupancy resolution (m)')
    ap.add_argument('--z-min', type=float, default=0.05, help='Min Z (m) for occupancy slice (ENU up)')
    ap.add_argument('--z-max', type=float, default=1.50, help='Max Z (m) for occupancy slice (ENU up)')

    args = ap.parse_args()

    # ---------------- ZED init ----------------
    zed = sl.Camera()
    init_params = sl.InitParameters()
    if args.svo:
        init_params.set_from_svo_file(args.svo)
        init_params.svo_real_time_mode = True
        print(f"Using SVO: {args.svo}")
    res_map = {
        'HD2K': sl.RESOLUTION.HD2K,
        'HD1080': sl.RESOLUTION.HD1080,
        'HD720': sl.RESOLUTION.HD720,
        'VGA': sl.RESOLUTION.VGA,
    }
    init_params.camera_resolution = res_map[args.resolution]
    init_params.depth_mode = sl.DEPTH_MODE.NEURAL
    init_params.coordinate_units = sl.UNIT.METER
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
    init_params.depth_maximum_distance = float(args.depth_max)

    status = zed.open(init_params)
    if status != sl.ERROR_CODE.SUCCESS:
        raise SystemExit(f"Failed to open ZED: {status}")

    # Positional tracking
    tracking_params = sl.PositionalTrackingParameters()
    status = zed.enable_positional_tracking(tracking_params)
    sensors_enabled = (status == sl.ERROR_CODE.SUCCESS)
    print(f"Positional tracking enabled: {sensors_enabled}")

    # Buffers
    runtime_params = sl.RuntimeParameters()
    cam_info = zed.get_camera_information()
    point_cloud_res = cam_info.camera_configuration.resolution

    point_cloud_res = sl.Resolution(min(point_cloud_res.width, 720), min(point_cloud_res.height, 404))

    pc_mat = sl.Mat()
    zed_pose = sl.Pose()
    sensors_data = sl.SensorsData()

    viewer = None
    if HAS_GL_VIEWER:
        viewer = gl.GLViewer()
        viewer.init(cam_info.camera_model, point_cloud_res)
        print("✓ OpenGL 3D viewer initialized")

    # EKF
    ekf = LocFusionEKF2D(EKFConfig())
    last_t = time.time()

    align_ready = False
    yaw0 = 0.0
    R_align = np.eye(2, dtype=np.float32)
    print("\n=== Running (press Ctrl+C to stop) ===")
    frame = 0
    try:
        while True:
            if zed.grab(runtime_params) != sl.ERROR_CODE.SUCCESS:
                if args.svo:
                    print("End of SVO")
                break

            # Retrieve point cloud (CPU) and current camera pose in WORLD frame
            zed.retrieve_measure(pc_mat, sl.MEASURE.XYZRGBA, sl.MEM.CPU, point_cloud_res)
            zed.get_position(zed_pose, sl.REFERENCE_FRAME.WORLD)

            if HAS_GL_VIEWER and viewer and viewer.is_available():
                viewer.updateData(pc_mat, None) 

            # Sensors (IMU)
            zed.get_sensors_data(sensors_data, sl.TIME_REFERENCE.CURRENT)
            imu = sensors_data.get_imu_data()
            ang_vel = imu.get_angular_velocity()  
            lin_acc = imu.get_linear_acceleration()  

            # Convert point cloud to numpy (always ENU for Nav2)
            xyz, rgb = mat_to_numpy_xyzrgba(pc_mat)
            xyz_enu = zed_right_handed_y_up_to_ros_enu(xyz)

            # Build VBO if requested (once)
            if args.save_vbo and frame == 0:
                vbo = build_interleaved_vbo(xyz_enu, rgb)

            # Save PLY (once)
            if args.save_ply and frame == 0:
                save_ply(args.save_ply, xyz_enu, rgb)
                print(f"Saved PLY: {args.save_ply} ({len(xyz_enu)} pts)")

            # Occupancy slice (once)
            if args.save_occ and frame == 0:
                occ, bounds = occupancy_slice(xyz_enu, res=args.occ_res, z_min=args.z_min, z_max=args.z_max)
                cv2.imwrite(args.save_occ, occ)
                print(f"Saved occupancy PNG: {args.save_occ}  bounds={bounds}")

            T_wc = zed_pose.pose_data()
            T_wc_np = np.array(T_wc.m, dtype=np.float32).reshape((4, 4))
            t = T_wc_np[:3, 3]
            R_wc = T_wc_np[:3, :3]

            t_enu = zed_right_handed_y_up_to_ros_enu(t.reshape(1, 3))[0]
            x_meas, y_meas = float(t_enu[0]), float(t_enu[1])

            forward_zed = (R_wc @ np.array([0, 0, -1.0], dtype=np.float32))
            f_enu = zed_right_handed_y_up_to_ros_enu(forward_zed.reshape(1, 3))[0]
            yaw_meas = float(np.arctan2(f_enu[1], f_enu[0])) 
            
            if not align_ready:
                yaw0 = yaw_meas
                if np.cos(yaw0) < 0.0:
                    yaw0 = (yaw0 + np.pi) % (2*np.pi)
                c, s = np.cos(-yaw0), np.sin(-yaw0) 
                R_align = np.array([[c, -s], [s, c]], dtype=np.float32)
                align_ready = True

            x_meas, y_meas = (R_align @ np.array([x_meas, y_meas], dtype=np.float32)).tolist()
            yaw_meas = (yaw_meas - yaw0 + np.pi) % (2*np.pi) - np.pi

            gyro_z = float(ang_vel[2])  
            ax_body = -float(lin_acc[2])  
            ay_body = float(-lin_acc[0]) 

            now = time.time()
            dt = now - last_t
            last_t = now

            ekf.predict(dt, ax_body=ax_body, ay_body=ay_body, gyro_z=gyro_z)
            ekf.update_with_zed_vio(x_meas, y_meas, yaw_meas)
            xf, yf, yawf = ekf.get_pose()

            if frame % 15 == 0:
                print(f"FUSED pose odom (ENU): x={xf: .2f} m, y={yf: .2f} m, yaw={yawf: .2f} rad  | pts={len(xyz_enu)}")

            frame += 1
            if args.frames > 0 and frame >= args.frames:
                break
    
        if args.save_vbo and 'vbo' in locals():
            with open(args.save_vbo, 'wb') as f:
                f.write(vbo.tobytes())
            print(f"Saved VBO buffer: {args.save_vbo}  (float32 {vbo.size} values)")

    except KeyboardInterrupt:
        pass
    finally:
        if HAS_GL_VIEWER and viewer:
            viewer.exit()
        cv2.destroyAllWindows()
        zed.close()
        print("ZED closed.")


if __name__ == '__main__':
    main()
