import cv2
import numpy as np
from scipy.interpolate import griddata
from scipy.spatial.transform import Rotation as R

class TerrainReconstructor:
    """
    Reconstructs a dense 2.5D Digital Elevation Model (DEM) and fused
    orthophoto texture directly from georeferenced camera frames.
    """
    def __init__(self, grid_resolution_m: float = 0.5, hfov_deg: float = 80.0, vfov_deg: float = 60.0):
        self.grid_res = grid_resolution_m
        self.hfov_rad = np.radians(hfov_deg)
        self.vfov_rad = np.radians(vfov_deg)

    def reconstruct(self, keyframes: list):
        if not keyframes:
            raise ValueError("No keyframes provided for reconstruction")

        all_points_x = []
        all_points_y = []
        all_points_z = []
        all_colors = [] # RGB

        # Compute point projections for each keyframe
        for kf in keyframes:
            img = kf["image"]
            h, w = img.shape[:2]
            pos = kf["pos_enu"] # [X_east, Y_north, Z_up]
            quat = kf["quat"]   # [qx, qy, qz, qw]

            rot_mat = R.from_quat(quat).as_matrix()

            # Approximate camera focal lengths
            fx = (w * 0.5) / np.tan(self.hfov_rad * 0.5)
            fy = (h * 0.5) / np.tan(self.vfov_rad * 0.5)
            cx, cy = w * 0.5, h * 0.5

            # Sample points across image grid
            step = 16
            u_grid, v_grid = np.meshgrid(np.arange(0, w, step), np.arange(0, h, step))
            u_flat = u_grid.flatten()
            v_flat = v_grid.flatten()

            # Normalized camera ray directions
            x_cam = (u_flat - cx) / fx
            y_cam = (v_flat - cy) / fy
            z_cam = np.ones_like(x_cam)
            rays_cam = np.stack([x_cam, y_cam, z_cam], axis=1)
            rays_cam /= np.linalg.norm(rays_cam, axis=1, keepdims=True)

            # Transform rays to world ENU frame (optical: Z forward, X right, Y down)
            # Standard ROS optical frame: X=East/Right, Y=Down, Z=Forward
            R_optical_to_flu = np.array([
                [0, 0, 1],
                [-1, 0, 0],
                [0, -1, 0]
            ])
            rays_body = (R_optical_to_flu @ rays_cam.T).T
            rays_world = (rot_mat @ rays_body.T).T

            # Project rays to ground plane / relative depth (assume ground is below camera)
            # Ray: p = pos + t * ray. If ray_z < 0, intersects z=0 at t = -pos_z / ray_z
            valid_mask = rays_world[:, 2] < -0.05
            t_ground = -pos[2] / rays_world[valid_mask, 2]
            t_ground = np.clip(t_ground, 0.5, 200.0)

            pts_world = pos + t_ground[:, None] * rays_world[valid_mask]
            
            # Extract corresponding RGB colors
            colors = img[v_flat[valid_mask], u_flat[valid_mask]]
            
            all_points_x.extend(pts_world[:, 0])
            all_points_y.extend(pts_world[:, 1])
            all_points_z.extend(pts_world[:, 2])
            all_colors.extend(colors)

        pts_x = np.array(all_points_x)
        pts_y = np.array(all_points_y)
        pts_z = np.array(all_points_z)
        colors = np.array(all_colors)

        # Compute bounding grid
        x_min, x_max = pts_x.min(), pts_x.max()
        y_min, y_max = pts_y.min(), pts_y.max()

        cols = int(np.ceil((x_max - x_min) / self.grid_res)) + 1
        rows = int(np.ceil((y_max - y_min) / self.grid_res)) + 1

        cols = max(cols, 10)
        rows = max(rows, 10)

        grid_x, grid_y = np.meshgrid(
            np.linspace(x_min, x_max, cols),
            np.linspace(y_min, y_max, rows)
        )

        # Interpolate elevation grid
        grid_z = griddata(
            (pts_x, pts_y), pts_z, (grid_x, grid_y),
            method='linear', fill_value=np.median(pts_z)
        )

        # Build fused orthophoto texture
        ortho_texture = np.zeros((rows, cols, 3), dtype=np.uint8)
        for i_ch in range(3):
            ch_grid = griddata(
                (pts_x, pts_y), colors[:, i_ch], (grid_x, grid_y),
                method='nearest', fill_value=128
            )
            ortho_texture[:, :, i_ch] = np.clip(ch_grid, 0, 255).astype(np.uint8)

        # Enforce smooth boundary elevation
        nan_mask = np.isnan(grid_z)
        if np.any(nan_mask):
            grid_z_near = griddata((pts_x, pts_y), pts_z, (grid_x, grid_y), method='nearest')
            grid_z[nan_mask] = grid_z_near[nan_mask]

        return grid_x, grid_y, grid_z, ortho_texture
