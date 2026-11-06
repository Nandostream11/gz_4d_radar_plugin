import os
import cv2
import numpy as np

class MeshGenerator:
    """
    Generates a 3D Wavefront (.obj) mesh, material (.mtl) descriptor,
    and texture map from the reconstructed 2.5D DEM surface.
    """
    def __init__(self, output_dir: str, model_name: str = "reconstructed_terrain"):
        self.output_dir = output_dir
        self.model_name = model_name
        self.mesh_dir = os.path.join(output_dir, "meshes")
        os.makedirs(self.mesh_dir, exist_ok=True)

    def export(self, grid_x: np.ndarray, grid_y: np.ndarray, grid_z: np.ndarray, ortho_texture: np.ndarray):
        rows, cols = grid_z.shape
        obj_path = os.path.join(self.mesh_dir, "terrain.obj")
        mtl_path = os.path.join(self.mesh_dir, "terrain.mtl")
        tex_path = os.path.join(self.mesh_dir, "terrain_diffuse.png")

        # Save texture image
        cv2.imwrite(tex_path, ortho_texture)

        # Write Material File (.mtl)
        with open(mtl_path, "w", encoding="utf-8") as f_mtl:
            f_mtl.write("# Wavefront Material File for Gazebo Terrain\n")
            f_mtl.write("newmtl terrain_material\n")
            f_mtl.write("Ka 1.0 1.0 1.0\n")
            f_mtl.write("Kd 1.0 1.0 1.0\n")
            f_mtl.write("Ks 0.05 0.05 0.05\n")
            f_mtl.write("d 1.0\n")
            f_mtl.write("illum 2\n")
            f_mtl.write("map_Kd terrain_diffuse.png\n")

        # Write Mesh File (.obj)
        with open(obj_path, "w", encoding="utf-8") as f_obj:
            f_obj.write("# Gazebo 3D Terrain Surface Mesh\n")
            f_obj.write("mtllib terrain.mtl\n")
            f_obj.write(f"o {self.model_name}\n\n")

            # 1. Vertices (X, Y, Z in meters)
            for r in range(rows):
                for c in range(cols):
                    f_obj.write(f"v {grid_x[r, c]:.4f} {grid_y[r, c]:.4f} {grid_z[r, c]:.4f}\n")

            # 2. UV Texture Coordinates
            for r in range(rows):
                for c in range(cols):
                    u = c / float(max(1, cols - 1))
                    v = 1.0 - (r / float(max(1, rows - 1))) # Invert V for OpenGL/Gazebo
                    f_obj.write(f"vt {u:.6f} {v:.6f}\n")

            # 3. Upward Surface Normal
            f_obj.write("\nvn 0.0 0.0 1.0\n\n")
            f_obj.write("usemtl terrain_material\n")
            f_obj.write("s 1\n")

            # 4. Triangulated Quad Grid Faces
            for r in range(rows - 1):
                for c in range(cols - 1):
                    v1 = r * cols + c + 1
                    v2 = (r + 1) * cols + c + 1
                    v3 = (r + 1) * cols + (c + 1) + 1
                    v4 = r * cols + (c + 1) + 1

                    # Two triangles per grid quad
                    f_obj.write(f"f {v1}/{v1}/1 {v2}/{v2}/1 {v3}/{v3}/1\n")
                    f_obj.write(f"f {v1}/{v1}/1 {v3}/{v3}/1 {v4}/{v4}/1\n")

        return obj_path
