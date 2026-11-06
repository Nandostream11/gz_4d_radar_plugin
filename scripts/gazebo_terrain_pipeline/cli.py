import argparse
import sys
import os
from .bag_parser import BagParser
from .terrain_reconstructor import TerrainReconstructor
from .mesh_generator import MeshGenerator
from .gazebo_exporter import GazeboExporter

def main():
    parser = argparse.ArgumentParser(
        description="Reconstruct a 3D Gazebo Terrain Mesh (SDF) directly from a ROS 2 MCAP/DB3 bag."
    )
    parser.add_argument(
        "--mcap", "-b", required=True,
        help="Path to the input ROS 2 MCAP / DB3 bag file or directory."
    )
    parser.add_argument(
        "--output-dir", "-o", default="./models/reconstructed_terrain",
        help="Directory to save the generated Gazebo model and meshes."
    )
    parser.add_argument(
        "--model-name", "-m", default="reconstructed_terrain",
        help="Name of the Gazebo model."
    )
    parser.add_argument(
        "--grid-res", "-r", type=float, default=0.5,
        help="Grid resolution in meters per cell (default: 0.5m)."
    )
    parser.add_argument(
        "--min-spatial-step", "-s", type=float, default=0.5,
        help="Minimum distance in meters between consecutive camera keyframes."
    )

    args = parser.parse_args()

    if not os.path.exists(args.mcap):
        print(f"[Error] Bag file not found: {args.mcap}", file=sys.stderr)
        sys.exit(1)

    print(f"[*] Parsing bag: {args.mcap}")
    parser = BagParser(args.mcap)
    keyframes, georef = parser.parse(min_spatial_step_m=args.min_spatial_step)
    print(f"[+] Extracted {len(keyframes)} synchronized camera keyframes.")

    print(f"[*] Reconstructing DEM elevation grid (Resolution: {args.grid_res}m)...")
    reconstructor = TerrainReconstructor(grid_resolution_m=args.grid_res)
    grid_x, grid_y, grid_z, ortho_texture = reconstructor.reconstruct(keyframes)
    print(f"[+] Reconstructed surface grid size: {grid_z.shape[0]}x{grid_z.shape[1]} cells.")
    print(f"    Elevation range: {grid_z.min():.2f}m to {grid_z.max():.2f}m (span: {grid_z.max()-grid_z.min():.2f}m).")

    print(f"[*] Generating 3D Mesh (.obj) and material (.mtl)...")
    mesh_gen = MeshGenerator(args.output_dir, model_name=args.model_name)
    obj_path = mesh_gen.export(grid_x, grid_y, grid_z, ortho_texture)
    print(f"[+] Exported mesh: {obj_path}")

    print(f"[*] Generating Gazebo Sim model.sdf and world.sdf...")
    exporter = GazeboExporter(args.output_dir, model_name=args.model_name)
    sdf_path, world_path = exporter.export()
    print(f"[+] Gazebo model created at: {args.output_dir}")
    print(f"    - Model SDF: {sdf_path}")
    print(f"    - Demo World: {world_path}")
    print("\n[SUCCESS] Terrain reconstruction complete. Ready to load into Gazebo Sim.")

if __name__ == "__main__":
    main()
