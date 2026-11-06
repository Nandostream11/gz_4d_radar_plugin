import os
import sys
import unittest
import shutil
import cv2
import numpy as np
from rosbags.rosbag2 import Writer
from rosbags.typesys import get_typestore, Stores

# Add script directory to sys.path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from gazebo_terrain_pipeline.georeference import GeoReference
from gazebo_terrain_pipeline.bag_parser import BagParser
from gazebo_terrain_pipeline.terrain_reconstructor import TerrainReconstructor
from gazebo_terrain_pipeline.mesh_generator import MeshGenerator
from gazebo_terrain_pipeline.gazebo_exporter import GazeboExporter

class SmokeTestTerrainPipeline(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.test_dir = os.path.join(os.path.dirname(__file__), "test_output")
        cls.bag_dir = os.path.join(cls.test_dir, "sample_flight_bag")
        cls.model_dir = os.path.join(cls.test_dir, "sample_terrain_model")
        
        os.makedirs(cls.test_dir, exist_ok=True)
        if os.path.exists(cls.bag_dir):
            shutil.rmtree(cls.bag_dir)
        if os.path.exists(cls.model_dir):
            shutil.rmtree(cls.model_dir)

        # Generate synthetic ROS 2 bag
        cls._create_synthetic_bag(cls.bag_dir)

    @classmethod
    def _create_synthetic_bag(cls, bag_path: str):
        typestore = get_typestore(Stores.ROS2_HUMBLE)
        
        with Writer(bag_path, version=9) as writer:
            # Register topic connections
            conn_gps = writer.add_connection(
                "/fmu/in/sensorgps",
                "sensor_msgs/msg/NavSatFix",
                typestore=typestore
            )
            conn_imu = writer.add_connection(
                "/imu/data",
                "sensor_msgs/msg/Imu",
                typestore=typestore
            )
            conn_cam = writer.add_connection(
                "/camera/image_rect",
                "sensor_msgs/msg/Image",
                typestore=typestore
            )

            # Generate 15 waypoints along a 100m trajectory
            base_lat, base_lon, base_alt = 37.7749, -122.4194, 50.0 # San Francisco 50m MSL
            t_base_ns = 1700000000000000000 # Timestamp in ns

            for step in range(15):
                t_ns = t_base_ns + step * 200_000_000 # 200ms steps (5 Hz)
                lat = base_lat + (step * 0.0001)      # ~11m per step north
                lon = base_lon + (step * 0.00005)     # ~5m per step east
                alt = base_alt + 5.0 * np.sin(step * 0.5) # undulating altitude

                # 1. NavSatFix GPS
                gps_msg = typestore.types['sensor_msgs/msg/NavSatFix'](
                    header=typestore.types['std_msgs/msg/Header'](
                        stamp=typestore.types['builtin_interfaces/msg/Time'](
                            sec=int(t_ns // 1_000_000_000),
                            nanosec=int(t_ns % 1_000_000_000)
                        ),
                        frame_id="gps_link"
                    ),
                    status=typestore.types['sensor_msgs/msg/NavSatStatus'](status=0, service=1),
                    latitude=lat,
                    longitude=lon,
                    altitude=alt,
                    position_covariance=np.zeros(9, dtype=np.float64),
                    position_covariance_type=0
                )
                writer.write(conn_gps, t_ns, typestore.serialize_cdr(gps_msg, 'sensor_msgs/msg/NavSatFix'))

                # 2. Imu orientation
                imu_msg = typestore.types['sensor_msgs/msg/Imu'](
                    header=typestore.types['std_msgs/msg/Header'](
                        stamp=typestore.types['builtin_interfaces/msg/Time'](
                            sec=int(t_ns // 1_000_000_000),
                            nanosec=int(t_ns % 1_000_000_000)
                        ),
                        frame_id="imu_link"
                    ),
                    orientation=typestore.types['geometry_msgs/msg/Quaternion'](x=0.0, y=0.0, z=0.0, w=1.0),
                    orientation_covariance=np.zeros(9, dtype=np.float64),
                    angular_velocity=typestore.types['geometry_msgs/msg/Vector3'](x=0.0, y=0.0, z=0.0),
                    angular_velocity_covariance=np.zeros(9, dtype=np.float64),
                    linear_acceleration=typestore.types['geometry_msgs/msg/Vector3'](x=0.0, y=0.0, z=9.81),
                    linear_acceleration_covariance=np.zeros(9, dtype=np.float64)
                )
                writer.write(conn_imu, t_ns, typestore.serialize_cdr(imu_msg, 'sensor_msgs/msg/Imu'))

                # 3. Camera Image (Synthetic terrain tile 128x128 BGR)
                synthetic_img = np.zeros((128, 128, 3), dtype=np.uint8)
                synthetic_img[:, :, 0] = np.uint8(30 + step * 5)   # Blue
                synthetic_img[:, :, 1] = np.uint8(100 + step * 8)  # Green (vegetation)
                synthetic_img[:, :, 2] = np.uint8(60 + step * 4)   # Red (dirt)

                cam_msg = typestore.types['sensor_msgs/msg/Image'](
                    header=typestore.types['std_msgs/msg/Header'](
                        stamp=typestore.types['builtin_interfaces/msg/Time'](
                            sec=int(t_ns // 1_000_000_000),
                            nanosec=int(t_ns % 1_000_000_000)
                        ),
                        frame_id="camera_optical_link"
                    ),
                    height=128,
                    width=128,
                    encoding="bgr8",
                    is_bigendian=0,
                    step=128 * 3,
                    data=synthetic_img.reshape(-1)
                )
                writer.write(conn_cam, t_ns, typestore.serialize_cdr(cam_msg, 'sensor_msgs/msg/Image'))

    def test_01_georeference(self):
        georef = GeoReference(37.7749, -122.4194, 50.0)
        enu = georef.lla_to_enu(37.7749, -122.4194, 50.0)
        self.assertAlmostEqual(enu[0], 0.0, delta=1e-3)
        self.assertAlmostEqual(enu[1], 0.0, delta=1e-3)
        self.assertAlmostEqual(enu[2], 0.0, delta=1e-3)

    def test_02_bag_parser(self):
        parser = BagParser(self.bag_dir)
        keyframes, georef = parser.parse(min_spatial_step_m=0.5)
        self.assertGreater(len(keyframes), 5)
        self.assertEqual(keyframes[0]["image"].shape, (128, 128, 3))
        self.assertEqual(len(keyframes[0]["pos_enu"]), 3)

    def test_03_end_to_end_reconstruction(self):
        # 1. Parse
        parser = BagParser(self.bag_dir)
        keyframes, _ = parser.parse(min_spatial_step_m=0.5)

        # 2. Reconstruct DEM
        reconstructor = TerrainReconstructor(grid_resolution_m=1.0)
        grid_x, grid_y, grid_z, ortho_texture = reconstructor.reconstruct(keyframes)
        
        self.assertGreater(grid_z.shape[0], 5)
        self.assertGreater(grid_z.shape[1], 5)
        self.assertEqual(ortho_texture.shape[:2], grid_z.shape)

        # 3. Export Mesh
        mesh_gen = MeshGenerator(self.model_dir, model_name="test_terrain")
        obj_path = mesh_gen.export(grid_x, grid_y, grid_z, ortho_texture)
        self.assertTrue(os.path.exists(obj_path))

        # Check OBJ contents
        with open(obj_path, "r", encoding="utf-8") as f:
            obj_lines = f.readlines()
        v_lines = [l for l in obj_lines if l.startswith("v ")]
        f_lines = [l for l in obj_lines if l.startswith("f ")]
        self.assertGreater(len(v_lines), 20)
        self.assertGreater(len(f_lines), 20)

        # 4. Export Gazebo SDF
        exporter = GazeboExporter(self.model_dir, model_name="test_terrain")
        sdf_path, world_path = exporter.export()
        self.assertTrue(os.path.exists(sdf_path))
        self.assertTrue(os.path.exists(world_path))

        # Validate SDF contains mesh URI
        with open(sdf_path, "r", encoding="utf-8") as f:
            sdf_text = f.read()
        self.assertIn("model://test_terrain/meshes/terrain.obj", sdf_text)
        self.assertIn("<static>true</static>", sdf_text)

    def test_04_cli_execution(self):
        # Run CLI directly
        from gazebo_terrain_pipeline.cli import main as cli_main
        cli_out = os.path.join(self.test_dir, "cli_model")
        
        saved_argv = sys.argv
        try:
            sys.argv = [
                "cli.py",
                "--mcap", self.bag_dir,
                "--output-dir", cli_out,
                "--model-name", "cli_terrain",
                "--grid-res", "1.0"
            ]
            cli_main()
            self.assertTrue(os.path.exists(os.path.join(cli_out, "model.sdf")))
            self.assertTrue(os.path.exists(os.path.join(cli_out, "meshes", "terrain.obj")))
            self.assertTrue(os.path.exists(os.path.join(cli_out, "meshes", "terrain_diffuse.png")))
        finally:
            sys.argv = saved_argv

    @classmethod
    def tearDownClass(cls):
        try:
            if os.path.exists(cls.test_dir):
                shutil.rmtree(cls.test_dir, ignore_errors=True)
        except Exception:
            pass

if __name__ == "__main__":
    unittest.main()
