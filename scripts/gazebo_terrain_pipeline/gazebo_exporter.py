import os

class GazeboExporter:
    """
    Creates standard Gazebo Sim (gz-sim) model and world definitions
    referencing the reconstructed 3D mesh.
    """
    def __init__(self, output_dir: str, model_name: str = "reconstructed_terrain"):
        self.output_dir = output_dir
        self.model_name = model_name

    def export(self):
        config_path = os.path.join(self.output_dir, "model.config")
        sdf_path = os.path.join(self.output_dir, "model.sdf")
        world_path = os.path.join(self.output_dir, "world.sdf")

        # 1. model.config
        config_content = f"""<?xml version="1.0"?>
<model>
  <name>{self.model_name}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <author>
    <name>Gazebo Terrain Pipeline</name>
    <email>dev@example.com</email>
  </author>
  <description>3D Terrain Surface Mesh Reconstructed from ROS 2 MCAP Bag</description>
</model>
"""
        with open(config_path, "w", encoding="utf-8") as f:
            f.write(config_content)

        # 2. model.sdf
        sdf_content = f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{self.model_name}">
    <static>true</static>
    <link name="terrain_link">
      <collision name="collision">
        <geometry>
          <mesh>
            <uri>model://{self.model_name}/meshes/terrain.obj</uri>
          </mesh>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>1.0</mu>
              <mu2>1.0</mu2>
            </ode>
          </friction>
        </surface>
      </collision>
      <visual name="visual">
        <geometry>
          <mesh>
            <uri>model://{self.model_name}/meshes/terrain.obj</uri>
          </mesh>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
"""
        with open(sdf_path, "w", encoding="utf-8") as f:
            f.write(sdf_content)

        # 3. world.sdf (Ready to run with 'gz sim world.sdf')
        world_content = f"""<?xml version="1.0"?>
<sdf version="1.9">
  <world name="terrain_world">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 100 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <include>
      <uri>model://{self.model_name}</uri>
      <pose>0 0 0 0 0 0</pose>
    </include>
  </world>
</sdf>
"""
        with open(world_path, "w", encoding="utf-8") as f:
            f.write(world_content)

        return sdf_path, world_path
