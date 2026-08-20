# Radar Plugin Integration Guide

This guide describes how to integrate the 4D Imaging Radar plugin into custom robot models, worlds, and multi-sensor configurations in Gazebo Sim and ROS 2.

---

## 1. Adding to an SDF Robot Model

Insert the `<plugin>` block inside the desired `<link>` element in your robot's SDF file:

```xml
<sdf version="1.9">
  <model name="my_vehicle">
    <link name="base_link">
      <!-- Robot link inertia, visual, collision -->
    </link>

    <!-- Radar Link -->
    <link name="radar_front_link">
      <pose relative_to="base_link">2.0 0.0 0.5 0 0 0</pose>
      <inertial>
        <mass>0.3</mass>
        <inertia>
          <ixx>0.0003</ixx>
          <iyy>0.0003</iyy>
          <izz>0.0005</izz>
        </inertia>
      </inertial>
      
      <visual name="visual">
        <geometry>
          <box><size>0.10 0.08 0.03</size></box>
        </geometry>
      </visual>

      <!-- 4D Radar System Plugin -->
      <plugin
        filename="libgazebo_4d_radar_plugin.so"
        name="gazebo_4d_radar_plugin::Gazebo4DRadarPlugin">
        <sensor_name>front_radar</sensor_name>
        <frame_id>radar_front_link</frame_id>
        <scan_topic>/radar/front/scan</scan_topic>
        <pointcloud_topic>/radar/front/points</pointcloud_topic>
        <cycle_rate_hz>20.0</cycle_rate_hz>
        <max_range_m>150.0</max_range_m>
        <min_range_m>0.3</min_range_m>
        <fov_azimuth_rad>2.0944</fov_azimuth_rad>   <!-- 120° -->
        <fov_elevation_rad>0.5236</fov_elevation_rad> <!-- 30° -->
        <v_max_mps>50.0</v_max_mps>
        <enable_doppler_folding>true</enable_doppler_folding>
      </plugin>
    </link>

    <!-- Fixed joint to base_link -->
    <joint name="radar_front_joint" type="fixed">
      <parent>base_link</parent>
      <child>radar_front_link</child>
    </joint>
  </model>
</sdf>
```

---

## 2. Multi-Radar Configuration

For 360° coverage (e.g. 4 corner radars or front/rear setup), instantiate multiple plugins with distinct `<sensor_name>`, `<frame_id>`, and topic names:

```xml
<!-- Front Radar -->
<plugin filename="libgazebo_4d_radar_plugin.so" name="gazebo_4d_radar_plugin::Gazebo4DRadarPlugin">
  <sensor_name>radar_front</sensor_name>
  <frame_id>radar_front_link</frame_id>
  <scan_topic>/radar/front/scan</scan_topic>
  <pointcloud_topic>/radar/front/points</pointcloud_topic>
</plugin>

<!-- Rear Radar -->
<plugin filename="libgazebo_4d_radar_plugin.so" name="gazebo_4d_radar_plugin::Gazebo4DRadarPlugin">
  <sensor_name>radar_rear</sensor_name>
  <frame_id>radar_rear_link</frame_id>
  <scan_topic>/radar/rear/scan</scan_topic>
  <pointcloud_topic>/radar/rear/points</pointcloud_topic>
</plugin>
```

---

## 3. Visualizing in RViz2

The plugin publishes `sensor_msgs/msg/PointCloud2` on `/radar/points`. In RViz2:

1. Set **Fixed Frame** to `radar_link` (or your robot's `base_link` / `odom` frame).
2. Add a **PointCloud2** display.
3. Set **Topic** to `/radar/points`.
4. Set **Channel Name** to `doppler` or `rcs`.
5. Set **Color Transformer** to `AxisColor` or `Intensity` to visually distinguish approaching vs receding scatterers.
