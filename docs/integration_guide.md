# Radar Plugin Integration Guide

How to add the 4D imaging radar to your own robot models and worlds in Gazebo Sim and ROS 2.

---

## 1. Adding the Radar to a Robot Model

The plugin is a **model-level** Gazebo system: put the `<plugin>` block directly under
`<model>` (SDF does not allow plugins inside `<link>`; Gazebo ignores them there).
Point it at the link that carries the radar with `<link_name>`, and place the radar on
that link with `<mounting_pose>`.

```xml
<sdf version="1.9">
  <model name="my_vehicle">
    <link name="base_link">
      <!-- inertia, visual, collision -->
    </link>

    <plugin filename="libgazebo_4d_radar_plugin.so"
            name="gazebo_4d_radar_plugin::Gazebo4DRadarPlugin">
      <sensor_name>front_radar</sensor_name>
      <link_name>base_link</link_name>
      <!-- Radar 2 m ahead, 0.5 m up, tilted 10 deg down (x y z roll pitch yaw). -->
      <mounting_pose>2.0 0.0 0.5 0 0.1745 0</mounting_pose>
      <frame_id>radar_front_link</frame_id>
      <scan_topic>/radar/front/scan</scan_topic>
      <pointcloud_topic>/radar/front/points</pointcloud_topic>
      <ground_truth_topic>/radar/front/ground_truth_velocity</ground_truth_topic>
      <cycle_rate_hz>20.0</cycle_rate_hz>
      <max_range_m>150.0</max_range_m>
      <fov_azimuth_rad>2.0944</fov_azimuth_rad>     <!-- 120 deg -->
      <fov_elevation_rad>0.5236</fov_elevation_rad> <!-- 30 deg -->
      <v_max_mps>55.55</v_max_mps>
      <enable_doppler_folding>true</enable_doppler_folding>
    </plugin>
  </model>
</sdf>
```

### Who applies the mounting pose?

The plugin does. Published detections are already expressed in the radar frame
(`frame_id`), and the ground-truth velocity is the velocity of the radar origin,
including the ω × r lever-arm term. If your real radar also stores mounting parameters
internally, configure the downstream consumer so it does **not** rotate the data a
second time.

If you prefer a dedicated radar link (for visuals or TF), give it a fixed joint to the
body, set `<link_name>` to it and leave `<mounting_pose>` at zero.

---

## 2. Multiple Radars

Add one `<plugin>` block per radar, each with its own `<sensor_name>`, `<mounting_pose>`,
`<frame_id>` and topics:

```xml
<plugin filename="libgazebo_4d_radar_plugin.so" name="gazebo_4d_radar_plugin::Gazebo4DRadarPlugin">
  <sensor_name>radar_front</sensor_name>
  <mounting_pose>2.0 0 0.5 0 0 0</mounting_pose>
  <frame_id>radar_front_link</frame_id>
  <scan_topic>/radar/front/scan</scan_topic>
  <pointcloud_topic>/radar/front/points</pointcloud_topic>
  <ground_truth_topic>/radar/front/ground_truth_velocity</ground_truth_topic>
</plugin>

<plugin filename="libgazebo_4d_radar_plugin.so" name="gazebo_4d_radar_plugin::Gazebo4DRadarPlugin">
  <sensor_name>radar_rear</sensor_name>
  <mounting_pose>-2.0 0 0.5 0 0 3.14159</mounting_pose>
  <frame_id>radar_rear_link</frame_id>
  <scan_topic>/radar/rear/scan</scan_topic>
  <pointcloud_topic>/radar/rear/points</pointcloud_topic>
  <ground_truth_topic>/radar/rear/ground_truth_velocity</ground_truth_topic>
</plugin>
```

The radars do not see their own host model (`<ignore_own_model>` defaults to `true`).

---

## 3. Making Objects Look Right to the Radar

The radar sees **collision** geometry, not visuals. Give every object you want detected a
`<collision>` (plane, box, sphere, cylinder or mesh). Tune how strongly it returns with
per-model surface overrides inside the plugin block:

```xml
<surface model="parked_truck">
  <nominal_rcs_dbm2>20.0</nominal_rcs_dbm2>
  <incidence_exponent>2.0</incidence_exponent>  <!-- painted metal: fairly specular -->
  <classification>1</classification>
</surface>
<surface model="lake">
  <reflectivity>0.9</reflectivity>
  <incidence_exponent>8.0</incidence_exponent>  <!-- calm water: almost no return off-normal -->
</surface>
```

`incidence_exponent` = 1 is a rough, diffuse surface (natural terrain); larger values make
the surface more mirror-like, so it returns little energy at oblique incidence.

Moving objects (any non-static model) are tracked every cycle; their returns carry the
velocity of the hit point, including rotation of the object.

---

## 4. Driving a Platform for Tests

The bundled worlds use the gz-sim `VelocityControl` system on a gravity-free platform:

```bash
gz topic -t /model/ego_radar_platform/cmd_vel -m gz.msgs.Twist -p 'linear: {x: 5}'
```

The command is in the platform's body frame.

---

## 5. Visualising in RViz2

`ros2 launch gazebo_4d_radar_plugin radar_sim.launch.py` opens RViz with the fixed frame
set to `radar_link`, so no TF tree is needed. To do it by hand:

1. Set **Fixed Frame** to the radar `frame_id` (or a frame connected to it by TF).
2. Add a **PointCloud2** display on `/radar/points`.
3. Use the **Intensity** colour transformer with channel `doppler` (or `rcs`) to tell
   approaching and receding scatterers apart.

---

## 6. Using the Ego-Velocity Estimator

```bash
ros2 run gazebo_4d_radar_plugin radar_ego_velocity_estimator.py \
  --ros-args -p scan_topic:=/radar/scan -p output_topic:=/radar/ego_velocity
```

Compare `/radar/ego_velocity` with `/radar/ground_truth_velocity` to evaluate a solver.
