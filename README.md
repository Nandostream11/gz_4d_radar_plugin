# Gazebo 4D Imaging Radar Plugin

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble%20%7C%20Iron%20%7C%20Jazzy%20%7C%20Rolling-blue.svg)](https://docs.ros.org/)
[![Gazebo Sim](https://img.shields.io/badge/Gazebo%20Sim-Harmonic%20%7C%20Fortress%20%7C%20Garden-orange.svg)](https://gazebosim.org/)
[![License](https://img.shields.io/badge/License-Apache%202.0-green.svg)](LICENSE)

A Gazebo Sim (`gz-sim`) sensor system plugin that simulates **4D FMCW MIMO Automotive Imaging Radars** (77 GHz / 24 GHz / 60 GHz / 79 GHz). It generates realistic point-level radar returns with 3D Cartesian coordinates, range, azimuth, elevation, Doppler radial velocity, and RCS, published over standard ROS 2 interfaces.

Designed for testing and validating downstream **radar ego-velocity estimators** (e.g. 3-point Doppler RANSAC solvers), Radar-Inertial Odometry (RIO), and multi-sensor perception pipelines.

---

## Features

- **Kinematic Doppler Accuracy**: Exact line-of-sight range-rate ($\dot{r} = -\mathbf{v}_s \cdot \hat{\mathbf{u}}$) driven solely by relative translation; sensor rotations about its own origin produce zero radial velocity.
- **FMCW Doppler Folding**: Models Nyquist velocity ambiguity wrapping periodically across $[-v_{\max}, +v_{\max}]$.
- **Radar Range Equation**: Incorporates $1/R^4$ power falloff, Lambertian cosine angle-of-incidence attenuation, and material reflectivity.
- **Sensor Non-Idealities**: Configurable Gaussian measurement noise, quantization bins, Poisson clutter/false detections, and detection dropouts.
- **Standard ROS 2 Interfaces**: Publishes structured `radar_msgs/msg/RadarScan` and standard `sensor_msgs/msg/PointCloud2`.

---

## Supported Environments

| Software | Supported Versions |
|---|---|
| **ROS 2** | Humble, Iron, Jazzy, Rolling |
| **Gazebo Sim** | Gz Harmonic (`gz-sim8`), Gz Fortress (`gz-sim6`), Gz Garden (`gz-sim7`) |
| **OS** | Ubuntu 22.04 / 24.04 (Linux / WSL2) |

---

## Quick Start

### 1. Build the Workspace
Clone into your ROS 2 workspace `src/` directory and build:

```bash
cd ~/ros2_ws/src
git clone https://github.com/Nandostream11/gz_4d_radar_plugin.git
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 2. Run the Demo Simulation
Launch Gazebo Sim with a sample test world and RViz2 visualization:

```bash
ros2 launch gazebo_4d_radar_plugin radar_sim.launch.py
```

### 3. Run Unit Tests
```bash
colcon test --packages-select radar_physics_core
colcon test-result --verbose
```

---

## SDF Integration

To attach the radar to any robot link in Gazebo, insert the `<plugin>` block:

```xml
<link name="radar_link">
  <!-- Sensor visual and collision definitions -->
  
  <plugin
    filename="libgazebo_4d_radar_plugin.so"
    name="gazebo_4d_radar_plugin::Gazebo4DRadarPlugin">
    <sensor_name>radar</sensor_name>
    <frame_id>radar_link</frame_id>
    <scan_topic>/radar/scan</scan_topic>
    <pointcloud_topic>/radar/points</pointcloud_topic>
    <cycle_rate_hz>20.0</cycle_rate_hz>
    <max_range_m>150.0</max_range_m>
    <min_range_m>0.3</min_range_m>
    <fov_azimuth_rad>2.0944</fov_azimuth_rad>     <!-- 120 deg -->
    <fov_elevation_rad>0.5236</fov_elevation_rad>  <!-- 30 deg -->
    <v_max_mps>50.0</v_max_mps>
    <enable_doppler_folding>true</enable_doppler_folding>
  </plugin>
</link>
```

---

## ROS 2 Interfaces

### Published Topics

| Topic | Type | Description |
|---|---|---|
| `/radar/scan` | `radar_msgs/msg/RadarScan` | Per-detection range, azimuth, elevation, Doppler, amplitude |
| `/radar/points` | `sensor_msgs/msg/PointCloud2` | 3D points with `x, y, z, doppler, range, azimuth, elevation, rcs` fields |

---

## Configuration Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `<sensor_name>` | string | `radar` | Sensor node identifier |
| `<frame_id>` | string | `radar_link` | TF coordinate frame |
| `<scan_topic>` | string | `/radar/scan` | Topic for standard radar scan messages |
| `<pointcloud_topic>` | string | `/radar/points` | Topic for PointCloud2 messages |
| `<cycle_rate_hz>` | double | `20.0` | Update frequency (10–50 Hz) |
| `<max_range_m>` | double | `300.0` | Maximum operational detection range (m) |
| `<min_range_m>` | double | `0.2` | Minimum detection range (m) |
| `<fov_azimuth_rad>` | double | `2.0944` | Horizontal field-of-view (rad) |
| `<fov_elevation_rad>` | double | `0.5236` | Vertical field-of-view (rad) |
| `<v_max_mps>` | double | `50.0` | Maximum unambiguous Doppler velocity ($v_{\max}$) |
| `<enable_doppler_folding>`| bool | `true` | Enable FMCW Nyquist velocity folding |
| `<closing_velocity_positive>` | bool | `false` | Invert Doppler sign convention ($\dot{r} < 0$ approaching) |
| `<range_sigma_m>` | double | `0.05` | Range Gaussian noise standard deviation (m) |
| `<azimuth_sigma_rad>` | double | `0.0052` | Azimuth noise standard deviation (rad) |
| `<elevation_sigma_rad>` | double | `0.0105` | Elevation noise standard deviation (rad) |
| `<range_rate_sigma_mps>` | double | `0.05` | Doppler noise standard deviation (m/s) |
| `<range_resolution_m>` | double | `0.02` | Range quantization resolution (m) |
| `<azimuth_resolution_rad>`| double | `0.00175` | Azimuth quantization resolution (rad) |
| `<elevation_resolution_rad>`| double | `0.00349` | Elevation quantization resolution (rad) |
| `<range_rate_resolution_mps>`| double | `0.01` | Doppler quantization resolution (m/s) |
| `<false_detection_rate_per_cycle>` | double | `2.0` | Mean Poisson false/clutter detections per cycle |
| `<detection_dropout_probability>` | double | `0.03` | Probability of target dropout $[0, 1)$ |
| `<min_rcs_threshold>` | double | `-25.0` | Minimum RCS detection threshold ($\text{dBm}^2$) |

---

## Documentation & Wiki

Detailed technical documentation and derivations are available in the [`docs/`](docs/) directory:

- [Physics Models & Sensor Non-Idealities](docs/physics_models.md): Complete mathematical derivations of kinematics, Doppler folding, and RCS falloff.
- [Integration Guide](docs/integration_guide.md): Multi-radar setups, coordinate transforms, and RViz visualization.
- [Algorithms & References](docs/algorithms_and_citations.md): Doppler velocity solvers (3-point RANSAC) and academic citations.

---

## License

This project is licensed under the Apache 2.0 License - see the [LICENSE](LICENSE) file for details.
