# Gazebo 4D Imaging Radar Plugin

A modular, high-fidelity Gazebo Sim (`gz-sim`) sensor plugin simulating **4D FMCW MIMO Automotive Imaging Radars** (77 GHz / 24 GHz / 60 GHz / 79 GHz). It outputs per-detection range, azimuth, elevation, and direct range-rate (Doppler), publishing to standard ROS 2 interfaces (`radar_msgs/msg/RadarScan`, `sensor_msgs/msg/PointCloud2`).

This plugin is specifically engineered for downstream **instantaneous ego-velocity estimators** (e.g. 3-point RANSAC Doppler solvers such as Kellner et al., REVE, and Radar-Inertial Odometry).

---

## 1. Physics Engine & Math Specifications

### 1.1 Translation-Only Doppler (Range-Rate)
For a static world point at position $\mathbf{p}$ relative to the sensor origin, where the sensor translates with velocity $\mathbf{v}_s$ (world frame):
$$\hat{\mathbf{u}} = \frac{\mathbf{p}}{\|\mathbf{p}\|} \quad (\text{unit line-of-sight})$$
$$\dot{r} = -\mathbf{v}_s \cdot \hat{\mathbf{u}}$$

- **Platform Rotation Invariance**: Sensor rotation about its own origin produces zero distance change to fixed world targets and therefore does **not** contribute to $\dot{r}$.
- **Sign Convention**: Approaching targets have negative range-rate ($\dot{r} = \frac{dr}{dt} < 0$). Exposes `<closing_velocity_positive>` parameter to invert if desired.

### 1.2 Doppler Ambiguity & Nyquist Folding
FMCW radars exhibit velocity ambiguity determined by the chirp repetition interval. Speeds exceeding $v_{\max}$ fold periodically:
$$\dot{r}_{\text{reported}} = ((\dot{r}_{\text{true}} + v_{\max}) \bmod 2v_{\max}) - v_{\max}$$

The plugin models this non-ideality to prevent sim-to-real divergence in RANSAC solvers, with an enable/disable switch `<enable_doppler_folding>`.

### 1.3 Scatterer Energy & RCS Falloff
The apparent received power is modeled following the radar range equation and Lambertian/diffuse cosine scattering:
$$P_{\text{rx}} \propto \frac{\sigma \cdot \cos(\alpha) \cdot \rho}{R^4}$$
where $\alpha = \arccos(-\hat{\mathbf{u}} \cdot \hat{\mathbf{n}})$ is the surface incidence angle and $\rho$ is surface reflectivity. Returns below `<min_rcs_threshold>` are filtered out.

---

## 2. Package Architecture

```
plugin/
├── radar_msgs/                      # Standard ROS 2 radar message package
│   ├── msg/
│   │   ├── RadarReturn.msg         # Single 4D radar return (range, az, el, doppler, amp)
│   │   └── RadarScan.msg           # Array of radar returns with standard Header
│   ├── CMakeLists.txt
│   └── package.xml
│
├── radar_physics_core/              # Modular C++ physics, noise & folding engine
│   ├── include/radar_physics_core/
│   │   ├── radar_types.hpp         # Configuration and detection structures
│   │   └── radar_physics.hpp       # Kinematics, folding, and RCS methods
│   ├── src/
│   │   └── radar_physics.cpp
│   ├── test/
│   │   └── test_radar_physics.cpp  # GTest physics validation suite
│   ├── CMakeLists.txt
│   └── package.xml
│
├── gazebo_4d_radar_plugin/          # Gazebo Sim System Plugin & ROS 2 Bridge
│   ├── include/gazebo_4d_radar_plugin/
│   │   └── Gazebo4DRadarPlugin.hpp
│   ├── src/
│   │   └── Gazebo4DRadarPlugin.cpp
│   ├── models/
│   │   └── generic_4d_radar/       # Generic 4D radar model definition
│   ├── worlds/                     # Demo simulation world with ground clutter
│   ├── launch/                     # ROS 2 launch files
│   ├── rviz/                       # RViz2 Doppler pointcloud visualization
│   ├── CMakeLists.txt
│   └── package.xml
│
└── scripts/
    └── verify_radar_physics.py      # Standalone unittest verification suite
```

---

## 3. Configuration Parameters

| SDF Element | Type | Default | Description |
|---|---|---|---|
| `<sensor_name>` | string | `radar` | Name of radar sensor |
| `<frame_id>` | string | `radar_link` | ROS 2 TF frame ID |
| `<scan_topic>` | string | `/radar/scan` | Standard `radar_msgs/msg/RadarScan` topic |
| `<pointcloud_topic>` | string | `/radar/points` | Standard `sensor_msgs/msg/PointCloud2` topic |
| `<publish_radar_scan>` | bool | `true` | Publish standard RadarScan messages |
| `<publish_pointcloud>` | bool | `true` | Publish standard PointCloud2 messages |
| `<cycle_rate_hz>` | double | `20.0` | Scan rate in Hz (10–50 Hz) |
| `<max_range_m>` | double | `300.0` | Maximum operational range |
| `<min_range_m>` | double | `0.2` | Minimum detection range |
| `<fov_azimuth_rad>` | double | `2.0944` | Azimuth FOV (120°, ±60°) |
| `<fov_elevation_rad>` | double | `0.5236` | Elevation FOV (30°, ±15°) |
| `<v_max_mps>` | double | `50.0` | Maximum unambiguous velocity |
| `<enable_doppler_folding>`| bool | `true` | Enable FMCW velocity folding |
| `<closing_velocity_positive>` | bool | `false` | Sign convention toggle |
| `<range_sigma_m>` | double | `0.05` | Range Gaussian noise standard dev |
| `<azimuth_sigma_rad>` | double | `0.0052` | Azimuth noise (~0.3°) |
| `<elevation_sigma_rad>` | double | `0.0105` | Elevation noise (~0.6°) |
| `<range_rate_sigma_mps>` | double | `0.05` | Doppler noise standard dev (5 cm/s) |
| `<range_resolution_m>` | double | `0.02` | Range quantization bin |
| `<azimuth_resolution_rad>`| double | `0.00175`| Azimuth quantization bin (~0.1°) |
| `<elevation_resolution_rad>`| double | `0.00349`| Elevation quantization bin (~0.2°) |
| `<range_rate_resolution_mps>`| double | `0.01` | Doppler quantization bin (1 cm/s) |
| `<false_detection_rate_per_cycle>` | double | `2.0` | Poisson mean clutter count |
| `<detection_dropout_probability>` | double | `0.03` | Random dropout probability |
| `<min_rcs_threshold>` | double | `-25.0` | Minimum RCS cutoff in dBm² |

---

## 4. Build and Run

### 4.1 Building in a ROS 2 Workspace
```bash
colcon build --symlink-install
source install/setup.bash
```

### 4.2 Running the Physics Test Suite
```bash
# Run GTest suite
colcon test --packages-select radar_physics_core
colcon test-result --verbose

# Run standalone Python verification
python3 scripts/verify_radar_physics.py
```

### 4.3 Launching Demo Simulation in Gazebo Sim & RViz2
```bash
ros2 launch gazebo_4d_radar_plugin radar_sim.launch.py
```

### 4.4 Inspecting Topics
```bash
# Echo generic radar scan
ros2 topic echo /radar/scan

# Check publication rate (20 Hz)
ros2 topic hz /radar/scan
```

---

## 5. References

- Kellner, Barjenbruch, Klappstein, Dickmann, Dietmayer, *Instantaneous Ego-Motion Estimation using Doppler Radar*, ITSC 2013.
- Kellner et al., *Instantaneous Ego-Motion Estimation using Multiple Doppler Radars*, ICRA 2014.
- Doer & Trommer, *An EKF Based Approach to Radar Inertial Odometry*, MFI 2020. Reference: [reve](https://github.com/christopherdoer/reve).
- Nyquist-zone Doppler folding: arXiv:2408.05811 §III-A.

---

## License

This project is licensed under the Apache 2.0 License - see the [LICENSE](LICENSE) file for details.
