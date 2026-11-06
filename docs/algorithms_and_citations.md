# Algorithms & Academic References

The 4D Imaging Radar plugin is designed according to radar physics and ego-motion estimation literature.

---

## 1. Instantaneous Ego-Velocity Estimation (Doppler Solvers)

Doppler imaging radars provide instantaneous linear velocity constraints per detection without requiring multi-frame feature matching or scan registration:

$$\dot{r}_i = -\mathbf{v}_s \cdot \hat{\mathbf{u}}_i$$

Where:
- $\dot{r}_i$ is the measured radial velocity of static scatterer $i$.
- $\hat{\mathbf{u}}_i$ is the unit line-of-sight vector to scatterer $i$.
- $\mathbf{v}_s$ is the unknown 3D sensor translational velocity vector.

Given $N \ge 3$ non-coplanar static detections, $\mathbf{v}_s$ is solvable in closed-form using least-squares or 3-point RANSAC:

$$\begin{bmatrix} \hat{\mathbf{u}}_1^T \\ \hat{\mathbf{u}}_2^T \\ \vdots \\ \hat{\mathbf{u}}_N^T \end{bmatrix} \mathbf{v}_s = -\begin{bmatrix} \dot{r}_1 \\ \dot{r}_2 \\ \vdots \\ \dot{r}_N \end{bmatrix}$$

---

## 2. Key Academic Citations

1. **Kellner et al., 2013**
   - *Instantaneous Ego-Motion Estimation using Doppler Radar*, IEEE International Conference on Intelligent Transportation Systems (ITSC), 2013.
   - Foundation for single-radar 3-point RANSAC Doppler velocity estimation.

2. **Kellner et al., 2014**
   - *Instantaneous Ego-Motion Estimation using Multiple Doppler Radars*, IEEE International Conference on Robotics and Automation (ICRA), 2014.
   - Extends Doppler velocity solvers to multi-radar setups for full 6-DOF linear and angular velocity recovery.

3. **Doer & Trommer, 2020**
   - *An EKF Based Approach to Radar Inertial Odometry*, IEEE International Conference on Multisensor Fusion and Integration (MFI), 2020.
   - Open source implementation: [REVE: Radar Ego-Velocity Estimator](https://github.com/christopherdoer/reve).

4. **Kramer et al., 2020**
   - *Radar-Inertial Ego-Velocity Estimation for Visually Degraded Environments*, IEEE International Conference on Robotics and Automation (ICRA), 2020.
   - Fuses radar Doppler ego-velocity with IMU data.

5. **Nyquist-zone augmentation for folded Doppler**
   - arXiv:2408.05811, Section III-A (referenced in the design handoff for handling Doppler folding in the solver).

6. **ARS548 ROS driver** — arXiv:2404.04589 and the `ars548_ros` package: message structures of a real ARS548 detection stream.

---

## 3. Reference Solver Shipped with This Package

`scripts/radar_ego_velocity_estimator.py` implements the Kellner-style 3-point RANSAC
solver (with least-squares refinement on the inliers) and runs as a ROS 2 node:

```bash
ros2 run gazebo_4d_radar_plugin radar_ego_velocity_estimator.py
# subscribes /radar/scan, publishes geometry_msgs/TwistStamped on /radar/ego_velocity
```

It is also the acceptance test used by `validate_radar_plugin.py` (round-trip
validation against Gazebo ground truth).
