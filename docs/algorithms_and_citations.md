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

4. **Kramer et al., 2022**
   - *Radar-Inertial State Estimation with Offline Calibration*, IEEE Robotics and Automation Letters (RA-L), 2022.
   - Tight coupling of 4D radar Doppler measurements with IMU kinematics.

5. **Nyquist Velocity Ambiguity Modeling**
   - *Doppler Velocity Disambiguation in FMCW Automotive Radar Systems*, arXiv:2408.05811.
