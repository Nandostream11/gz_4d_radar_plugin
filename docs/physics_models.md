# 4D Radar Physics & Sensor Non-Idealities

This document details the mathematical formulation, kinematic derivations, and physical scattering models implemented in `radar_physics_core`.

---

## 1. Kinematics & Doppler Velocity (Range-Rate)

### 1.1 Line-of-Sight Range-Rate
For a sensor located at position $\mathbf{p}_s \in \mathbb{R}^3$ with translational velocity $\mathbf{v}_s \in \mathbb{R}^3$ (expressed in the world frame), and a target located at position $\mathbf{p}_t \in \mathbb{R}^3$ with velocity $\mathbf{v}_t \in \mathbb{R}^3$:

The relative line-of-sight (LOS) position vector is:
$$\mathbf{r} = \mathbf{p}_t - \mathbf{p}_s$$

The scalar distance is:
$$r = \|\mathbf{r}\| = \sqrt{\mathbf{r}^T \mathbf{r}}$$

The unit line-of-sight vector pointing from sensor to target is:
$$\hat{\mathbf{u}} = \frac{\mathbf{r}}{\|\mathbf{r}\|}$$

Taking the time derivative of the range $r$:
$$\dot{r} = \frac{d}{dt} \sqrt{\mathbf{r}^T \mathbf{r}} = \frac{\mathbf{r}^T \dot{\mathbf{r}}}{\|\mathbf{r}\|} = \hat{\mathbf{u}} \cdot (\mathbf{v}_t - \mathbf{v}_s)$$

Defining relative velocity $\mathbf{v}_{\text{rel}} = \mathbf{v}_s - \mathbf{v}_t$:
$$\dot{r} = -\mathbf{v}_{\text{rel}} \cdot \hat{\mathbf{u}}$$

For stationary targets ($\mathbf{v}_t = \mathbf{0}$):
$$\dot{r} = -\mathbf{v}_s \cdot \hat{\mathbf{u}}$$

### 1.2 Invariance to Pure Sensor Rotation
If the sensor rotates about its own origin with angular velocity $\boldsymbol{\omega}_s$ without translating ($\mathbf{v}_s = \mathbf{0}$):
$$\dot{\mathbf{p}}_s = \mathbf{0} \implies \dot{\mathbf{r}} = \mathbf{0} \implies \dot{r} = 0$$

Sensor angular velocity about the sensor frame origin does **not** alter the distance to stationary world targets and therefore does not generate radial Doppler velocity.

### 1.3 Lever Arm (Mounting Offset)
The radar is mounted at offset $\mathbf{r}_m$ (from `<mounting_pose>`, rotated into the world
frame) from the origin of the link that carries it. The radar origin therefore moves with
$$\mathbf{v}_s = \mathbf{v}_{\text{link}} + \boldsymbol{\omega}_{\text{link}} \times \mathbf{r}_m$$
which is what enters $\dot r$. With zero offset the rotation term vanishes (Section 1.2).

### 1.4 Moving Targets
For a hit point $\mathbf{p}$ on a moving rigid body (link origin $\mathbf{o}$, linear velocity
$\mathbf{v}_b$, angular velocity $\boldsymbol{\omega}_b$, all from Gazebo's world-frame velocity
components), the target velocity is that of the material point:
$$\mathbf{v}_t = \mathbf{v}_b + \boldsymbol{\omega}_b \times (\mathbf{p} - \mathbf{o})$$
and $\dot r = (\mathbf{v}_t - \mathbf{v}_s)\cdot\hat{\mathbf{u}}$ as in Section 1.1.

---

## 1b. Scene Geometry

Each cycle a ray fan of `<ray_samples_azimuth>` × `<ray_samples_elevation>` directions
spanning the field of view is cast from the radar origin. Each ray returns the nearest
intersection with the world's **collision** shapes (planes, oriented boxes, spheres,
cylinders, triangle meshes accelerated with a bounding-volume hierarchy), together with the
true surface normal at the hit — which feeds the incidence-angle model in Section 3.
Static models are captured once; non-static models have their pose and velocity refreshed
every cycle. The radar's own model is excluded.

---

## 2. FMCW Velocity Ambiguity & Nyquist Doppler Folding

Automotive FMCW chirps have a maximum unambiguous radial velocity $v_{\max}$ determined by the chirp repetition time $T_c$ and carrier wavelength $\lambda$:
$$v_{\max} = \frac{\lambda}{4 T_c}$$

When the true relative radial velocity $|\dot{r}_{\text{true}}| > v_{\max}$, the phase shift across consecutive chirps exceeds $\pi$ radians, causing the detected velocity to wrap into the periodic interval $[-v_{\max}, +v_{\max}]$:

$$\dot{r}_{\text{reported}} = \left( (\dot{r}_{\text{true}} + v_{\max}) \bmod 2v_{\max} \right) - v_{\max}$$

The plugin implements this non-ideality to ensure downstream estimators (e.g. 3-point RANSAC Doppler solvers) encounter realistic velocity ambiguities.

---

## 3. Radar Range Equation & Lambertian Scattering

The received power $P_{\text{rx}}$ from a scatterer of Radar Cross Section (RCS) $\sigma$ at range $R$ follows the radar equation:
$$P_{\text{rx}} = \frac{P_{\text{tx}} G_{\text{tx}} G_{\text{rx}} \lambda^2 \sigma}{(4\pi)^3 R^4}$$

### 3.1 Angle of Incidence, Specularity & Surface Reflectivity
The apparent return strength combines the incidence angle, a per-surface specularity
exponent $n$ and the material reflectivity $\rho$:
$$\sigma_{\text{apparent}} = \sigma_{\text{nominal}} \cdot \cos^{n}(\alpha) \cdot \rho \cdot \left(\frac{R_{\text{ref}}}{\max(R, R_{\text{ref}})}\right)^{k}$$
where $\alpha = \arccos(-\hat{\mathbf{u}} \cdot \hat{\mathbf{n}})$ is the incidence angle relative
to the surface normal $\hat{\mathbf{n}}$, $R_{\text{ref}} = 10$ m and $k$ is
`<power_falloff_exponent>` (4 for the radar equation). In dB this is what the code computes.

- $n = 1$: rough, diffuse (Lambertian) surface — natural terrain still returns at grazing angles.
- $n \gg 1$: smooth surface (calm water, asphalt, glass) — strong only near normal incidence,
  so a downward-tilted radar over such surfaces gets few detections, as the design spec requires.

$\cos\alpha$ is clamped to $[0.02, 1]$ to keep the logarithm finite.

### 3.2 Signal-to-Noise Ratio (SNR) Thresholding
Detections whose received power or apparent RCS falls below `<min_rcs_threshold>` (in $\text{dBm}^2$) are dropped before output.

---

## 4. Measurement Noise & Clutter Models

### 4.1 Gaussian Measurement Jitter
Zero-mean Gaussian noise is applied to each measured dimension:
$$r_{\text{meas}} = r_{\text{true}} + \mathcal{N}(0, \sigma_r^2)$$
$$\theta_{\text{meas}} = \theta_{\text{true}} + \mathcal{N}(0, \sigma_{\theta}^2)$$
$$\phi_{\text{meas}} = \phi_{\text{true}} + \mathcal{N}(0, \sigma_{\phi}^2)$$
$$\dot{r}_{\text{meas}} = \dot{r}_{\text{folded}} + \mathcal{N}(0, \sigma_{\dot{r}}^2)$$

### 4.2 Quantization & Binning
Measurements are quantized to configurable resolution bins:
$$x_{\text{reported}} = \text{round}\left(\frac{x_{\text{meas}}}{\Delta x}\right) \cdot \Delta x$$

### 4.3 Poisson False Detections (Clutter)
Each scan cycle generates $K \sim \text{Poisson}(\lambda_{\text{clutter}})$ false returns randomly distributed across the radar FOV and Doppler space.

### 4.4 Probabilistic Detection Dropouts
Physical targets are dropped with probability $P_{\text{dropout}} \in [0, 1)$ to simulate specular reflection loss and multipath cancellation.
