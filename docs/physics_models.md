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

### 3.1 Angle of Incidence & Surface Reflectivity
For extended diffuse surfaces, apparent RCS incorporates Lambertian cosine scattering and material reflectivity $\rho$:
$$\sigma_{\text{apparent}} = \sigma_{\text{nominal}} \cdot \cos(\alpha) \cdot \rho$$
where $\alpha = \arccos(-\hat{\mathbf{u}} \cdot \hat{\mathbf{n}})$ is the incidence angle relative to the surface normal $\hat{\mathbf{n}}$.

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
