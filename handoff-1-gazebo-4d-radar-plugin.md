# Handoff: Gazebo plugin for a 4D automotive imaging radar

## Objective

Build a Gazebo sensor plugin that simulates a 77 GHz MIMO imaging radar and
publishes a **per-cycle detection list** with range, azimuth, elevation and
**range-rate (Doppler)** for each detection, in a ROS 2 message.

The consumer is an ego-velocity estimator that solves for platform velocity from
the Doppler-vs-bearing relationship across static-world detections, typically
via 3-point RANSAC. The plugin must therefore produce data faithful enough that
a solver validated in simulation still works on hardware.

## Why a new plugin

No stock Gazebo sensor produces radar detections with per-detection Doppler.
Lidar plugins give range and bearing but no range-rate, and the range-rate is
the entire point — it is a direct velocity observation whose error does not grow
with time and does not depend on feature tracking.

## Target sensor characteristics

Model against a Continental ARS548-class device:

| Property | Value |
| --- | --- |
| Carrier | 77 GHz (λ ≈ 3.9 mm) |
| Technique | FMCW with digital beamforming across a MIMO array |
| Outputs per detection | range, azimuth, elevation, range-rate, RCS, quality flags |
| Max range | ~300 m |
| Cycle rate | 20 Hz factory default (10 Hz configurable) |
| Mounting in the target application | tilted ~45° downward, so ground clutter dominates returns |

Obtain the interface specification for exact field layouts, resolutions and the
unambiguous range-rate interval. It is distributed under NDA with the sensor
rather than published.

## Physics the plugin must get right

### 1. Range-rate — exact, and the most important output

For a static world point at position `p` relative to the sensor origin, with the
sensor origin translating at velocity `v_s` (world frame):

```
û  = p / |p|                    unit line-of-sight
ṙ  = −v_s · û                   range rate
```

Two things worth stating explicitly because they are commonly got wrong:

- **Platform rotation does not contribute.** Range is a scalar distance to a
  fixed point; rotating the sensor about its own origin does not change it. Only
  translation of the sensor origin appears in `ṙ`. Angular rate matters when the
  consumer converts sensor velocity to body velocity via a lever arm, but that
  is downstream of this plugin.
- **Sign convention must match the real device.** Some radars report closing
  velocity as positive, others report `d(range)/dt`. Getting this backwards
  produces an ego-velocity estimate that is exactly negated, which is obvious;
  getting the azimuth sign backwards instead produces an error only in the
  lateral component, which on a mostly-forward trajectory looks like a small
  persistent bias rather than a failure. Verify both against the interface spec.

### 2. Doppler ambiguity — do not skip this

FMCW radars have an unambiguous velocity interval set by the chirp repetition
interval. Beyond it, range-rate **folds**:

```
ṙ_reported = ((ṙ_true + v_max) mod 2·v_max) − v_max
```

This is the single most important non-ideality to model. If the simulator
reports unfolded range-rate, a RANSAC ego-velocity solver will appear to work
perfectly in simulation and then fail on hardware in a way that is very hard to
diagnose — when Doppler folds, the inlier set splits into two internally
consistent populations at different velocities and the solver picks one, so the
symptom is an intermittent velocity bias, not an error or an outlier spike.

Take `v_max` from the interface spec. Make it a configurable parameter and
provide a switch to disable folding, so the same world can be run both ways to
isolate solver behaviour.

### 3. Detections are scatterers, not surface samples

A ray-cast returns a surface intersection. A radar returns energy from
scatterers, and the mapping between the two is not one-to-one:

- Smooth surfaces at oblique incidence return very little. Rough surfaces return
  diffusely. A downward-tilted radar over natural terrain gets good returns;
  the same radar aimed at calm water or smooth asphalt at grazing incidence gets
  almost nothing.
- Detection **count** varies enormously with scene — from a handful in a sparse
  or specular environment to hundreds in clutter. Since RANSAC ego-velocity
  needs a minimum inlier count to solve at all, reproducing plausible detection
  density under varied terrain is as important as reproducing detection accuracy.
- Model an RCS-like weight per detection and threshold on it, rather than
  emitting every ray hit.

## Suggested architecture

```
1. Cast a ray fan over the sensor's azimuth × elevation FOV at
   the configured angular resolution.
2. For each hit: compute range, azimuth, elevation in the sensor frame.
3. Compute ṙ analytically from the sensor origin's world velocity
   projected onto the LOS. Apply folding.
4. Apply an RCS/return-strength model (incidence angle, surface
   property, 1/R⁴ falloff) and threshold.
5. Add per-detection noise: range, azimuth, elevation, range-rate —
   independent sigmas, azimuth typically worse than range.
6. Inject false detections at a configurable rate, and drop a
   configurable fraction of true detections.
7. Quantise to the device's reported resolutions.
8. Publish in the real driver's message structure at the cycle rate.
```

Step 8 matters: emit the **same message type the real driver publishes**, so the
downstream consumer is bit-identical between sim and hardware and no
conditional code paths are needed.

## Noise and non-ideality parameters to expose

All configurable, all with defaults from the interface spec where it gives them:

- `range_sigma_m`, `azimuth_sigma_rad`, `elevation_sigma_rad`, `range_rate_sigma_mps`
- `range_resolution_m`, `azimuth_resolution_rad`, `elevation_resolution_rad`
- `v_max_mps` and `enable_doppler_folding`
- `false_detection_rate_per_cycle`
- `detection_dropout_probability`
- `min_rcs_threshold`
- `max_range_m`, `fov_azimuth_rad`, `fov_elevation_rad`
- `cycle_rate_hz`
- `mounting_pose` — and be explicit about whether the plugin applies it or the
  consumer does, because the real device also has internal mounting-parameter
  configuration and it is easy to end up rotating twice

## Validation plan

Validate the plugin before trusting anything built on it:

1. **Static platform.** All range-rates should be zero within noise. Any bias
   here is a frame or sign error.
2. **Pure translation at known velocity toward a flat wall.** Range-rate at
   boresight should equal the speed, with the cosine falloff across the beam
   matching `−v·û` analytically. Plot reported `ṙ` against azimuth and confirm
   it traces the expected cosine — this is the same relationship the consumer's
   RANSAC exploits, so it is the most direct test.
3. **Pure rotation about the sensor origin.** Range-rates should stay at zero.
   If they do not, rotation is leaking into the Doppler computation.
4. **Velocity ramp through `v_max`.** Confirm folding occurs at the right speed
   and wraps in the right direction.
5. **Round-trip.** Feed the output to the actual ego-velocity solver and compare
   its estimate against Gazebo ground-truth velocity. This is the real
   acceptance test.

## Performance

At 20 Hz with a wide FOV, a naive dense ray fan is expensive. Consider casting
at reduced angular density and treating each hit as a detection cluster, rather
than casting at the device's full angular resolution. The consumer cares about
detection **distribution and Doppler accuracy**, not about reproducing the exact
beamforming pattern.

## Deliberate fidelity ceiling

Do not attempt to model: the beamforming process itself, sidelobes, multipath,
interference between radars, micro-Doppler from rotating parts, or weather
attenuation. None affect an ego-velocity solver enough to justify the cost. If a
downstream consumer later needs any of these, revisit.

## Reference reading

- Kellner, Barjenbruch, Klappstein, Dickmann, Dietmayer, *Instantaneous
  Ego-Motion Estimation using Doppler Radar*, ITSC 2013 — the origin of the
  Doppler-vs-azimuth RANSAC approach the plugin is feeding.
- Kellner et al., *Instantaneous Ego-Motion Estimation using Multiple Doppler
  Radars*, ICRA 2014 — adds the covariance derivation.
- Doer & Trommer, *An EKF Based Approach to Radar Inertial Odometry*, MFI 2020.
  Reference implementation: https://github.com/christopherdoer/reve
- `ars548_ros` driver and its paper (arXiv:2404.04589) — for the message
  structures and byte-order handling of a real ARS548 stream.
- Nyquist-zone augmentation for folded Doppler: arXiv:2408.05811 §III-A.
