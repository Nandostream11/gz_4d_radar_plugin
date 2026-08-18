#include "radar_physics_core/radar_physics.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace radar_physics_core {

RadarPhysicsEngine::RadarPhysicsEngine(const RadarConfig& config)
    : config_(config) {
  uint32_t default_seed = static_cast<uint32_t>(
      std::chrono::system_clock::now().time_since_epoch().count());
  seed(default_seed);
}

void RadarPhysicsEngine::set_config(const RadarConfig& config) {
  config_ = config;
}

void RadarPhysicsEngine::seed(uint32_t seed_val) {
  rng_.seed(seed_val);
}

double RadarPhysicsEngine::compute_true_range_rate(
    const double sensor_linear_vel_world[3],
    const double unit_los_world[3],
    bool closing_positive) {
  // Line of sight velocity: v_rel_los = v_sensor . u_los
  // When closing toward a target, range decreases: dr/dt = - v_sensor . u_los
  double dot = sensor_linear_vel_world[0] * unit_los_world[0] +
               sensor_linear_vel_world[1] * unit_los_world[1] +
               sensor_linear_vel_world[2] * unit_los_world[2];

  if (closing_positive) {
    return dot;
  }
  return -dot;
}

double RadarPhysicsEngine::fold_doppler(double v_true, double v_max) {
  if (v_max <= 1e-6) {
    return v_true;
  }
  double period = 2.0 * v_max;
  // Proper mathematical modulo for negative and positive floats
  double val = std::fmod(v_true + v_max, period);
  if (val < 0.0) {
    val += period;
  }
  return val - v_max;
}

double RadarPhysicsEngine::quantize(double value, double resolution) {
  if (resolution <= 1e-9) {
    return value;
  }
  return std::round(value / resolution) * resolution;
}

double RadarPhysicsEngine::compute_apparent_rcs(
    double nominal_rcs_dbm2,
    double range_m,
    const double unit_los_world[3],
    const double normal_world[3],
    double surface_reflectivity,
    double power_falloff_exp) {
  // Dot product of unit LOS and surface normal for incidence angle
  // LOS is directed from sensor to target, normal points outward from surface
  double cos_inc = -(unit_los_world[0] * normal_world[0] +
                     unit_los_world[1] * normal_world[1] +
                     unit_los_world[2] * normal_world[2]);
  cos_inc = std::clamp(cos_inc, 0.02, 1.0);

  double refl = std::clamp(surface_reflectivity, 0.01, 1.0);
  double r_norm = std::max(1.0, range_m / 10.0);

  // Apparent RCS in dBm^2 including 1/R^falloff and diffuse cosine scattering
  double rcs_apparent = nominal_rcs_dbm2 +
                        10.0 * std::log10(cos_inc) +
                        10.0 * std::log10(refl) -
                        10.0 * power_falloff_exp * std::log10(r_norm);

  return rcs_apparent;
}

std::optional<RadarDetection> RadarPhysicsEngine::process_ray_hit(
    const double sensor_pos_world[3],
    const double sensor_rot_world_to_sensor[9],
    const double sensor_linear_vel_world[3],
    const RawRayHit& raw_hit,
    uint16_t measurement_id) {
  // 1. Compute relative position vector in world frame: p_w = hit - sensor_pos
  double p_w[3] = {
      raw_hit.hit_point_world[0] - sensor_pos_world[0],
      raw_hit.hit_point_world[1] - sensor_pos_world[1],
      raw_hit.hit_point_world[2] - sensor_pos_world[2]};

  double range_true = std::sqrt(p_w[0] * p_w[0] + p_w[1] * p_w[1] + p_w[2] * p_w[2]);

  if (range_true < config_.min_range_m || range_true > config_.max_range_m) {
    return std::nullopt;
  }

  // 2. Unit Line of Sight in world frame
  double unit_los_w[3] = {
      p_w[0] / range_true,
      p_w[1] / range_true,
      p_w[2] / range_true};

  // 3. Transform relative vector to sensor frame: p_s = R * p_w
  // R is 3x3 row-major rotation matrix from world to sensor frame
  double p_s[3] = {
      sensor_rot_world_to_sensor[0] * p_w[0] + sensor_rot_world_to_sensor[1] * p_w[1] + sensor_rot_world_to_sensor[2] * p_w[2],
      sensor_rot_world_to_sensor[3] * p_w[0] + sensor_rot_world_to_sensor[4] * p_w[1] + sensor_rot_world_to_sensor[5] * p_w[2],
      sensor_rot_world_to_sensor[6] * p_w[0] + sensor_rot_world_to_sensor[7] * p_w[1] + sensor_rot_world_to_sensor[8] * p_w[2]};

  // 4. Compute true azimuth and elevation angles in sensor frame
  // Sensor frame standard: +x forward (boresight), +y left, +z up
  double r_xy = std::sqrt(p_s[0] * p_s[0] + p_s[1] * p_s[1]);
  if (r_xy < 1e-4) {
    r_xy = 1e-4;
  }

  double az_true = std::atan2(p_s[1], p_s[0]);
  double el_true = std::atan2(p_s[2], r_xy);

  // Check FOV boundaries
  if (az_true < config_.fov_azimuth_min_rad || az_true > config_.fov_azimuth_max_rad) {
    return std::nullopt;
  }
  if (el_true < config_.fov_elevation_min_rad || el_true > config_.fov_elevation_max_rad) {
    return std::nullopt;
  }

  // 5. Stochastic detection dropout check
  if (config_.detection_dropout_probability > 0.0) {
    if (uniform_dist_(rng_) < config_.detection_dropout_probability) {
      return std::nullopt;
    }
  }

  // 6. RCS computation & thresholding
  double app_rcs = compute_apparent_rcs(
      raw_hit.nominal_rcs_dbm2,
      range_true,
      unit_los_w,
      raw_hit.hit_normal_world,
      raw_hit.surface_reflectivity,
      config_.power_falloff_exponent);

  if (app_rcs < config_.min_rcs_threshold_dbm2) {
    return std::nullopt;
  }

  // 7. True Doppler range-rate (strictly translation of sensor origin)
  double v_true = compute_true_range_rate(
      sensor_linear_vel_world,
      unit_los_w,
      config_.closing_velocity_positive);

  // 8. Doppler folding / ambiguity
  double v_reported = config_.enable_doppler_folding ? fold_doppler(v_true, config_.v_max_mps) : v_true;

  // 9. Gaussian measurement noise
  double noisy_range = range_true + norm_dist_(rng_) * config_.range_sigma_m;
  double noisy_az = az_true + norm_dist_(rng_) * config_.azimuth_sigma_rad;
  double noisy_el = el_true + norm_dist_(rng_) * config_.elevation_sigma_rad;
  double noisy_vr = v_reported + norm_dist_(rng_) * config_.range_rate_sigma_mps;

  // 10. Quantization to sensor reporting resolution
  double q_range = quantize(noisy_range, config_.range_resolution_m);
  double q_az = quantize(noisy_az, config_.azimuth_resolution_rad);
  double q_el = quantize(noisy_el, config_.elevation_resolution_rad);
  double q_vr = quantize(noisy_vr, config_.range_rate_resolution_mps);

  q_range = std::max(config_.min_range_m, q_range);

  // Build detection
  RadarDetection det;
  det.range_m = q_range;
  det.azimuth_rad = q_az;
  det.elevation_rad = q_el;
  det.range_rate_mps = q_vr;

  det.range_std_m = config_.range_sigma_m;
  det.azimuth_std_rad = config_.azimuth_sigma_rad;
  det.elevation_std_rad = config_.elevation_sigma_rad;
  det.range_rate_std_mps = config_.range_rate_sigma_mps;

  // Derived Cartesian coordinates in sensor frame
  det.x_m = q_range * std::cos(q_el) * std::cos(q_az);
  det.y_m = q_range * std::cos(q_el) * std::sin(q_az);
  det.z_m = q_range * std::sin(q_el);

  det.rcs_dbm2 = static_cast<int8_t>(std::clamp(std::round(app_rcs), -50.0, 50.0));
  det.measurement_id = measurement_id;
  det.positive_predictive_value = static_cast<uint8_t>(std::clamp(
      std::round(95.0 + norm_dist_(rng_) * 3.0), 50.0, 100.0));
  det.invalid_flags = 0;
  det.classification = raw_hit.classification;

  return det;
}

std::vector<RadarDetection> RadarPhysicsEngine::generate_false_detections(
    uint16_t start_measurement_id) {
  std::vector<RadarDetection> false_dets;
  if (config_.false_detection_rate_per_cycle <= 0.0) {
    return false_dets;
  }

  std::poisson_distribution<int> poisson_dist(config_.false_detection_rate_per_cycle);
  int count = poisson_dist(rng_);

  for (int i = 0; i < count; ++i) {
    double az = config_.fov_azimuth_min_rad +
                uniform_dist_(rng_) * (config_.fov_azimuth_max_rad - config_.fov_azimuth_min_rad);
    double el = config_.fov_elevation_min_rad +
                uniform_dist_(rng_) * (config_.fov_elevation_max_rad - config_.fov_elevation_min_rad);
    double range = config_.min_range_m +
                   uniform_dist_(rng_) * (config_.max_range_m * 0.5 - config_.min_range_m);

    double vr = -config_.v_max_mps + uniform_dist_(rng_) * (2.0 * config_.v_max_mps);

    RadarDetection det;
    det.range_m = quantize(range, config_.range_resolution_m);
    det.azimuth_rad = quantize(az, config_.azimuth_resolution_rad);
    det.elevation_rad = quantize(el, config_.elevation_resolution_rad);
    det.range_rate_mps = quantize(vr, config_.range_rate_resolution_mps);

    det.range_std_m = config_.range_sigma_m * 2.0;
    det.azimuth_std_rad = config_.azimuth_sigma_rad * 2.0;
    det.elevation_std_rad = config_.elevation_sigma_rad * 2.0;
    det.range_rate_std_mps = config_.range_rate_sigma_mps * 2.0;

    det.x_m = det.range_m * std::cos(det.elevation_rad) * std::cos(det.azimuth_rad);
    det.y_m = det.range_m * std::cos(det.elevation_rad) * std::sin(det.azimuth_rad);
    det.z_m = det.range_m * std::sin(det.elevation_rad);

    det.rcs_dbm2 = static_cast<int8_t>(std::round(-20.0 + uniform_dist_(rng_) * 10.0));
    det.measurement_id = static_cast<uint16_t>(start_measurement_id + i);
    det.positive_predictive_value = static_cast<uint8_t>(30 + uniform_dist_(rng_) * 25);
    det.invalid_flags = 0;
    det.classification = 0;

    false_dets.push_back(det);
  }

  return false_dets;
}

}  // namespace radar_physics_core
