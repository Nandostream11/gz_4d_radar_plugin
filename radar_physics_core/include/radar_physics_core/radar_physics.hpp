#ifndef RADAR_PHYSICS_CORE_RADAR_PHYSICS_HPP_
#define RADAR_PHYSICS_CORE_RADAR_PHYSICS_HPP_

#include <cmath>
#include <memory>
#include <optional>
#include <random>
#include <vector>

#include "radar_physics_core/radar_types.hpp"

namespace radar_physics_core {

/**
 * @brief Core physics and signal engine for 4D FMCW MIMO Radar.
 * Computes exact Doppler range-rate, folding, antenna/scatterer physics,
 * RCS attenuation, non-idealities and stochastic noise.
 */
class RadarPhysicsEngine {
 public:
  explicit RadarPhysicsEngine(const RadarConfig& config = RadarConfig{});
  ~RadarPhysicsEngine() = default;

  /**
   * @brief Update engine configuration.
   */
  void set_config(const RadarConfig& config);
  const RadarConfig& get_config() const { return config_; }

  /**
   * @brief Re-seed random number generator.
   */
  void seed(uint32_t seed_val);

  /**
   * @brief Analytically compute true range-rate without noise or folding.
   * @param sensor_linear_vel_world [vx, vy, vz] translation of sensor origin in world frame.
   * @param unit_los_world [ux, uy, uz] unit line of sight from sensor to target in world frame.
   * @param closing_positive Whether approaching velocity is positive.
   * @return True line-of-sight range-rate (m/s).
   */
  static double compute_true_range_rate(
      const double sensor_linear_vel_world[3],
      const double unit_los_world[3],
      bool closing_positive = false);

  /**
   * @brief Apply Doppler Nyquist folding / wrap-around due to FMCW chirp periodicity.
   * Reported velocity wraps across [-v_max, +v_max].
   * @param v_true True relative range-rate (m/s).
   * @param v_max Maximum unambiguous velocity interval boundary (m/s).
   * @return Folded range-rate (m/s).
   */
  static double fold_doppler(double v_true, double v_max);

  /**
   * @brief Quantize a continuous physical measurement to device reporting resolution.
   * @param value Continuous input value.
   * @param resolution Bin width.
   * @return Quantized value.
   */
  static double quantize(double value, double resolution);

  /**
   * @brief Compute received signal strength (RCS) considering 1/R^4 falloff,
   * surface incidence angle, and material reflectivity.
   * @param nominal_rcs_dbm2 Object base RCS in dBm^2.
   * @param range_m Distance to target in meters.
   * @param unit_los_world Unit vector from sensor to target.
   * @param normal_world Surface normal at hit point.
   * @param surface_reflectivity Material reflection coefficient [0..1].
   * @param power_falloff_exp Falloff exponent (e.g. 4.0 for radar).
   * @return Received apparent RCS in dBm^2.
   */
  static double compute_apparent_rcs(
      double nominal_rcs_dbm2,
      double range_m,
      const double unit_los_world[3],
      const double normal_world[3],
      double surface_reflectivity = 1.0,
      double power_falloff_exp = 4.0);

  /**
   * @brief Process a raw raycast hit into a full 4D radar detection.
   * Applies geometric transforms, Doppler calculation, folding, RCS thresholding,
   * Gaussian noise, dropouts, and resolution quantization.
   *
   * @param sensor_pos_world Sensor origin position [x, y, z] in world frame.
   * @param sensor_rot_world_to_sensor 3x3 rotation matrix from world to sensor frame (row-major).
   * @param sensor_linear_vel_world Sensor origin translation velocity [vx, vy, vz] in world frame.
   * @param raw_hit Surface hit information.
   * @param measurement_id Sequential measurement index.
   * @return Valid RadarDetection or std::nullopt if dropped (FOV/RCS/dropout).
   */
  std::optional<RadarDetection> process_ray_hit(
      const double sensor_pos_world[3],
      const double sensor_rot_world_to_sensor[9],
      const double sensor_linear_vel_world[3],
      const RawRayHit& raw_hit,
      uint16_t measurement_id);

  /**
   * @brief Generate false / clutter detections for the current cycle.
   * Injects clutter uniformly distributed across FOV and range with realistic noise.
   * @param start_measurement_id Starting measurement ID for false detections.
   * @return Vector of synthetic clutter detections.
   */
  std::vector<RadarDetection> generate_false_detections(uint16_t start_measurement_id);

 private:
  RadarConfig config_;
  std::mt19937 rng_;
  std::normal_distribution<double> norm_dist_{0.0, 1.0};
  std::uniform_real_distribution<double> uniform_dist_{0.0, 1.0};
};

}  // namespace radar_physics_core

#endif  // RADAR_PHYSICS_CORE_RADAR_PHYSICS_HPP_
