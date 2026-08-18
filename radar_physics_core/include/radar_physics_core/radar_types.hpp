#ifndef RADAR_PHYSICS_CORE_RADAR_TYPES_HPP_
#define RADAR_PHYSICS_CORE_RADAR_TYPES_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace radar_physics_core {

/**
 * @brief Configuration parameters for the 4D Imaging Radar physics simulation.
 * Modeled after Continental ARS548 77 GHz MIMO automotive radar.
 */
struct RadarConfig {
  // Sensor carrier & cycle
  double carrier_frequency_ghz{77.0};
  double cycle_rate_hz{20.0};

  // Field of view (sensor frame: +x forward, +y left, +z up)
  double fov_azimuth_min_rad{-1.04719755};  // -60 deg
  double fov_azimuth_max_rad{1.04719755};   // +60 deg
  double fov_elevation_min_rad{-0.261799};  // -15 deg
  double fov_elevation_max_rad{0.261799};   // +15 deg

  // Range bounds
  double min_range_m{0.2};
  double max_range_m{300.0};

  // Doppler & Velocity Ambiguity
  double v_max_mps{55.55};                  // Max unambiguous velocity (~200 km/h)
  bool enable_doppler_folding{true};        // Toggle Doppler wrap-around
  bool closing_velocity_positive{false};    // false: r_dot = -v_s * u (closing is negative)

  // Noise standard deviations
  double range_sigma_m{0.05};               // 5 cm range noise
  double azimuth_sigma_rad{0.005236};       // ~0.3 deg azimuth noise
  double elevation_sigma_rad{0.010472};     // ~0.6 deg elevation noise
  double range_rate_sigma_mps{0.05};        // 5 cm/s velocity noise

  // Quantization / reported resolutions
  double range_resolution_m{0.02};          // 2 cm bin
  double azimuth_resolution_rad{0.001745};  // ~0.1 deg
  double elevation_resolution_rad{0.003491};// ~0.2 deg
  double range_rate_resolution_mps{0.01};   // 1 cm/s

  // Radar cross section & Energy falloff
  double min_rcs_threshold_dbm2{-25.0};     // Detection cutoff threshold
  double rcs_ref_dbm2{10.0};                // Base nominal RCS
  double power_falloff_exponent{4.0};       // 1/R^4 radar equation falloff

  // Stochastic Non-idealities
  double false_detection_rate_per_cycle{2.0}; // Mean Poisson clutter count
  double detection_dropout_probability{0.03}; // 3% random dropout rate

  // Frame ID
  std::string frame_id{"radar_link"};
};

/**
 * @brief Single 4D radar detection data structure.
 */
struct RadarDetection {
  // Spherical coordinates in sensor frame
  double range_m{0.0};              // Radial distance r
  double azimuth_rad{0.0};          // Azimuth angle phi
  double elevation_rad{0.0};        // Elevation angle theta

  // Kinematics
  double range_rate_mps{0.0};       // Doppler radial velocity r_dot

  // Variances / Standard deviations
  double range_std_m{0.0};
  double azimuth_std_rad{0.0};
  double elevation_std_rad{0.0};
  double range_rate_std_mps{0.0};

  // Cartesian coordinates in sensor frame (derived for convenience/RViz)
  double x_m{0.0};
  double y_m{0.0};
  double z_m{0.0};

  // Quality & classification
  int8_t rcs_dbm2{0};               // Radar cross section (-50..50 dBm^2)
  uint16_t measurement_id{0};       // ID within scan
  uint8_t positive_predictive_value{95}; // Existence probability (0..100)
  uint8_t invalid_flags{0};         // 0 = valid
  uint8_t classification{0};        // 0=point, 1=car, etc.
};

/**
 * @brief Raw geometric raycast hit input to the physics engine.
 */
struct RawRayHit {
  double hit_point_world[3]{0, 0, 0};
  double hit_normal_world[3]{0, 0, 0};
  double surface_reflectivity{1.0};  // [0..1]
  double nominal_rcs_dbm2{10.0};     // Object base RCS
  uint8_t classification{0};
};

}  // namespace radar_physics_core

#endif  // RADAR_PHYSICS_CORE_RADAR_TYPES_HPP_
