#include <gtest/gtest.h>
#include <cmath>
#include <vector>

#include "radar_physics_core/radar_physics.hpp"

using namespace radar_physics_core;

class RadarPhysicsTest : public ::testing::Test {
 protected:
  void SetUp() override {
    config_.range_sigma_m = 0.0;
    config_.azimuth_sigma_rad = 0.0;
    config_.elevation_sigma_rad = 0.0;
    config_.range_rate_sigma_mps = 0.0;
    config_.detection_dropout_probability = 0.0;
    config_.false_detection_rate_per_cycle = 0.0;
    config_.range_resolution_m = 0.0;
    config_.azimuth_resolution_rad = 0.0;
    config_.elevation_resolution_rad = 0.0;
    config_.range_rate_resolution_mps = 0.0;
    config_.v_max_mps = 50.0;
    config_.enable_doppler_folding = true;
    config_.min_rcs_threshold_dbm2 = -40.0;

    engine_.set_config(config_);
    engine_.seed(42);
  }

  RadarConfig config_;
  RadarPhysicsEngine engine_;
};

// ============================================================================
// Validation Test 1: Static Platform (Doppler must be exactly zero)
// ============================================================================
TEST_F(RadarPhysicsTest, StaticPlatformZeroDoppler) {
  double sensor_pos[3] = {0.0, 0.0, 1.0};
  double rot_identity[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  double sensor_vel[3] = {0.0, 0.0, 0.0};  // Zero translation velocity

  // Test hits in different directions across FOV
  std::vector<std::pair<double, double>> angles = {
      {0.0, 0.0}, {-0.5, 0.0}, {0.5, 0.0}, {0.0, 0.1}, {-0.3, -0.1}};

  for (size_t i = 0; i < angles.size(); ++i) {
    double az = angles[i].first;
    double el = angles[i].second;
    double range = 20.0;

    RawRayHit hit;
    hit.hit_point_world[0] = sensor_pos[0] + range * std::cos(el) * std::cos(az);
    hit.hit_point_world[1] = sensor_pos[1] + range * std::cos(el) * std::sin(az);
    hit.hit_point_world[2] = sensor_pos[2] + range * std::sin(el);
    hit.hit_normal_world[0] = -std::cos(az);
    hit.hit_normal_world[1] = -std::sin(az);
    hit.hit_normal_world[2] = 0.0;
    hit.nominal_rcs_dbm2 = 10.0;
    hit.surface_reflectivity = 1.0;

    auto det = engine_.process_ray_hit(sensor_pos, rot_identity, sensor_vel, hit, static_cast<uint16_t>(i));
    ASSERT_TRUE(det.has_value());
    EXPECT_NEAR(det->range_rate_mps, 0.0, 1e-6);
    EXPECT_NEAR(det->range_m, range, 1e-4);
    EXPECT_NEAR(det->azimuth_rad, az, 1e-4);
    EXPECT_NEAR(det->elevation_rad, el, 1e-4);
  }
}

// ============================================================================
// Validation Test 2: Pure Translation Toward Flat Wall (Cosine Doppler profile)
// ============================================================================
TEST_F(RadarPhysicsTest, PureTranslationCosineDopplerProfile) {
  double sensor_pos[3] = {0.0, 0.0, 1.0};
  double rot_identity[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  double forward_speed = 15.0;  // 15 m/s along +x
  double sensor_vel[3] = {forward_speed, 0.0, 0.0};

  // Wall located at x = 30m
  double wall_x = 30.0;

  for (double az_deg = -50.0; az_deg <= 50.0; az_deg += 10.0) {
    double az_rad = az_deg * M_PI / 180.0;
    double range = wall_x / std::cos(az_rad);

    RawRayHit hit;
    hit.hit_point_world[0] = wall_x;
    hit.hit_point_world[1] = sensor_pos[1] + wall_x * std::tan(az_rad);
    hit.hit_point_world[2] = sensor_pos[2];
    hit.hit_normal_world[0] = -1.0;  // Normal pointing back toward sensor
    hit.hit_normal_world[1] = 0.0;
    hit.hit_normal_world[2] = 0.0;
    hit.nominal_rcs_dbm2 = 10.0;
    hit.surface_reflectivity = 1.0;

    auto det = engine_.process_ray_hit(sensor_pos, rot_identity, sensor_vel, hit, 1);
    ASSERT_TRUE(det.has_value());

    // Expected Doppler is -v_s . u_los = -forward_speed * cos(azimuth)
    double expected_doppler = -forward_speed * std::cos(az_rad);
    EXPECT_NEAR(det->range_rate_mps, expected_doppler, 1e-4);
  }
}

// ============================================================================
// Validation Test 3: Pure Sensor Origin Rotation Invariance
// ============================================================================
TEST_F(RadarPhysicsTest, PureRotationInvariance) {
  // Rotating the sensor about its own origin does not change target range distance.
  // Translating velocity v_s = [0, 0, 0].
  double sensor_pos[3] = {5.0, 2.0, 1.0};
  double rot_yaw_45[9] = {
      std::cos(M_PI / 4), -std::sin(M_PI / 4), 0.0,
      std::sin(M_PI / 4),  std::cos(M_PI / 4), 0.0,
      0.0,                 0.0,                1.0};
  double sensor_vel[3] = {0.0, 0.0, 0.0};  // Zero translation

  RawRayHit hit;
  hit.hit_point_world[0] = 20.0;
  hit.hit_point_world[1] = 10.0;
  hit.hit_point_world[2] = 1.0;
  hit.hit_normal_world[0] = -1.0;
  hit.hit_normal_world[1] = 0.0;
  hit.hit_normal_world[2] = 0.0;
  hit.nominal_rcs_dbm2 = 15.0;

  auto det = engine_.process_ray_hit(sensor_pos, rot_yaw_45, sensor_vel, hit, 1);
  ASSERT_TRUE(det.has_value());
  EXPECT_NEAR(det->range_rate_mps, 0.0, 1e-6);
}

// ============================================================================
// Validation Test 4: Doppler Folding / Nyquist Ambiguity
// ============================================================================
TEST_F(RadarPhysicsTest, DopplerFoldingPeriodicModulo) {
  double v_max = 50.0;  // unambiguous limit [-50, +50] m/s

  // Unfolded tests (inside bounds)
  EXPECT_NEAR(RadarPhysicsEngine::fold_doppler(0.0, v_max), 0.0, 1e-6);
  EXPECT_NEAR(RadarPhysicsEngine::fold_doppler(25.0, v_max), 25.0, 1e-6);
  EXPECT_NEAR(RadarPhysicsEngine::fold_doppler(-25.0, v_max), -25.0, 1e-6);

  // Boundary conditions: +50 wraps to -50
  EXPECT_NEAR(RadarPhysicsEngine::fold_doppler(50.0, v_max), -50.0, 1e-6);
  EXPECT_NEAR(RadarPhysicsEngine::fold_doppler(-50.0, v_max), -50.0, 1e-6);

  // Exceeding positive boundary: +55 -> -45
  EXPECT_NEAR(RadarPhysicsEngine::fold_doppler(55.0, v_max), -45.0, 1e-6);

  // Exceeding negative boundary: -55 -> +45
  EXPECT_NEAR(RadarPhysicsEngine::fold_doppler(-55.0, v_max), 45.0, 1e-6);

  // 2x v_max: +110 -> +10
  EXPECT_NEAR(RadarPhysicsEngine::fold_doppler(110.0, v_max), 10.0, 1e-6);
}

// ============================================================================
// Validation Test 5: RCS 1/R^4 Falloff and Incidence Angle Cutoff
// ============================================================================
TEST_F(RadarPhysicsTest, RcsFalloffAndThresholding) {
  double sensor_pos[3] = {0.0, 0.0, 0.0};
  double rot_identity[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  double sensor_vel[3] = {0.0, 0.0, 0.0};

  // Close target (10m) -> Strong return -> Accepted
  RawRayHit close_hit;
  close_hit.hit_point_world[0] = 10.0;
  close_hit.hit_point_world[1] = 0.0;
  close_hit.hit_point_world[2] = 0.0;
  close_hit.hit_normal_world[0] = -1.0;
  close_hit.nominal_rcs_dbm2 = 10.0;
  close_hit.surface_reflectivity = 1.0;

  auto det_close = engine_.process_ray_hit(sensor_pos, rot_identity, sensor_vel, close_hit, 1);
  ASSERT_TRUE(det_close.has_value());

  // Grazing incidence angle: normal perpendicular to LOS -> Low return -> Dropped if below threshold
  RawRayHit grazing_hit;
  grazing_hit.hit_point_world[0] = 10.0;
  grazing_hit.hit_point_world[1] = 0.0;
  grazing_hit.hit_point_world[2] = 0.0;
  // Normal perpendicular (dot product with LOS is 0)
  grazing_hit.hit_normal_world[0] = 0.0;
  grazing_hit.hit_normal_world[1] = 1.0;
  grazing_hit.hit_normal_world[2] = 0.0;
  grazing_hit.nominal_rcs_dbm2 = -20.0;
  grazing_hit.surface_reflectivity = 0.1;

  auto det_grazing = engine_.process_ray_hit(sensor_pos, rot_identity, sensor_vel, grazing_hit, 2);
  EXPECT_FALSE(det_grazing.has_value());
}

// ============================================================================
// Validation Test 6: Noise and Quantization
// ============================================================================
TEST_F(RadarPhysicsTest, NoiseAndQuantizationEnabled) {
  config_.range_sigma_m = 0.1;
  config_.range_rate_sigma_mps = 0.1;
  config_.range_resolution_m = 0.05;
  config_.range_rate_resolution_mps = 0.02;

  engine_.set_config(config_);

  double sensor_pos[3] = {0.0, 0.0, 0.0};
  double rot_identity[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  double sensor_vel[3] = {10.0, 0.0, 0.0};

  RawRayHit hit;
  hit.hit_point_world[0] = 20.0;
  hit.hit_point_world[1] = 0.0;
  hit.hit_point_world[2] = 0.0;
  hit.hit_normal_world[0] = -1.0;
  hit.nominal_rcs_dbm2 = 10.0;

  auto det = engine_.process_ray_hit(sensor_pos, rot_identity, sensor_vel, hit, 1);
  ASSERT_TRUE(det.has_value());

  // Confirm quantization grid (modulo of resolution should be ~0)
  double r_rem = std::fmod(det->range_m, config_.range_resolution_m);
  if (r_rem > config_.range_resolution_m * 0.5) r_rem -= config_.range_resolution_m;
  EXPECT_NEAR(r_rem, 0.0, 1e-4);

  double v_rem = std::fmod(std::abs(det->range_rate_mps), config_.range_rate_resolution_mps);
  if (v_rem > config_.range_rate_resolution_mps * 0.5) v_rem -= config_.range_rate_resolution_mps;
  EXPECT_NEAR(v_rem, 0.0, 1e-4);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
