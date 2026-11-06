#ifndef GAZEBO_4D_RADAR_PLUGIN_GAZEBO_4D_RADAR_PLUGIN_HPP_
#define GAZEBO_4D_RADAR_PLUGIN_GAZEBO_4D_RADAR_PLUGIN_HPP_

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/System.hh>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <radar_msgs/msg/radar_scan.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "radar_physics_core/radar_physics.hpp"
#include "radar_physics_core/radar_types.hpp"
#include "radar_physics_core/scene_raycaster.hpp"

namespace gazebo_4d_radar_plugin {

/// Radar surface properties applied to every collision of a named model.
struct SurfaceOverride {
  std::optional<double> reflectivity;
  std::optional<double> nominal_rcs_dbm2;
  std::optional<double> incidence_exponent;
  std::optional<int> classification;
};

/**
 * @brief Gazebo Sim system simulating a 4D FMCW MIMO automotive imaging radar.
 *
 * Attach it to a <model>; <link_name> selects the link carrying the radar
 * (default: the model's canonical link). SDF does not allow plugins on <link>.
 * Each cycle it casts a ray fan against the true collision geometry of the world
 * (planes, boxes, spheres, cylinders, meshes), computes exact range-rate from the
 * relative velocity of sensor origin and scatterer, and publishes
 * radar_msgs/RadarScan, sensor_msgs/PointCloud2 and an optional ground-truth
 * sensor velocity for validating ego-velocity estimators.
 */
class Gazebo4DRadarPlugin
    : public gz::sim::System,
      public gz::sim::ISystemConfigure,
      public gz::sim::ISystemPreUpdate,
      public gz::sim::ISystemPostUpdate {
 public:
  Gazebo4DRadarPlugin();
  ~Gazebo4DRadarPlugin() override = default;

  void Configure(const gz::sim::Entity& entity,
                 const std::shared_ptr<const sdf::Element>& sdf,
                 gz::sim::EntityComponentManager& ecm,
                 gz::sim::EventManager& eventMgr) override;

  void PreUpdate(const gz::sim::UpdateInfo& info,
                 gz::sim::EntityComponentManager& ecm) override;

  void PostUpdate(const gz::sim::UpdateInfo& info,
                  const gz::sim::EntityComponentManager& ecm) override;

 private:
  /// Collision shape tracked in the ray-cast scene (parallel to scene_ shapes).
  struct TrackedShape {
    gz::sim::Entity collision{gz::sim::kNullEntity};
    gz::sim::Entity link{gz::sim::kNullEntity};
    bool is_static{true};
    gz::math::Pose3d geometry_offset{};  ///< Extra local offset (mesh submesh centring).
  };

  void ParseSdfParameters(const std::shared_ptr<const sdf::Element>& sdf);
  void InitializeRos();
  bool ResolveSensorLink(gz::sim::EntityComponentManager& ecm);
  void RebuildScene(gz::sim::EntityComponentManager& ecm);
  std::optional<radar_physics_core::SceneShape> MakeShape(
      const gz::sim::EntityComponentManager& ecm, gz::sim::Entity collision,
      gz::math::Pose3d& geometry_offset);
  std::shared_ptr<const radar_physics_core::TriangleMeshBvh> LoadMesh(
      const std::string& path, const std::string& submesh, bool center_submesh,
      const gz::math::Vector3d& scale, gz::math::Vector3d& submesh_center);
  void UpdateDynamicShapes(const gz::sim::EntityComponentManager& ecm);
  bool SensorState(const gz::sim::UpdateInfo& info,
                   const gz::sim::EntityComponentManager& ecm,
                   gz::math::Pose3d& sensor_pose,
                   gz::math::Vector3d& sensor_vel_world,
                   gz::math::Vector3d& sensor_ang_vel_world);
  void PerformRadarScan(const gz::sim::UpdateInfo& info,
                        const gz::sim::EntityComponentManager& ecm);
  void PublishRosMessages(const std::vector<radar_physics_core::RadarDetection>& detections,
                          const gz::sim::UpdateInfo& info,
                          const gz::math::Pose3d& sensor_pose,
                          const gz::math::Vector3d& sensor_vel_world,
                          const gz::math::Vector3d& sensor_ang_vel_world);

  // --- Configuration -------------------------------------------------------
  std::string sensor_name_{"radar"};
  std::string link_name_;  ///< Link carrying the radar; empty = model's canonical link.
  std::string frame_id_{"radar_link"};
  std::string scan_topic_{"/radar/scan"};
  std::string pointcloud_topic_{"/radar/points"};
  std::string ground_truth_topic_{"/radar/ground_truth_velocity"};
  bool publish_radar_scan_{true};
  bool publish_pointcloud_{true};
  bool publish_ground_truth_{true};
  /// Broadcast tf_parent_frame -> frame_id from the true radar pose (for RViz demos).
  bool publish_tf_{false};
  std::string tf_parent_frame_{"world"};
  bool ignore_own_model_{true};
  /// Pose of the radar frame relative to the link the plugin is attached to.
  /// The plugin applies it; downstream consumers must NOT apply it again.
  gz::math::Pose3d mounting_pose_{};
  int ray_samples_azimuth_{120};
  int ray_samples_elevation_{30};
  std::optional<uint32_t> random_seed_;

  radar_physics_core::SurfaceProperties default_surface_{};
  radar_physics_core::SurfaceProperties ground_surface_{0.8, 5.0, 1.0, 0};
  std::map<std::string, SurfaceOverride> surface_overrides_;

  radar_physics_core::RadarConfig config_;
  std::unique_ptr<radar_physics_core::RadarPhysicsEngine> physics_engine_;

  // --- Gazebo state ---------------------------------------------------------
  gz::sim::Entity plugin_entity_{gz::sim::kNullEntity};
  gz::sim::Entity sensor_link_{gz::sim::kNullEntity};
  gz::sim::Entity own_model_{gz::sim::kNullEntity};

  radar_physics_core::SceneRaycaster scene_;
  std::vector<TrackedShape> tracked_;
  std::size_t last_collision_count_{0};
  bool scene_dirty_{true};
  std::unordered_map<std::string, std::shared_ptr<const radar_physics_core::TriangleMeshBvh>>
      mesh_cache_;
  std::unordered_map<std::string, gz::math::Vector3d> mesh_center_cache_;

  double update_period_sec_{0.05};
  std::optional<double> last_update_time_sec_;

  // Finite-difference fallback for the sensor velocity during the first steps,
  // before the physics system has populated the velocity components.
  std::optional<gz::math::Vector3d> prev_sensor_pos_;
  double prev_sensor_time_sec_{0.0};

  // --- ROS 2 -------------------------------------------------------------------
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Publisher<radar_msgs::msg::RadarScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr ground_truth_pub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;
};

}  // namespace gazebo_4d_radar_plugin

#endif  // GAZEBO_4D_RADAR_PLUGIN_GAZEBO_4D_RADAR_PLUGIN_HPP_
