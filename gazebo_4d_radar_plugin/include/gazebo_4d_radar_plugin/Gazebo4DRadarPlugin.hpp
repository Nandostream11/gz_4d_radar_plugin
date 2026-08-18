#ifndef GAZEBO_4D_RADAR_PLUGIN_GAZEBO_4D_RADAR_PLUGIN_HPP_
#define GAZEBO_4D_RADAR_PLUGIN_GAZEBO_4D_RADAR_PLUGIN_HPP_

#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <thread>

// Gazebo Sim includes
#include <gz/sim/System.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/transport/Node.hh>

// ROS 2 includes
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/header.hpp>

// Package includes
#include "radar_msgs/msg/radar_scan.hpp"
#include "radar_msgs/msg/radar_return.hpp"
#include "radar_physics_core/radar_physics.hpp"
#include "radar_physics_core/radar_types.hpp"

namespace gazebo_4d_radar_plugin {

/**
 * @brief Gazebo Sim System Plugin simulating a 4D FMCW MIMO Automotive Imaging Radar.
 * Publishes standard ROS 2 outputs: radar_msgs/msg/RadarScan and sensor_msgs/msg/PointCloud2.
 */
class Gazebo4DRadarPlugin
    : public gz::sim::System,
      public gz::sim::ISystemConfigure,
      public gz::sim::ISystemPreUpdate,
      public gz::sim::ISystemPostUpdate {
 public:
  Gazebo4DRadarPlugin();
  ~Gazebo4DRadarPlugin() override;

  // gz::sim::ISystemConfigure
  void Configure(
      const gz::sim::Entity& entity,
      const std::shared_ptr<const sdf::Element>& sdf,
      gz::sim::EntityComponentManager& ecm,
      gz::sim::EventManager& eventMgr) override;

  // gz::sim::ISystemPreUpdate
  void PreUpdate(
      const gz::sim::UpdateInfo& info,
      gz::sim::EntityComponentManager& ecm) override;

  // gz::sim::ISystemPostUpdate
  void PostUpdate(
      const gz::sim::UpdateInfo& info,
      const gz::sim::EntityComponentManager& ecm) override;

 private:
  void ParseSdfParameters(const std::shared_ptr<const sdf::Element>& sdf);
  void InitializeRos();
  void PerformRadarScan(
      const gz::sim::UpdateInfo& info,
      const gz::sim::EntityComponentManager& ecm);
  void PublishRosMessages(
      const std::vector<radar_physics_core::RadarDetection>& detections,
      const gz::sim::UpdateInfo& info);

  // Gazebo entities
  gz::sim::Entity entity_{gz::sim::kNullEntity};
  gz::sim::Entity model_entity_{gz::sim::kNullEntity};
  std::string sensor_name_{"radar"};
  std::string frame_id_{"radar_link"};
  std::string scan_topic_{"/radar/scan"};
  std::string pointcloud_topic_{"/radar/points"};

  bool publish_radar_scan_{true};
  bool publish_pointcloud_{true};

  // Physics engine & config
  radar_physics_core::RadarConfig config_;
  std::unique_ptr<radar_physics_core::RadarPhysicsEngine> physics_engine_;

  // Ray scan grid configuration
  int ray_samples_azimuth_{120};
  int ray_samples_elevation_{30};
  double last_update_time_sec_{0.0};
  double update_period_sec_{0.05};  // 20 Hz default

  // ROS 2 node and publishers
  std::shared_ptr<rclcpp::Node> ros_node_;
  rclcpp::Publisher<radar_msgs::msg::RadarScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  std::thread ros_spin_thread_;
  std::mutex ros_mutex_;

  // State tracking
  double prev_pos_world_[3]{0, 0, 0};
  double prev_time_sec_{0.0};
  bool has_prev_pos_{false};
};

}  // namespace gazebo_4d_radar_plugin

#endif  // GAZEBO_4D_RADAR_PLUGIN_GAZEBO_4D_RADAR_PLUGIN_HPP_
