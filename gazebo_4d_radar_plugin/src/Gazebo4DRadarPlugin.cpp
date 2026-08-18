#include "gazebo_4d_radar_plugin/Gazebo4DRadarPlugin.hpp"

#include <gz/plugin/Register.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/Geometry.hh>
#include <gz/sim/Util.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Matrix3.hh>

#include <cmath>
#include <chrono>

namespace gazebo_4d_radar_plugin {

Gazebo4DRadarPlugin::Gazebo4DRadarPlugin() {
  physics_engine_ = std::make_unique<radar_physics_core::RadarPhysicsEngine>(config_);
}

Gazebo4DRadarPlugin::~Gazebo4DRadarPlugin() {
  if (ros_node_ && rclcpp::ok()) {
    rclcpp::shutdown();
  }
  if (ros_spin_thread_.joinable()) {
    ros_spin_thread_.join();
  }
}

void Gazebo4DRadarPlugin::Configure(
    const gz::sim::Entity& entity,
    const std::shared_ptr<const sdf::Element>& sdf,
    gz::sim::EntityComponentManager& /*ecm*/,
    gz::sim::EventManager& /*eventMgr*/) {
  entity_ = entity;

  ParseSdfParameters(sdf);
  InitializeRos();
}

void Gazebo4DRadarPlugin::ParseSdfParameters(const std::shared_ptr<const sdf::Element>& sdf) {
  if (!sdf) return;

  if (sdf->HasElement("sensor_name")) {
    sensor_name_ = sdf->Get<std::string>("sensor_name");
  }
  if (sdf->HasElement("frame_id")) {
    frame_id_ = sdf->Get<std::string>("frame_id");
  }
  if (sdf->HasElement("scan_topic")) {
    scan_topic_ = sdf->Get<std::string>("scan_topic");
  }
  if (sdf->HasElement("pointcloud_topic")) {
    pointcloud_topic_ = sdf->Get<std::string>("pointcloud_topic");
  }

  if (sdf->HasElement("publish_radar_scan")) {
    publish_radar_scan_ = sdf->Get<bool>("publish_radar_scan");
  }
  if (sdf->HasElement("publish_pointcloud")) {
    publish_pointcloud_ = sdf->Get<bool>("publish_pointcloud");
  }

  // Update rates & cycles
  if (sdf->HasElement("cycle_rate_hz")) {
    config_.cycle_rate_hz = sdf->Get<double>("cycle_rate_hz");
    if (config_.cycle_rate_hz > 0.0) {
      update_period_sec_ = 1.0 / config_.cycle_rate_hz;
    }
  }

  // FOV & Range
  if (sdf->HasElement("max_range_m")) {
    config_.max_range_m = sdf->Get<double>("max_range_m");
  }
  if (sdf->HasElement("min_range_m")) {
    config_.min_range_m = sdf->Get<double>("min_range_m");
  }
  if (sdf->HasElement("fov_azimuth_rad")) {
    double fov_az = sdf->Get<double>("fov_azimuth_rad");
    config_.fov_azimuth_min_rad = -fov_az * 0.5;
    config_.fov_azimuth_max_rad = fov_az * 0.5;
  }
  if (sdf->HasElement("fov_elevation_rad")) {
    double fov_el = sdf->Get<double>("fov_elevation_rad");
    config_.fov_elevation_min_rad = -fov_el * 0.5;
    config_.fov_elevation_max_rad = fov_el * 0.5;
  }

  // Doppler & Ambiguity
  if (sdf->HasElement("v_max_mps")) {
    config_.v_max_mps = sdf->Get<double>("v_max_mps");
  }
  if (sdf->HasElement("enable_doppler_folding")) {
    config_.enable_doppler_folding = sdf->Get<bool>("enable_doppler_folding");
  }
  if (sdf->HasElement("closing_velocity_positive")) {
    config_.closing_velocity_positive = sdf->Get<bool>("closing_velocity_positive");
  }

  // Noise sigmas
  if (sdf->HasElement("range_sigma_m")) {
    config_.range_sigma_m = sdf->Get<double>("range_sigma_m");
  }
  if (sdf->HasElement("azimuth_sigma_rad")) {
    config_.azimuth_sigma_rad = sdf->Get<double>("azimuth_sigma_rad");
  }
  if (sdf->HasElement("elevation_sigma_rad")) {
    config_.elevation_sigma_rad = sdf->Get<double>("elevation_sigma_rad");
  }
  if (sdf->HasElement("range_rate_sigma_mps")) {
    config_.range_rate_sigma_mps = sdf->Get<double>("range_rate_sigma_mps");
  }

  // Resolutions
  if (sdf->HasElement("range_resolution_m")) {
    config_.range_resolution_m = sdf->Get<double>("range_resolution_m");
  }
  if (sdf->HasElement("azimuth_resolution_rad")) {
    config_.azimuth_resolution_rad = sdf->Get<double>("azimuth_resolution_rad");
  }
  if (sdf->HasElement("elevation_resolution_rad")) {
    config_.elevation_resolution_rad = sdf->Get<double>("elevation_resolution_rad");
  }
  if (sdf->HasElement("range_rate_resolution_mps")) {
    config_.range_rate_resolution_mps = sdf->Get<double>("range_rate_resolution_mps");
  }

  // Non-idealities & Clutter
  if (sdf->HasElement("false_detection_rate_per_cycle")) {
    config_.false_detection_rate_per_cycle = sdf->Get<double>("false_detection_rate_per_cycle");
  }
  if (sdf->HasElement("detection_dropout_probability")) {
    config_.detection_dropout_probability = sdf->Get<double>("detection_dropout_probability");
  }
  if (sdf->HasElement("min_rcs_threshold")) {
    config_.min_rcs_threshold_dbm2 = sdf->Get<double>("min_rcs_threshold");
  }

  // Scan ray fan density
  if (sdf->HasElement("ray_samples_azimuth")) {
    ray_samples_azimuth_ = sdf->Get<int>("ray_samples_azimuth");
  }
  if (sdf->HasElement("ray_samples_elevation")) {
    ray_samples_elevation_ = sdf->Get<int>("ray_samples_elevation");
  }

  config_.frame_id = frame_id_;
  physics_engine_->set_config(config_);
}

void Gazebo4DRadarPlugin::InitializeRos() {
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  ros_node_ = rclcpp::Node::make_shared(sensor_name_ + "_node");

  if (publish_radar_scan_) {
    scan_pub_ = ros_node_->create_publisher<radar_msgs::msg::RadarScan>(
        scan_topic_, rclcpp::QoS(10));
  }
  if (publish_pointcloud_) {
    pointcloud_pub_ = ros_node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        pointcloud_topic_, rclcpp::QoS(10));
  }

  ros_spin_thread_ = std::thread([this]() {
    rclcpp::spin(ros_node_);
  });
}

void Gazebo4DRadarPlugin::PreUpdate(
    const gz::sim::UpdateInfo& /*info*/,
    gz::sim::EntityComponentManager& /*ecm*/) {
}

void Gazebo4DRadarPlugin::PostUpdate(
    const gz::sim::UpdateInfo& info,
    const gz::sim::EntityComponentManager& ecm) {
  if (info.paused) return;

  double current_time_sec = std::chrono::duration<double>(info.simTime).count();
  if (current_time_sec - last_update_time_sec_ < update_period_sec_) {
    return;
  }
  last_update_time_sec_ = current_time_sec;

  PerformRadarScan(info, ecm);
}

void Gazebo4DRadarPlugin::PerformRadarScan(
    const gz::sim::UpdateInfo& info,
    const gz::sim::EntityComponentManager& ecm) {
  gz::math::Pose3d sensor_world_pose = gz::sim::worldPose(entity_, ecm);
  gz::math::Vector3d sensor_pos = sensor_world_pose.Pos();
  gz::math::Quaterniond sensor_rot = sensor_world_pose.Rot();

  gz::math::Vector3d sensor_vel_world(0, 0, 0);
  auto lin_vel_comp = ecm.Component<gz::sim::components::LinearVelocity>(entity_);
  if (lin_vel_comp) {
    sensor_vel_world = lin_vel_comp->Data();
  } else {
    double current_time = std::chrono::duration<double>(info.simTime).count();
    if (has_prev_pos_ && (current_time > prev_time_sec_)) {
      double dt = current_time - prev_time_sec_;
      sensor_vel_world.Set(
          (sensor_pos.X() - prev_pos_world_[0]) / dt,
          (sensor_pos.Y() - prev_pos_world_[1]) / dt,
          (sensor_pos.Z() - prev_pos_world_[2]) / dt);
    }
    prev_pos_world_[0] = sensor_pos.X();
    prev_pos_world_[1] = sensor_pos.Y();
    prev_pos_world_[2] = sensor_pos.Z();
    prev_time_sec_ = current_time;
    has_prev_pos_ = true;
  }

  double sensor_pos_raw[3] = {sensor_pos.X(), sensor_pos.Y(), sensor_pos.Z()};
  double sensor_vel_raw[3] = {sensor_vel_world.X(), sensor_vel_world.Y(), sensor_vel_world.Z()};

  gz::math::Matrix3d rot_mat = sensor_rot.Inverse().Matrix3();
  double rot_raw[9] = {
      rot_mat(0, 0), rot_mat(0, 1), rot_mat(0, 2),
      rot_mat(1, 0), rot_mat(1, 1), rot_mat(1, 2),
      rot_mat(2, 0), rot_mat(2, 1), rot_mat(2, 2)};

  std::vector<radar_physics_core::RadarDetection> detections;
  uint16_t measurement_id = 0;

  // Collect world obstacle bounding boxes & velocities from ECM
  struct ObstacleEntity {
    gz::math::Vector3d pos;
    gz::math::Vector3d vel;
    gz::math::Vector3d size;
    double rcs;
    uint8_t classification;
  };
  std::vector<ObstacleEntity> obstacles;

  ecm.Each<gz::sim::components::Model, gz::sim::components::Pose>(
      [&](const gz::sim::Entity& ent,
          const gz::sim::components::Model* /*model*/,
          const gz::sim::components::Pose* pose) -> bool {
        if (ent == entity_ || ent == model_entity_) {
          return true;
        }

        ObstacleEntity obs;
        obs.pos = pose->Data().Pos();
        obs.vel.Set(0, 0, 0);
        obs.size.Set(2.0, 2.0, 1.5);
        obs.rcs = 15.0;
        obs.classification = 1;

        auto lin_vel = ecm.Component<gz::sim::components::LinearVelocity>(ent);
        if (lin_vel) {
          obs.vel = lin_vel->Data();
        }

        obstacles.push_back(obs);
        return true;
      });

  // Cast ray fan across Azimuth x Elevation FOV
  double az_step = (config_.fov_azimuth_max_rad - config_.fov_azimuth_min_rad) /
                   std::max(1, ray_samples_azimuth_ - 1);
  double el_step = (config_.fov_elevation_max_rad - config_.fov_elevation_min_rad) /
                   std::max(1, ray_samples_elevation_ - 1);

  for (int i_el = 0; i_el < ray_samples_elevation_; ++i_el) {
    double el = config_.fov_elevation_min_rad + i_el * el_step;
    for (int i_az = 0; i_az < ray_samples_azimuth_; ++i_az) {
      double az = config_.fov_azimuth_min_rad + i_az * az_step;

      gz::math::Vector3d dir_s(
          std::cos(el) * std::cos(az),
          std::cos(el) * std::sin(az),
          std::sin(el));

      gz::math::Vector3d dir_w = sensor_rot * dir_s;
      dir_w.Normalize();

      double nearest_dist = config_.max_range_m;
      radar_physics_core::RawRayHit nearest_hit;
      gz::math::Vector3d target_vel_w(0, 0, 0);
      bool has_hit = false;

      // Ground intersection (z = 0 plane)
      if (dir_w.Z() < -1e-4) {
        double t_ground = (0.0 - sensor_pos.Z()) / dir_w.Z();
        if (t_ground > config_.min_range_m && t_ground < nearest_dist) {
          nearest_dist = t_ground;
          nearest_hit.hit_point_world[0] = sensor_pos.X() + t_ground * dir_w.X();
          nearest_hit.hit_point_world[1] = sensor_pos.Y() + t_ground * dir_w.Y();
          nearest_hit.hit_point_world[2] = 0.0;
          nearest_hit.hit_normal_world[0] = 0.0;
          nearest_hit.hit_normal_world[1] = 0.0;
          nearest_hit.hit_normal_world[2] = 1.0;
          nearest_hit.nominal_rcs_dbm2 = 5.0;
          nearest_hit.surface_reflectivity = 0.8;
          nearest_hit.classification = 0;
          target_vel_w.Set(0, 0, 0);
          has_hit = true;
        }
      }

      // Obstacle bounding volume intersections
      for (const auto& obs : obstacles) {
        gz::math::Vector3d box_min = obs.pos - obs.size * 0.5;
        gz::math::Vector3d box_max = obs.pos + obs.size * 0.5;

        double t_min = (box_min.X() - sensor_pos.X()) / (std::abs(dir_w.X()) > 1e-6 ? dir_w.X() : 1e-6);
        double t_max = (box_max.X() - sensor_pos.X()) / (std::abs(dir_w.X()) > 1e-6 ? dir_w.X() : 1e-6);
        if (t_min > t_max) std::swap(t_min, t_max);

        double ty_min = (box_min.Y() - sensor_pos.Y()) / (std::abs(dir_w.Y()) > 1e-6 ? dir_w.Y() : 1e-6);
        double ty_max = (box_max.Y() - sensor_pos.Y()) / (std::abs(dir_w.Y()) > 1e-6 ? dir_w.Y() : 1e-6);
        if (ty_min > ty_max) std::swap(ty_min, ty_max);

        if (t_min > ty_max || ty_min > t_max) continue;
        if (ty_min > t_min) t_min = ty_min;
        if (ty_max < t_max) t_max = ty_max;

        double tz_min = (box_min.Z() - sensor_pos.Z()) / (std::abs(dir_w.Z()) > 1e-6 ? dir_w.Z() : 1e-6);
        double tz_max = (box_max.Z() - sensor_pos.Z()) / (std::abs(dir_w.Z()) > 1e-6 ? dir_w.Z() : 1e-6);
        if (tz_min > tz_max) std::swap(tz_min, tz_max);

        if (t_min > tz_max || tz_min > t_max) continue;
        if (tz_min > t_min) t_min = tz_min;
        if (tz_max < t_max) t_max = tz_max;

        if (t_min > config_.min_range_m && t_min < nearest_dist) {
          nearest_dist = t_min;
          nearest_hit.hit_point_world[0] = sensor_pos.X() + t_min * dir_w.X();
          nearest_hit.hit_point_world[1] = sensor_pos.Y() + t_min * dir_w.Y();
          nearest_hit.hit_point_world[2] = sensor_pos.Z() + t_min * dir_w.Z();
          nearest_hit.hit_normal_world[0] = -dir_w.X();
          nearest_hit.hit_normal_world[1] = -dir_w.Y();
          nearest_hit.hit_normal_world[2] = -dir_w.Z();
          nearest_hit.nominal_rcs_dbm2 = obs.rcs;
          nearest_hit.surface_reflectivity = 1.0;
          nearest_hit.classification = obs.classification;
          target_vel_w = obs.vel;
          has_hit = true;
        }
      }

      if (has_hit) {
        double rel_vel_raw[3] = {
            sensor_vel_raw[0] - target_vel_w.X(),
            sensor_vel_raw[1] - target_vel_w.Y(),
            sensor_vel_raw[2] - target_vel_w.Z()};

        auto det = physics_engine_->process_ray_hit(
            sensor_pos_raw, rot_raw, rel_vel_raw, nearest_hit, measurement_id++);
        if (det.has_value()) {
          detections.push_back(*det);
        }
      }
    }
  }

  // Inject false / clutter detections
  auto false_dets = physics_engine_->generate_false_detections(measurement_id);
  detections.insert(detections.end(), false_dets.begin(), false_dets.end());

  // Publish to ROS 2
  PublishRosMessages(detections, info);
}

void Gazebo4DRadarPlugin::PublishRosMessages(
    const std::vector<radar_physics_core::RadarDetection>& detections,
    const gz::sim::UpdateInfo& info) {
  std_msgs::msg::Header header;
  int64_t sim_nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
  header.stamp.sec = static_cast<int32_t>(sim_nanos / 1000000000LL);
  header.stamp.nanosec = static_cast<uint32_t>(sim_nanos % 1000000000LL);
  header.frame_id = frame_id_;

  // 1. Publish Generic Standard radar_msgs/msg/RadarScan
  if (scan_pub_) {
    radar_msgs::msg::RadarScan scan_msg;
    scan_msg.header = header;
    scan_msg.returns.reserve(detections.size());

    for (const auto& d : detections) {
      radar_msgs::msg::RadarReturn ret;
      ret.range = static_cast<float>(d.range_m);
      ret.azimuth = static_cast<float>(d.azimuth_rad);
      ret.elevation = static_cast<float>(d.elevation_rad);
      ret.doppler_velocity = static_cast<float>(d.range_rate_mps);
      ret.amplitude = static_cast<float>(d.rcs_dbm2);
      scan_msg.returns.push_back(ret);
    }
    scan_pub_->publish(scan_msg);
  }

  // 2. Publish Standard sensor_msgs/msg/PointCloud2
  if (pointcloud_pub_) {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = header;
    cloud.height = 1;
    cloud.width = static_cast<uint32_t>(detections.size());
    cloud.is_dense = true;
    cloud.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(
        8,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "doppler", 1, sensor_msgs::msg::PointField::FLOAT32,
        "range", 1, sensor_msgs::msg::PointField::FLOAT32,
        "azimuth", 1, sensor_msgs::msg::PointField::FLOAT32,
        "elevation", 1, sensor_msgs::msg::PointField::FLOAT32,
        "rcs", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(detections.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_doppler(cloud, "doppler");
    sensor_msgs::PointCloud2Iterator<float> iter_range(cloud, "range");
    sensor_msgs::PointCloud2Iterator<float> iter_az(cloud, "azimuth");
    sensor_msgs::PointCloud2Iterator<float> iter_el(cloud, "elevation");
    sensor_msgs::PointCloud2Iterator<float> iter_rcs(cloud, "rcs");

    for (const auto& d : detections) {
      *iter_x = static_cast<float>(d.x_m);
      *iter_y = static_cast<float>(d.y_m);
      *iter_z = static_cast<float>(d.z_m);
      *iter_doppler = static_cast<float>(d.range_rate_mps);
      *iter_range = static_cast<float>(d.range_m);
      *iter_az = static_cast<float>(d.azimuth_rad);
      *iter_el = static_cast<float>(d.elevation_rad);
      *iter_rcs = static_cast<float>(d.rcs_dbm2);

      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++iter_doppler;
      ++iter_range;
      ++iter_az;
      ++iter_el;
      ++iter_rcs;
    }

    pointcloud_pub_->publish(cloud);
  }
}

}  // namespace gazebo_4d_radar_plugin

GZ_ADD_PLUGIN(
    gazebo_4d_radar_plugin::Gazebo4DRadarPlugin,
    gz::sim::System,
    gazebo_4d_radar_plugin::Gazebo4DRadarPlugin::ISystemConfigure,
    gazebo_4d_radar_plugin::Gazebo4DRadarPlugin::ISystemPreUpdate,
    gazebo_4d_radar_plugin::Gazebo4DRadarPlugin::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(
    gazebo_4d_radar_plugin::Gazebo4DRadarPlugin,
    "gazebo_4d_radar_plugin::Gazebo4DRadarPlugin")
