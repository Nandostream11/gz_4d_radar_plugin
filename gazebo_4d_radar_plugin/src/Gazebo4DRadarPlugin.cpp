#include "gazebo_4d_radar_plugin/Gazebo4DRadarPlugin.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>

#include <gz/common/Console.hh>
#include <gz/common/Mesh.hh>
#include <gz/common/MeshManager.hh>
#include <gz/common/SubMesh.hh>
#include <gz/math/Matrix3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/Geometry.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Static.hh>
#include <sdf/Box.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Geometry.hh>
#include <sdf/Mesh.hh>
#include <sdf/Plane.hh>
#include <sdf/Sphere.hh>
#include <sdf/Capsule.hh>
#include <sdf/Ellipsoid.hh>

#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace gazebo_4d_radar_plugin {

namespace rpc = radar_physics_core;

namespace {

rpc::Vec3 to_vec(const gz::math::Vector3d& v) { return {v.X(), v.Y(), v.Z()}; }

rpc::RigidPose to_rigid(const gz::math::Pose3d& pose) {
  rpc::RigidPose rp;
  rp.p = to_vec(pose.Pos());
  const gz::math::Matrix3d m(pose.Rot());
  rp.R = {m(0, 0), m(0, 1), m(0, 2), m(1, 0), m(1, 1), m(1, 2), m(2, 0), m(2, 1), m(2, 2)};
  return rp;
}

/// world_T_child = world_T_parent * parent_T_child, written out explicitly so the
/// result does not depend on gz-math's operator* convention.
gz::math::Pose3d compose(const gz::math::Pose3d& world_T_parent,
                         const gz::math::Pose3d& parent_T_child) {
  return gz::math::Pose3d(
      world_T_parent.Pos() + world_T_parent.Rot().RotateVector(parent_T_child.Pos()),
      world_T_parent.Rot() * parent_T_child.Rot());
}

double sim_seconds(const gz::sim::UpdateInfo& info) {
  return std::chrono::duration<double>(info.simTime).count();
}

}  // namespace

Gazebo4DRadarPlugin::Gazebo4DRadarPlugin()
    : physics_engine_(std::make_unique<rpc::RadarPhysicsEngine>(config_)) {}

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

void Gazebo4DRadarPlugin::Configure(const gz::sim::Entity& entity,
                                    const std::shared_ptr<const sdf::Element>& sdf,
                                    gz::sim::EntityComponentManager& ecm,
                                    gz::sim::EventManager& /*eventMgr*/) {
  plugin_entity_ = entity;
  ParseSdfParameters(sdf);
  if (!ResolveSensorLink(ecm)) {
    gzerr << "[" << sensor_name_ << "] 4D radar plugin must be attached to a <model> "
          << "(entity " << entity << "). Radar disabled.\n";
    return;
  }
  InitializeRos();
  gzmsg << "[" << sensor_name_ << "] 4D radar ready: " << ray_samples_azimuth_ << "x"
        << ray_samples_elevation_ << " rays @ " << config_.cycle_rate_hz << " Hz, topic "
        << scan_topic_ << "\n";
}

void Gazebo4DRadarPlugin::ParseSdfParameters(const std::shared_ptr<const sdf::Element>& sdf_in) {
  if (!sdf_in) return;
  // Clone so we can iterate repeated child elements with the non-const API.
  sdf::ElementPtr sdf = sdf_in->Clone();

  auto get_str = [&](const char* key, std::string& out) {
    if (sdf->HasElement(key)) out = sdf->Get<std::string>(key);
  };
  auto get_bool = [&](const char* key, bool& out) {
    if (sdf->HasElement(key)) out = sdf->Get<bool>(key);
  };
  auto get_dbl = [&](const char* key, double& out) {
    if (sdf->HasElement(key)) out = sdf->Get<double>(key);
  };
  auto get_int = [&](const char* key, int& out) {
    if (sdf->HasElement(key)) out = sdf->Get<int>(key);
  };

  get_str("sensor_name", sensor_name_);
  get_str("link_name", link_name_);
  get_str("frame_id", frame_id_);
  get_str("scan_topic", scan_topic_);
  get_str("pointcloud_topic", pointcloud_topic_);
  get_str("ground_truth_topic", ground_truth_topic_);
  get_bool("publish_radar_scan", publish_radar_scan_);
  get_bool("publish_pointcloud", publish_pointcloud_);
  get_bool("publish_ground_truth", publish_ground_truth_);
  get_bool("publish_tf", publish_tf_);
  get_str("tf_parent_frame", tf_parent_frame_);
  get_bool("ignore_own_model", ignore_own_model_);
  if (sdf->HasElement("mounting_pose")) {
    mounting_pose_ = sdf->Get<gz::math::Pose3d>("mounting_pose");
  }

  get_dbl("cycle_rate_hz", config_.cycle_rate_hz);
  if (config_.cycle_rate_hz > 0.0) {
    update_period_sec_ = 1.0 / config_.cycle_rate_hz;
  }

  get_dbl("max_range_m", config_.max_range_m);
  get_dbl("min_range_m", config_.min_range_m);
  if (sdf->HasElement("fov_azimuth_rad")) {
    const double fov = sdf->Get<double>("fov_azimuth_rad");
    config_.fov_azimuth_min_rad = -0.5 * fov;
    config_.fov_azimuth_max_rad = 0.5 * fov;
  }
  if (sdf->HasElement("fov_elevation_rad")) {
    const double fov = sdf->Get<double>("fov_elevation_rad");
    config_.fov_elevation_min_rad = -0.5 * fov;
    config_.fov_elevation_max_rad = 0.5 * fov;
  }

  get_dbl("v_max_mps", config_.v_max_mps);
  get_bool("enable_doppler_folding", config_.enable_doppler_folding);
  get_bool("closing_velocity_positive", config_.closing_velocity_positive);

  get_dbl("range_sigma_m", config_.range_sigma_m);
  get_dbl("azimuth_sigma_rad", config_.azimuth_sigma_rad);
  get_dbl("elevation_sigma_rad", config_.elevation_sigma_rad);
  get_dbl("range_rate_sigma_mps", config_.range_rate_sigma_mps);

  get_dbl("range_resolution_m", config_.range_resolution_m);
  get_dbl("azimuth_resolution_rad", config_.azimuth_resolution_rad);
  get_dbl("elevation_resolution_rad", config_.elevation_resolution_rad);
  get_dbl("range_rate_resolution_mps", config_.range_rate_resolution_mps);

  get_dbl("false_detection_rate_per_cycle", config_.false_detection_rate_per_cycle);
  get_dbl("detection_dropout_probability", config_.detection_dropout_probability);
  get_dbl("min_rcs_threshold", config_.min_rcs_threshold_dbm2);
  get_dbl("power_falloff_exponent", config_.power_falloff_exponent);

  get_int("ray_samples_azimuth", ray_samples_azimuth_);
  get_int("ray_samples_elevation", ray_samples_elevation_);
  ray_samples_azimuth_ = std::max(1, ray_samples_azimuth_);
  ray_samples_elevation_ = std::max(1, ray_samples_elevation_);

  if (sdf->HasElement("random_seed")) {
    random_seed_ = static_cast<uint32_t>(sdf->Get<int>("random_seed"));
  }

  // Surface defaults: <default_surface> applies to everything except planes,
  // which use <ground_surface>. Per-model overrides via <surface model="name">.
  auto parse_surface = [](const sdf::ElementPtr& e, rpc::SurfaceProperties& s) {
    if (e->HasElement("reflectivity")) s.reflectivity = e->Get<double>("reflectivity");
    if (e->HasElement("nominal_rcs_dbm2")) s.nominal_rcs_dbm2 = e->Get<double>("nominal_rcs_dbm2");
    if (e->HasElement("incidence_exponent"))
      s.incidence_exponent = e->Get<double>("incidence_exponent");
    if (e->HasElement("classification"))
      s.classification = static_cast<uint8_t>(e->Get<int>("classification"));
  };
  if (sdf->HasElement("default_surface")) parse_surface(sdf->GetElement("default_surface"), default_surface_);
  if (sdf->HasElement("ground_surface")) parse_surface(sdf->GetElement("ground_surface"), ground_surface_);

  if (sdf->HasElement("surface")) {
    for (sdf::ElementPtr e = sdf->GetElement("surface"); e; e = e->GetNextElement("surface")) {
      if (!e->HasAttribute("model")) {
        gzwarn << "[" << sensor_name_ << "] <surface> without model=\"...\" attribute ignored\n";
        continue;
      }
      const std::string model = e->GetAttribute("model")->GetAsString();
      SurfaceOverride o;
      if (e->HasElement("reflectivity")) o.reflectivity = e->Get<double>("reflectivity");
      if (e->HasElement("nominal_rcs_dbm2")) o.nominal_rcs_dbm2 = e->Get<double>("nominal_rcs_dbm2");
      if (e->HasElement("incidence_exponent")) o.incidence_exponent = e->Get<double>("incidence_exponent");
      if (e->HasElement("classification")) o.classification = e->Get<int>("classification");
      surface_overrides_[model] = o;
    }
  }

  config_.frame_id = frame_id_;
  physics_engine_->set_config(config_);
  if (random_seed_) physics_engine_->seed(*random_seed_);
}

bool Gazebo4DRadarPlugin::ResolveSensorLink(gz::sim::EntityComponentManager& ecm) {
  if (ecm.Component<gz::sim::components::Model>(plugin_entity_)) {
    const gz::sim::Model model(plugin_entity_);
    if (!link_name_.empty()) {
      sensor_link_ = model.LinkByName(ecm, link_name_);
      if (sensor_link_ == gz::sim::kNullEntity) {
        gzerr << "[" << sensor_name_ << "] <link_name> '" << link_name_
              << "' not found in model '" << model.Name(ecm) << "'\n";
        return false;
      }
    } else {
      sensor_link_ = model.CanonicalLink(ecm);
    }
  } else if (ecm.Component<gz::sim::components::Link>(plugin_entity_)) {
    sensor_link_ = plugin_entity_;
  }
  if (sensor_link_ == gz::sim::kNullEntity) return false;

  own_model_ = gz::sim::topLevelModel(sensor_link_, ecm);
  // Ask the physics system to publish world-frame velocities for the sensor link.
  gz::sim::Link(sensor_link_).EnableVelocityChecks(ecm, true);
  return true;
}

void Gazebo4DRadarPlugin::InitializeRos() {
  if (!rclcpp::ok()) {
    // Do not install ROS signal handlers: they would swallow Ctrl+C meant for gz sim.
    rclcpp::init(0, nullptr, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  }
  // Only publishers are used, so no executor / spin thread is needed. The node is
  // released with the plugin; the global ROS context is left untouched so other
  // ROS-based systems in the same process keep working.
  ros_node_ = rclcpp::Node::make_shared(sensor_name_ + "_radar_sim");

  const auto qos = rclcpp::SensorDataQoS().keep_last(10).reliable();
  if (publish_radar_scan_) {
    scan_pub_ = ros_node_->create_publisher<radar_msgs::msg::RadarScan>(scan_topic_, qos);
  }
  if (publish_pointcloud_) {
    pointcloud_pub_ = ros_node_->create_publisher<sensor_msgs::msg::PointCloud2>(pointcloud_topic_, qos);
  }
  if (publish_ground_truth_) {
    ground_truth_pub_ =
        ros_node_->create_publisher<geometry_msgs::msg::TwistStamped>(ground_truth_topic_, qos);
  }
  if (publish_tf_) {
    // Same QoS as tf2_ros::TransformBroadcaster.
    tf_pub_ = ros_node_->create_publisher<tf2_msgs::msg::TFMessage>("/tf", rclcpp::QoS(100));
  }
}

// ---------------------------------------------------------------------------
// Scene construction from the ECM
// ---------------------------------------------------------------------------

void Gazebo4DRadarPlugin::PreUpdate(const gz::sim::UpdateInfo& /*info*/,
                                    gz::sim::EntityComponentManager& ecm) {
  if (sensor_link_ == gz::sim::kNullEntity) return;

  std::size_t n = 0;
  ecm.Each<gz::sim::components::Collision>(
      [&](const gz::sim::Entity&, const gz::sim::components::Collision*) -> bool {
        ++n;
        return true;
      });
  if (scene_dirty_ || n != last_collision_count_ || ecm.HasEntitiesMarkedForRemoval()) {
    RebuildScene(ecm);
    last_collision_count_ = n;
    scene_dirty_ = false;
  }
}

void Gazebo4DRadarPlugin::RebuildScene(gz::sim::EntityComponentManager& ecm) {
  scene_.clear();
  tracked_.clear();

  std::vector<gz::sim::Entity> collisions;
  ecm.Each<gz::sim::components::Collision, gz::sim::components::Geometry>(
      [&](const gz::sim::Entity& e, const gz::sim::components::Collision*,
          const gz::sim::components::Geometry*) -> bool {
        collisions.push_back(e);
        return true;
      });

  std::size_t skipped_own = 0;
  for (const auto c : collisions) {
    const gz::sim::Entity link = ecm.ParentEntity(c);
    if (link == gz::sim::kNullEntity) continue;
    const gz::sim::Entity top = gz::sim::topLevelModel(link, ecm);
    if (ignore_own_model_ && top == own_model_) {
      ++skipped_own;
      continue;
    }

    gz::math::Pose3d offset;
    auto shape = MakeShape(ecm, c, offset);
    if (!shape) continue;

    bool is_static = false;
    if (auto st = ecm.Component<gz::sim::components::Static>(top)) is_static = st->Data();

    // Surface properties: defaults by geometry, then per-model overrides.
    rpc::SurfaceProperties surf =
        shape->type == rpc::ShapeType::kPlane ? ground_surface_ : default_surface_;
    std::string model_name;
    if (auto nm = ecm.Component<gz::sim::components::Name>(top)) model_name = nm->Data();
    auto it = surface_overrides_.find(model_name);
    if (it != surface_overrides_.end()) {
      const SurfaceOverride& o = it->second;
      if (o.reflectivity) surf.reflectivity = *o.reflectivity;
      if (o.nominal_rcs_dbm2) surf.nominal_rcs_dbm2 = *o.nominal_rcs_dbm2;
      if (o.incidence_exponent) surf.incidence_exponent = *o.incidence_exponent;
      if (o.classification) surf.classification = static_cast<uint8_t>(*o.classification);
    }
    shape->surface = surf;
    shape->owner_id = c;
    shape->pose = to_rigid(compose(gz::sim::worldPose(c, ecm), offset));

    if (!is_static) gz::sim::Link(link).EnableVelocityChecks(ecm, true);

    scene_.add_shape(*shape);
    tracked_.push_back(TrackedShape{c, link, is_static, offset});
  }

  gzdbg << "[" << sensor_name_ << "] radar scene rebuilt: " << scene_.size()
        << " collision shapes (" << skipped_own << " on own model ignored)\n";
}

std::optional<rpc::SceneShape> Gazebo4DRadarPlugin::MakeShape(
    const gz::sim::EntityComponentManager& ecm, gz::sim::Entity collision,
    gz::math::Pose3d& geometry_offset) {
  geometry_offset = gz::math::Pose3d::Zero;
  const auto* geom_comp = ecm.Component<gz::sim::components::Geometry>(collision);
  if (!geom_comp) return std::nullopt;
  const sdf::Geometry& geom = geom_comp->Data();

  rpc::SceneShape s;
  switch (geom.Type()) {
    case sdf::GeometryType::PLANE: {
      const sdf::Plane* p = geom.PlaneShape();
      if (!p) return std::nullopt;
      s.type = rpc::ShapeType::kPlane;
      s.plane_normal = to_vec(p->Normal()).normalized();
      // Physics engines treat <plane> as infinite; honour that unless it is tiny.
      s.plane_half_x = 0.0;
      s.plane_half_y = 0.0;
      return s;
    }
    case sdf::GeometryType::BOX: {
      const sdf::Box* b = geom.BoxShape();
      if (!b) return std::nullopt;
      s.type = rpc::ShapeType::kBox;
      s.box_half_extents = to_vec(b->Size() * 0.5);
      return s;
    }
    case sdf::GeometryType::SPHERE: {
      const sdf::Sphere* sp = geom.SphereShape();
      if (!sp) return std::nullopt;
      s.type = rpc::ShapeType::kSphere;
      s.radius = sp->Radius();
      return s;
    }
    case sdf::GeometryType::CYLINDER: {
      const sdf::Cylinder* cy = geom.CylinderShape();
      if (!cy) return std::nullopt;
      s.type = rpc::ShapeType::kCylinder;
      s.radius = cy->Radius();
      s.half_length = 0.5 * cy->Length();
      return s;
    }
    case sdf::GeometryType::CAPSULE: {
      // Approximated by a cylinder spanning the full capsule length.
      const sdf::Capsule* ca = geom.CapsuleShape();
      if (!ca) return std::nullopt;
      s.type = rpc::ShapeType::kCylinder;
      s.radius = ca->Radius();
      s.half_length = 0.5 * ca->Length() + ca->Radius();
      return s;
    }
    case sdf::GeometryType::ELLIPSOID: {
      // Approximated by its bounding box.
      const sdf::Ellipsoid* el = geom.EllipsoidShape();
      if (!el) return std::nullopt;
      s.type = rpc::ShapeType::kBox;
      s.box_half_extents = to_vec(el->Radii());
      return s;
    }
    case sdf::GeometryType::MESH: {
      const sdf::Mesh* m = geom.MeshShape();
      if (!m) return std::nullopt;
      const std::string path = gz::sim::asFullPath(m->Uri(), m->FilePath());
      gz::math::Vector3d center;
      auto bvh = LoadMesh(path, m->Submesh(), m->CenterSubmesh(), m->Scale(), center);
      if (!bvh) return std::nullopt;
      s.type = rpc::ShapeType::kMesh;
      s.mesh = bvh;
      return s;
    }
    default: {
      std::string name = "?";
      if (auto nm = ecm.Component<gz::sim::components::Name>(collision)) name = nm->Data();
      gzwarn << "[" << sensor_name_ << "] collision '" << name
             << "' has a geometry type the radar does not support yet (e.g. heightmap, "
                "polyline); it will be invisible to the radar.\n";
      return std::nullopt;
    }
  }
}

std::shared_ptr<const rpc::TriangleMeshBvh> Gazebo4DRadarPlugin::LoadMesh(
    const std::string& path, const std::string& submesh, bool center_submesh,
    const gz::math::Vector3d& scale, gz::math::Vector3d& submesh_center) {
  std::ostringstream key;
  key << path << "|" << submesh << "|" << center_submesh << "|" << scale;
  auto cached = mesh_cache_.find(key.str());
  if (cached != mesh_cache_.end()) {
    submesh_center = mesh_center_cache_[key.str()];
    return cached->second;
  }

  const gz::common::Mesh* mesh = gz::common::MeshManager::Instance()->Load(path);
  if (!mesh) {
    gzerr << "[" << sensor_name_ << "] failed to load collision mesh '" << path
          << "'; it will be invisible to the radar.\n";
    mesh_cache_[key.str()] = nullptr;
    return nullptr;
  }

  std::vector<rpc::Vec3> vertices;
  std::vector<std::array<uint32_t, 3>> triangles;
  submesh_center = gz::math::Vector3d::Zero;

  for (unsigned int i = 0; i < mesh->SubMeshCount(); ++i) {
    auto sm = mesh->SubMeshByIndex(i).lock();
    if (!sm) continue;
    if (!submesh.empty() && sm->Name() != submesh) continue;
    if (sm->SubMeshPrimitiveType() != gz::common::SubMesh::TRIANGLES) continue;

    gz::math::Vector3d c = gz::math::Vector3d::Zero;
    if (!submesh.empty() && center_submesh && sm->VertexCount() > 0) {
      gz::math::Vector3d mn = sm->Vertex(0), mx = sm->Vertex(0);
      for (unsigned int v = 1; v < sm->VertexCount(); ++v) {
        mn.Min(sm->Vertex(v));
        mx.Max(sm->Vertex(v));
      }
      c = (mn + mx) * 0.5;
      submesh_center = c;
    }

    const uint32_t base = static_cast<uint32_t>(vertices.size());
    for (unsigned int v = 0; v < sm->VertexCount(); ++v) {
      const gz::math::Vector3d p = (sm->Vertex(v) - c) * scale;
      vertices.push_back(to_vec(p));
    }
    for (unsigned int k = 0; k + 2 < sm->IndexCount(); k += 3) {
      triangles.push_back({base + static_cast<uint32_t>(sm->Index(k)),
                           base + static_cast<uint32_t>(sm->Index(k + 1)),
                           base + static_cast<uint32_t>(sm->Index(k + 2))});
    }
  }

  if (triangles.empty()) {
    gzwarn << "[" << sensor_name_ << "] mesh '" << path << "' has no triangles"
           << (submesh.empty() ? "" : " in submesh '" + submesh + "'") << "\n";
    mesh_cache_[key.str()] = nullptr;
    return nullptr;
  }

  auto bvh = std::make_shared<const rpc::TriangleMeshBvh>(std::move(vertices), std::move(triangles));
  gzmsg << "[" << sensor_name_ << "] radar loaded mesh '" << path << "' ("
        << bvh->triangle_count() << " triangles)\n";
  mesh_cache_[key.str()] = bvh;
  mesh_center_cache_[key.str()] = submesh_center;
  return bvh;
}

// ---------------------------------------------------------------------------
// Per-cycle update
// ---------------------------------------------------------------------------

void Gazebo4DRadarPlugin::UpdateDynamicShapes(const gz::sim::EntityComponentManager& ecm) {
  for (std::size_t i = 0; i < tracked_.size(); ++i) {
    const TrackedShape& t = tracked_[i];
    if (t.is_static) continue;
    rpc::SceneShape& s = scene_.shape(i);
    s.pose = to_rigid(compose(gz::sim::worldPose(t.collision, ecm), t.geometry_offset));
    s.velocity_origin = to_vec(gz::sim::worldPose(t.link, ecm).Pos());
    s.linear_velocity = {0, 0, 0};
    s.angular_velocity = {0, 0, 0};
    if (auto v = ecm.Component<gz::sim::components::WorldLinearVelocity>(t.link)) {
      s.linear_velocity = to_vec(v->Data());
    }
    if (auto w = ecm.Component<gz::sim::components::WorldAngularVelocity>(t.link)) {
      s.angular_velocity = to_vec(w->Data());
    }
  }
}

bool Gazebo4DRadarPlugin::SensorState(const gz::sim::UpdateInfo& info,
                                      const gz::sim::EntityComponentManager& ecm,
                                      gz::math::Pose3d& sensor_pose,
                                      gz::math::Vector3d& sensor_vel_world,
                                      gz::math::Vector3d& sensor_ang_vel_world) {
  const gz::math::Pose3d link_pose = gz::sim::worldPose(sensor_link_, ecm);
  sensor_pose = compose(link_pose, mounting_pose_);

  const auto* lin = ecm.Component<gz::sim::components::WorldLinearVelocity>(sensor_link_);
  const auto* ang = ecm.Component<gz::sim::components::WorldAngularVelocity>(sensor_link_);
  const double now = sim_seconds(info);

  sensor_ang_vel_world = ang ? ang->Data() : gz::math::Vector3d::Zero;
  if (lin) {
    // Velocity of the radar origin = link origin velocity + w x lever arm.
    const gz::math::Vector3d lever_world = link_pose.Rot().RotateVector(mounting_pose_.Pos());
    sensor_vel_world = lin->Data() + sensor_ang_vel_world.Cross(lever_world);
  } else if (prev_sensor_pos_ && now > prev_sensor_time_sec_) {
    sensor_vel_world = (sensor_pose.Pos() - *prev_sensor_pos_) / (now - prev_sensor_time_sec_);
  } else {
    sensor_vel_world = gz::math::Vector3d::Zero;
  }
  prev_sensor_pos_ = sensor_pose.Pos();
  prev_sensor_time_sec_ = now;
  return true;
}

void Gazebo4DRadarPlugin::PostUpdate(const gz::sim::UpdateInfo& info,
                                     const gz::sim::EntityComponentManager& ecm) {
  if (info.paused || sensor_link_ == gz::sim::kNullEntity || !ros_node_) return;

  const double now = sim_seconds(info);
  if (last_update_time_sec_ && now - *last_update_time_sec_ < update_period_sec_ - 1e-9) {
    return;
  }
  // Keep a fixed cadence even if the physics step does not divide the period.
  last_update_time_sec_ =
      last_update_time_sec_ ? *last_update_time_sec_ + update_period_sec_ : now;
  if (now - *last_update_time_sec_ > update_period_sec_) last_update_time_sec_ = now;

  PerformRadarScan(info, ecm);
}

void Gazebo4DRadarPlugin::PerformRadarScan(const gz::sim::UpdateInfo& info,
                                           const gz::sim::EntityComponentManager& ecm) {
  UpdateDynamicShapes(ecm);

  gz::math::Pose3d sensor_pose;
  gz::math::Vector3d v_sensor, w_sensor;
  SensorState(info, ecm, sensor_pose, v_sensor, w_sensor);

  const rpc::Vec3 origin = to_vec(sensor_pose.Pos());
  const rpc::Vec3 v_s = to_vec(v_sensor);
  const gz::math::Quaterniond& q = sensor_pose.Rot();

  const double sensor_pos_raw[3] = {origin.x, origin.y, origin.z};
  const gz::math::Matrix3d w2s(q.Inverse());
  const double rot_raw[9] = {w2s(0, 0), w2s(0, 1), w2s(0, 2), w2s(1, 0), w2s(1, 1),
                             w2s(1, 2), w2s(2, 0), w2s(2, 1), w2s(2, 2)};

  std::vector<rpc::RadarDetection> detections;
  detections.reserve(static_cast<std::size_t>(ray_samples_azimuth_ * ray_samples_elevation_) / 4);
  uint16_t measurement_id = 0;

  const double az_span = config_.fov_azimuth_max_rad - config_.fov_azimuth_min_rad;
  const double el_span = config_.fov_elevation_max_rad - config_.fov_elevation_min_rad;
  const double az_step = ray_samples_azimuth_ > 1 ? az_span / (ray_samples_azimuth_ - 1) : 0.0;
  const double el_step = ray_samples_elevation_ > 1 ? el_span / (ray_samples_elevation_ - 1) : 0.0;
  const double az0 = ray_samples_azimuth_ > 1 ? config_.fov_azimuth_min_rad : 0.5 *
      (config_.fov_azimuth_min_rad + config_.fov_azimuth_max_rad);
  const double el0 = ray_samples_elevation_ > 1 ? config_.fov_elevation_min_rad : 0.5 *
      (config_.fov_elevation_min_rad + config_.fov_elevation_max_rad);

  for (int i_el = 0; i_el < ray_samples_elevation_; ++i_el) {
    const double el = el0 + i_el * el_step;
    for (int i_az = 0; i_az < ray_samples_azimuth_; ++i_az) {
      const double az = az0 + i_az * az_step;
      const gz::math::Vector3d dir_s(std::cos(el) * std::cos(az), std::cos(el) * std::sin(az),
                                     std::sin(el));
      const rpc::Vec3 dir = to_vec(q.RotateVector(dir_s)).normalized();

      auto hit = scene_.cast(origin, dir, config_.min_range_m, config_.max_range_m);
      if (!hit) continue;

      const rpc::SceneShape& shape = scene_.shape(hit->shape_index);
      rpc::RawRayHit raw;
      raw.hit_point_world[0] = hit->point.x;
      raw.hit_point_world[1] = hit->point.y;
      raw.hit_point_world[2] = hit->point.z;
      raw.hit_normal_world[0] = hit->normal.x;
      raw.hit_normal_world[1] = hit->normal.y;
      raw.hit_normal_world[2] = hit->normal.z;
      raw.surface_reflectivity = shape.surface.reflectivity;
      raw.nominal_rcs_dbm2 = shape.surface.nominal_rcs_dbm2;
      raw.incidence_exponent = shape.surface.incidence_exponent;
      raw.classification = shape.surface.classification;

      // Range-rate is driven by the velocity of the sensor origin relative to the
      // scatterer; static scatterers have zero velocity.
      const rpc::Vec3 rel = v_s - shape.point_velocity(hit->point);
      const double rel_raw[3] = {rel.x, rel.y, rel.z};

      auto det = physics_engine_->process_ray_hit(sensor_pos_raw, rot_raw, rel_raw, raw,
                                                  measurement_id);
      if (det) {
        detections.push_back(*det);
        ++measurement_id;
      }
    }
  }

  auto false_dets = physics_engine_->generate_false_detections(measurement_id);
  detections.insert(detections.end(), false_dets.begin(), false_dets.end());

  PublishRosMessages(detections, info, sensor_pose, v_sensor, w_sensor);
}

void Gazebo4DRadarPlugin::PublishRosMessages(const std::vector<rpc::RadarDetection>& detections,
                                             const gz::sim::UpdateInfo& info,
                                             const gz::math::Pose3d& sensor_pose,
                                             const gz::math::Vector3d& sensor_vel_world,
                                             const gz::math::Vector3d& sensor_ang_vel_world) {
  std_msgs::msg::Header header;
  const int64_t ns = std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
  header.stamp.sec = static_cast<int32_t>(ns / 1000000000LL);
  header.stamp.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
  header.frame_id = frame_id_;

  if (scan_pub_) {
    radar_msgs::msg::RadarScan scan;
    scan.header = header;
    scan.returns.reserve(detections.size());
    for (const auto& d : detections) {
      radar_msgs::msg::RadarReturn r;
      r.range = static_cast<float>(d.range_m);
      r.azimuth = static_cast<float>(d.azimuth_rad);
      r.elevation = static_cast<float>(d.elevation_rad);
      r.doppler_velocity = static_cast<float>(d.range_rate_mps);
      r.amplitude = static_cast<float>(d.rcs_dbm2);
      scan.returns.push_back(r);
    }
    scan_pub_->publish(scan);
  }

  if (pointcloud_pub_) {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = header;
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.is_bigendian = false;
    sensor_msgs::PointCloud2Modifier mod(cloud);
    mod.setPointCloud2Fields(
        8, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
        sensor_msgs::msg::PointField::FLOAT32, "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "doppler", 1, sensor_msgs::msg::PointField::FLOAT32, "range", 1,
        sensor_msgs::msg::PointField::FLOAT32, "azimuth", 1, sensor_msgs::msg::PointField::FLOAT32,
        "elevation", 1, sensor_msgs::msg::PointField::FLOAT32, "rcs", 1,
        sensor_msgs::msg::PointField::FLOAT32);
    mod.resize(detections.size());
    sensor_msgs::PointCloud2Iterator<float> ix(cloud, "x"), iy(cloud, "y"), iz(cloud, "z"),
        idop(cloud, "doppler"), ir(cloud, "range"), iaz(cloud, "azimuth"),
        iel(cloud, "elevation"), ircs(cloud, "rcs");
    for (const auto& d : detections) {
      *ix = static_cast<float>(d.x_m);
      *iy = static_cast<float>(d.y_m);
      *iz = static_cast<float>(d.z_m);
      *idop = static_cast<float>(d.range_rate_mps);
      *ir = static_cast<float>(d.range_m);
      *iaz = static_cast<float>(d.azimuth_rad);
      *iel = static_cast<float>(d.elevation_rad);
      *ircs = static_cast<float>(d.rcs_dbm2);
      ++ix; ++iy; ++iz; ++idop; ++ir; ++iaz; ++iel; ++ircs;
    }
    pointcloud_pub_->publish(cloud);
  }

  if (tf_pub_) {
    tf2_msgs::msg::TFMessage tf;
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = header.stamp;
    t.header.frame_id = tf_parent_frame_;
    t.child_frame_id = frame_id_;
    t.transform.translation.x = sensor_pose.Pos().X();
    t.transform.translation.y = sensor_pose.Pos().Y();
    t.transform.translation.z = sensor_pose.Pos().Z();
    t.transform.rotation.w = sensor_pose.Rot().W();
    t.transform.rotation.x = sensor_pose.Rot().X();
    t.transform.rotation.y = sensor_pose.Rot().Y();
    t.transform.rotation.z = sensor_pose.Rot().Z();
    tf.transforms.push_back(t);
    tf_pub_->publish(tf);
  }

  if (ground_truth_pub_) {
    // True velocity of the radar origin, expressed in the radar frame — the
    // quantity an ego-velocity estimator should recover from the Doppler returns.
    const gz::math::Quaterniond q_inv = sensor_pose.Rot().Inverse();
    const gz::math::Vector3d v = q_inv.RotateVector(sensor_vel_world);
    const gz::math::Vector3d w = q_inv.RotateVector(sensor_ang_vel_world);
    geometry_msgs::msg::TwistStamped gt;
    gt.header = header;
    gt.twist.linear.x = v.X();
    gt.twist.linear.y = v.Y();
    gt.twist.linear.z = v.Z();
    gt.twist.angular.x = w.X();
    gt.twist.angular.y = w.Y();
    gt.twist.angular.z = w.Z();
    ground_truth_pub_->publish(gt);
  }
}

}  // namespace gazebo_4d_radar_plugin

GZ_ADD_PLUGIN(gazebo_4d_radar_plugin::Gazebo4DRadarPlugin, gz::sim::System,
              gazebo_4d_radar_plugin::Gazebo4DRadarPlugin::ISystemConfigure,
              gazebo_4d_radar_plugin::Gazebo4DRadarPlugin::ISystemPreUpdate,
              gazebo_4d_radar_plugin::Gazebo4DRadarPlugin::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(gazebo_4d_radar_plugin::Gazebo4DRadarPlugin,
                    "gazebo_4d_radar_plugin::Gazebo4DRadarPlugin")
