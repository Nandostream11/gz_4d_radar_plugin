#ifndef RADAR_PHYSICS_CORE_SCENE_RAYCASTER_HPP_
#define RADAR_PHYSICS_CORE_SCENE_RAYCASTER_HPP_

#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

namespace radar_physics_core {

/// Minimal 3-vector used by the geometry layer (kept free of Gazebo types so the
/// ray caster can be unit-tested without a simulator).
struct Vec3 {
  double x{0.0}, y{0.0}, z{0.0};

  Vec3() = default;
  Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

  Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
  Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
  Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
  Vec3 operator-() const { return {-x, -y, -z}; }
  double operator[](int i) const { return i == 0 ? x : (i == 1 ? y : z); }
  double dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
  Vec3 cross(const Vec3& o) const {
    return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x};
  }
  double norm() const { return std::sqrt(dot(*this)); }
  Vec3 normalized() const {
    const double n = norm();
    return n > 1e-12 ? (*this) * (1.0 / n) : Vec3{0.0, 0.0, 0.0};
  }
};

/// Rigid transform. `R` is the row-major rotation from the local (shape) frame to
/// the world frame, `p` is the local origin expressed in the world frame.
struct RigidPose {
  Vec3 p{};
  std::array<double, 9> R{1, 0, 0, 0, 1, 0, 0, 0, 1};

  Vec3 rotate(const Vec3& v) const {
    return {R[0] * v.x + R[1] * v.y + R[2] * v.z,
            R[3] * v.x + R[4] * v.y + R[5] * v.z,
            R[6] * v.x + R[7] * v.y + R[8] * v.z};
  }
  Vec3 inverse_rotate(const Vec3& v) const {
    return {R[0] * v.x + R[3] * v.y + R[6] * v.z,
            R[1] * v.x + R[4] * v.y + R[7] * v.z,
            R[2] * v.x + R[5] * v.y + R[8] * v.z};
  }
  Vec3 to_world(const Vec3& local) const { return rotate(local) + p; }
  Vec3 to_local(const Vec3& world) const { return inverse_rotate(world - p); }
};

/// Radar-relevant surface description attached to every shape in the scene.
struct SurfaceProperties {
  double reflectivity{1.0};        ///< Material reflection coefficient [0..1].
  double nominal_rcs_dbm2{10.0};   ///< Base return strength of a scatterer on this surface.
  /// Exponent n applied to cos(incidence): 1 = diffuse/rough (Lambertian),
  /// larger values = smoother, more specular surfaces (water, asphalt, glass)
  /// that return little energy at oblique incidence.
  double incidence_exponent{1.0};
  uint8_t classification{0};
};

/// Static triangle mesh with a bounding-volume hierarchy. Vertices are in the
/// mesh's local frame (any scale already applied).
class TriangleMeshBvh {
 public:
  TriangleMeshBvh(std::vector<Vec3> vertices, std::vector<std::array<uint32_t, 3>> triangles);

  /// Nearest intersection with the local-frame ray. Returns (t, triangle index).
  std::optional<std::pair<double, uint32_t>> intersect(
      const Vec3& origin, const Vec3& dir, double t_min, double t_max) const;

  Vec3 triangle_normal(uint32_t tri) const;
  std::size_t triangle_count() const { return triangles_.size(); }
  std::size_t vertex_count() const { return vertices_.size(); }

 private:
  struct Node {
    Vec3 bmin, bmax;
    uint32_t left{0}, right{0};    // child node indices (internal nodes)
    uint32_t first{0}, count{0};   // triangle range in order_ (leaf when count > 0)
  };

  uint32_t build(uint32_t first, uint32_t count, int depth);

  std::vector<Vec3> vertices_;
  std::vector<std::array<uint32_t, 3>> triangles_;
  std::vector<Vec3> centroids_;
  std::vector<uint32_t> order_;
  std::vector<Node> nodes_;
};

enum class ShapeType { kPlane, kBox, kSphere, kCylinder, kMesh };

/// One collision shape in the scene, with its current pose and motion.
struct SceneShape {
  ShapeType type{ShapeType::kBox};
  RigidPose pose{};

  // Geometry parameters (local frame).
  Vec3 plane_normal{0, 0, 1};     ///< kPlane: unit normal.
  double plane_half_x{0.0};       ///< kPlane: half extents along the in-plane
  double plane_half_y{0.0};       ///<         basis; <= 0 means unbounded.
  Vec3 box_half_extents{0.5, 0.5, 0.5};
  double radius{0.5};             ///< kSphere / kCylinder.
  double half_length{0.5};        ///< kCylinder (along local z).
  std::shared_ptr<const TriangleMeshBvh> mesh;  ///< kMesh.

  SurfaceProperties surface{};

  /// Rigid-body motion of the link that owns this shape (world frame).
  Vec3 linear_velocity{0, 0, 0};   ///< Velocity of `velocity_origin`.
  Vec3 angular_velocity{0, 0, 0};
  Vec3 velocity_origin{0, 0, 0};   ///< Usually the owning link's origin.

  uint64_t owner_id{0};            ///< Opaque id (e.g. Gazebo collision entity).

  /// Velocity of the material point `p_world` on this shape.
  Vec3 point_velocity(const Vec3& p_world) const {
    return linear_velocity + angular_velocity.cross(p_world - velocity_origin);
  }
};

struct SceneHit {
  double t{0.0};          ///< Distance along the (unit) ray.
  Vec3 point{};           ///< World-frame hit point.
  Vec3 normal{};          ///< World-frame unit normal, oriented against the ray.
  std::size_t shape_index{0};
};

/// CPU ray caster over a set of analytic primitives and meshes.
class SceneRaycaster {
 public:
  std::size_t add_shape(const SceneShape& shape);
  SceneShape& shape(std::size_t i) { return shapes_[i]; }
  const SceneShape& shape(std::size_t i) const { return shapes_[i]; }
  std::size_t size() const { return shapes_.size(); }
  void clear() { shapes_.clear(); }

  /// Nearest hit of the world-frame ray origin + t*dir, t in [t_min, t_max].
  /// `dir` must be unit length.
  std::optional<SceneHit> cast(const Vec3& origin, const Vec3& dir,
                               double t_min, double t_max) const;

 private:
  std::vector<SceneShape> shapes_;
};

/// Local-frame single-shape intersection (exposed for tests).
/// Returns t and the local-frame (unoriented) normal.
std::optional<std::pair<double, Vec3>> intersect_shape_local(
    const SceneShape& s, const Vec3& o, const Vec3& d, double t_min, double t_max);

/// Velocity of a sensor whose origin is offset `mount_offset_world` (world
/// frame vector from the link origin to the sensor origin) from a rigid link
/// moving with `link_linear_velocity` / `link_angular_velocity`.
inline Vec3 sensor_origin_velocity(const Vec3& link_linear_velocity,
                                   const Vec3& link_angular_velocity,
                                   const Vec3& mount_offset_world) {
  return link_linear_velocity + link_angular_velocity.cross(mount_offset_world);
}

}  // namespace radar_physics_core

#endif  // RADAR_PHYSICS_CORE_SCENE_RAYCASTER_HPP_
