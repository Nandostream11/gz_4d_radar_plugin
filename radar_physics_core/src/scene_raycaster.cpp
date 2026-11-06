#include "radar_physics_core/scene_raycaster.hpp"

#include <algorithm>
#include <limits>
#include <numeric>

namespace radar_physics_core {

namespace {

constexpr double kEps = 1e-12;

// In-plane basis for a plane with unit normal n, matching the convention of
// rotating local +z onto n with the minimal rotation (as Gazebo does when it
// orients a plane's <size> rectangle).
void plane_basis(const Vec3& n, Vec3& u, Vec3& v) {
  const Vec3 z{0, 0, 1};
  const double c = z.dot(n);
  if (c > 1.0 - 1e-9) {
    u = {1, 0, 0};
    v = {0, 1, 0};
    return;
  }
  if (c < -1.0 + 1e-9) {
    u = {1, 0, 0};
    v = {0, -1, 0};
    return;
  }
  // Rodrigues rotation of x and y about axis k = (z x n)/|z x n| by acos(c).
  const Vec3 k = z.cross(n).normalized();
  const double s = std::sqrt(std::max(0.0, 1.0 - c * c));
  auto rot = [&](const Vec3& a) {
    return a * c + k.cross(a) * s + k * (k.dot(a) * (1.0 - c));
  };
  u = rot({1, 0, 0});
  v = rot({0, 1, 0});
}

bool slab(const Vec3& bmin, const Vec3& bmax, const Vec3& o, const Vec3& inv_d,
          double t_min, double t_max, double& t_enter, double& t_exit) {
  double t0 = t_min, t1 = t_max;
  for (int a = 0; a < 3; ++a) {
    double tn = (bmin[a] - o[a]) * inv_d[a];
    double tf = (bmax[a] - o[a]) * inv_d[a];
    if (std::isnan(tn) || std::isnan(tf)) {
      // Ray parallel to the slab and exactly on its boundary: treat as inside.
      if (o[a] < bmin[a] || o[a] > bmax[a]) return false;
      continue;
    }
    if (tn > tf) std::swap(tn, tf);
    t0 = std::max(t0, tn);
    t1 = std::min(t1, tf);
    if (t0 > t1) return false;
  }
  t_enter = t0;
  t_exit = t1;
  return true;
}

Vec3 safe_inverse(const Vec3& d) {
  auto inv = [](double x) {
    return std::abs(x) > kEps ? 1.0 / x : std::copysign(std::numeric_limits<double>::infinity(), x);
  };
  return {inv(d.x), inv(d.y), inv(d.z)};
}

// Möller–Trumbore, two-sided.
bool ray_triangle(const Vec3& o, const Vec3& d, const Vec3& a, const Vec3& b,
                  const Vec3& c, double& t) {
  const Vec3 e1 = b - a;
  const Vec3 e2 = c - a;
  const Vec3 p = d.cross(e2);
  const double det = e1.dot(p);
  if (std::abs(det) < 1e-14) return false;
  const double inv_det = 1.0 / det;
  const Vec3 s = o - a;
  const double u = s.dot(p) * inv_det;
  if (u < 0.0 || u > 1.0) return false;
  const Vec3 q = s.cross(e1);
  const double v = d.dot(q) * inv_det;
  if (v < 0.0 || u + v > 1.0) return false;
  t = e2.dot(q) * inv_det;
  return true;
}

}  // namespace

// ---------------------------------------------------------------------------
// TriangleMeshBvh
// ---------------------------------------------------------------------------

TriangleMeshBvh::TriangleMeshBvh(std::vector<Vec3> vertices,
                                 std::vector<std::array<uint32_t, 3>> triangles)
    : vertices_(std::move(vertices)) {
  // Drop degenerate / out-of-range triangles up front.
  triangles_.reserve(triangles.size());
  for (const auto& t : triangles) {
    if (t[0] >= vertices_.size() || t[1] >= vertices_.size() || t[2] >= vertices_.size()) {
      continue;
    }
    const Vec3 n = (vertices_[t[1]] - vertices_[t[0]]).cross(vertices_[t[2]] - vertices_[t[0]]);
    if (n.norm() < 1e-14) continue;
    triangles_.push_back(t);
  }

  centroids_.resize(triangles_.size());
  for (std::size_t i = 0; i < triangles_.size(); ++i) {
    const auto& t = triangles_[i];
    centroids_[i] = (vertices_[t[0]] + vertices_[t[1]] + vertices_[t[2]]) * (1.0 / 3.0);
  }
  order_.resize(triangles_.size());
  std::iota(order_.begin(), order_.end(), 0u);
  nodes_.reserve(2 * triangles_.size() + 1);
  if (!triangles_.empty()) {
    build(0, static_cast<uint32_t>(triangles_.size()), 0);
  }
}

uint32_t TriangleMeshBvh::build(uint32_t first, uint32_t count, int depth) {
  const uint32_t idx = static_cast<uint32_t>(nodes_.size());
  nodes_.push_back(Node{});

  Vec3 bmin{std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
            std::numeric_limits<double>::max()};
  Vec3 bmax = -bmin;
  Vec3 cmin = bmin, cmax = bmax;
  for (uint32_t i = first; i < first + count; ++i) {
    const auto& t = triangles_[order_[i]];
    for (int k = 0; k < 3; ++k) {
      const Vec3& v = vertices_[t[k]];
      bmin = {std::min(bmin.x, v.x), std::min(bmin.y, v.y), std::min(bmin.z, v.z)};
      bmax = {std::max(bmax.x, v.x), std::max(bmax.y, v.y), std::max(bmax.z, v.z)};
    }
    const Vec3& c = centroids_[order_[i]];
    cmin = {std::min(cmin.x, c.x), std::min(cmin.y, c.y), std::min(cmin.z, c.z)};
    cmax = {std::max(cmax.x, c.x), std::max(cmax.y, c.y), std::max(cmax.z, c.z)};
  }
  // Pad flat boxes slightly so slab tests on axis-aligned planar meshes are robust.
  const double pad = 1e-9;
  bmin = bmin - Vec3{pad, pad, pad};
  bmax = bmax + Vec3{pad, pad, pad};
  nodes_[idx].bmin = bmin;
  nodes_[idx].bmax = bmax;

  const Vec3 ext = cmax - cmin;
  int axis = 0;
  if (ext.y > ext.x) axis = 1;
  if (ext.z > ext[axis]) axis = 2;

  if (count <= 4 || depth > 48 || ext[axis] < 1e-12) {
    nodes_[idx].first = first;
    nodes_[idx].count = count;
    return idx;
  }

  const uint32_t mid = first + count / 2;
  std::nth_element(order_.begin() + first, order_.begin() + mid, order_.begin() + first + count,
                   [&](uint32_t a, uint32_t b) { return centroids_[a][axis] < centroids_[b][axis]; });

  const uint32_t left = build(first, mid - first, depth + 1);
  const uint32_t right = build(mid, first + count - mid, depth + 1);
  nodes_[idx].left = left;
  nodes_[idx].right = right;
  nodes_[idx].count = 0;
  return idx;
}

std::optional<std::pair<double, uint32_t>> TriangleMeshBvh::intersect(
    const Vec3& o, const Vec3& d, double t_min, double t_max) const {
  if (nodes_.empty()) return std::nullopt;
  const Vec3 inv_d = safe_inverse(d);

  double best_t = t_max;
  uint32_t best_tri = 0;
  bool found = false;

  uint32_t stack[128];
  int sp = 0;
  stack[sp++] = 0;
  while (sp > 0) {
    const Node& n = nodes_[stack[--sp]];
    double te, tx;
    if (!slab(n.bmin, n.bmax, o, inv_d, t_min, best_t, te, tx)) continue;
    if (n.count > 0) {
      for (uint32_t i = n.first; i < n.first + n.count; ++i) {
        const uint32_t tri = order_[i];
        const auto& t = triangles_[tri];
        double th;
        if (ray_triangle(o, d, vertices_[t[0]], vertices_[t[1]], vertices_[t[2]], th) &&
            th >= t_min && th < best_t) {
          best_t = th;
          best_tri = tri;
          found = true;
        }
      }
    } else if (sp + 2 <= 128) {
      stack[sp++] = n.left;
      stack[sp++] = n.right;
    }
  }
  if (!found) return std::nullopt;
  return std::make_pair(best_t, best_tri);
}

Vec3 TriangleMeshBvh::triangle_normal(uint32_t tri) const {
  const auto& t = triangles_[tri];
  return (vertices_[t[1]] - vertices_[t[0]]).cross(vertices_[t[2]] - vertices_[t[0]]).normalized();
}

// ---------------------------------------------------------------------------
// Primitive intersection (local frame)
// ---------------------------------------------------------------------------

std::optional<std::pair<double, Vec3>> intersect_shape_local(
    const SceneShape& s, const Vec3& o, const Vec3& d, double t_min, double t_max) {
  switch (s.type) {
    case ShapeType::kPlane: {
      const Vec3& n = s.plane_normal;
      const double denom = n.dot(d);
      if (std::abs(denom) < kEps) return std::nullopt;
      const double t = -n.dot(o) / denom;
      if (t < t_min || t > t_max) return std::nullopt;
      if (s.plane_half_x > 0.0 && s.plane_half_y > 0.0) {
        Vec3 u, v;
        plane_basis(n, u, v);
        const Vec3 p = o + d * t;
        if (std::abs(p.dot(u)) > s.plane_half_x || std::abs(p.dot(v)) > s.plane_half_y) {
          return std::nullopt;
        }
      }
      return std::make_pair(t, n);
    }

    case ShapeType::kBox: {
      const Vec3 h = s.box_half_extents;
      const Vec3 inv_d = safe_inverse(d);
      double te, tx;
      if (!slab(-h, h, o, inv_d, -std::numeric_limits<double>::infinity(),
                std::numeric_limits<double>::infinity(), te, tx)) {
        return std::nullopt;
      }
      double t;
      if (te >= t_min && te <= t_max) {
        t = te;
      } else if (tx >= t_min && tx <= t_max) {
        t = tx;  // ray starts inside the box
      } else {
        return std::nullopt;
      }
      const Vec3 p = o + d * t;
      // Normal = axis whose face the point lies on (largest normalised coordinate).
      int axis = 0;
      double best = -1.0;
      for (int a = 0; a < 3; ++a) {
        const double r = h[a] > kEps ? std::abs(p[a]) / h[a] : 0.0;
        if (r > best) {
          best = r;
          axis = a;
        }
      }
      Vec3 nrm{0, 0, 0};
      const double sign = p[axis] >= 0.0 ? 1.0 : -1.0;
      if (axis == 0) nrm.x = sign;
      if (axis == 1) nrm.y = sign;
      if (axis == 2) nrm.z = sign;
      return std::make_pair(t, nrm);
    }

    case ShapeType::kSphere: {
      const double b = o.dot(d);
      const double c = o.dot(o) - s.radius * s.radius;
      const double disc = b * b - c;
      if (disc < 0.0) return std::nullopt;
      const double sq = std::sqrt(disc);
      double t = -b - sq;
      if (t < t_min) t = -b + sq;
      if (t < t_min || t > t_max) return std::nullopt;
      return std::make_pair(t, (o + d * t).normalized());
    }

    case ShapeType::kCylinder: {
      const double r = s.radius;
      const double hl = s.half_length;
      double best_t = std::numeric_limits<double>::infinity();
      Vec3 best_n{};
      // Side wall.
      const double a = d.x * d.x + d.y * d.y;
      if (a > kEps) {
        const double b = o.x * d.x + o.y * d.y;
        const double c = o.x * o.x + o.y * o.y - r * r;
        const double disc = b * b - a * c;
        if (disc >= 0.0) {
          const double sq = std::sqrt(disc);
          for (double t : {(-b - sq) / a, (-b + sq) / a}) {
            if (t < t_min || t > t_max || t >= best_t) continue;
            const Vec3 p = o + d * t;
            if (std::abs(p.z) <= hl) {
              best_t = t;
              best_n = Vec3{p.x, p.y, 0.0}.normalized();
            }
          }
        }
      }
      // Caps.
      if (std::abs(d.z) > kEps) {
        for (double zc : {hl, -hl}) {
          const double t = (zc - o.z) / d.z;
          if (t < t_min || t > t_max || t >= best_t) continue;
          const Vec3 p = o + d * t;
          if (p.x * p.x + p.y * p.y <= r * r) {
            best_t = t;
            best_n = {0.0, 0.0, zc > 0 ? 1.0 : -1.0};
          }
        }
      }
      if (!std::isfinite(best_t)) return std::nullopt;
      return std::make_pair(best_t, best_n);
    }

    case ShapeType::kMesh: {
      if (!s.mesh) return std::nullopt;
      auto hit = s.mesh->intersect(o, d, t_min, t_max);
      if (!hit) return std::nullopt;
      return std::make_pair(hit->first, s.mesh->triangle_normal(hit->second));
    }
  }
  return std::nullopt;
}

// ---------------------------------------------------------------------------
// SceneRaycaster
// ---------------------------------------------------------------------------

std::size_t SceneRaycaster::add_shape(const SceneShape& shape) {
  shapes_.push_back(shape);
  return shapes_.size() - 1;
}

std::optional<SceneHit> SceneRaycaster::cast(const Vec3& origin, const Vec3& dir,
                                             double t_min, double t_max) const {
  std::optional<SceneHit> best;
  double best_t = t_max;
  for (std::size_t i = 0; i < shapes_.size(); ++i) {
    const SceneShape& s = shapes_[i];
    // Rigid transforms preserve length, so t is identical in local and world frames.
    const Vec3 o_l = s.pose.to_local(origin);
    const Vec3 d_l = s.pose.inverse_rotate(dir);
    auto hit = intersect_shape_local(s, o_l, d_l, t_min, best_t);
    if (!hit || hit->first > best_t) continue;
    best_t = hit->first;
    SceneHit h;
    h.t = hit->first;
    h.point = origin + dir * hit->first;
    Vec3 n_w = s.pose.rotate(hit->second).normalized();
    if (n_w.dot(dir) > 0.0) n_w = -n_w;  // orient against the incoming ray
    h.normal = n_w;
    h.shape_index = i;
    best = h;
  }
  return best;
}

}  // namespace radar_physics_core
