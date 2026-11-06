#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <random>

#include "radar_physics_core/scene_raycaster.hpp"

using namespace radar_physics_core;

namespace {

RigidPose yaw_pose(double x, double y, double z, double yaw) {
  RigidPose p;
  p.p = {x, y, z};
  const double c = std::cos(yaw), s = std::sin(yaw);
  p.R = {c, -s, 0, s, c, 0, 0, 0, 1};
  return p;
}

// Axis-aligned unit square in the z=0 plane split into two triangles, tiled n x n
// over [-half, half]^2 with a height function.
std::shared_ptr<TriangleMeshBvh> make_grid(int n, double half, double (*h)(double, double)) {
  std::vector<Vec3> v;
  std::vector<std::array<uint32_t, 3>> t;
  for (int j = 0; j <= n; ++j) {
    for (int i = 0; i <= n; ++i) {
      const double x = -half + 2.0 * half * i / n;
      const double y = -half + 2.0 * half * j / n;
      v.push_back({x, y, h(x, y)});
    }
  }
  auto id = [n](int i, int j) { return static_cast<uint32_t>(j * (n + 1) + i); };
  for (int j = 0; j < n; ++j) {
    for (int i = 0; i < n; ++i) {
      t.push_back({id(i, j), id(i + 1, j), id(i + 1, j + 1)});
      t.push_back({id(i, j), id(i + 1, j + 1), id(i, j + 1)});
    }
  }
  return std::make_shared<TriangleMeshBvh>(std::move(v), std::move(t));
}

double flat(double, double) { return 0.0; }
double slope(double x, double) { return 0.25 * x; }  // rises along +x

}  // namespace

TEST(SceneRaycaster, PlaneHitAndNormalOrientation) {
  SceneRaycaster scene;
  SceneShape ground;
  ground.type = ShapeType::kPlane;
  scene.add_shape(ground);

  const Vec3 o{0, 0, 2};
  const Vec3 d = Vec3{1, 0, -1}.normalized();
  auto hit = scene.cast(o, d, 0.0, 100.0);
  ASSERT_TRUE(hit.has_value());
  EXPECT_NEAR(hit->point.x, 2.0, 1e-9);
  EXPECT_NEAR(hit->point.z, 0.0, 1e-9);
  EXPECT_NEAR(hit->t, 2.0 * std::sqrt(2.0), 1e-9);
  EXPECT_NEAR(hit->normal.z, 1.0, 1e-9);

  // Looking up never hits a ground plane below.
  EXPECT_FALSE(scene.cast(o, Vec3{0, 0, 1}, 0.0, 100.0).has_value());
}

TEST(SceneRaycaster, BoundedPlaneRespectsSize) {
  SceneRaycaster scene;
  SceneShape ground;
  ground.type = ShapeType::kPlane;
  ground.plane_half_x = 5.0;
  ground.plane_half_y = 5.0;
  scene.add_shape(ground);
  EXPECT_TRUE(scene.cast({4, 0, 1}, {0, 0, -1}, 0.0, 10.0).has_value());
  EXPECT_FALSE(scene.cast({6, 0, 1}, {0, 0, -1}, 0.0, 10.0).has_value());
}

TEST(SceneRaycaster, RotatedBoxUsesTrueExtentsAndFaceNormal) {
  // Wall 0.5 x 40 x 5 centred at x=30, like the demo world.
  SceneRaycaster scene;
  SceneShape wall;
  wall.type = ShapeType::kBox;
  wall.pose = yaw_pose(30, 0, 2.5, 0.0);
  wall.box_half_extents = {0.25, 20.0, 2.5};
  scene.add_shape(wall);

  // 15 m off-centre still hits (the old plugin's 2x2 placeholder box would miss).
  auto hit = scene.cast({0, 15, 1}, {1, 0, 0}, 0.0, 100.0);
  ASSERT_TRUE(hit.has_value());
  EXPECT_NEAR(hit->t, 29.75, 1e-9);
  EXPECT_NEAR(hit->normal.x, -1.0, 1e-9);

  // Rotate the wall 90 deg: now it spans x in [10, 50] and is thin along x.
  scene.shape(0).pose = yaw_pose(30, 0, 2.5, M_PI / 2);
  auto hit2 = scene.cast({45, -10, 1}, {0, 1, 0}, 0.0, 100.0);
  ASSERT_TRUE(hit2.has_value());
  EXPECT_NEAR(hit2->t, 9.75, 1e-9);
  EXPECT_NEAR(hit2->normal.y, -1.0, 1e-9);
  EXPECT_FALSE(scene.cast({0, 15, 1}, {1, 0, 0}, 0.0, 100.0).has_value());
}

TEST(SceneRaycaster, ObliqueBoxFaceGivesObliqueNormal) {
  // The old plugin always used normal = -ray, so incidence was always 0 deg.
  SceneRaycaster scene;
  SceneShape box;
  box.type = ShapeType::kBox;
  box.pose = yaw_pose(10, 0, 0, 0.0);
  box.box_half_extents = {1, 50, 50};
  scene.add_shape(box);
  const Vec3 d = Vec3{1, 1, 0}.normalized();
  auto hit = scene.cast({0, 0, 0}, d, 0.0, 100.0);
  ASSERT_TRUE(hit.has_value());
  EXPECT_NEAR(-hit->normal.dot(d), std::cos(M_PI / 4), 1e-9);
}

TEST(SceneRaycaster, SphereAndCylinder) {
  SceneRaycaster scene;
  SceneShape sph;
  sph.type = ShapeType::kSphere;
  sph.radius = 1.0;
  sph.pose = yaw_pose(10, 0, 0, 0);
  scene.add_shape(sph);
  auto h = scene.cast({0, 0, 0}, {1, 0, 0}, 0.0, 100.0);
  ASSERT_TRUE(h.has_value());
  EXPECT_NEAR(h->t, 9.0, 1e-9);
  EXPECT_NEAR(h->normal.x, -1.0, 1e-9);

  SceneRaycaster scene2;
  SceneShape cyl;
  cyl.type = ShapeType::kCylinder;
  cyl.radius = 0.5;
  cyl.half_length = 1.0;
  cyl.pose = yaw_pose(5, 0, 0, 0);
  scene2.add_shape(cyl);
  auto side = scene2.cast({0, 0, 0.5}, {1, 0, 0}, 0.0, 100.0);
  ASSERT_TRUE(side.has_value());
  EXPECT_NEAR(side->t, 4.5, 1e-9);
  auto cap = scene2.cast({5, 0, 10}, {0, 0, -1}, 0.0, 100.0);
  ASSERT_TRUE(cap.has_value());
  EXPECT_NEAR(cap->t, 9.0, 1e-9);
  EXPECT_NEAR(cap->normal.z, 1.0, 1e-9);
  EXPECT_FALSE(scene2.cast({0, 0, 1.5}, {1, 0, 0}, 0.0, 100.0).has_value());
}

TEST(SceneRaycaster, NearestOfSeveralShapesWins) {
  SceneRaycaster scene;
  SceneShape ground;
  ground.type = ShapeType::kPlane;
  scene.add_shape(ground);
  SceneShape box;
  box.type = ShapeType::kBox;
  box.pose = yaw_pose(3, 0, 0.5, 0);
  box.box_half_extents = {0.5, 0.5, 0.5};
  const std::size_t box_idx = scene.add_shape(box);

  // Ray from 1 m height going down-forward hits the box before the ground.
  const Vec3 d = Vec3{1, 0, -0.2}.normalized();
  auto hit = scene.cast({0, 0, 1}, d, 0.0, 100.0);
  ASSERT_TRUE(hit.has_value());
  EXPECT_EQ(hit->shape_index, box_idx);
  EXPECT_NEAR(hit->point.x, 2.5, 1e-9);
}

TEST(SceneRaycaster, MeshMatchesAnalyticPlaneAndSlope) {
  SceneRaycaster scene;
  SceneShape terrain;
  terrain.type = ShapeType::kMesh;
  terrain.mesh = make_grid(40, 50.0, flat);
  scene.add_shape(terrain);

  std::mt19937 rng(7);
  std::uniform_real_distribution<double> u(-0.9, 0.9);
  for (int k = 0; k < 200; ++k) {
    const Vec3 o{u(rng) * 20, u(rng) * 20, 3.0};
    const Vec3 d = Vec3{u(rng), u(rng), -1.0}.normalized();
    auto hit = scene.cast(o, d, 0.0, 200.0);
    ASSERT_TRUE(hit.has_value());
    EXPECT_NEAR(hit->t, -3.0 / d.z, 1e-9);
    EXPECT_NEAR(hit->normal.z, 1.0, 1e-9);
  }

  // Sloped mesh: z = 0.25 x. Normal is (-0.25, 0, 1)/|.|.
  SceneRaycaster sloped;
  SceneShape s2;
  s2.type = ShapeType::kMesh;
  s2.mesh = make_grid(20, 50.0, slope);
  sloped.add_shape(s2);
  auto hit = sloped.cast({4, 0, 10}, {0, 0, -1}, 0.0, 100.0);
  ASSERT_TRUE(hit.has_value());
  EXPECT_NEAR(hit->point.z, 1.0, 1e-9);
  const Vec3 n_exp = Vec3{-0.25, 0, 1}.normalized();
  EXPECT_NEAR(hit->normal.dot(n_exp), 1.0, 1e-9);
}

TEST(SceneRaycaster, MeshPoseIsApplied) {
  SceneRaycaster scene;
  SceneShape terrain;
  terrain.type = ShapeType::kMesh;
  terrain.mesh = make_grid(4, 1.0, flat);  // 2 x 2 m patch
  terrain.pose = yaw_pose(100, 0, -2, 0.0);
  scene.add_shape(terrain);
  EXPECT_FALSE(scene.cast({0, 0, 0}, {0, 0, -1}, 0.0, 100.0).has_value());
  auto hit = scene.cast({100.5, 0.5, 0}, {0, 0, -1}, 0.0, 100.0);
  ASSERT_TRUE(hit.has_value());
  EXPECT_NEAR(hit->t, 2.0, 1e-9);
}

TEST(SceneRaycaster, RespectsRangeWindow) {
  SceneRaycaster scene;
  SceneShape ground;
  ground.type = ShapeType::kPlane;
  scene.add_shape(ground);
  EXPECT_FALSE(scene.cast({0, 0, 10}, {0, 0, -1}, 0.0, 5.0).has_value());
  EXPECT_FALSE(scene.cast({0, 0, 10}, {0, 0, -1}, 11.0, 50.0).has_value());
  EXPECT_TRUE(scene.cast({0, 0, 10}, {0, 0, -1}, 0.0, 10.5).has_value());
}

TEST(SceneShape, PointVelocityIncludesRotation) {
  SceneShape s;
  s.linear_velocity = {1, 0, 0};
  s.angular_velocity = {0, 0, 1};   // spinning about z
  s.velocity_origin = {0, 0, 0};
  const Vec3 v = s.point_velocity({0, 2, 0});  // w x r = (0,0,1)x(0,2,0) = (-2,0,0)
  EXPECT_NEAR(v.x, -1.0, 1e-12);
  EXPECT_NEAR(v.y, 0.0, 1e-12);
}
