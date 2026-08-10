#include <gtest/gtest.h>

#include <Eigen/Core>

#include "nav2_teb_controller/g2o_types/edge_esdf_obstacle.h"
#include "nav2_teb_controller/g2o_types/vertex_pose.h"
#include "nav2_teb_controller/obstacles/esdf.hpp"
#include "nav2_teb_controller/core/footprint.hpp"
#include "test_jacobian_utils.hpp"

using namespace nav2_teb_controller;

namespace {

// 21x21 grid @ 0.1 m, origin (0,0). Obstacle: 3x3 lethal block centered at cell (10,10)
// -> world (1.0, 1.0).
ObstacleMap2D makeEsdf() {
  constexpr unsigned nx = 21, ny = 21;
  std::vector<uint8_t> data(nx * ny, 0);
  for (unsigned y = 9; y <= 11; ++y)
    for (unsigned x = 9; x <= 11; ++x)
      data[y * nx + x] = 254;

  ObstacleMap2D esdf;
  esdf.update(data.data(), nx, ny, 0.1, 0.0, 0.0, 254);
  return esdf;
}

teb_controller::Params makeParams() {
  teb_controller::Params p;
  p.FollowPath.obstacles.min_obstacle_dist = 0.2;
  // Note: with the small 2.1 x 2.1 m test grid the max EDT distance is ~1.47 m,
  // so inflation_dist must be reachable, otherwise every pose is penalized.
  p.FollowPath.obstacles.inflation_dist = 0.5;
  p.FollowPath.obstacles.cost_exponent = 5.0;
  return p;
}

}  // namespace

TEST(EdgeESDFObstacle, ErrorDimensionCirclesPlusInflation) {
  Footprint fp(false, "circles", "[ [0, 0, 0.1] ]");
  EdgeESDFObstacle edge;
  edge.resize(fp.circles().size());

  EXPECT_EQ(edge.error().size(), 2);  // 1 circle + 1 inflation

  Footprint fp2(false, "circles", "[ [0, 0, 0.1], [0.5, 0, 0.1] ]");
  EdgeESDFObstacle edge2;
  edge2.resize(fp2.circles().size());
  EXPECT_EQ(edge2.error().size(), 3);  // 2 circles + 1 inflation
}

TEST(EdgeESDFObstacle, FarFromObstacleZeroError) {
  auto esdf = makeEsdf();
  Footprint fp(false, "circles", "[ [0, 0, 0.1] ]");
  auto params = makeParams();

  // Corner of the grid, in bounds, dist ~1.27 m > inflation 0.5
  VertexPose *v = new VertexPose();
  v->setEstimate(PoseSE2(2.0, 2.0, 0.0));

  EdgeESDFObstacle edge;
  edge.resize(fp.circles().size());
  edge.setVertex(0, v);
  edge.setObstacle(esdf);
  edge.setFootprint(fp);
  edge.setTebConfig(params);
  edge.computeError();

  EXPECT_NEAR(edge.error()[0], 0.0, 1e-9);  // circle far away
  EXPECT_NEAR(edge.error()[1], 0.0, 1e-9);  // center inflation far away

  delete v;
}

TEST(EdgeESDFObstacle, OutOfBoundsClampsToBoundary) {
  auto esdf = makeEsdf();
  Footprint fp(false, "circles", "[ [0, 0, 0.1] ]");
  auto params = makeParams();
  params.FollowPath.obstacles.inflation_dist = 1.7;

  // (5, 5) is outside the 2.1 x 2.1 m grid: query() clamps to the corner cell
  // (19,19) -> EDT = sqrt(8^2 + 8^2) * 0.1 = 1.1314 (does NOT grow)
  VertexPose *v = new VertexPose();
  v->setEstimate(PoseSE2(5.0, 5.0, 0.0));

  EdgeESDFObstacle edge;
  edge.resize(fp.circles().size());
  edge.setVertex(0, v);
  edge.setObstacle(esdf);
  edge.setFootprint(fp);
  edge.setTebConfig(params);
  edge.computeError();

  const double clamped_dist = std::sqrt(8.0 * 8.0 + 8.0 * 8.0) * 0.1;
  EXPECT_NEAR(edge.error()[1], 1.7 - clamped_dist, 1e-6);

  delete v;
}

TEST(EdgeESDFObstacle, OnObstacleMaximumError) {
  auto esdf = makeEsdf();
  Footprint fp(false, "circles", "[ [0, 0, 0.1] ]");
  auto params = makeParams();

  VertexPose *v = new VertexPose();
  v->setEstimate(PoseSE2(1.0, 1.0, 0.0));  // dead center of the 3x3 block

  EdgeESDFObstacle edge;
  edge.resize(fp.circles().size());
  edge.setVertex(0, v);
  edge.setObstacle(esdf);
  edge.setFootprint(fp);
  edge.setTebConfig(params);
  edge.computeError();

  // Circle: penaltyBoundFromBelow(0, min_dist+radius=0.3, 0) = 0.3
  //         + expm1(5 * (0.3 - 0)) / 5 = (e^1.5 - 1) / 5 ≈ 0.69634
  const double expected_circle = 0.3 + std::expm1(5.0 * 0.3) / 5.0;
  EXPECT_NEAR(edge.error()[0], expected_circle, 1e-4);
  // Center inflation: penaltyBoundFromBelow(0, 0.5, 0) = 0.5
  EXPECT_NEAR(edge.error()[1], 0.5, 1e-9);

  delete v;
}

TEST(EdgeESDFObstacle, NearObstaclePartialPenalty) {
  auto esdf = makeEsdf();
  Footprint fp(false, "circles", "[ [0, 0, 0.1] ]");
  auto params = makeParams();

  // (0.8, 1.0): bilinear-interpolated distance to the block ≈ 0.15
  VertexPose *v = new VertexPose();
  v->setEstimate(PoseSE2(0.8, 1.0, 0.0));

  EdgeESDFObstacle edge;
  edge.resize(fp.circles().size());
  edge.setVertex(0, v);
  edge.setObstacle(esdf);
  edge.setFootprint(fp);
  edge.setTebConfig(params);
  edge.computeError();

  // penaltyBoundFromBelow(0.15, 0.3, 0) = 0.15
  // + expm1(5 * (0.3 - 0.15)) / 5 = (e^0.75 - 1) / 5 ≈ 0.22344
  const double expected = 0.15 + std::expm1(5.0 * 0.15) / 5.0;
  EXPECT_NEAR(edge.error()[0], expected, 1e-4);
  // Center distance 0.15 (bilinear interp at (0.8, 1.0)) < inflation 0.5
  EXPECT_NEAR(edge.error()[1], 0.5 - 0.15, 1e-4);

  delete v;
}


// Analytic Jacobian vs finite differences. Pose at (0.8, 1.0): bilinear
// interpolation cell (distance ~0.15 < thresholds, both penalties active),
// 0.05 m away from interpolation cell boundaries, gradient smooth.
TEST(EdgeESDFObstacle, JacobianMatchesNumeric) {
  auto esdf = makeEsdf();
  Footprint fp(false, "circles", "[ [0, 0, 0.1] ]");
  auto params = makeParams();

  VertexPose *v = new VertexPose();
  v->setEstimate(PoseSE2(0.8, 1.0, 0.3));

  EdgeESDFObstacle edge;
  edge.resize(fp.circles().size());
  edge.setVertex(0, v);
  edge.setObstacle(esdf);
  edge.setFootprint(fp);
  edge.setTebConfig(params);
  edge.computeError();

  expectAnalyticJacobianMatchesNumericUnary(edge);

  delete v;
}
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
