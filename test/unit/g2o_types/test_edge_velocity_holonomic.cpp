#include <gtest/gtest.h>

#include <Eigen/Core>

#include "nav2_teb_controller/g2o_types/edge_velocity_holonomic.h"
#include "nav2_teb_controller/g2o_types/vertex_pose.h"
#include "nav2_teb_controller/g2o_types/vertex_timediff.h"
#include "test_jacobian_utils.hpp"

using namespace nav2_teb_controller;

namespace {

teb_controller::Params makeParams() {
  teb_controller::Params p;
  p.FollowPath.robot.v_max_x = 0.5;
  p.FollowPath.robot.v_max_x_backwards = 0.2;
  p.FollowPath.robot.v_max_y = 0.4;
  p.FollowPath.robot.v_max_theta = 1.0;
  p.FollowPath.optimizer.penalty_epsilon = 0.1;
  p.FollowPath.optimizer.exact_arc_length = false;
  return p;
}

}  // namespace

TEST(EdgeVelocityHolonomic, WithinLimitsZeroError) {
  auto params = makeParams();

  VertexPose *p0 = new VertexPose();
  p0->setEstimate(PoseSE2(0, 0, 0));
  VertexPose *p1 = new VertexPose();
  p1->setEstimate(PoseSE2(0.3, 0.2, 0.1));
  VertexTimeDiff *dt = new VertexTimeDiff(2.0);

  EdgeVelocityHolonomic edge;
  edge.setVertex(0, p0);
  edge.setVertex(1, p1);
  edge.setVertex(2, dt);
  edge.setTebConfig(params);
  edge.computeError();

  // vx = 0.15, vy = 0.1, omega = 0.05: all within bounds
  EXPECT_NEAR(edge.error()[0], 0.0, 1e-9);
  EXPECT_NEAR(edge.error()[1], 0.0, 1e-9);
  EXPECT_NEAR(edge.error()[2], 0.0, 1e-9);

  delete p0;
  delete p1;
  delete dt;
}

TEST(EdgeVelocityHolonomic, LateralVelocityPenalized) {
  auto params = makeParams();

  VertexPose *p0 = new VertexPose();
  p0->setEstimate(PoseSE2(0, 0, 0));
  VertexPose *p1 = new VertexPose();
  p1->setEstimate(PoseSE2(0, 0.5, 0));  // pure lateral motion in robot frame
  VertexTimeDiff *dt = new VertexTimeDiff(1.0);

  EdgeVelocityHolonomic edge;
  edge.setVertex(0, p0);
  edge.setVertex(1, p1);
  edge.setVertex(2, dt);
  edge.setTebConfig(params);
  edge.computeError();

  // r_dx = 0 -> vx = 0 ok; r_dy = 0.5 -> vy = 0.5 > v_max_y 0.4
  EXPECT_NEAR(edge.error()[0], 0.0, 1e-9);
  EXPECT_NEAR(edge.error()[1], 0.5 - 0.4, 1e-9);
  EXPECT_NEAR(edge.error()[2], 0.0, 1e-9);

  delete p0;
  delete p1;
  delete dt;
}

TEST(EdgeVelocityHolonomic, RotatedFrame) {
  auto params = makeParams();

  // Robot heading 90 deg, displacement along its +x axis -> in world frame this is +y
  VertexPose *p0 = new VertexPose();
  p0->setEstimate(PoseSE2(0, 0, M_PI_2));
  VertexPose *p1 = new VertexPose();
  p1->setEstimate(PoseSE2(0, 0.4, M_PI_2));
  VertexTimeDiff *dt = new VertexTimeDiff(1.0);

  EdgeVelocityHolonomic edge;
  edge.setVertex(0, p0);
  edge.setVertex(1, p1);
  edge.setVertex(2, dt);
  edge.setTebConfig(params);
  edge.computeError();

  // r_dx = 0.4 -> vx = 0.4 within limits; r_dy = 0 -> vy = 0
  EXPECT_NEAR(edge.error()[0], 0.0, 1e-9);
  EXPECT_NEAR(edge.error()[1], 0.0, 1e-9);

  delete p0;
  delete p1;
  delete dt;
}

// Analytic Jacobian vs finite differences. Robot-frame velocities
// vx = 1.0 > 0.5 - eps, vy = 2.0 > 0.4, omega = 1.6 > 1.0 - eps: all active.
// theta1 = 0 far from wrap boundaries, vy penalty has epsilon 0 -> keep away
// from the v_max_y cusp.
TEST(EdgeVelocityHolonomic, JacobianMatchesNumeric) {
  auto params = makeParams();

  VertexPose *p0 = new VertexPose();
  p0->setEstimate(PoseSE2(0, 0, 0));
  VertexPose *p1 = new VertexPose();
  p1->setEstimate(PoseSE2(0.5, 1.0, 0.8));
  VertexTimeDiff *dt = new VertexTimeDiff(0.5);

  EdgeVelocityHolonomic edge;
  edge.setVertex(0, p0);
  edge.setVertex(1, p1);
  edge.setVertex(2, dt);
  edge.setTebConfig(params);
  edge.computeError();

  expectAnalyticJacobianMatchesNumericMulti(edge);

  delete p0;
  delete p1;
  delete dt;
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
