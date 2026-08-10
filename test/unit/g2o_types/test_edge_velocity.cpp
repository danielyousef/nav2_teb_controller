#include <gtest/gtest.h>

#include <Eigen/Core>

#include "nav2_teb_controller/g2o_types/edge_velocity.h"
#include "nav2_teb_controller/g2o_types/vertex_pose.h"
#include "nav2_teb_controller/g2o_types/vertex_timediff.h"
#include "nav2_teb_controller/math_utils.hpp"
#include "test_jacobian_utils.hpp"

using namespace nav2_teb_controller;

namespace {

teb_controller::Params makeParams() {
  teb_controller::Params p;
  p.FollowPath.robot.v_max_x = 0.5;
  p.FollowPath.robot.v_max_x_backwards = 0.2;
  p.FollowPath.robot.v_max_theta = 1.0;
  p.FollowPath.optimizer.penalty_epsilon = 0.1;
  p.FollowPath.optimizer.exact_arc_length = false;
  return p;
}

}  // namespace

TEST(EdgeVelocity, StraightWithinLimitsZeroError) {
  auto params = makeParams();

  VertexPose *p0 = new VertexPose();
  p0->setEstimate(PoseSE2(0, 0, 0));
  VertexPose *p1 = new VertexPose();
  p1->setEstimate(PoseSE2(0.5, 0, 0));
  VertexTimeDiff *dt = new VertexTimeDiff(2.0);  // 0.25 m/s forward

  EdgeVelocity edge;
  edge.setVertex(0, p0);
  edge.setVertex(1, p1);
  edge.setVertex(2, dt);
  edge.setTebConfig(params);
  edge.computeError();

  // vel = 0.25 (forward -> sigmoid ~0.9901), within [-0.2+eps, 0.5-eps]
  EXPECT_NEAR(edge.error()[0], 0.0, 1e-9);
  EXPECT_NEAR(edge.error()[1], 0.0, 1e-9);

  delete p0;
  delete p1;
  delete dt;
}

TEST(EdgeVelocity, OverSpeedPenalized) {
  auto params = makeParams();

  VertexPose *p0 = new VertexPose();
  p0->setEstimate(PoseSE2(0, 0, 0));
  VertexPose *p1 = new VertexPose();
  p1->setEstimate(PoseSE2(2.0, 0, 0));
  VertexTimeDiff *dt = new VertexTimeDiff(1.0);  // 2 m/s forward

  EdgeVelocity edge;
  edge.setVertex(0, p0);
  edge.setVertex(1, p1);
  edge.setVertex(2, dt);
  edge.setTebConfig(params);
  edge.computeError();

  const double vel = 2.0 * fast_sigmoid(100.0 * 2.0);
  // penaltyBoundToInterval(vel, -0.2, 0.5, 0.1): vel > 0.5-0.1 -> vel - 0.4
  EXPECT_NEAR(edge.error()[0], vel - 0.4, 1e-9);
  EXPECT_NEAR(edge.error()[1], 0.0, 1e-9);

  delete p0;
  delete p1;
  delete dt;
}

TEST(EdgeVelocity, BackwardMotionSignFlipsVelocity) {
  auto params = makeParams();

  // Backward motion: sigmoid(-100) = -0.9901 -> vel = -0.9901 < -(v_max_back - eps)
  VertexPose *p0 = new VertexPose();
  p0->setEstimate(PoseSE2(0, 0, 0));
  VertexPose *p1 = new VertexPose();
  p1->setEstimate(PoseSE2(-1.0, 0, 0));  // backward motion
  VertexTimeDiff *dt = new VertexTimeDiff(1.0);

  EdgeVelocity edge;
  edge.setVertex(0, p0);
  edge.setVertex(1, p1);
  edge.setVertex(2, dt);
  edge.setTebConfig(params);
  edge.computeError();

  const double vel = 1.0 * fast_sigmoid(-100.0);
  // below -a+eps branch: error = -vel - (v_max_x_backwards - eps) = 0.9901 - 0.1
  EXPECT_NEAR(edge.error()[0], -vel - (0.2 - 0.1), 1e-9);

  delete p0;
  delete p1;
  delete dt;
}

TEST(EdgeVelocity, HighOmegaPenalized) {
  auto params = makeParams();

  VertexPose *p0 = new VertexPose();
  p0->setEstimate(PoseSE2(0, 0, 0));
  VertexPose *p1 = new VertexPose();
  p1->setEstimate(PoseSE2(1.0, 0, 1.5));  // 1.5 rad / 0.5 s = 3 rad/s
  VertexTimeDiff *dt = new VertexTimeDiff(0.5);

  EdgeVelocity edge;
  edge.setVertex(0, p0);
  edge.setVertex(1, p1);
  edge.setVertex(2, dt);
  edge.setTebConfig(params);
  edge.computeError();

  const double omega = 1.5 / 0.5;
  // penaltyBoundToInterval(omega, 1.0, 0.1): omega > 1.0-0.1 -> omega - 0.9
  EXPECT_NEAR(edge.error()[1], omega - 0.9, 1e-9);

  delete p0;
  delete p1;
  delete dt;
}

// Analytic Jacobian vs finite differences. Both error components active:
// v = 2 m/s > v_max_x - eps and omega = 1.6 > v_max_theta - eps. Sigmoid
// argument s_dir = 2 >> 0 (smooth), theta far from wrap boundaries.
TEST(EdgeVelocity, JacobianMatchesNumeric) {
  auto params = makeParams();

  VertexPose *p0 = new VertexPose();
  p0->setEstimate(PoseSE2(0, 0, 0));
  VertexPose *p1 = new VertexPose();
  p1->setEstimate(PoseSE2(1.0, 0, 0.8));
  VertexTimeDiff *dt = new VertexTimeDiff(0.5);

  EdgeVelocity edge;
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
