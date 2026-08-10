#include <gtest/gtest.h>

#include <Eigen/Core>

#include "nav2_teb_controller/g2o_types/edge_acceleration.h"
#include "nav2_teb_controller/g2o_types/vertex_pose.h"
#include "nav2_teb_controller/g2o_types/vertex_timediff.h"
#include "nav2_teb_controller/math_utils.hpp"
#include "test_jacobian_utils.hpp"

using namespace nav2_teb_controller;

namespace {

teb_controller::Params makeParams() {
  teb_controller::Params p;
  p.FollowPath.robot.a_max_x = 1.0;
  p.FollowPath.robot.a_max_theta = 1.0;
  p.FollowPath.optimizer.penalty_epsilon = 0.1;
  p.FollowPath.optimizer.exact_arc_length = false;
  return p;
}

}  // namespace

TEST(EdgeAcceleration, ConstantVelocityZeroError) {
  auto params = makeParams();

  VertexPose *p1 = new VertexPose();
  p1->setEstimate(PoseSE2(0, 0, 0));
  VertexPose *p2 = new VertexPose();
  p2->setEstimate(PoseSE2(1, 0, 0));
  VertexPose *p3 = new VertexPose();
  p3->setEstimate(PoseSE2(2, 0, 0));
  VertexTimeDiff *dt1 = new VertexTimeDiff(1.0);
  VertexTimeDiff *dt2 = new VertexTimeDiff(1.0);

  EdgeAcceleration edge;
  edge.setVertex(0, p1);
  edge.setVertex(1, p2);
  edge.setVertex(2, p3);
  edge.setVertex(3, dt1);
  edge.setVertex(4, dt2);
  edge.setTebConfig(params);
  edge.computeError();

  EXPECT_NEAR(edge.error()[0], 0.0, 1e-9);
  EXPECT_NEAR(edge.error()[1], 0.0, 1e-9);

  delete p1;
  delete p2;
  delete p3;
  delete dt1;
  delete dt2;
}

TEST(EdgeAcceleration, HighLinearAccelerationPenalized) {
  auto params = makeParams();

  // dist1 = 1, dist2 = 3, dt = 1:
  // vel1 = 1*sigmoid(100), vel2 = 3*sigmoid(300) -> acc = (vel2-vel1)*2/2
  VertexPose *p1 = new VertexPose();
  p1->setEstimate(PoseSE2(0, 0, 0));
  VertexPose *p2 = new VertexPose();
  p2->setEstimate(PoseSE2(1, 0, 0));
  VertexPose *p3 = new VertexPose();
  p3->setEstimate(PoseSE2(4, 0, 0));
  VertexTimeDiff *dt1 = new VertexTimeDiff(1.0);
  VertexTimeDiff *dt2 = new VertexTimeDiff(1.0);

  EdgeAcceleration edge;
  edge.setVertex(0, p1);
  edge.setVertex(1, p2);
  edge.setVertex(2, p3);
  edge.setVertex(3, dt1);
  edge.setVertex(4, dt2);
  edge.setTebConfig(params);
  edge.computeError();

  const double vel1 = 1.0 * fast_sigmoid(100.0 * 1.0);
  const double vel2 = 3.0 * fast_sigmoid(100.0 * 3.0);
  const double acc = (vel2 - vel1) * 2.0 / 2.0;
  EXPECT_NEAR(edge.error()[0], acc - 0.9, 1e-9);  // acc - (a_max - eps)
  EXPECT_NEAR(edge.error()[1], 0.0, 1e-9);

  delete p1;
  delete p2;
  delete p3;
  delete dt1;
  delete dt2;
}

TEST(EdgeAcceleration, HighRotationalAccelerationPenalized) {
  auto params = makeParams();

  // omega1 = 1, omega2 = 2 -> acc_rot = (2-1)*2/2 = 1.0 > a_max_theta - eps
  VertexPose *p1 = new VertexPose();
  p1->setEstimate(PoseSE2(0, 0, 0));
  VertexPose *p2 = new VertexPose();
  p2->setEstimate(PoseSE2(1, 0, 1.0));
  VertexPose *p3 = new VertexPose();
  p3->setEstimate(PoseSE2(2, 0, 3.0));
  VertexTimeDiff *dt1 = new VertexTimeDiff(1.0);
  VertexTimeDiff *dt2 = new VertexTimeDiff(1.0);

  EdgeAcceleration edge;
  edge.setVertex(0, p1);
  edge.setVertex(1, p2);
  edge.setVertex(2, p3);
  edge.setVertex(3, dt1);
  edge.setVertex(4, dt2);
  edge.setTebConfig(params);
  edge.computeError();

  EXPECT_NEAR(edge.error()[0], 0.0, 1e-9);
  EXPECT_NEAR(edge.error()[1], 1.0 - 0.9, 1e-9);  // acc_rot - (a_max_theta - eps)

  delete p1;
  delete p2;
  delete p3;
  delete dt1;
  delete dt2;
}

// Analytic Jacobian vs finite differences. acc_x = 6 > a_max_x - eps and
// acc_rot = 3.2 > a_max_theta - eps (both active); sigmoid args positive.
TEST(EdgeAcceleration, JacobianMatchesNumeric) {
  auto params = makeParams();

  VertexPose *p1 = new VertexPose();
  p1->setEstimate(PoseSE2(0, 0, 0));
  VertexPose *p2 = new VertexPose();
  p2->setEstimate(PoseSE2(0.5, 0, 0.4));
  VertexPose *p3 = new VertexPose();
  p3->setEstimate(PoseSE2(2.0, 0, 1.2));
  VertexTimeDiff *dt1 = new VertexTimeDiff(0.5);
  VertexTimeDiff *dt2 = new VertexTimeDiff(0.5);

  EdgeAcceleration edge;
  edge.setVertex(0, p1);
  edge.setVertex(1, p2);
  edge.setVertex(2, p3);
  edge.setVertex(3, dt1);
  edge.setVertex(4, dt2);
  edge.setTebConfig(params);
  edge.computeError();

  expectAnalyticJacobianMatchesNumericMulti(edge);

  delete p1;
  delete p2;
  delete p3;
  delete dt1;
  delete dt2;
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
