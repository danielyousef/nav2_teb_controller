#include <gtest/gtest.h>

#include <Eigen/Core>

#include "nav2_teb_controller/g2o_types/edge_acceleration_holonomic.h"
#include "nav2_teb_controller/g2o_types/vertex_pose.h"
#include "nav2_teb_controller/g2o_types/vertex_timediff.h"

using namespace nav2_teb_controller;

namespace {

teb_controller::Params makeParams() {
  teb_controller::Params p;
  p.FollowPath.robot.a_max_x = 1.0;
  p.FollowPath.robot.a_max_y = 0.5;
  p.FollowPath.robot.a_max_theta = 1.0;
  p.FollowPath.optimizer.penalty_epsilon = 0.1;
  p.FollowPath.optimizer.exact_arc_length = false;
  return p;
}

}  // namespace

TEST(EdgeAccelerationHolonomic, HighLinearAccelerationPenalized) {
  auto params = makeParams();

  // No sigmoid in the holonomic edge: vel1_x = 1, vel2_x = 3 -> acc_x = 2
  VertexPose *p1 = new VertexPose();
  p1->setEstimate(PoseSE2(0, 0, 0));
  VertexPose *p2 = new VertexPose();
  p2->setEstimate(PoseSE2(1, 0, 0));
  VertexPose *p3 = new VertexPose();
  p3->setEstimate(PoseSE2(4, 0, 0));
  VertexTimeDiff *dt1 = new VertexTimeDiff(1.0);
  VertexTimeDiff *dt2 = new VertexTimeDiff(1.0);

  EdgeAccelerationHolonomic edge;
  edge.setVertex(0, p1);
  edge.setVertex(1, p2);
  edge.setVertex(2, p3);
  edge.setVertex(3, dt1);
  edge.setVertex(4, dt2);
  edge.setTebConfig(params);
  edge.computeError();

  EXPECT_NEAR(edge.error()[0], 2.0 - 0.9, 1e-9);
  EXPECT_NEAR(edge.error()[1], 0.0, 1e-9);
  EXPECT_NEAR(edge.error()[2], 0.0, 1e-9);

  delete p1;
  delete p2;
  delete p3;
  delete dt1;
  delete dt2;
}

TEST(EdgeAccelerationHolonomic, LateralAccelerationPenalized) {
  auto params = makeParams();

  // vy: 0.5 -> 1.0, acc_y = (1.0-0.5)*2/2 = 0.5 > a_max_y - eps
  VertexPose *p1 = new VertexPose();
  p1->setEstimate(PoseSE2(0, 0, 0));
  VertexPose *p2 = new VertexPose();
  p2->setEstimate(PoseSE2(0, 0.5, 0));
  VertexPose *p3 = new VertexPose();
  p3->setEstimate(PoseSE2(0, 1.5, 0));
  VertexTimeDiff *dt1 = new VertexTimeDiff(1.0);
  VertexTimeDiff *dt2 = new VertexTimeDiff(1.0);

  EdgeAccelerationHolonomic edge;
  edge.setVertex(0, p1);
  edge.setVertex(1, p2);
  edge.setVertex(2, p3);
  edge.setVertex(3, dt1);
  edge.setVertex(4, dt2);
  edge.setTebConfig(params);
  edge.computeError();

  EXPECT_NEAR(edge.error()[0], 0.0, 1e-9);
  EXPECT_NEAR(edge.error()[1], 0.5 - 0.4, 1e-9);  // acc_y - (a_max_y - eps)
  EXPECT_NEAR(edge.error()[2], 0.0, 1e-9);

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
