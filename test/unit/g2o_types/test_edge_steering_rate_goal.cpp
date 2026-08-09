#include <gtest/gtest.h>

#include <Eigen/Core>

#include "nav2_teb_controller/g2o_types/edge_steering_rate_goal.h"
#include "nav2_teb_controller/g2o_types/vertex_pose.h"
#include "nav2_teb_controller/g2o_types/vertex_timediff.h"

using namespace nav2_teb_controller;

namespace {

teb_controller::Params makeParams() {
  teb_controller::Params p;
  p.FollowPath.robot.wheelbase = 1.055;
  p.FollowPath.robot.steering_rate_max = 0.25;
  p.FollowPath.optimizer.penalty_epsilon = 0.1;
  p.FollowPath.optimizer.exact_arc_length = false;
  return p;
}

}  // namespace

TEST(EdgeSteeringRateGoal, MatchingGoalAngleZeroError) {
  auto params = makeParams();

  VertexPose *p1 = new VertexPose();
  p1->setEstimate(PoseSE2(0, 0, 0));
  VertexPose *p2 = new VertexPose();
  p2->setEstimate(PoseSE2(1, 0, 0.2));
  VertexTimeDiff *dt = new VertexTimeDiff(1.0);

  EdgeSteeringRateGoal edge;
  edge.setVertex(0, p1);
  edge.setVertex(1, p2);
  edge.setVertex(2, dt);
  edge.setGoalSteeringAngle(0.2);
  edge.setTebConfig(params);
  edge.computeError();

  EXPECT_NEAR(edge.error()[0], 0.0, 1e-9);

  delete p1;
  delete p2;
  delete dt;
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
