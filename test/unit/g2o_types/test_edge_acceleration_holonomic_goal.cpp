#include <gtest/gtest.h>

#include <Eigen/Core>
#include <geometry_msgs/msg/twist.hpp>

#include "nav2_teb_controller/g2o_types/edge_acceleration_holonomic_goal.h"
#include "nav2_teb_controller/g2o_types/vertex_pose.h"
#include "nav2_teb_controller/g2o_types/vertex_timediff.h"
#include "test_jacobian_utils.hpp"

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

TEST(EdgeAccelerationHolonomicGoal, ZeroErrorOnMatchingGoalVelocity) {
  auto params = makeParams();

  VertexPose *p1 = new VertexPose();
  p1->setEstimate(PoseSE2(0, 0, 0));
  VertexPose *p2 = new VertexPose();
  p2->setEstimate(PoseSE2(1, 0, 0));
  VertexTimeDiff *dt = new VertexTimeDiff(1.0);

  geometry_msgs::msg::Twist vel_goal;
  vel_goal.linear.x = 1.0;  // matches vel1 = 1.0
  vel_goal.linear.y = 0.0;
  vel_goal.angular.z = 0.0;

  EdgeAccelerationHolonomicGoal edge;
  edge.setVertex(0, p1);
  edge.setVertex(1, p2);
  edge.setVertex(2, dt);
  edge.setGoalVelocity(vel_goal);
  edge.setTebConfig(params);
  edge.computeError();

  EXPECT_NEAR(edge.error()[0], 0.0, 1e-9);
  EXPECT_NEAR(edge.error()[1], 0.0, 1e-9);
  EXPECT_NEAR(edge.error()[2], 0.0, 1e-9);

  delete p1;
  delete p2;
  delete dt;
}


// Analytic Jacobian vs finite differences. acc_x = -3 and acc_y = -1.4
// (active), acc_rot inactive (zero row verified too).
TEST(EdgeAccelerationHolonomicGoal, JacobianMatchesNumeric) {
  auto params = makeParams();

  geometry_msgs::msg::Twist vel_goal;
  vel_goal.linear.x = 0.5;
  vel_goal.linear.y = 0.3;
  vel_goal.angular.z = 0.3;

  VertexPose *p1 = new VertexPose();
  p1->setEstimate(PoseSE2(0, 0, 0));
  VertexPose *p2 = new VertexPose();
  p2->setEstimate(PoseSE2(1.0, 0.5, 0.3));
  VertexTimeDiff *dt = new VertexTimeDiff(0.5);

  EdgeAccelerationHolonomicGoal edge;
  edge.setGoalVelocity(vel_goal);
  edge.setVertex(0, p1);
  edge.setVertex(1, p2);
  edge.setVertex(2, dt);
  edge.setTebConfig(params);
  edge.computeError();

  expectAnalyticJacobianMatchesNumericMulti(edge);

  delete p1;
  delete p2;
  delete dt;
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
