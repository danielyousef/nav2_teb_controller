#include <gtest/gtest.h>

#include <Eigen/Core>
#include <geometry_msgs/msg/twist.hpp>

#include "nav2_teb_controller/g2o_types/edge_acceleration_goal.h"
#include "nav2_teb_controller/g2o_types/vertex_pose.h"
#include "nav2_teb_controller/g2o_types/vertex_timediff.h"
#include "nav2_teb_controller/math_utils.hpp"

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

TEST(EdgeAccelerationGoal, HighGoalAccelerationPenalized) {
  auto params = makeParams();

  // dist 2.0 in 0.5 s -> vel1 = 4*sigmoid(200), vel_goal = 0 -> acc = -vel1/0.5
  VertexPose *p1 = new VertexPose();
  p1->setEstimate(PoseSE2(0, 0, 0));
  VertexPose *p2 = new VertexPose();
  p2->setEstimate(PoseSE2(2, 0, 0));
  VertexTimeDiff *dt = new VertexTimeDiff(0.5);

  geometry_msgs::msg::Twist vel_goal;
  vel_goal.linear.x = 0.0;
  vel_goal.angular.z = 0.0;

  EdgeAccelerationGoal edge;
  edge.setVertex(0, p1);
  edge.setVertex(1, p2);
  edge.setVertex(2, dt);
  edge.setGoalVelocity(vel_goal);
  edge.setTebConfig(params);
  edge.computeError();

  const double vel1 = (2.0 / 0.5) * fast_sigmoid(100.0 * 2.0);
  const double acc = -vel1 / 0.5;
  // below -a+eps branch: error = -acc - (a_max - eps)
  EXPECT_NEAR(edge.error()[0], -acc - 0.9, 1e-9);

  delete p1;
  delete p2;
  delete dt;
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
