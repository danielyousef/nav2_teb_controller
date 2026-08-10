#include <gtest/gtest.h>

#include <Eigen/Core>
#include <geometry_msgs/msg/twist.hpp>

#include "nav2_teb_controller/g2o_types/edge_jerk_new.h"
#include "nav2_teb_controller/g2o_types/vertex_pose.h"
#include "nav2_teb_controller/g2o_types/vertex_timediff.h"
#include "nav2_teb_controller/math_utils.hpp"
#include "test_jacobian_utils.hpp"

using namespace nav2_teb_controller;

namespace {

teb_controller::Params makeParams() {
  teb_controller::Params p;
  p.FollowPath.robot.jerk_max_x = 1.0;
  p.FollowPath.robot.jerk_max_theta = 2.0;
  p.FollowPath.optimizer.penalty_epsilon = 0.1;
  p.FollowPath.optimizer.exact_arc_length = false;
  return p;
}

}  // namespace

TEST(EdgeJerk, ConstantAccelerationZeroError) {
  auto params = makeParams();

  // vels 1, 2, 3 -> accs 1, 1 -> jerk 0
  VertexPose *p1 = new VertexPose();
  p1->setEstimate(PoseSE2(0, 0, 0));
  VertexPose *p2 = new VertexPose();
  p2->setEstimate(PoseSE2(1, 0, 0));
  VertexPose *p3 = new VertexPose();
  p3->setEstimate(PoseSE2(3, 0, 0));
  VertexPose *p4 = new VertexPose();
  p4->setEstimate(PoseSE2(6, 0, 0));
  VertexTimeDiff *dt1 = new VertexTimeDiff(1.0);
  VertexTimeDiff *dt2 = new VertexTimeDiff(1.0);
  VertexTimeDiff *dt3 = new VertexTimeDiff(1.0);

  EdgeJerk edge;
  edge.setVertex(0, p1);
  edge.setVertex(1, p2);
  edge.setVertex(2, p3);
  edge.setVertex(3, p4);
  edge.setVertex(4, dt1);
  edge.setVertex(5, dt2);
  edge.setVertex(6, dt3);
  edge.setTebConfig(params);
  edge.computeError();

  EXPECT_NEAR(edge.error()[0], 0.0, 1e-9);
  EXPECT_NEAR(edge.error()[1], 0.0, 1e-9);

  delete p1;
  delete p2;
  delete p3;
  delete p4;
  delete dt1;
  delete dt2;
  delete dt3;
}

TEST(EdgeJerk, JerkChangePenalized) {
  auto params = makeParams();

  // dists 1, 2, 4 -> vels 1, 2, 4 scaled by sigmoid:
  // accs (sigmoid-weighted) ~ 0.99995, 1.99997 -> jerk ~ 1.00002
  VertexPose *p1 = new VertexPose();
  p1->setEstimate(PoseSE2(0, 0, 0));
  VertexPose *p2 = new VertexPose();
  p2->setEstimate(PoseSE2(1, 0, 0));
  VertexPose *p3 = new VertexPose();
  p3->setEstimate(PoseSE2(3, 0, 0));
  VertexPose *p4 = new VertexPose();
  p4->setEstimate(PoseSE2(7, 0, 0));
  VertexTimeDiff *dt1 = new VertexTimeDiff(1.0);
  VertexTimeDiff *dt2 = new VertexTimeDiff(1.0);
  VertexTimeDiff *dt3 = new VertexTimeDiff(1.0);

  EdgeJerk edge;
  edge.setVertex(0, p1);
  edge.setVertex(1, p2);
  edge.setVertex(2, p3);
  edge.setVertex(3, p4);
  edge.setVertex(4, dt1);
  edge.setVertex(5, dt2);
  edge.setVertex(6, dt3);
  edge.setTebConfig(params);
  edge.computeError();

  const double vel1 = 1.0 * fast_sigmoid(100.0 * 1.0);
  const double vel2 = 2.0 * fast_sigmoid(100.0 * 2.0);
  const double vel3 = 4.0 * fast_sigmoid(100.0 * 4.0);
  const double acc1 = (vel2 - vel1) * 2.0 / 2.0;
  const double acc2 = (vel3 - vel2) * 2.0 / 2.0;
  const double jerk = (acc2 - acc1) / (2.0 / 4.0 + 2.0 / 4.0);
  EXPECT_NEAR(edge.error()[0], jerk - 0.9, 1e-9);  // jerk - (jerk_max_x - eps)
  EXPECT_NEAR(edge.error()[1], 0.0, 1e-9);

  delete p1;
  delete p2;
  delete p3;
  delete p4;
  delete dt1;
  delete dt2;
  delete dt3;
}

TEST(EdgeJerk, RotationalJerkPenalized) {
  auto params = makeParams();

  // omegas 0.5, 1.0, 2.0 (dt=1) -> acc_rot 0.5, 1.0 -> jerk_rot = 0.5
  VertexPose *p1 = new VertexPose();
  p1->setEstimate(PoseSE2(0, 0, 0.0));
  VertexPose *p2 = new VertexPose();
  p2->setEstimate(PoseSE2(1, 0, 0.5));
  VertexPose *p3 = new VertexPose();
  p3->setEstimate(PoseSE2(2, 0, 1.5));
  VertexPose *p4 = new VertexPose();
  p4->setEstimate(PoseSE2(3, 0, 3.5));
  VertexTimeDiff *dt1 = new VertexTimeDiff(1.0);
  VertexTimeDiff *dt2 = new VertexTimeDiff(1.0);
  VertexTimeDiff *dt3 = new VertexTimeDiff(1.0);

  EdgeJerk edge;
  edge.setVertex(0, p1);
  edge.setVertex(1, p2);
  edge.setVertex(2, p3);
  edge.setVertex(3, p4);
  edge.setVertex(4, dt1);
  edge.setVertex(5, dt2);
  edge.setVertex(6, dt3);
  edge.setTebConfig(params);
  edge.computeError();

  EXPECT_NEAR(edge.error()[0], 0.0, 1e-9);
  // jerk_rot = 0.5 < jerk_max_theta - eps = 1.9 -> 0
  EXPECT_NEAR(edge.error()[1], 0.0, 1e-9);

  delete p1;
  delete p2;
  delete p3;
  delete p4;
  delete dt1;
  delete dt2;
  delete dt3;
}

TEST(EdgeJerkStart, SmoothStartZeroError) {
  auto params = makeParams();

  // vel0 = 0, vels 1, 2 -> acc0 = 1, acc1 = 1 -> jerk 0
  VertexPose *p1 = new VertexPose();
  p1->setEstimate(PoseSE2(0, 0, 0));
  VertexPose *p2 = new VertexPose();
  p2->setEstimate(PoseSE2(1, 0, 0));
  VertexPose *p3 = new VertexPose();
  p3->setEstimate(PoseSE2(3, 0, 0));
  VertexTimeDiff *dt1 = new VertexTimeDiff(1.0);
  VertexTimeDiff *dt2 = new VertexTimeDiff(1.0);

  geometry_msgs::msg::Twist vel_start;
  vel_start.linear.x = 0.0;
  vel_start.angular.z = 0.0;

  EdgeJerkStart edge;
  edge.setVertex(0, p1);
  edge.setVertex(1, p2);
  edge.setVertex(2, p3);
  edge.setVertex(3, dt1);
  edge.setVertex(4, dt2);
  edge.setInitialVelocity(vel_start);
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

TEST(EdgeJerkStart, StartJerkPenalized) {
  auto params = makeParams();

  // vel0 = 0 (measurement), dists 1, 4 -> vels 1, 4 scaled by sigmoid
  // acc0 ~ 0.9901, acc1 ~ 2.99992 -> jerk ~ 2.00982
  VertexPose *p1 = new VertexPose();
  p1->setEstimate(PoseSE2(0, 0, 0));
  VertexPose *p2 = new VertexPose();
  p2->setEstimate(PoseSE2(1, 0, 0));
  VertexPose *p3 = new VertexPose();
  p3->setEstimate(PoseSE2(5, 0, 0));
  VertexTimeDiff *dt1 = new VertexTimeDiff(1.0);
  VertexTimeDiff *dt2 = new VertexTimeDiff(1.0);

  geometry_msgs::msg::Twist vel_start;
  vel_start.linear.x = 0.0;
  vel_start.angular.z = 0.0;

  EdgeJerkStart edge;
  edge.setVertex(0, p1);
  edge.setVertex(1, p2);
  edge.setVertex(2, p3);
  edge.setVertex(3, dt1);
  edge.setVertex(4, dt2);
  edge.setInitialVelocity(vel_start);
  edge.setTebConfig(params);
  edge.computeError();

  const double vel1 = 1.0 * fast_sigmoid(100.0 * 1.0);
  const double vel2 = 4.0 * fast_sigmoid(100.0 * 4.0);
  const double acc0 = vel1 / 1.0;
  const double acc1 = (vel2 - vel1) * 2.0 / 2.0;
  const double jerk = (acc1 - acc0) / (1.0 / 2.0 + 2.0 / 4.0);
  EXPECT_NEAR(edge.error()[0], jerk - 0.9, 1e-9);

  delete p1;
  delete p2;
  delete p3;
  delete dt1;
  delete dt2;
}

TEST(EdgeJerkGoal, GoalJerkPenalized) {
  auto params = makeParams();

  // dists 1, 2 -> vels 1, 2 scaled by sigmoid, vel_goal = 5 (measurement)
  // acc1 ~ 0.99995, acc2 ~ 3.00995 -> jerk ~ 2.01
  VertexPose *p1 = new VertexPose();
  p1->setEstimate(PoseSE2(0, 0, 0));
  VertexPose *p2 = new VertexPose();
  p2->setEstimate(PoseSE2(1, 0, 0));
  VertexPose *p3 = new VertexPose();
  p3->setEstimate(PoseSE2(3, 0, 0));
  VertexTimeDiff *dt1 = new VertexTimeDiff(1.0);
  VertexTimeDiff *dt2 = new VertexTimeDiff(1.0);

  geometry_msgs::msg::Twist vel_goal;
  vel_goal.linear.x = 5.0;
  vel_goal.angular.z = 0.0;

  EdgeJerkGoal edge;
  edge.setVertex(0, p1);
  edge.setVertex(1, p2);
  edge.setVertex(2, p3);
  edge.setVertex(3, dt1);
  edge.setVertex(4, dt2);
  edge.setGoalVelocity(vel_goal);
  edge.setTebConfig(params);
  edge.computeError();

  const double vel1 = 1.0 * fast_sigmoid(100.0 * 1.0);
  const double vel2 = 2.0 * fast_sigmoid(100.0 * 2.0);
  const double acc1 = (vel2 - vel1) * 2.0 / 2.0;
  const double acc2 = (5.0 - vel2) / 1.0;
  const double jerk = (acc2 - acc1) / (2.0 / 4.0 + 1.0 / 2.0);
  EXPECT_NEAR(edge.error()[0], jerk - 0.9, 1e-9);

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
