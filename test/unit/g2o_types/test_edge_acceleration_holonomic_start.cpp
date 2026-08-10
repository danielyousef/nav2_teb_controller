#include <gtest/gtest.h>

#include <Eigen/Core>
#include <geometry_msgs/msg/twist.hpp>

#include "nav2_teb_controller/g2o_types/edge_acceleration_holonomic_start.h"
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

TEST(EdgeAccelerationHolonomicStart, RotatedFrame) {
  auto params = makeParams();

  // pose1 heading pi/2, displacement along +y (robot +x): vel1_x = 1.0
  VertexPose *p1 = new VertexPose();
  p1->setEstimate(PoseSE2(0, 0, M_PI_2));
  VertexPose *p2 = new VertexPose();
  p2->setEstimate(PoseSE2(0, 1, M_PI_2));
  VertexTimeDiff *dt = new VertexTimeDiff(1.0);

  geometry_msgs::msg::Twist vel_start;
  vel_start.linear.x = 1.0;  // matches vel2 -> zero error
  vel_start.linear.y = 0.0;
  vel_start.angular.z = 0.0;

  EdgeAccelerationHolonomicStart edge;
  edge.setVertex(0, p1);
  edge.setVertex(1, p2);
  edge.setVertex(2, dt);
  edge.setInitialVelocity(vel_start);
  edge.setTebConfig(params);
  edge.computeError();

  EXPECT_NEAR(edge.error()[0], 0.0, 1e-9);
  EXPECT_NEAR(edge.error()[1], 0.0, 1e-9);
  EXPECT_NEAR(edge.error()[2], 0.0, 1e-9);

  delete p1;
  delete p2;
  delete dt;
}


// Analytic Jacobian vs finite differences. acc_x = 3.6, acc_y = 1.8,
// acc_rot = 1.4: all three components active.
TEST(EdgeAccelerationHolonomicStart, JacobianMatchesNumeric) {
  auto params = makeParams();

  geometry_msgs::msg::Twist vel_start;
  vel_start.linear.x = 0.2;
  vel_start.linear.y = 0.1;
  vel_start.angular.z = -0.1;

  VertexPose *p1 = new VertexPose();
  p1->setEstimate(PoseSE2(0, 0, 0));
  VertexPose *p2 = new VertexPose();
  p2->setEstimate(PoseSE2(1.0, 0.5, 0.3));
  VertexTimeDiff *dt = new VertexTimeDiff(0.5);

  EdgeAccelerationHolonomicStart edge;
  edge.setInitialVelocity(vel_start);
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
