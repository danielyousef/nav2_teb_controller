#include <gtest/gtest.h>

#include <Eigen/Core>
#include <cmath>

#include "nav2_teb_controller/g2o_types/edge_start_steering_angle.h"
#include "nav2_teb_controller/g2o_types/vertex_pose.h"
#include "nav2_teb_controller/g2o_types/vertex_timediff.h"
#include "nav2_teb_controller/math_utils.hpp"

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

TEST(EdgeStartSteeringAngle, StraightStartZeroError) {
  auto params = makeParams();

  VertexPose *p0 = new VertexPose();
  p0->setEstimate(PoseSE2(0, 0, 0));
  VertexPose *p1 = new VertexPose();
  p1->setEstimate(PoseSE2(1, 0, 0));
  VertexPose *p2 = new VertexPose();
  p2->setEstimate(PoseSE2(2, 0, 0));
  VertexTimeDiff *dt0 = new VertexTimeDiff(1.0);
  VertexTimeDiff *dt1 = new VertexTimeDiff(1.0);

  EdgeStartSteeringAngle edge;
  edge.setVertex(0, p0);
  edge.setVertex(1, p1);
  edge.setVertex(2, p2);
  edge.setVertex(3, dt0);
  edge.setVertex(4, dt1);
  edge.setInitialSteeringAngle(0.0);
  edge.setTebConfig(params);
  edge.computeError();

  EXPECT_NEAR(edge.error()[0], 0.0, 1e-9);
  EXPECT_DOUBLE_EQ(edge.error()[1], 0.0);  // second error term is hardcoded to zero

  delete p0;
  delete p1;
  delete p2;
  delete dt0;
  delete dt1;
}

TEST(EdgeStartSteeringAngle, HighRequiredRatePenalized) {
  auto params = makeParams();

  // planned_omega = 1.0 rad/s, planned_vx = 2 m/s * fast_sigmoid(100)
  // cmd = atan2(1.0*1.055, vx) -> rate = cmd / 0.5
  VertexPose *p0 = new VertexPose();
  p0->setEstimate(PoseSE2(0, 0, 0));
  VertexPose *p1 = new VertexPose();
  p1->setEstimate(PoseSE2(1, 0, 0.5));
  VertexPose *p2 = new VertexPose();
  p2->setEstimate(PoseSE2(2, 0, 0.5));
  VertexTimeDiff *dt0 = new VertexTimeDiff(0.5);
  VertexTimeDiff *dt1 = new VertexTimeDiff(0.5);

  EdgeStartSteeringAngle edge;
  edge.setVertex(0, p0);
  edge.setVertex(1, p1);
  edge.setVertex(2, p2);
  edge.setVertex(3, dt0);
  edge.setVertex(4, dt1);
  edge.setInitialSteeringAngle(0.0);
  edge.setTebConfig(params);
  edge.computeError();

  // penaltyBoundFromBelow(steering_rate_max=0.25, |rate|, eps=0.1)
  const double vx = (1.0 / 0.5) * fast_sigmoid(100.0 * 1.0);
  const double cmd = std::atan2(1.0 * 1.055, vx);
  const double rate = cmd / 0.5;
  const double expected = rate + 0.1 - 0.25;
  EXPECT_NEAR(edge.error()[0], expected, 1e-6);

  delete p0;
  delete p1;
  delete p2;
  delete dt0;
  delete dt1;
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
