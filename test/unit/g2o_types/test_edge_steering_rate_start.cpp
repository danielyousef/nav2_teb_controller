#include <gtest/gtest.h>

#include <Eigen/Core>
#include <cmath>

#include "nav2_teb_controller/g2o_types/edge_steering_rate_start.h"
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

TEST(EdgeSteeringRateStart, MatchingInitialAngleZeroError) {
  auto params = makeParams();

  // phi = atan(1.055*0.2) ~ 0.208, measured = 0.2 -> rate ~ 0.008
  VertexPose *p1 = new VertexPose();
  p1->setEstimate(PoseSE2(0, 0, 0));
  VertexPose *p2 = new VertexPose();
  p2->setEstimate(PoseSE2(1, 0, 0.2));
  VertexTimeDiff *dt = new VertexTimeDiff(1.0);

  EdgeSteeringRateStart edge;
  edge.setVertex(0, p1);
  edge.setVertex(1, p2);
  edge.setVertex(2, dt);
  edge.setInitialSteeringAngle(0.2);
  edge.setTebConfig(params);
  edge.computeError();

  EXPECT_NEAR(edge.error()[0], 0.0, 1e-9);

  delete p1;
  delete p2;
  delete dt;
}

TEST(EdgeSteeringRateStart, HighInitialRatePenalized) {
  auto params = makeParams();

  VertexPose *p1 = new VertexPose();
  p1->setEstimate(PoseSE2(0, 0, 0));
  VertexPose *p2 = new VertexPose();
  p2->setEstimate(PoseSE2(1, 0, 0.2));
  VertexTimeDiff *dt = new VertexTimeDiff(1.0);

  EdgeSteeringRateStart edge;
  edge.setVertex(0, p1);
  edge.setVertex(1, p2);
  edge.setVertex(2, dt);
  edge.setInitialSteeringAngle(0.0);
  edge.setTebConfig(params);
  edge.computeError();

  // phi = atan(1.055*0.2) * fast_sigmoid(100), rate = phi/1 > 0.25 - eps
  const double phi = std::atan(1.055 * 0.2) * fast_sigmoid(100.0);
  const double expected = phi - (0.25 - 0.1);
  EXPECT_NEAR(edge.error()[0], expected, 1e-6);

  delete p1;
  delete p2;
  delete dt;
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
