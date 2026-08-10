#include <gtest/gtest.h>

#include <Eigen/Core>
#include <cmath>

#include "nav2_teb_controller/g2o_types/edge_steering_angle_goal.h"
#include "nav2_teb_controller/g2o_types/vertex_pose.h"
#include "test_jacobian_utils.hpp"

using namespace nav2_teb_controller;

namespace {

teb_controller::Params makeParams() {
  teb_controller::Params p;
  p.FollowPath.robot.wheelbase = 1.055;
  p.FollowPath.optimizer.penalty_epsilon = 0.1;
  p.FollowPath.optimizer.exact_arc_length = false;
  return p;
}

}  // namespace

TEST(EdgeSteeringAngleGoal, CollinearPointsStillPenalizedQuirk) {
  auto params = makeParams();

  // Known quirk: a least-squares circle fit through collinear points is NOT the
  // straight-line case (R > 1e6). The LS solution for x = 0..4 on y = 0 is the
  // circle centered at (2, 0) with R^2 = 2 -> kappa = 1/sqrt(2) -> penalty.
  VertexPose *p1 = new VertexPose();
  p1->setEstimate(PoseSE2(0, 0, 0));
  VertexPose *p2 = new VertexPose();
  p2->setEstimate(PoseSE2(1, 0, 0));
  VertexPose *p3 = new VertexPose();
  p3->setEstimate(PoseSE2(2, 0, 0));
  VertexPose *p4 = new VertexPose();
  p4->setEstimate(PoseSE2(3, 0, 0));
  VertexPose *p5 = new VertexPose();
  p5->setEstimate(PoseSE2(4, 0, 0));

  EdgeSteeringAngleGoal edge;
  edge.setVertex(0, p1);
  edge.setVertex(1, p2);
  edge.setVertex(2, p3);
  edge.setVertex(3, p4);
  edge.setVertex(4, p5);
  edge.setTebConfig(params);
  edge.computeError();

  const double steering_angle = std::atan(1.055 / std::sqrt(2.0));
  const double expected = steering_angle - (M_PI_2 / 6.0 - 0.1);
  EXPECT_NEAR(edge.error()[0], expected, 1e-6);

  delete p1;
  delete p2;
  delete p3;
  delete p4;
  delete p5;
}

TEST(EdgeSteeringAngleGoal, WideArcZeroError) {
  auto params = makeParams();

  // Points on a circle of radius 10 -> steering angle ~ 0.105 < bound - eps
  const double a = 10.0 * M_PI / 180.0;
  const double r = 10.0;
  VertexPose *p1 = new VertexPose();
  p1->setEstimate(PoseSE2(r * std::cos(0 * a), r * std::sin(0 * a), 0));
  VertexPose *p2 = new VertexPose();
  p2->setEstimate(PoseSE2(r * std::cos(1 * a), r * std::sin(1 * a), 0));
  VertexPose *p3 = new VertexPose();
  p3->setEstimate(PoseSE2(r * std::cos(2 * a), r * std::sin(2 * a), 0));
  VertexPose *p4 = new VertexPose();
  p4->setEstimate(PoseSE2(r * std::cos(3 * a), r * std::sin(3 * a), 0));
  VertexPose *p5 = new VertexPose();
  p5->setEstimate(PoseSE2(r * std::cos(4 * a), r * std::sin(4 * a), 0));

  EdgeSteeringAngleGoal edge;
  edge.setVertex(0, p1);
  edge.setVertex(1, p2);
  edge.setVertex(2, p3);
  edge.setVertex(3, p4);
  edge.setVertex(4, p5);
  edge.setTebConfig(params);
  edge.computeError();

  EXPECT_NEAR(edge.error()[0], 0.0, 1e-6);

  delete p1;
  delete p2;
  delete p3;
  delete p4;
  delete p5;
}

TEST(EdgeSteeringAngleGoal, TightCirclePenalized) {
  auto params = makeParams();

  // Points on a unit circle -> kappa = 1 -> steering angle = atan(1.055) ~ 0.813
  // > pi/12 - eps -> penalized
  const double a = 18.0 * M_PI / 180.0;
  const double r = 1.0;
  VertexPose *p1 = new VertexPose();
  p1->setEstimate(PoseSE2(r * std::cos(0 * a), r * std::sin(0 * a), 0));
  VertexPose *p2 = new VertexPose();
  p2->setEstimate(PoseSE2(r * std::cos(1 * a), r * std::sin(1 * a), 0));
  VertexPose *p3 = new VertexPose();
  p3->setEstimate(PoseSE2(r * std::cos(2 * a), r * std::sin(2 * a), 0));
  VertexPose *p4 = new VertexPose();
  p4->setEstimate(PoseSE2(r * std::cos(3 * a), r * std::sin(3 * a), 0));
  VertexPose *p5 = new VertexPose();
  p5->setEstimate(PoseSE2(r * std::cos(4 * a), r * std::sin(4 * a), 0));

  EdgeSteeringAngleGoal edge;
  edge.setVertex(0, p1);
  edge.setVertex(1, p2);
  edge.setVertex(2, p3);
  edge.setVertex(3, p4);
  edge.setVertex(4, p5);
  edge.setTebConfig(params);
  edge.computeError();

  const double expected = std::atan(1.055) - (M_PI_2 / 6.0 - 0.1);
  EXPECT_NEAR(edge.error()[0], expected, 1e-6);

  delete p1;
  delete p2;
  delete p3;
  delete p4;
  delete p5;
}


// Analytic Jacobian vs finite differences. Five points on a radius-5 circle:
// A full rank, kappa = 0.2, steering 0.208 active (> pi/12 - eps) and away
// from the penalty cusp; positions only (theta components stay zero).
TEST(EdgeSteeringAngleGoal, JacobianMatchesNumeric) {
  auto params = makeParams();

  const double a = 18.0 * M_PI / 180.0;
  const double r = 5.0;
  VertexPose *p1 = new VertexPose();
  p1->setEstimate(PoseSE2(r * std::cos(0 * a), r * std::sin(0 * a), 0));
  VertexPose *p2 = new VertexPose();
  p2->setEstimate(PoseSE2(r * std::cos(1 * a), r * std::sin(1 * a), 0));
  VertexPose *p3 = new VertexPose();
  p3->setEstimate(PoseSE2(r * std::cos(2 * a), r * std::sin(2 * a), 0));
  VertexPose *p4 = new VertexPose();
  p4->setEstimate(PoseSE2(r * std::cos(3 * a), r * std::sin(3 * a), 0));
  VertexPose *p5 = new VertexPose();
  p5->setEstimate(PoseSE2(r * std::cos(4 * a), r * std::sin(4 * a), 0));

  EdgeSteeringAngleGoal edge;
  edge.setVertex(0, p1);
  edge.setVertex(1, p2);
  edge.setVertex(2, p3);
  edge.setVertex(3, p4);
  edge.setVertex(4, p5);
  edge.setTebConfig(params);
  edge.computeError();

  expectAnalyticJacobianMatchesNumericMulti(edge);

  delete p1;
  delete p2;
  delete p3;
  delete p4;
  delete p5;
}
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
