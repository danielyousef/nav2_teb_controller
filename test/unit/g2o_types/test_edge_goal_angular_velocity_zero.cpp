#include <gtest/gtest.h>

#include <Eigen/Core>
#include <cmath>
#include <vector>

#include "nav2_teb_controller/g2o_types/edge_goal_angular_velocity_zero.h"
#include "nav2_teb_controller/g2o_types/vertex_pose.h"
#include "nav2_teb_controller/g2o_types/vertex_timediff.h"
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

TEST(EdgeGoalAngularVelocityZero, StandingStillZeroError) {
  auto params = makeParams();

  VertexPose *p1 = new VertexPose();
  p1->setEstimate(PoseSE2(0, 0, 0));
  VertexPose *p2 = new VertexPose();
  p2->setEstimate(PoseSE2(0, 0, 0));
  VertexPose *p3 = new VertexPose();
  p3->setEstimate(PoseSE2(0, 0, 0));
  VertexPose *p4 = new VertexPose();
  p4->setEstimate(PoseSE2(0, 0, 0));
  VertexPose *p5 = new VertexPose();
  p5->setEstimate(PoseSE2(0, 0, 0));
  VertexTimeDiff *dt1 = new VertexTimeDiff(1.0);
  VertexTimeDiff *dt2 = new VertexTimeDiff(1.0);
  VertexTimeDiff *dt3 = new VertexTimeDiff(1.0);
  VertexTimeDiff *dt4 = new VertexTimeDiff(1.0);

  EdgeGoalAngularVelocityZero edge;
  edge.setVertex(0, p1);
  edge.setVertex(1, p2);
  edge.setVertex(2, p3);
  edge.setVertex(3, p4);
  edge.setVertex(4, p5);
  edge.setVertex(5, dt1);
  edge.setVertex(6, dt2);
  edge.setVertex(7, dt3);
  edge.setVertex(8, dt4);
  edge.setTebConfig(params);
  edge.computeError();

  EXPECT_DOUBLE_EQ(edge.error()[0], 0.0);  // v ~ 0 -> early return

  delete p1;
  delete p2;
  delete p3;
  delete p4;
  delete p5;
  delete dt1;
  delete dt2;
  delete dt3;
  delete dt4;
}

TEST(EdgeGoalAngularVelocityZero, RotatingTrajectoryPenalized) {
  auto params = makeParams();

  // theta profile 0, 0.5, 1.0, 1.5, 2.0 with dt = 1.
  // Known quirk: the unwrap double-adds the reference (theta[i+1] = theta[i] +
  // normalize(theta[i+1] + theta[i])) and normalize_angle wraps values > pi,
  // so the regression slope comes out as ~ -0.0416 instead of 0.5. We replicate
  // the buggy recurrence here to lock in the current behavior.
  VertexPose *p1 = new VertexPose();
  p1->setEstimate(PoseSE2(0, 0, 0.0));
  VertexPose *p2 = new VertexPose();
  p2->setEstimate(PoseSE2(1, 0, 0.5));
  VertexPose *p3 = new VertexPose();
  p3->setEstimate(PoseSE2(2, 0, 1.0));
  VertexPose *p4 = new VertexPose();
  p4->setEstimate(PoseSE2(3, 0, 1.5));
  VertexPose *p5 = new VertexPose();
  p5->setEstimate(PoseSE2(4, 0, 2.0));
  VertexTimeDiff *dt1 = new VertexTimeDiff(1.0);
  VertexTimeDiff *dt2 = new VertexTimeDiff(1.0);
  VertexTimeDiff *dt3 = new VertexTimeDiff(1.0);
  VertexTimeDiff *dt4 = new VertexTimeDiff(1.0);

  EdgeGoalAngularVelocityZero edge;
  edge.setVertex(0, p1);
  edge.setVertex(1, p2);
  edge.setVertex(2, p3);
  edge.setVertex(3, p4);
  edge.setVertex(4, p5);
  edge.setVertex(5, dt1);
  edge.setVertex(6, dt2);
  edge.setVertex(7, dt3);
  edge.setVertex(8, dt4);
  edge.setTebConfig(params);
  edge.computeError();

  std::vector<double> theta(5);
  theta[0] = 0.0;
  theta[1] = theta[0] + std::remainder(0.5 + theta[0], 2.0 * M_PI);
  theta[2] = theta[1] + std::remainder(1.0 + theta[1], 2.0 * M_PI);
  theta[3] = theta[2] + std::remainder(1.5 + theta[2], 2.0 * M_PI);
  theta[4] = theta[3] + std::remainder(2.0 + theta[3], 2.0 * M_PI);

  const std::vector<double> t = {0.0, 1.0, 2.0, 3.0, 4.0};
  const double t_mean = 2.0;
  double theta_mean = 0.0;
  for (double th : theta) { theta_mean += th; }
  theta_mean /= 5.0;
  double num = 0.0, den = 0.0;
  for (int i = 0; i < 5; ++i) {
    num += (t[i] - t_mean) * (theta[i] - theta_mean);
    den += (t[i] - t_mean) * (t[i] - t_mean);
  }
  const double omega = num / den;
  const double v = 4.0 / 4.0;
  // hand-computed check: slope ~ -0.0416 -> steering angle ~ -0.0439 rad
  EXPECT_NEAR(edge.error()[0], std::atan2(omega * 1.055, v), 1e-9);
  EXPECT_NEAR(edge.error()[0], -0.0439, 1e-3);

  delete p1;
  delete p2;
  delete p3;
  delete p4;
  delete p5;
  delete dt1;
  delete dt2;
  delete dt3;
  delete dt4;
}


// Analytic Jacobian vs finite differences. Gentle arc: v ~ 0.92 m/s (above the
// 1e-4 guard), non-trivial omega from the LSQ slope; all normalize_angle()
// arguments of the theta recursion are >= 0.15 rad away from +/-pi.
TEST(EdgeGoalAngularVelocityZero, JacobianMatchesNumeric) {
  auto params = makeParams();

  VertexPose *p1 = new VertexPose();
  p1->setEstimate(PoseSE2(0.0, 0.0, 1.0));
  VertexPose *p2 = new VertexPose();
  p2->setEstimate(PoseSE2(0.8, 0.1, 1.2));
  VertexPose *p3 = new VertexPose();
  p3->setEstimate(PoseSE2(1.6, 0.4, 1.4));
  VertexPose *p4 = new VertexPose();
  p4->setEstimate(PoseSE2(2.4, 0.9, 1.6));
  VertexPose *p5 = new VertexPose();
  p5->setEstimate(PoseSE2(3.2, 1.6, 1.8));
  VertexTimeDiff *dt1 = new VertexTimeDiff(1.0);
  VertexTimeDiff *dt2 = new VertexTimeDiff(1.0);
  VertexTimeDiff *dt3 = new VertexTimeDiff(1.0);
  VertexTimeDiff *dt4 = new VertexTimeDiff(1.0);

  EdgeGoalAngularVelocityZero edge;
  edge.setVertex(0, p1);
  edge.setVertex(1, p2);
  edge.setVertex(2, p3);
  edge.setVertex(3, p4);
  edge.setVertex(4, p5);
  edge.setVertex(5, dt1);
  edge.setVertex(6, dt2);
  edge.setVertex(7, dt3);
  edge.setVertex(8, dt4);
  edge.setTebConfig(params);
  edge.computeError();

  expectAnalyticJacobianMatchesNumericMulti(edge);

  delete p1;
  delete p2;
  delete p3;
  delete p4;
  delete p5;
  delete dt1;
  delete dt2;
  delete dt3;
  delete dt4;
}
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
