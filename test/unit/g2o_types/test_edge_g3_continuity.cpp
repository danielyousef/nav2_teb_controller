#include <gtest/gtest.h>

#include <Eigen/Core>

#include "nav2_teb_controller/g2o_types/edge_g3_continuity.h"
#include "nav2_teb_controller/g2o_types/vertex_pose.h"
#include "nav2_teb_controller/g2o_types/vertex_timediff.h"

using namespace nav2_teb_controller;

namespace {

teb_controller::Params makeParams() {
  teb_controller::Params p;
  p.FollowPath.robot.wheelbase = 1.055;
  p.FollowPath.robot.steering_rate_max = 0.25;
  p.FollowPath.optimizer.penalty_epsilon = 0.1;
  return p;
}

}  // namespace

TEST(EdgeG3Continuity, StraightLineZeroError) {
  auto params = makeParams();

  VertexPose *p0 = new VertexPose();
  p0->setEstimate(PoseSE2(0, 0, 0));
  VertexPose *p1 = new VertexPose();
  p1->setEstimate(PoseSE2(1, 0, 0));
  VertexPose *p2 = new VertexPose();
  p2->setEstimate(PoseSE2(2, 0, 0));
  VertexTimeDiff *dt1 = new VertexTimeDiff(1.0);
  VertexTimeDiff *dt2 = new VertexTimeDiff(1.0);

  EdgeG3Continuity edge;
  edge.setVertex(0, p0);
  edge.setVertex(1, p1);
  edge.setVertex(2, p2);
  edge.setVertex(3, dt1);
  edge.setVertex(4, dt2);
  edge.setTebConfig(params);
  edge.computeError();

  EXPECT_NEAR(edge.error()[0], 0.0, 1e-9);

  delete p0;
  delete p1;
  delete p2;
  delete dt1;
  delete dt2;
}

TEST(EdgeG3Continuity, DegeneratePosesZeroError) {
  auto params = makeParams();

  VertexPose *p0 = new VertexPose();
  p0->setEstimate(PoseSE2(1, 1, 0.3));
  VertexPose *p1 = new VertexPose();
  p1->setEstimate(PoseSE2(1, 1, 0.3));
  VertexPose *p2 = new VertexPose();
  p2->setEstimate(PoseSE2(1, 1, 0.3));
  VertexTimeDiff *dt1 = new VertexTimeDiff(1.0);
  VertexTimeDiff *dt2 = new VertexTimeDiff(1.0);

  EdgeG3Continuity edge;
  edge.setVertex(0, p0);
  edge.setVertex(1, p1);
  edge.setVertex(2, p2);
  edge.setVertex(3, dt1);
  edge.setVertex(4, dt2);
  edge.setTebConfig(params);
  edge.computeError();

  // dist < 1e-4 -> early return with zero error
  EXPECT_DOUBLE_EQ(edge.error()[0], 0.0);

  delete p0;
  delete p1;
  delete p2;
  delete dt1;
  delete dt2;
}

TEST(EdgeG3Continuity, ConstantCurvatureZeroError) {
  auto params = makeParams();

  // Same curvature both segments -> steer angles identical -> rate 0 -> no penalty
  VertexPose *p0 = new VertexPose();
  p0->setEstimate(PoseSE2(0, 0, 0));
  VertexPose *p1 = new VertexPose();
  p1->setEstimate(PoseSE2(1, 0, 0.2));
  VertexPose *p2 = new VertexPose();
  p2->setEstimate(PoseSE2(2, 0, 0.4));
  VertexTimeDiff *dt1 = new VertexTimeDiff(1.0);
  VertexTimeDiff *dt2 = new VertexTimeDiff(1.0);

  EdgeG3Continuity edge;
  edge.setVertex(0, p0);
  edge.setVertex(1, p1);
  edge.setVertex(2, p2);
  edge.setVertex(3, dt1);
  edge.setVertex(4, dt2);
  edge.setTebConfig(params);
  edge.computeError();

  EXPECT_NEAR(edge.error()[0], 0.0, 1e-9);

  delete p0;
  delete p1;
  delete p2;
  delete dt1;
  delete dt2;
}

TEST(EdgeG3Continuity, HighSteeringRatePenalized) {
  auto params = makeParams();

  // Segment 1: delta_theta=0.6 over 1m  -> phi1 = atan(0.6*1.055)        ≈ 0.56397
  // Segment 2: delta_theta=0.4 over 2.236m -> phi2 = atan(0.4/2.236*1.055) ≈ 0.18656
  // steer_rate = |phi2 - phi1| / avg_dt ≈ 0.37741 > 0.25 - eps -> penalized
  VertexPose *p0 = new VertexPose();
  p0->setEstimate(PoseSE2(0, 0, 0));
  VertexPose *p1 = new VertexPose();
  p1->setEstimate(PoseSE2(1, 0, 0.6));
  VertexPose *p2 = new VertexPose();
  p2->setEstimate(PoseSE2(3, 1, 1.0));
  VertexTimeDiff *dt1 = new VertexTimeDiff(1.0);
  VertexTimeDiff *dt2 = new VertexTimeDiff(1.0);

  EdgeG3Continuity edge;
  edge.setVertex(0, p0);
  edge.setVertex(1, p1);
  edge.setVertex(2, p2);
  edge.setVertex(3, dt1);
  edge.setVertex(4, dt2);
  edge.setTebConfig(params);
  edge.computeError();

  // penaltyBoundFromBelow(steering_rate_max=0.25, steer_rate, eps=0.1):
  // 0.25 >= rate + 0.1 ? 0 : (rate + 0.1 - 0.25)
  const double phi1 = std::atan(0.6 * 1.055);
  const double phi2 = std::atan(0.4 / std::sqrt(2.0 * 2.0 + 1.0) * 1.055);
  const double steer_rate = std::abs(phi2 - phi1);
  const double expected = (0.25 >= steer_rate + 0.1) ? 0.0 : steer_rate + 0.1 - 0.25;
  EXPECT_NEAR(edge.error()[0], expected, 1e-6);

  delete p0;
  delete p1;
  delete p2;
  delete dt1;
  delete dt2;
}

TEST(EdgeG3Continuity, UnwrapSteeringAngle) {
  // Quirk: normalize(phi + pi - ref) and normalize(phi - pi - ref) are the same
  // residue mod 2pi, so d1 == d2 up to ~1 ULP of floating-point rounding. The
  // tie is therefore broken by FP noise: sometimes phi is kept, sometimes
  // phi +- pi is returned. Deterministic cases:
  EXPECT_DOUBLE_EQ(EdgeG3Continuity::unwrapSteeringAngle(0.7, 0.5), 0.7);
  EXPECT_DOUBLE_EQ(EdgeG3Continuity::unwrapSteeringAngle(-2.5, 0.5), -2.5);
  EXPECT_DOUBLE_EQ(EdgeG3Continuity::unwrapSteeringAngle(-0.5, 0.5), -0.5);

  // FP-noise case: result must be one of the equivalent candidates (any would
  // be acceptable semantically), observed on this platform: 2.8 + pi.
  const double r = EdgeG3Continuity::unwrapSteeringAngle(2.8, 0.5);
  const bool valid = (std::abs(r - 2.8) < 1e-12) || (std::abs(r - (2.8 + M_PI)) < 1e-12) ||
                     (std::abs(r - (2.8 - M_PI)) < 1e-12);
  EXPECT_TRUE(valid);
  EXPECT_NEAR(r, 2.8 + M_PI, 1e-12);  // observed behavior on this platform
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
