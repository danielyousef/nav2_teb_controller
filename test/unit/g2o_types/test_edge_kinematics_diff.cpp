#include <gtest/gtest.h>

#include <Eigen/Core>

#include "nav2_teb_controller/g2o_types/edge_kinematics_diff.h"
#include "nav2_teb_controller/g2o_types/vertex_pose.h"
#include "test_jacobian_utils.hpp"

using namespace nav2_teb_controller;

namespace {

teb_controller::Params makeParams() {
  teb_controller::Params p;
  p.FollowPath.optimizer.exact_arc_length = false;
  return p;
}

}  // namespace

TEST(EdgeKinematicsDiffDrive, StraightForwardZeroError) {
  auto params = makeParams();

  VertexPose *p0 = new VertexPose();
  p0->setEstimate(PoseSE2(0, 0, 0));
  VertexPose *p1 = new VertexPose();
  p1->setEstimate(PoseSE2(1, 0, 0));

  EdgeKinematicsDiffDrive edge;
  edge.setVertex(0, p0);
  edge.setVertex(1, p1);
  edge.setTebConfig(params);
  edge.computeError();

  EXPECT_NEAR(edge.error()[0], 0.0, 1e-9);
  EXPECT_NEAR(edge.error()[1], 0.0, 1e-9);

  delete p0;
  delete p1;
}

TEST(EdgeKinematicsDiffDrive, BackwardMotionPenalized) {
  auto params = makeParams();

  VertexPose *p0 = new VertexPose();
  p0->setEstimate(PoseSE2(0, 0, 0));
  VertexPose *p1 = new VertexPose();
  p1->setEstimate(PoseSE2(-1, 0, 0));

  EdgeKinematicsDiffDrive edge;
  edge.setVertex(0, p0);
  edge.setVertex(1, p1);
  edge.setTebConfig(params);
  edge.computeError();

  EXPECT_NEAR(edge.error()[0], 0.0, 1e-9);
  EXPECT_NEAR(edge.error()[1], 1.0, 1e-9);  // dot = -1 -> bound 0 with eps 0

  delete p0;
  delete p1;
}

TEST(EdgeKinematicsDiffDrive, LateralMotionNonHolonomicViolation) {
  auto params = makeParams();

  // p0 faces +x, p1 faces +y, displacement is diagonal: non-holonomic violation
  VertexPose *p0 = new VertexPose();
  p0->setEstimate(PoseSE2(0, 0, 0));
  VertexPose *p1 = new VertexPose();
  p1->setEstimate(PoseSE2(1, 1, M_PI_2));

  EdgeKinematicsDiffDrive edge;
  edge.setVertex(0, p0);
  edge.setVertex(1, p1);
  edge.setTebConfig(params);
  edge.computeError();

  // |(cos0+cos90)*1 - (sin0+sin90)*1| = |1 - 1| = 0
  EXPECT_NEAR(edge.error()[0], 0.0, 1e-9);
  // forward component positive -> no backward penalty
  EXPECT_NEAR(edge.error()[1], 0.0, 1e-9);

  // Perpendicular lateral motion: p1 straight up, headings along +x
  p1->setEstimate(PoseSE2(0, 1, 0));
  edge.computeError();
  // |(1+1)*1 - 0| = 2
  EXPECT_NEAR(edge.error()[0], 2.0, 1e-9);
  EXPECT_NEAR(edge.error()[1], 0.0, 1e-9);

  delete p0;
  delete p1;
}


// Analytic Jacobian vs finite differences. Both rows active: non-holonomic
// bracket = 0.539 != 0, forward-drive dot < 0 (penalized).
TEST(EdgeKinematicsDiffDrive, JacobianMatchesNumeric) {
  auto params = makeParams();

  VertexPose *p0 = new VertexPose();
  p0->setEstimate(PoseSE2(0, 0, 0.1));
  VertexPose *p1 = new VertexPose();
  p1->setEstimate(PoseSE2(-1, 0.05, 0.15));

  EdgeKinematicsDiffDrive edge;
  edge.setVertex(0, p0);
  edge.setVertex(1, p1);
  edge.setTebConfig(params);
  edge.computeError();

  expectAnalyticJacobianMatchesNumericBinary(edge);

  delete p0;
  delete p1;
}
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
