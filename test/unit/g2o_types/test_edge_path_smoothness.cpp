#include <gtest/gtest.h>

#include <Eigen/Core>

#include "nav2_teb_controller/g2o_types/edge_path_smoothness.h"
#include "nav2_teb_controller/g2o_types/vertex_pose.h"
#include "test_jacobian_utils.hpp"

using namespace nav2_teb_controller;

TEST(EdgePathSmoothness, ComputeError) {
  EdgePathSmoothness edge;

  VertexPose *v1 = new VertexPose();
  v1->setEstimate(PoseSE2(0, 0, 0.1));
  v1->setId(0);

  VertexPose *v2 = new VertexPose();
  v2->setEstimate(PoseSE2(0, 0, 0.5));
  v2->setId(1);

  edge.setVertex(0, v1);
  edge.setVertex(1, v2);

  edge.computeError();

  // Error is abs(normalize_angle(0.1 - 0.5)) = 0.4
  EXPECT_NEAR(edge.error()[0], 0.4, 1e-9);

  delete v1;
  delete v2;
}

// Analytic Jacobian vs finite differences: error = |normalize(t1 - t2)|,
// smooth away from the 0 and +/-pi wrap boundaries.
TEST(EdgePathSmoothness, JacobianMatchesNumeric) {
  EdgePathSmoothness edge;

  VertexPose *v1 = new VertexPose();
  v1->setEstimate(PoseSE2(0, 0, 0.2));

  VertexPose *v2 = new VertexPose();
  v2->setEstimate(PoseSE2(0, 0, 0.8));

  edge.setVertex(0, v1);
  edge.setVertex(1, v2);
  edge.computeError();

  expectAnalyticJacobianMatchesNumericBinary(edge);

  delete v1;
  delete v2;
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
