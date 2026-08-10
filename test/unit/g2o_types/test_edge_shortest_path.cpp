#include <gtest/gtest.h>

#include <Eigen/Core>

#include "nav2_teb_controller/g2o_types/edge_shortest_path.h"
#include "nav2_teb_controller/g2o_types/vertex_pose.h"
#include "test_jacobian_utils.hpp"

using namespace nav2_teb_controller;

TEST(EdgeShortestPath, ComputeError) {
  EdgeShortestPath edge;

  VertexPose *v1 = new VertexPose();
  v1->setEstimate(PoseSE2(0, 0, 0));
  v1->setId(0);

  VertexPose *v2 = new VertexPose();
  v2->setEstimate(PoseSE2(3, 4, 0));
  v2->setId(1);

  edge.setVertex(0, v1);
  edge.setVertex(1, v2);

  edge.computeError();

  // Euclidean distance between (0,0) and (3,4) is 5.0
  EXPECT_DOUBLE_EQ(edge.error()[0], 5.0);

  delete v1;
  delete v2;
}

// Analytic Jacobian vs finite differences: error = |p2 - p1|, smooth for dist > 0.
TEST(EdgeShortestPath, JacobianMatchesNumeric) {
  EdgeShortestPath edge;

  VertexPose *v1 = new VertexPose();
  v1->setEstimate(PoseSE2(0, 0, 0.2));

  VertexPose *v2 = new VertexPose();
  v2->setEstimate(PoseSE2(1.5, 0.5, 0.3));

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
