#include <gtest/gtest.h>

#include <Eigen/Core>

#include "nav2_teb_controller/g2o_types/edge_kinematic_car_like.h"
#include "nav2_teb_controller/g2o_types/vertex_pose.h"

using namespace nav2_teb_controller;

namespace {

teb_controller::Params makeParams(bool exact_arc_length) {
  teb_controller::Params p;
  p.FollowPath.robot.min_turning_radius = 1.0;
  p.FollowPath.optimizer.exact_arc_length = exact_arc_length;
  return p;
}

}  // namespace

TEST(EdgeKinematicsCarlike, StraightLineZeroError) {
  auto params = makeParams(false);

  VertexPose *p0 = new VertexPose();
  p0->setEstimate(PoseSE2(0, 0, 0));
  VertexPose *p1 = new VertexPose();
  p1->setEstimate(PoseSE2(2, 0, 0));

  EdgeKinematicsCarlike edge;
  edge.setVertex(0, p0);
  edge.setVertex(1, p1);
  edge.setTebConfig(params);
  edge.computeError();

  EXPECT_NEAR(edge.error()[0], 0.0, 1e-9);
  EXPECT_NEAR(edge.error()[1], 0.0, 1e-9);  // angle_diff == 0 -> straight line

  delete p0;
  delete p1;
}

TEST(EdgeKinematicsCarlike, TightTurnPenalized) {
  VertexPose *p0 = new VertexPose();
  p0->setEstimate(PoseSE2(0, 0, 0));
  VertexPose *p1 = new VertexPose();
  p1->setEstimate(PoseSE2(1, 0, M_PI_2));

  // radius = |dS| / angle_diff = 1 / (pi/2) = 0.6366 < 1.0 -> penalized
  {
    auto params = makeParams(false);  // approximate arc length
    EdgeKinematicsCarlike edge;
    edge.setVertex(0, p0);
    edge.setVertex(1, p1);
    edge.setTebConfig(params);
    edge.computeError();

    const double radius = 1.0 / M_PI_2;
    EXPECT_NEAR(edge.error()[1], 1.0 - radius, 1e-9);
  }

  // exact arc length: radius = |dS| / (2 sin(angle/2)) = 1 / sqrt(2) = 0.7071
  {
    auto params = makeParams(true);
    EdgeKinematicsCarlike edge;
    edge.setVertex(0, p0);
    edge.setVertex(1, p1);
    edge.setTebConfig(params);
    edge.computeError();

    const double radius = 1.0 / (2.0 * std::sin(M_PI_2 / 2.0));
    EXPECT_NEAR(edge.error()[1], 1.0 - radius, 1e-9);
  }

  delete p0;
  delete p1;
}

TEST(EdgeKinematicsCarlike, WideTurnZeroError) {
  auto params = makeParams(false);

  VertexPose *p0 = new VertexPose();
  p0->setEstimate(PoseSE2(0, 0, 0));
  VertexPose *p1 = new VertexPose();
  p1->setEstimate(PoseSE2(2, 0, M_PI_2));

  EdgeKinematicsCarlike edge;
  edge.setVertex(0, p0);
  edge.setVertex(1, p1);
  edge.setTebConfig(params);
  edge.computeError();

  // radius = 2/(pi/2) = 1.273 > 1.0 -> ok
  EXPECT_NEAR(edge.error()[1], 0.0, 1e-9);

  delete p0;
  delete p1;
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
