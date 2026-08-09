#include <gtest/gtest.h>

#include <Eigen/Core>
#include <cmath>

#include "nav2_teb_controller/g2o_types/edge_steering_rate.h"
#include "nav2_teb_controller/g2o_types/vertex_pose.h"
#include "nav2_teb_controller/g2o_types/vertex_timediff.h"

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

TEST(EdgeSteeringRate, StraightLineZeroError) {
  auto params = makeParams();

  VertexPose *p1 = new VertexPose();
  p1->setEstimate(PoseSE2(0, 0, 0));
  VertexPose *p2 = new VertexPose();
  p2->setEstimate(PoseSE2(1, 0, 0));
  VertexPose *p3 = new VertexPose();
  p3->setEstimate(PoseSE2(2, 0, 0));
  VertexTimeDiff *dt1 = new VertexTimeDiff(1.0);
  VertexTimeDiff *dt2 = new VertexTimeDiff(1.0);

  EdgeSteeringRate edge;
  edge.setVertex(0, p1);
  edge.setVertex(1, p2);
  edge.setVertex(2, p3);
  edge.setVertex(3, dt1);
  edge.setVertex(4, dt2);
  edge.setTebConfig(params);
  edge.computeError();

  EXPECT_NEAR(edge.error()[0], 0.0, 1e-9);

  delete p1;
  delete p2;
  delete p3;
  delete dt1;
  delete dt2;
}

TEST(EdgeSteeringRate, DegenerateSegmentZeroError) {
  auto params = makeParams();

  // dist2 = 0 -> early return, zero error
  VertexPose *p1 = new VertexPose();
  p1->setEstimate(PoseSE2(0, 0, 0));
  VertexPose *p2 = new VertexPose();
  p2->setEstimate(PoseSE2(1, 0, 0.3));
  VertexPose *p3 = new VertexPose();
  p3->setEstimate(PoseSE2(1, 0, 0.3));
  VertexTimeDiff *dt1 = new VertexTimeDiff(1.0);
  VertexTimeDiff *dt2 = new VertexTimeDiff(1.0);

  EdgeSteeringRate edge;
  edge.setVertex(0, p1);
  edge.setVertex(1, p2);
  edge.setVertex(2, p3);
  edge.setVertex(3, dt1);
  edge.setVertex(4, dt2);
  edge.setTebConfig(params);
  edge.computeError();

  EXPECT_DOUBLE_EQ(edge.error()[0], 0.0);

  delete p1;
  delete p2;
  delete p3;
  delete dt1;
  delete dt2;
}

TEST(EdgeSteeringRate, HighRatePenalized) {
  auto params = makeParams();

  // phi1 = atan(1.055*0.5)       ~ 0.4857
  // phi2 = atan(1.055*0.1/2.01)  ~ 0.0525
  // steer_rate = (phi2 - phi1) ~ -0.4332 < -(0.25 - 0.1)
  VertexPose *p1 = new VertexPose();
  p1->setEstimate(PoseSE2(0, 0, 0));
  VertexPose *p2 = new VertexPose();
  p2->setEstimate(PoseSE2(1, 0, 0.5));
  VertexPose *p3 = new VertexPose();
  p3->setEstimate(PoseSE2(3, 0.2, 0.6));
  VertexTimeDiff *dt1 = new VertexTimeDiff(1.0);
  VertexTimeDiff *dt2 = new VertexTimeDiff(1.0);

  EdgeSteeringRate edge;
  edge.setVertex(0, p1);
  edge.setVertex(1, p2);
  edge.setVertex(2, p3);
  edge.setVertex(3, dt1);
  edge.setVertex(4, dt2);
  edge.setTebConfig(params);
  edge.computeError();

  const double phi1 = std::atan(1.055 * 0.5);
  const double phi2 = std::atan(1.055 * 0.1 / std::sqrt(4.0 + 0.04));
  const double steer_rate = phi2 - phi1;
  // var < -a+eps branch: error = -var - (a - eps)
  const double expected = -steer_rate - (0.25 - 0.1);
  EXPECT_NEAR(edge.error()[0], expected, 1e-6);

  delete p1;
  delete p2;
  delete p3;
  delete dt1;
  delete dt2;
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
