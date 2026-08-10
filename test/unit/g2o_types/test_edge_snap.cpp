#include <gtest/gtest.h>

#include <Eigen/Core>

#include "nav2_teb_controller/g2o_types/edge_snap.h"
#include "nav2_teb_controller/g2o_types/vertex_pose.h"
#include "nav2_teb_controller/g2o_types/vertex_timediff.h"
#include "nav2_teb_controller/math_utils.hpp"
#include "test_jacobian_utils.hpp"

using namespace nav2_teb_controller;

namespace {

teb_controller::Params makeParams() {
  teb_controller::Params p;
  p.FollowPath.robot.snap_max_x = 1.0;
  p.FollowPath.robot.snap_max_theta = 2.0;
  p.FollowPath.optimizer.penalty_epsilon = 0.1;
  p.FollowPath.optimizer.exact_arc_length = false;
  return p;
}

}  // namespace

TEST(EdgeSnap, ConstantMotionZeroError) {
  auto params = makeParams();

  // Uniform linear motion: all vel = 1 m/s, accelerations = 0 -> snap 0
  VertexPose *p0 = new VertexPose();
  p0->setEstimate(PoseSE2(0, 0, 0));
  VertexPose *p1 = new VertexPose();
  p1->setEstimate(PoseSE2(1, 0, 0));
  VertexPose *p2 = new VertexPose();
  p2->setEstimate(PoseSE2(2, 0, 0));
  VertexPose *p3 = new VertexPose();
  p3->setEstimate(PoseSE2(3, 0, 0));
  VertexPose *p4 = new VertexPose();
  p4->setEstimate(PoseSE2(4, 0, 0));
  VertexTimeDiff *dt1 = new VertexTimeDiff(1.0);
  VertexTimeDiff *dt2 = new VertexTimeDiff(1.0);
  VertexTimeDiff *dt3 = new VertexTimeDiff(1.0);
  VertexTimeDiff *dt4 = new VertexTimeDiff(1.0);

  EdgeSnap edge;
  edge.setVertex(0, p0);
  edge.setVertex(1, p1);
  edge.setVertex(2, p2);
  edge.setVertex(3, p3);
  edge.setVertex(4, p4);
  edge.setVertex(5, dt1);
  edge.setVertex(6, dt2);
  edge.setVertex(7, dt3);
  edge.setVertex(8, dt4);
  edge.setTebConfig(params);
  edge.computeError();

  EXPECT_NEAR(edge.error()[0], 0.0, 1e-9);
  EXPECT_NEAR(edge.error()[1], 0.0, 1e-9);

  delete p0;
  delete p1;
  delete p2;
  delete p3;
  delete p4;
  delete dt1;
  delete dt2;
  delete dt3;
  delete dt4;
}

TEST(EdgeSnap, LinearAccelerationZeroSnap) {
  auto params = makeParams();

  // Constant acceleration: positions 0, 1, 3, 6, 10 with dt=1 -> vel 1,2,3,4
  VertexPose *p0 = new VertexPose();
  p0->setEstimate(PoseSE2(0, 0, 0));
  VertexPose *p1 = new VertexPose();
  p1->setEstimate(PoseSE2(1, 0, 0));
  VertexPose *p2 = new VertexPose();
  p2->setEstimate(PoseSE2(3, 0, 0));
  VertexPose *p3 = new VertexPose();
  p3->setEstimate(PoseSE2(6, 0, 0));
  VertexPose *p4 = new VertexPose();
  p4->setEstimate(PoseSE2(10, 0, 0));
  VertexTimeDiff *dt1 = new VertexTimeDiff(1.0);
  VertexTimeDiff *dt2 = new VertexTimeDiff(1.0);
  VertexTimeDiff *dt3 = new VertexTimeDiff(1.0);
  VertexTimeDiff *dt4 = new VertexTimeDiff(1.0);

  EdgeSnap edge;
  edge.setVertex(0, p0);
  edge.setVertex(1, p1);
  edge.setVertex(2, p2);
  edge.setVertex(3, p3);
  edge.setVertex(4, p4);
  edge.setVertex(5, dt1);
  edge.setVertex(6, dt2);
  edge.setVertex(7, dt3);
  edge.setVertex(8, dt4);
  edge.setTebConfig(params);
  edge.computeError();

  EXPECT_NEAR(edge.error()[0], 0.0, 1e-9);
  EXPECT_NEAR(edge.error()[1], 0.0, 1e-9);

  delete p0;
  delete p1;
  delete p2;
  delete p3;
  delete p4;
  delete dt1;
  delete dt2;
  delete dt3;
  delete dt4;
}

TEST(EdgeSnap, JerkChangePenalized) {
  auto params = makeParams();

  // dists 1, 2, 4, 13 with dt=1 -> vels (sigmoid-scaled) ~ 0.99, 1.99, 3.99, 12.98
  // -> accs ~ 1.0, 2.0, 8.99 -> jerks ~ 0.667, 4.66 -> snap ~ 2.0 > snap_max_x
  VertexPose *p0 = new VertexPose();
  p0->setEstimate(PoseSE2(0, 0, 0));
  VertexPose *p1 = new VertexPose();
  p1->setEstimate(PoseSE2(1, 0, 0));
  VertexPose *p2 = new VertexPose();
  p2->setEstimate(PoseSE2(3, 0, 0));
  VertexPose *p3 = new VertexPose();
  p3->setEstimate(PoseSE2(7, 0, 0));
  VertexPose *p4 = new VertexPose();
  p4->setEstimate(PoseSE2(20, 0, 0));
  VertexTimeDiff *dt1 = new VertexTimeDiff(1.0);
  VertexTimeDiff *dt2 = new VertexTimeDiff(1.0);
  VertexTimeDiff *dt3 = new VertexTimeDiff(1.0);
  VertexTimeDiff *dt4 = new VertexTimeDiff(1.0);

  EdgeSnap edge;
  edge.setVertex(0, p0);
  edge.setVertex(1, p1);
  edge.setVertex(2, p2);
  edge.setVertex(3, p3);
  edge.setVertex(4, p4);
  edge.setVertex(5, dt1);
  edge.setVertex(6, dt2);
  edge.setVertex(7, dt3);
  edge.setVertex(8, dt4);
  edge.setTebConfig(params);
  edge.computeError();

  const double vel1 = 1.0 * fast_sigmoid(100.0 * 1.0);
  const double vel2 = 2.0 * fast_sigmoid(100.0 * 2.0);
  const double vel3 = 4.0 * fast_sigmoid(100.0 * 4.0);
  const double vel4 = 13.0 * fast_sigmoid(100.0 * 13.0);
  const double acc1 = (vel2 - vel1) * 2.0 / 2.0;
  const double acc2 = (vel3 - vel2) * 2.0 / 2.0;
  const double acc3 = (vel4 - vel3) * 2.0 / 2.0;
  const double jerk1 = (acc2 - acc1) * 2.0 / 3.0;
  const double jerk2 = (acc3 - acc2) * 2.0 / 3.0;
  const double snap = (jerk2 - jerk1) * 2.0 / 4.0;
  EXPECT_NEAR(edge.error()[0], snap - 0.9, 1e-6);  // snap - (snap_max_x - eps)
  EXPECT_NEAR(edge.error()[1], 0.0, 1e-9);

  delete p0;
  delete p1;
  delete p2;
  delete p3;
  delete p4;
  delete dt1;
  delete dt2;
  delete dt3;
  delete dt4;
}

// Analytic Jacobian vs finite differences. Snap active (snap ~ 2.0 >
// snap_max_x - eps), all sigmoid arguments positive (smooth), unwrap chain
// with derivative 0 w.r.t. the reference angle is consistent between
// computeError() and linearizeOplus().
TEST(EdgeSnap, JacobianMatchesNumeric) {
  auto params = makeParams();

  VertexPose *p0 = new VertexPose();
  p0->setEstimate(PoseSE2(0, 0, 0));
  VertexPose *p1 = new VertexPose();
  p1->setEstimate(PoseSE2(1, 0, 0));
  VertexPose *p2 = new VertexPose();
  p2->setEstimate(PoseSE2(3, 0, 0));
  VertexPose *p3 = new VertexPose();
  p3->setEstimate(PoseSE2(7, 0, 0));
  VertexPose *p4 = new VertexPose();
  p4->setEstimate(PoseSE2(20, 0, 0.2));
  VertexTimeDiff *dt1 = new VertexTimeDiff(1.0);
  VertexTimeDiff *dt2 = new VertexTimeDiff(1.0);
  VertexTimeDiff *dt3 = new VertexTimeDiff(1.0);
  VertexTimeDiff *dt4 = new VertexTimeDiff(1.0);

  EdgeSnap edge;
  edge.setVertex(0, p0);
  edge.setVertex(1, p1);
  edge.setVertex(2, p2);
  edge.setVertex(3, p3);
  edge.setVertex(4, p4);
  edge.setVertex(5, dt1);
  edge.setVertex(6, dt2);
  edge.setVertex(7, dt3);
  edge.setVertex(8, dt4);
  edge.setTebConfig(params);
  edge.computeError();

  expectAnalyticJacobianMatchesNumericMulti(edge);

  delete p0;
  delete p1;
  delete p2;
  delete p3;
  delete p4;
  delete dt1;
  delete dt2;
  delete dt3;
  delete dt4;
}

TEST(EdgeSnap, UnwrapAngle) {
  EXPECT_NEAR(EdgeSnap::unwrap_angle(0.1, 0.0), 0.1, 1e-9);
  // 6.0 is closer to 0.0 via -6.28 -> 6.0 - 2pi = -0.283
  EXPECT_NEAR(EdgeSnap::unwrap_angle(6.0, 0.0), 6.0 - 2 * M_PI, 1e-9);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
