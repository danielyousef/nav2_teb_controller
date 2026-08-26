#include <gtest/gtest.h>

#include "nav2_teb_controller/band_controller/band_controller.hpp"
#include "nav2_teb_controller/band_controller/feed_forward_controller.hpp"
#include "nav2_teb_controller/core/timed_elastic_band.hpp"

using namespace nav2_teb_controller;

namespace {

teb_controller::Params makeParams() {
  teb_controller::Params p;
  p.FollowPath.robot.v_max_x = 0.5;
  p.FollowPath.robot.v_max_x_backwards = 0.2;
  p.FollowPath.robot.v_max_theta = 1.0;
  p.FollowPath.robot.v_max_y = 0.0;
  p.FollowPath.robot.steering_rate_max = 0.5;
  p.FollowPath.robot.wheelbase = 1.055;
  p.FollowPath.robot.use_proportional_saturation = true;
  p.FollowPath.trajectory.dt_ref = 0.3;
  p.FollowPath.trajectory.control_look_ahead_poses = 1;
  p.FollowPath.trajectory.control_min_look_ahead_time = 0.0;
  return p;
}

TimedElasticBand makeStraightBand() {
  TimedElasticBand teb;
  teb.addPose(PoseSE2(0, 0, 0));
  teb.addPoseAndTimeDiff(PoseSE2(0.5, 0, 0), 1.0);
  return teb;
}

geometry_msgs::msg::PoseStamped makePose() {
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.orientation.w = 1.0;
  return pose;
}

geometry_msgs::msg::Twist makeTwist() {
  return geometry_msgs::msg::Twist{};
}

}  // namespace

TEST(FeedForwardController, StraightLineForward) {
  auto params = makeParams();
  FeedForwardController ctrl;
  ctrl.configure(params);
  auto teb = makeStraightBand();
  auto cmd = ctrl.computeCommand(teb, makePose(), makeTwist(), 0.0, 0.1);
  EXPECT_NEAR(cmd.linear.x, 0.5, 1e-9);
  EXPECT_NEAR(cmd.linear.y, 0.0, 1e-9);
  EXPECT_NEAR(cmd.angular.z, 0.0, 1e-9);
}

TEST(FeedForwardController, SaturationClipsToLimit) {
  auto params = makeParams();
  FeedForwardController ctrl;
  ctrl.configure(params);
  TimedElasticBand teb;
  teb.addPose(PoseSE2(0, 0, 0));
  teb.addPoseAndTimeDiff(PoseSE2(5.0, 0, 0), 1.0);  // 5 m/s > v_max_x
  auto cmd = ctrl.computeCommand(teb, makePose(), makeTwist(), 0.0, 0.1);
  EXPECT_NEAR(cmd.linear.x, params.FollowPath.robot.v_max_x, 1e-9);
  EXPECT_LE(cmd.linear.x, params.FollowPath.robot.v_max_x + 1e-9);
}

TEST(FeedForwardController, ZeroBandYieldsZeroCommand) {
  auto params = makeParams();
  FeedForwardController ctrl;
  ctrl.configure(params);
  TimedElasticBand teb;
  teb.addPose(PoseSE2(0, 0, 0));  // single pose, no dt
  auto cmd = ctrl.computeCommand(teb, makePose(), makeTwist(), 0.0, 0.1);
  EXPECT_NEAR(cmd.linear.x, 0.0, 1e-9);
  EXPECT_NEAR(cmd.angular.z, 0.0, 1e-9);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
