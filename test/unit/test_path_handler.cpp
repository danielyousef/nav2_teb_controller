#include <gtest/gtest.h>
#include <tf2_ros/buffer.h>

#include <nav2_costmap_2d/costmap_2d.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/time.hpp>

#include "nav2_teb_controller/path_handler.hpp"

using namespace nav2_teb_controller;

namespace {

teb_controller::Params makeParams() {
  teb_controller::Params p;
  p.FollowPath.trajectory.global_plan_prune_distance = 0.5;
  p.FollowPath.trajectory.max_global_plan_lookahead_dist = 5.0;
  return p;
}

nav_msgs::msg::Path makeGlobalPlan() {
  nav_msgs::msg::Path plan;
  plan.header.frame_id = "plan";
  for (double x = 0.0; x <= 1.0; x += 0.25) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = "plan";
    ps.pose.position.x = x;
    plan.poses.push_back(ps);
  }
  return plan;
}

geometry_msgs::msg::PoseStamped makeRobot(double x) {
  geometry_msgs::msg::PoseStamped robot;
  robot.header.frame_id = "map";
  robot.pose.position.x = x;
  robot.pose.orientation.w = 1.0;
  return robot;
}

}  // namespace

TEST(PathHandler, TransformsAndTrimsPlan) {
  auto params = makeParams();
  tf2_ros::Buffer tf(std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME));
  geometry_msgs::msg::TransformStamped ts;
  ts.header.frame_id = "map";
  ts.child_frame_id = "plan";
  ts.header.stamp = rclcpp::Time(0, 0);
  ts.transform.rotation.w = 1.0;
  tf.setTransform(ts, "test");

  nav2_costmap_2d::Costmap2D costmap(10, 10, 0.1, 0.0, 0.0, 0);

  PathHandler ph(params, tf);

  auto global_plan = makeGlobalPlan();
  nav_msgs::msg::Path local;
  int goal_idx = -1;
  bool ok = ph.prepareLocalPlan(global_plan, makeRobot(0.6), geometry_msgs::msg::Twist{},
                                rclcpp::Time(0, 0), costmap, "map", local, goal_idx);
  EXPECT_TRUE(ok);
  ASSERT_FALSE(local.poses.empty());
  // start pose overwritten with the robot pose
  EXPECT_NEAR(local.poses.front().pose.position.x, 0.6, 1e-6);
  EXPECT_GE(goal_idx, 0);
}

TEST(PathHandler, TfFailureReturnsFalse) {
  auto params = makeParams();
  tf2_ros::Buffer tf(std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME));  // no transform set
  nav2_costmap_2d::Costmap2D costmap(10, 10, 0.1, 0.0, 0.0, 0);

  PathHandler ph(params, tf);
  auto global_plan = makeGlobalPlan();
  nav_msgs::msg::Path local;
  int goal_idx = -1;
  bool ok = ph.prepareLocalPlan(global_plan, makeRobot(0.6), geometry_msgs::msg::Twist{},
                                rclcpp::Time(0, 0), costmap, "map", local, goal_idx);
  EXPECT_FALSE(ok);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
