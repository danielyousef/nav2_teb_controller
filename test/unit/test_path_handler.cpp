#include <gtest/gtest.h>
#include <tf2_ros/buffer.h>

#include <nav2_costmap_2d/costmap_2d.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/time.hpp>

#include "nav2_teb_controller/path_handler.hpp"

using namespace nav2_teb_controller;

namespace {

teb_controller::Params makeParams(double local_goal_hysteresis = 1.0) {
  teb_controller::Params p;
  p.FollowPath.trajectory.global_plan_prune_distance = 0.5;
  p.FollowPath.trajectory.max_global_plan_lookahead_dist = 50.0;
  p.FollowPath.trajectory.local_goal_hysteresis = local_goal_hysteresis;
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

// Longer straight plan along +x in the "plan" frame (used to exercise goal-index hysteresis).
nav_msgs::msg::Path makeLongPlan(double x_end = 4.0, double step = 0.5) {
  nav_msgs::msg::Path plan;
  plan.header.frame_id = "plan";
  for (double x = 0.0; x <= x_end + 1e-9; x += step) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = "plan";
    ps.pose.position.x = x;
    plan.poses.push_back(ps);
  }
  return plan;
}

geometry_msgs::msg::PoseStamped makeRobot(double x, double y = 0.0) {
  geometry_msgs::msg::PoseStamped robot;
  robot.header.frame_id = "map";
  robot.pose.position.x = x;
  robot.pose.position.y = y;
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

// A large costmap (no distance cutoff) + a short lookahead window makes the natural lookahead
// goal recede when the robot's along-path projection moves back, and advance when it moves
// forward. With hysteresis the sticky goal is anchored by POSE, so it is kept across the
// per-tick prune step; only a recede larger than local_goal_hysteresis is honored.
TEST(PathHandler, StickyGoalKeptWhenRecedeBelowHysteresis) {
  auto params = makeParams(/*local_goal_hysteresis=*/1.0);
  params.FollowPath.trajectory.max_global_plan_lookahead_dist = 1.5;
  tf2_ros::Buffer tf(std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME));
  geometry_msgs::msg::TransformStamped ts;
  ts.header.frame_id = "map";
  ts.child_frame_id = "plan";
  ts.header.stamp = rclcpp::Time(0, 0);
  ts.transform.rotation.w = 1.0;
  tf.setTransform(ts, "test");
  nav2_costmap_2d::Costmap2D costmap(300, 300, 0.1, 0.0, 0.0, 0);

  PathHandler ph(params, tf);
  auto global_plan = makeLongPlan(10.0, 0.5);

  nav_msgs::msg::Path local;
  int goal_idx1 = -1;
  ASSERT_TRUE(ph.prepareLocalPlan(global_plan, makeRobot(2.0, 0.0), geometry_msgs::msg::Twist{},
                                  rclcpp::Time(0, 0), costmap, "map", local, goal_idx1));
  ASSERT_FALSE(local.poses.empty());
  const double goal_x1 = local.poses.back().pose.position.x;  // ~3.5 m

  nav_msgs::msg::Path local2;
  int goal_idx2 = -1;
  ASSERT_TRUE(ph.prepareLocalPlan(global_plan, makeRobot(1.0, 0.0), geometry_msgs::msg::Twist{},
                                  rclcpp::Time(0, 0), costmap, "map", local2, goal_idx2));
  // Robot projection moved back → natural goal recedes to ~2.5 m, but hysteresis keeps 3.5 m.
  ASSERT_FALSE(local2.poses.empty());
  EXPECT_NEAR(local2.poses.back().pose.position.x, goal_x1, 1e-6);
}

TEST(PathHandler, StickyGoalRecedesWhenRecedeAboveHysteresis) {
  auto params = makeParams(/*local_goal_hysteresis=*/0.2);  // smaller than the 1.0 m recede
  params.FollowPath.trajectory.max_global_plan_lookahead_dist = 1.5;
  tf2_ros::Buffer tf(std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME));
  geometry_msgs::msg::TransformStamped ts;
  ts.header.frame_id = "map";
  ts.child_frame_id = "plan";
  ts.header.stamp = rclcpp::Time(0, 0);
  ts.transform.rotation.w = 1.0;
  tf.setTransform(ts, "test");
  nav2_costmap_2d::Costmap2D costmap(300, 300, 0.1, 0.0, 0.0, 0);

  PathHandler ph(params, tf);
  auto global_plan = makeLongPlan(10.0, 0.5);

  nav_msgs::msg::Path local;
  int goal_idx1 = -1;
  ASSERT_TRUE(ph.prepareLocalPlan(global_plan, makeRobot(2.0, 0.0), geometry_msgs::msg::Twist{},
                                  rclcpp::Time(0, 0), costmap, "map", local, goal_idx1));
  ASSERT_FALSE(local.poses.empty());
  const double goal_x1 = local.poses.back().pose.position.x;  // ~3.5 m

  nav_msgs::msg::Path local2;
  int goal_idx2 = -1;
  ASSERT_TRUE(ph.prepareLocalPlan(global_plan, makeRobot(1.0, 0.0), geometry_msgs::msg::Twist{},
                                  rclcpp::Time(0, 0), costmap, "map", local2, goal_idx2));
  // Recede (1.0 m) exceeds the 0.2 m hysteresis → the goal is allowed to drop to ~2.5 m.
  ASSERT_FALSE(local2.poses.empty());
  EXPECT_LT(local2.poses.back().pose.position.x, goal_x1 - 0.5);
}

TEST(PathHandler, StickyGoalAdvances) {
  auto params = makeParams(/*local_goal_hysteresis=*/1.0);
  params.FollowPath.trajectory.max_global_plan_lookahead_dist = 1.5;
  tf2_ros::Buffer tf(std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME));
  geometry_msgs::msg::TransformStamped ts;
  ts.header.frame_id = "map";
  ts.child_frame_id = "plan";
  ts.header.stamp = rclcpp::Time(0, 0);
  ts.transform.rotation.w = 1.0;
  tf.setTransform(ts, "test");
  nav2_costmap_2d::Costmap2D costmap(300, 300, 0.1, 0.0, 0.0, 0);

  PathHandler ph(params, tf);
  auto global_plan = makeLongPlan(10.0, 0.5);

  nav_msgs::msg::Path local;
  int goal_idx1 = -1;
  ASSERT_TRUE(ph.prepareLocalPlan(global_plan, makeRobot(2.0, 0.0), geometry_msgs::msg::Twist{},
                                  rclcpp::Time(0, 0), costmap, "map", local, goal_idx1));
  ASSERT_FALSE(local.poses.empty());
  const double goal_x1 = local.poses.back().pose.position.x;  // ~3.5 m

  nav_msgs::msg::Path local2;
  int goal_idx2 = -1;
  ASSERT_TRUE(ph.prepareLocalPlan(global_plan, makeRobot(4.0, 0.0), geometry_msgs::msg::Twist{},
                                  rclcpp::Time(0, 0), costmap, "map", local2, goal_idx2));
  // Robot further along → the sticky goal advances to ~5.5 m.
  ASSERT_FALSE(local2.poses.empty());
  EXPECT_GT(local2.poses.back().pose.position.x, goal_x1 + 0.5);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
