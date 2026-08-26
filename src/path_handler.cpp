#include "nav2_teb_controller/path_handler.hpp"

#include <tf2/transform_datatypes.h>
#include <tf2/utils.h>

#include <cmath>
#include <nav2_util/geometry_utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace nav2_teb_controller {

PathHandler::PathHandler(const teb_controller::Params &params, tf2_ros::Buffer &tf)
    : params_(&params), tf_(tf) {}

bool PathHandler::prepareLocalPlan(const nav_msgs::msg::Path &global_plan,
                                   const geometry_msgs::msg::PoseStamped &robot_pose,
                                   const geometry_msgs::msg::Twist &robot_vel,
                                   const rclcpp::Time &now,
                                   const nav2_costmap_2d::Costmap2D &costmap,
                                   const std::string &global_frame,
                                   nav_msgs::msg::Path &out_local_plan, int &out_goal_idx) {
  const double prune_dist = params_->FollowPath.trajectory.global_plan_prune_distance;
  const double global_plan_lookahead =
      params_->FollowPath.trajectory.max_global_plan_lookahead_dist;
  const double path_length_time_based = robot_vel.linear.x * 5.0;
  const double path_length =
      std::clamp(1.0 * global_plan_lookahead, path_length_time_based, global_plan_lookahead);

  // Single latest-known transform lookup (plan frame -> global frame). The previous
  // stamped-time lookup (plan stamp) blocked up to its timeout whenever the exact stamped
  // transform was not cached yet. TimePointZero uses the newest available transform; the
  // plan frame is quasi-static, so this is equivalent except that it never blocks.
  const std::string plan_frame =
      global_plan.poses.empty() ? "" : global_plan.poses.front().header.frame_id;
  tf2::Transform plan_to_global;
  try {
    const geometry_msgs::msg::TransformStamped plan_to_global_stamped =
        tf_.lookupTransform(global_frame, plan_frame, tf2::TimePointZero);
    const auto &t = plan_to_global_stamped.transform;
    plan_to_global =
        tf2::Transform(tf2::Quaternion(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w),
                       tf2::Vector3(t.translation.x, t.translation.y, t.translation.z));
  } catch (const tf2::TransformException &ex) {
    RCLCPP_DEBUG(rclcpp::get_logger("TEBController"), "TF transform failed: %s", ex.what());
    out_local_plan = nav_msgs::msg::Path{};
    out_goal_idx = -1;
    return false;
  }

  nav_msgs::msg::Path pruned_plan = global_plan;
  pruneGlobalPlan(plan_to_global, robot_pose, pruned_plan, prune_dist);
  out_local_plan = transformAndTrimPlan(plan_to_global, pruned_plan, robot_pose, costmap,
                                        global_frame, now, path_length, &out_goal_idx);
  if (out_local_plan.poses.empty()) {
    return false;
  }

  // Overwrite start with actual robot pose so TEB can use plan as initial trajectory.
  if (out_local_plan.poses.size() == 1) {  // plan only contains the goal
    out_local_plan.poses.insert(out_local_plan.poses.begin(), geometry_msgs::msg::PoseStamped());
  }
  out_local_plan.poses.front() = robot_pose;
  return true;
}

bool PathHandler::pruneGlobalPlan(const tf2::Transform &plan_to_global,
                                  const geometry_msgs::msg::PoseStamped &robot_pose,
                                  nav_msgs::msg::Path &global_plan, double dist_behind_robot) {
  if (global_plan.poses.empty()) {
    return true;
  }
  // Robot pose (global frame) → plan frame by pure math (no TF lookup, no blocking)
  const tf2::Vector3 robot_global(robot_pose.pose.position.x, robot_pose.pose.position.y, 0.0);
  const tf2::Vector3 robot_plan = plan_to_global.inverse() * robot_global;
  const double dist_thresh_sq = dist_behind_robot * dist_behind_robot;
  auto it = global_plan.poses.begin();
  auto erase_end = it;
  // Track the nearest plan pose as a fallback: if no pose lies within dist_behind_robot
  // (robot slightly off-path), we still prune everything up to the closest pose so the
  // resulting plan starts at the robot's projection. This restores the pre-refactor
  // behavior (the old code mutated the plan member in place, so it stayed pruned).
  double best_sq = 1e300;
  while (it != global_plan.poses.end()) {
    double dx = robot_plan.x() - it->pose.position.x;
    double dy = robot_plan.y() - it->pose.position.y;
    const double d2 = dx * dx + dy * dy;
    if (d2 < dist_thresh_sq) {
      erase_end = it;
      break;
    }
    if (d2 < best_sq) {
      best_sq = d2;
      erase_end = it;
    }
    ++it;
  }
  if (erase_end != global_plan.poses.begin()) {
    global_plan.poses.erase(global_plan.poses.begin(), erase_end);
  }
  return true;
}

nav_msgs::msg::Path PathHandler::transformAndTrimPlan(
    const tf2::Transform &plan_to_global, const nav_msgs::msg::Path &global_plan,
    const geometry_msgs::msg::PoseStamped &global_pose, const nav2_costmap_2d::Costmap2D &costmap,
    const std::string &global_frame, const rclcpp::Time &now, double max_plan_length,
    int *current_goal_idx) {
  nav_msgs::msg::Path transformed_path;
  if (global_plan.poses.empty()) {
    return transformed_path;
  }

  // Robot pose (global frame) → plan frame by pure math (no TF lookup, no blocking)
  const tf2::Vector3 robot_global(global_pose.pose.position.x, global_pose.pose.position.y, 0.0);
  const tf2::Vector3 robot_plan = plan_to_global.inverse() * robot_global;

  const double dist_threshold =
      std::max(costmap.getSizeInCellsX() * costmap.getResolution() / 2.0,
               costmap.getSizeInCellsY() * costmap.getResolution() / 2.0) *
      0.85;
  const double sq_dist_threshold = dist_threshold * dist_threshold;

  int i = 0;
  double sq_dist = 1e10;
  bool robot_reached = false;
  for (int j = 0; j < static_cast<int>(global_plan.poses.size()); ++j) {
    double dx = robot_plan.x() - global_plan.poses[j].pose.position.x;
    double dy = robot_plan.y() - global_plan.poses[j].pose.position.y;
    double new_sq_dist = dx * dx + dy * dy;
    // NOTE: do NOT break on the first pose beyond dist_threshold. The plan is ordered
    // start->goal, but the robot may be anywhere along it (e.g. in the middle), so the
    // first pose can be far while the true nearest pose is much later. Scan the whole
    // plan and keep the global minimum instead.
    if (robot_reached && new_sq_dist > sq_dist) {
      break;
    }
    if (new_sq_dist < sq_dist) {
      sq_dist = new_sq_dist;
      i = j;
      if (sq_dist < 0.05) {
        robot_reached = true;
      }
    }
  }

  // tf2::Transform → stamped message once; doTransform below is pure math.
  geometry_msgs::msg::TransformStamped plan_to_global_msg;
  plan_to_global_msg.header.frame_id = global_frame;
  plan_to_global_msg.header.stamp = now;
  plan_to_global_msg.transform.translation.x = plan_to_global.getOrigin().x();
  plan_to_global_msg.transform.translation.y = plan_to_global.getOrigin().y();
  plan_to_global_msg.transform.translation.z = plan_to_global.getOrigin().z();
  const tf2::Quaternion plan_to_global_rot = plan_to_global.getRotation();
  plan_to_global_msg.transform.rotation.x = plan_to_global_rot.x();
  plan_to_global_msg.transform.rotation.y = plan_to_global_rot.y();
  plan_to_global_msg.transform.rotation.z = plan_to_global_rot.z();
  plan_to_global_msg.transform.rotation.w = plan_to_global_rot.w();

  geometry_msgs::msg::PoseStamped newer_pose;
  double plan_length = 0.0;
  while (i < static_cast<int>(global_plan.poses.size()) && sq_dist <= sq_dist_threshold &&
         (max_plan_length <= 0.0 || plan_length <= max_plan_length)) {
    tf2::doTransform(global_plan.poses[i], newer_pose, plan_to_global_msg);
    transformed_path.poses.push_back(newer_pose);
    double dx = robot_plan.x() - global_plan.poses[i].pose.position.x;
    double dy = robot_plan.y() - global_plan.poses[i].pose.position.y;
    sq_dist = dx * dx + dy * dy;
    if (i > 0 && max_plan_length > 0.0)
      plan_length += nav2_util::geometry_utils::euclidean_distance(global_plan.poses[i - 1].pose,
                                                                   global_plan.poses[i].pose);
    ++i;
  }

  const bool empty_branch = transformed_path.poses.empty();
  if (empty_branch) {
    tf2::doTransform(global_plan.poses.back(), newer_pose, plan_to_global_msg);
    transformed_path.poses.push_back(newer_pose);
  }
  if (current_goal_idx) {
    *current_goal_idx = empty_branch ? int(global_plan.poses.size()) - 1 : i - 1;
  }

  return transformed_path;
}

}  // namespace nav2_teb_controller
