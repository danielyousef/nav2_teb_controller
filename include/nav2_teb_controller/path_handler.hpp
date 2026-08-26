#pragma once

#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>

#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_teb_controller/teb_controller_parameters.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/time.hpp"

namespace nav2_teb_controller {

// Encapsulates step 2 of TEBController::computeVelocityCommands: extracting the local
// lookahead window from the global plan (TF lookup + prune + transform/trim + overwrite of
// the start pose with the measured robot pose).
//
// NOTE: the costmap is taken as an argument to prepareLocalPlan (queried live every tick by
// the controller) rather than captured at construction. A rolling/local Costmap2D object can
// be replaced or be zero-sized after configure(), so a configure-time reference would go
// stale and break the trim distance threshold.
class PathHandler {
public:
  PathHandler(const teb_controller::Params &params, tf2_ros::Buffer &tf);

  // Prepare the local plan. Returns false (and an empty plan) on TF failure. On success,
  // `out_local_plan` holds the transformed/trimmed plan (start pose == robot_pose) and
  // `out_goal_idx` is the index into the (pruned) global plan of the last included pose.
  // `costmap` / `global_frame` are the *live* costmap + its global frame, queried by the
  // caller each tick.
  bool prepareLocalPlan(const nav_msgs::msg::Path &global_plan,
                        const geometry_msgs::msg::PoseStamped &robot_pose,
                        const geometry_msgs::msg::Twist &robot_vel, const rclcpp::Time &now,
                        const nav2_costmap_2d::Costmap2D &costmap, const std::string &global_frame,
                        nav_msgs::msg::Path &out_local_plan, int &out_goal_idx);

  /// Reset the local-goal hysteresis state. Call when a new global plan is supplied so the
  /// sticky goal does not persist across missions (PathHandler::prepareLocalPlan keeps the
  /// last emitted goal index sticky across ticks within one plan).
  void reset();

private:
  /// @return number of poses erased from the front of @p global_plan (the prune offset), so
  ///         pruned-relative indices can be mapped back to absolute global-plan indices.
  static size_t pruneGlobalPlan(const tf2::Transform &plan_to_global,
                                const geometry_msgs::msg::PoseStamped &robot_pose,
                                nav_msgs::msg::Path &global_plan, double dist_behind_robot);

  nav_msgs::msg::Path transformAndTrimPlan(
      const tf2::Transform &plan_to_global, const nav_msgs::msg::Path &global_plan,
      const geometry_msgs::msg::PoseStamped &global_pose,
      const nav2_costmap_2d::Costmap2D &costmap, const std::string &global_frame,
      const rclcpp::Time &now, double max_plan_length, int *current_goal_idx);

  const teb_controller::Params *params_{nullptr};
  tf2_ros::Buffer &tf_;

  /// Local-goal hysteresis state (see local_goal_hysteresis param). The sticky goal is
  /// anchored by its POSE position (`last_goal_pos_`, plan frame), not by an index, so it
  /// survives the per-tick prune step that drops a variable number of leading poses.
  /// `has_sticky_` is false until the first plan is emitted (after reset() it is cleared).
  bool has_sticky_ = false;
  tf2::Vector3 last_goal_pos_{0.0, 0.0, 0.0};
};

}  // namespace nav2_teb_controller
