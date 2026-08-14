#pragma once

#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/time.hpp>

#include "nav2_teb_controller/core/footprint.hpp"
#include "nav2_teb_controller/core/timed_elastic_band.hpp"
#include "nav2_teb_controller/obstacles/esdf.hpp"

namespace nav2_teb_controller {

double estimateDeltaT(const PoseSE2 &start, const PoseSE2 &end, double max_vel_x,
                      double max_vel_theta);

bool initFromPath(TimedElasticBand &teb, const nav_msgs::msg::Path &path, double max_vel_x,
                  double max_vel_theta, bool estimate_orient, int min_samples,
                  bool guess_backwards_motion, bool fixed_goal);

// void autoResize(TimedElasticBand& teb, double dt_ref, double dt_hysteresis,
//   int min_samples, int max_samples, bool fast_mode);

void autoResize(TimedElasticBand &teb, double dt_ref, double dt_hysteresis, double min_seg_length,
                double max_seg_length, double max_angle_diff, int min_samples, int max_samples,
                bool fast_mode);

void updateAndPrune(TimedElasticBand &teb, const PoseSE2 &new_start, const PoseSE2 &new_goal,
                    int min_samples, double min_prune_distance);

geometry_msgs::msg::Twist extractVelocity(const PoseSE2 &pose1, const PoseSE2 &pose2, double dt,
                                          bool holonomic);

geometry_msgs::msg::Twist getVelocityCommand(const TimedElasticBand &teb, double dt_ref,
                                             int look_ahead_poses, double min_look_ahead_time,
                                             bool holonomic);

bool pruneGlobalPlan(const tf2::Transform &plan_to_global,
                     const geometry_msgs::msg::PoseStamped &robot_pose,
                     nav_msgs::msg::Path &global_plan, double dist_behind_robot = 1.0);

nav_msgs::msg::Path transformAndTrimPlan(const tf2::Transform &plan_to_global,
                                         const nav_msgs::msg::Path &global_plan,
                                         const geometry_msgs::msg::PoseStamped &global_pose,
                                         const nav2_costmap_2d::Costmap2D &costmap,
                                         const std::string &global_frame, const rclcpp::Time &now,
                                         double max_plan_length, int *current_goal_idx);

void saturateVelocity(geometry_msgs::msg::Twist &cmd_vel, double v_max_x, double v_max_y,
                      double v_max_theta, double v_max_x_backwards,
                      bool use_proportional_saturation);

geometry_msgs::msg::Twist convertAckermannToTwist(double wheelspeed, double angle,
                                                  double wheelbase);

std::pair<double, double> convertTwistToAckermann(
    const geometry_msgs::msg::Twist &twist_cmd, double wheelbase,
    std::optional<double> current_angle = std::nullopt);

void saturateSteeringAngle(geometry_msgs::msg::Twist &cmd_vel, double current_angle,
                           double steering_rate_max, double wheelbase, double dt);

double computeCurvature(const PoseSE2 &p1, const PoseSE2 &p2, const PoseSE2 &p3);

int checkFeasibility(const TimedElasticBand &teb, const ObstacleMap2D &esdf, const Footprint &fp,
                     double lookahead);

}  // namespace nav2_teb_controller
