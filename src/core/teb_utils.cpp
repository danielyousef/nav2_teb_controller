#include "nav2_teb_controller/core/teb_utils.hpp"

#include <angles/angles.h>
#include <tf2/utils.h>

#include <cmath>
#include <nav2_util/geometry_utils.hpp>
#include <optional>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace nav2_teb_controller {

double estimateDeltaT(const PoseSE2 &start, const PoseSE2 &end, double max_vel_x,
                      double max_vel_theta) {
  double dt = 0.05;
  if (max_vel_x > 0) {
    double trans_dist = (end.position() - start.position()).norm();
    dt = trans_dist / (1.0 * max_vel_x);  // 0.75
  }
  if (max_vel_theta > 0) {
    double rot_dist = std::abs(angles::normalize_angle(end.theta() - start.theta()));
    dt = std::max(dt, rot_dist / (1.0 * max_vel_theta));  // 0.5
  }
  return dt;
}

bool initFromPath(TimedElasticBand &teb, const nav_msgs::msg::Path &path, double max_vel_x,
                  double max_vel_theta, bool estimate_orient, int min_samples,
                  bool guess_backwards_motion, bool /*fixed_goal*/) {
  if (teb.isInit()) {
    return false;
  }

  PoseSE2 start(path.poses.front().pose);
  PoseSE2 goal(path.poses.back().pose);
  teb.addPose(start);

  bool backward_motion = (goal.position() - start.position()).dot(start.orientationUnitVec()) < 0;
  bool backwards = guess_backwards_motion && backward_motion;

  for (int i = 1; i < static_cast<int>(path.poses.size()) - 1; ++i) {
    double yaw;
    if (estimate_orient) {
      double dx = path.poses[i + 1].pose.position.x - path.poses[i].pose.position.x;
      double dy = path.poses[i + 1].pose.position.y - path.poses[i].pose.position.y;
      yaw = std::atan2(dy, dx);
      if (backwards) {
        yaw = angles::normalize_angle(yaw + M_PI);
      }
    } else
      yaw = tf2::getYaw(path.poses[i].pose.orientation);
    PoseSE2 intermediate_pose(path.poses[i].pose.position.x, path.poses[i].pose.position.y, yaw);
    double dt = estimateDeltaT(teb.backPose(), intermediate_pose, max_vel_x, max_vel_theta);
    teb.addPoseAndTimeDiff(intermediate_pose, dt);
  }
  RCLCPP_DEBUG(rclcpp::get_logger("optimal_planner"), "TEB Utils: Added path poses to teb.");
  while (teb.sizePoses() < static_cast<std::size_t>(min_samples) - 1) {
    PoseSE2 intermediate_pose = PoseSE2::average(teb.backPose(), goal);
    double dt = estimateDeltaT(teb.backPose(), intermediate_pose, max_vel_x, max_vel_theta);
    teb.addPoseAndTimeDiff(intermediate_pose, dt);
  }
  RCLCPP_DEBUG(rclcpp::get_logger("optimal_planner"),
               "TEB Utils: Added extra poses for min samples.");
  double dt = estimateDeltaT(teb.backPose(), goal, max_vel_x, max_vel_theta);
  teb.addPoseAndTimeDiff(goal, dt);
  return true;
}

void autoResize(TimedElasticBand &teb, double dt_ref, double dt_hysteresis, double min_seg_length,
                double max_seg_length, double max_angle_diff, int min_samples, int max_samples,
                bool fast_mode) {
  if (teb.sizeTimeDiffs() != 0 && teb.sizeTimeDiffs() + 1 != teb.sizePoses()) {
    RCLCPP_ERROR(rclcpp::get_logger("optimal_planner"), "TEB Utils: Auto resize not possible.");
    throw std::runtime_error("autoResize: TEB inconsistent — sizePoses != sizeTimeDiffs + 1");
  }

  bool modified = true;
  for (int rep = 0; rep < 100 && modified; ++rep) {
    modified = false;
    for (std::size_t i = 0; i < teb.sizeTimeDiffs(); ++i) {
      const double dt = teb.timeDiff(i);
      const double seg_len = (teb.pose(i + 1).position() - teb.pose(i).position()).norm();
      const double angle_diff =
          std::abs(angles::normalize_angle(teb.pose(i + 1).theta() - teb.pose(i).theta()));

      // --- INSERT ---
      // Zeit zu lang AND Segment zu lang, ODER Kurve zu scharf
      const bool time_too_long = dt > dt_ref + dt_hysteresis;
      const bool geom_too_long = seg_len > max_seg_length;
      const bool curve_too_sharp = angle_diff > max_angle_diff;

      const bool insert_ok = ((time_too_long && geom_too_long) || curve_too_sharp) &&
                             (teb.sizeTimeDiffs() < static_cast<std::size_t>(max_samples));

      if (insert_ok) {
        const double new_dt = 0.5 * dt;
        teb.timeDiff(i) = new_dt;
        teb.insertPose(i + 1, PoseSE2::average(teb.pose(i), teb.pose(i + 1)));
        teb.insertTimeDiff(i + 1, new_dt);
        modified = true;
        continue;  // dieses Segment nicht auch noch auf DELETE prüfen
      }

      // --- DELETE ---
      // Zeit zu kurz ODER Segment zu kurz — aber nur wenn kein signifikanter Winkel
      const bool time_too_short = dt < dt_ref - dt_hysteresis;
      const bool geom_too_short = seg_len < min_seg_length;
      const bool angle_significant =
          angle_diff > max_angle_diff;  // 0.01;  // ~0.6° — Kurven-Posen schützen

      const bool delete_ok = ((time_too_short || geom_too_short) && !angle_significant) &&
                             (teb.sizeTimeDiffs() > static_cast<std::size_t>(min_samples));

      if (delete_ok) {
        if (i + 1 < teb.sizeTimeDiffs()) {
          teb.timeDiff(i + 1) += teb.timeDiff(i);
          teb.deleteTimeDiff(i);
          teb.deletePose(i + 1);
        } else {
          // Letztes Segment — Zeit auf Vorgänger verschieben
          teb.timeDiff(i - 1) += teb.timeDiff(i);
          teb.deleteTimeDiff(i);
          teb.deletePose(i);
        }
        modified = true;
      }
    }
    if (fast_mode) {
      break;
    }
  }
}

void updateAndPrune(TimedElasticBand &teb, const PoseSE2 &new_start, const PoseSE2 &new_goal,
                    int min_samples, double min_prune_distance) {
  if (teb.sizePoses() == 0) {
    return;
  }
  // find nearest state (using l2-norm) in order to prune the trajectory
  const int max_lookahead = 15;
  const int last_idx = static_cast<int>(teb.sizePoses()) - 1;
  int lookahead = std::max(std::min(last_idx - min_samples + 1, max_lookahead), 0);
  double dist_cache = (new_start.position() - teb.pose(0).position()).norm();
  int nearest_idx = 0;
  for (int i = 1; i <= lookahead; ++i) {
    const double dist = (new_start.position() - teb.pose(i).position()).norm();
    if (dist < dist_cache) {
      dist_cache = dist;
      nearest_idx = i;
    } else {
      break;
    }
  }
  // prune trajectory at the beginning
  if (nearest_idx > 0) {
    teb.deletePoses(1, nearest_idx);
    teb.deleteTimeDiffs(1, nearest_idx);
  }
  // Bei hartem Rückwärts-/Quer-Versatz kann P1 sehr dicht an P0 kleben.
  if (teb.sizePoses() > 1) {
    const double dist_p1 = (teb.pose(1).position() - new_start.position()).norm();
    const double min_dist_p1 = min_prune_distance;  // z.B. 2-5cm, als Parameter konfigurierbar

    if (dist_p1<min_dist_p1 &&static_cast<int>(teb.sizePoses())> min_samples) {
      teb.deletePoses(1, 1);
      teb.deleteTimeDiffs(1, 1);
    }
  }
  // update start
  teb.pose(0) = new_start;
  teb.backPose() = new_goal;
}

geometry_msgs::msg::Twist extractVelocity(const PoseSE2 &pose1, const PoseSE2 &pose2, double dt,
                                          bool holonomic) {
  geometry_msgs::msg::Twist cmd_vel;
  if (dt < 1e-9) {
    return cmd_vel;
  }

  Eigen::Vector2d deltaS = pose2.position() - pose1.position();
  if (!holonomic) {
    Eigen::Vector2d conf1dir(std::cos(pose1.theta()), std::sin(pose1.theta()));
    double dir = deltaS.dot(conf1dir);
    cmd_vel.linear.x = std::copysign(1, dir) * deltaS.norm() / dt;
    // cmd_vel.linear.x = deltaS.dot(conf1dir) / dt;
  } else {
    double cos_theta1 = std::cos(pose1.theta());
    double sin_theta1 = std::sin(pose1.theta());
    double p1_dx = cos_theta1 * deltaS.x() + sin_theta1 * deltaS.y();
    double p1_dy = -sin_theta1 * deltaS.x() + cos_theta1 * deltaS.y();
    cmd_vel.linear.x = p1_dx / dt;
    cmd_vel.linear.y = p1_dy / dt;
  }
  cmd_vel.angular.z = angles::normalize_angle(pose2.theta() - pose1.theta()) / dt;
  return cmd_vel;
}

geometry_msgs::msg::Twist getVelocityCommand(const TimedElasticBand &teb, double dt_ref,
                                             int look_ahead_poses, double min_look_ahead_time,
                                             bool holonomic) {
  geometry_msgs::msg::Twist cmd_vel;
  if (teb.sizePoses() < 2) {
    return cmd_vel;
  }
  look_ahead_poses =
      std::max(1, std::min(look_ahead_poses, static_cast<int>(teb.sizePoses() - 1)));
  double dt = 0.0;
  for (int counter = 0; counter < look_ahead_poses; ++counter) {
    dt += teb.timeDiff(counter);
    if (dt >= dt_ref * look_ahead_poses && dt >= min_look_ahead_time) {
      // if(min_look_ahead_time >= dt_ref * look_ahead_poses) {
      look_ahead_poses = counter + 1;
      break;
    }
  }
  if (dt <= 1e-9) {
    return cmd_vel;
  }
  // Get velocity from the first two configurations
  // if (teb.accumulatedDistance() < 0.01) // add param later
  //   cmd_vel = extractVelocity(teb.pose(0), teb.pose(teb.sizePoses()-1), dt, holonomic);
  // else
  cmd_vel = extractVelocity(teb.pose(0), teb.pose(static_cast<std::size_t>(look_ahead_poses)), dt,
                            holonomic);
  return cmd_vel;
}

bool pruneGlobalPlan(const tf2_ros::Buffer &tf_buffer,
                     const geometry_msgs::msg::PoseStamped &robot_pose,
                     nav_msgs::msg::Path &global_plan, double dist_behind_robot) {
  if (global_plan.poses.empty()) {
    return true;
  }
  try {
    geometry_msgs::msg::PoseStamped robot =
        tf_buffer.transform(robot_pose, global_plan.poses.front().header.frame_id);
    const double dist_thresh_sq = dist_behind_robot * dist_behind_robot;
    auto it = global_plan.poses.begin();
    auto erase_end = it;
    while (it != global_plan.poses.end()) {
      double dx = robot.pose.position.x - it->pose.position.x;
      double dy = robot.pose.position.y - it->pose.position.y;
      if (dx * dx + dy * dy < dist_thresh_sq) {
        erase_end = it;
        break;
      }
      ++it;
    }
    if (erase_end == global_plan.poses.end()) {
      return false;
    }
    if (erase_end != global_plan.poses.begin()) {
      global_plan.poses.erase(global_plan.poses.begin(), erase_end);
    }
  } catch (const tf2::TransformException &ex) {
    RCLCPP_DEBUG(rclcpp::get_logger("optimal_planner"), "TF transform failed: %s", ex.what());
    return false;
  }
  return true;
}

nav_msgs::msg::Path transformAndTrimPlan(const tf2_ros::Buffer &tf_buffer,
                                         const nav_msgs::msg::Path &global_plan,
                                         const geometry_msgs::msg::PoseStamped &global_pose,
                                         const nav2_costmap_2d::Costmap2D &costmap,
                                         const std::string &global_frame, const rclcpp::Time &now,
                                         double max_plan_length, int *current_goal_idx) {
  nav_msgs::msg::Path transformed_path;
  if (global_plan.poses.empty()) {
    return transformed_path;
  }

  const auto &plan_pose = global_plan.poses.front();
  try {
    auto plan_to_global =
        tf_buffer.lookupTransform(global_frame, tf2_ros::fromMsg(now), plan_pose.header.frame_id,
                                  tf2_ros::fromMsg(plan_pose.header.stamp),
                                  plan_pose.header.frame_id, tf2::durationFromSec(0.5));

    geometry_msgs::msg::PoseStamped robot_pose =
        tf_buffer.transform(global_pose, plan_pose.header.frame_id);

    const double dist_threshold =
        std::max(costmap.getSizeInCellsX() * costmap.getResolution() / 2.0,
                 costmap.getSizeInCellsY() * costmap.getResolution() / 2.0) *
        0.85;
    const double sq_dist_threshold = dist_threshold * dist_threshold;

    int i = 0;
    double sq_dist = 1e10;
    bool robot_reached = false;
    for (int j = 0; j < static_cast<int>(global_plan.poses.size()); ++j) {
      double dx = robot_pose.pose.position.x - global_plan.poses[j].pose.position.x;
      double dy = robot_pose.pose.position.y - global_plan.poses[j].pose.position.y;
      double new_sq_dist = dx * dx + dy * dy;
      if (new_sq_dist > sq_dist_threshold) {
        break;
      }
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

    geometry_msgs::msg::PoseStamped newer_pose;
    double plan_length = 0.0;
    while (i < static_cast<int>(global_plan.poses.size()) && sq_dist <= sq_dist_threshold &&
           (max_plan_length <= 0.0 || plan_length <= max_plan_length)) {
      tf2::doTransform(global_plan.poses[i], newer_pose, plan_to_global);
      transformed_path.poses.push_back(newer_pose);
      double dx = robot_pose.pose.position.x - global_plan.poses[i].pose.position.x;
      double dy = robot_pose.pose.position.y - global_plan.poses[i].pose.position.y;
      sq_dist = dx * dx + dy * dy;
      if (i > 0 && max_plan_length > 0.0)
        plan_length += nav2_util::geometry_utils::euclidean_distance(global_plan.poses[i - 1].pose,
                                                                     global_plan.poses[i].pose);
      ++i;
    }

    if (transformed_path.poses.empty()) {
      tf2::doTransform(global_plan.poses.back(), newer_pose, plan_to_global);
      transformed_path.poses.push_back(newer_pose);
      if (current_goal_idx) {
        *current_goal_idx = int(global_plan.poses.size()) - 1;
      }
    } else {
      if (current_goal_idx) {
        *current_goal_idx = i - 1;
      }
    }
  } catch (const tf2::TransformException &) {
    return nav_msgs::msg::Path();
  }
  return transformed_path;
}

void saturateVelocity(geometry_msgs::msg::Twist &cmd_vel, double v_max_x, double v_max_y,
                      double v_max_theta, double v_max_x_backwards,
                      bool use_proportional_saturation) {
  double ratio_x = 1.0, ratio_omega = 1.0, ratio_y = 1.0;

  if (cmd_vel.linear.x > v_max_x) {
    ratio_x = v_max_x / cmd_vel.linear.x;
  }

  if (cmd_vel.linear.y > v_max_y || cmd_vel.linear.y < -v_max_y) {
    ratio_y = std::abs(v_max_y / cmd_vel.linear.y);
  }

  if (cmd_vel.angular.z > v_max_theta || cmd_vel.angular.z < -v_max_theta) {
    ratio_omega = std::abs(v_max_theta / cmd_vel.angular.z);
  }

  if (cmd_vel.linear.x < -v_max_x_backwards) {
    ratio_x = -v_max_x_backwards / cmd_vel.linear.x;
  }

  if (use_proportional_saturation) {
    double ratio = std::min({ratio_x, ratio_y, ratio_omega});
    cmd_vel.linear.x *= ratio;
    cmd_vel.linear.y *= ratio;
    cmd_vel.angular.z *= ratio;
  } else {
    cmd_vel.linear.x *= ratio_x;
    cmd_vel.linear.y *= ratio_y;
    cmd_vel.angular.z *= ratio_omega;
  }
}

geometry_msgs::msg::Twist convertAckermannToTwist(double wheelspeed, double angle,
                                                  double wheelbase) {
  geometry_msgs::msg::Twist twist_cmd;
  twist_cmd.linear.x = wheelspeed * std::cos(angle);
  twist_cmd.angular.z = wheelspeed * std::sin(angle) / wheelbase;
  return twist_cmd;
}

std::pair<double, double> convertTwistToAckermann(const geometry_msgs::msg::Twist &twist_cmd,
                                                  double wheelbase,
                                                  std::optional<double> current_angle) {
  const double lin_x = twist_cmd.linear.x;
  const double ang_z = twist_cmd.angular.z;
  double cmd_angle = 0.0;
  double cmd_speed = 0.0;
  if (std::abs(lin_x) < 1e-5 && std::abs(ang_z) > 1e-5) {
    cmd_angle = std::copysign(M_PI_2, ang_z);
    cmd_speed = std::abs(ang_z) * wheelbase;
    if (current_angle.has_value()) {
      if (std::abs(cmd_angle - current_angle.value()) > M_PI_2) {
        cmd_angle -= std::copysign(M_PI, cmd_angle);
        cmd_speed *= -1.0;
      }
    }
  } else {
    cmd_angle = std::atan2(ang_z * wheelbase, lin_x);
    cmd_speed = lin_x / std::cos(cmd_angle);
  }
  return {cmd_speed, cmd_angle};
}

void saturateSteeringAngle(geometry_msgs::msg::Twist &cmd_vel, double current_angle,
                           double steering_rate_max, double wheelbase, double dt) {
  auto [cmd_speed, cmd_angle] = convertTwistToAckermann(cmd_vel, wheelbase);
  double angle_diff = std::abs(angles::normalize_angle(cmd_angle - current_angle));
  double angle_rate = std::abs(angle_diff) / dt;
  if (angle_rate > steering_rate_max) {
    cmd_angle += std::copysign(steering_rate_max, angle_diff) * dt;
  }
  cmd_vel = convertAckermannToTwist(cmd_speed, cmd_angle, wheelbase);
}

double computeCurvature(const PoseSE2 &p1, const PoseSE2 &p2, const PoseSE2 &p3) {
  // Tangential vectors
  Eigen::Vector2d v1 = (p2.position() - p1.position()).normalized();
  Eigen::Vector2d v2 = (p3.position() - p2.position()).normalized();

  // Angle between tangents
  double angle1 = atan2(v1.y(), v1.x());
  double angle2 = atan2(v2.y(), v2.x());
  double d_angle = angles::normalize_angle(angle2 - angle1);

  // Finite difference curvature: κ = 2*sin(Δα/2) / chord_length
  double chord_length = (p3.position() - p1.position()).norm();
  if (chord_length < 1e-3) {
    return 0.0;  // Degenerate case
  }

  double kappa = 2.0 * sin(d_angle / 2.0) / chord_length;

  // Signed curvature (left/right turn)
  double cross_prod = v1.x() * v2.y() - v1.y() * v2.x();
  return copysign(kappa, cross_prod);
}

int checkFeasibility(const TimedElasticBand &teb, const ObstacleMap2D &esdf, const Footprint &fp,
                     double lookahead) {
  double progress = 0.0;
  for (size_t i = 0; i < teb.sizePoses(); i++) {
    if (i > 0) {
      progress += (teb.pose(i).position() - teb.pose(i - 1).position()).norm();
    }
    if (progress > lookahead) {
      return -1;
    }

    const auto &pose = teb.pose(i);
    const double px = pose.x();
    const double py = pose.y();
    const double ct = std::cos(pose.theta());
    const double st = std::sin(pose.theta());

    const auto &circles = fp.circles();
    for (const auto &c : circles) {
      const double wx = px + ct * c.offset.x() - st * c.offset.y();
      const double wy = py + st * c.offset.x() + ct * c.offset.y();

      const double dist = esdf.queryDistance(wx, wy) - c.radius;

      if (dist > 0.0) {
        continue;
      } else {
        return static_cast<int>(i);
      }
    }
  }
  return -1;
}
}  // namespace nav2_teb_controller
