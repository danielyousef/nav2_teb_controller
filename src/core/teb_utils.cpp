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

// NOTE: extractVelocity / getVelocityCommand / saturateVelocity / saturateSteeringAngle /
// convertAckermannToTwist / convertTwistToAckermann were moved into the BandController module
// (src/band_controller/band_controller.cpp). pruneGlobalPlan / transformAndTrimPlan were moved
// into PathHandler (src/path_handler.cpp).

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
