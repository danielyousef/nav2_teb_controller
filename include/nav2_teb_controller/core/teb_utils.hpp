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

// NOTE: extractVelocity / getVelocityCommand / saturateVelocity / saturateSteeringAngle /
// convertAckermannToTwist / convertTwistToAckermann were moved into the BandController module
// (include/nav2_teb_controller/band_controller). pruneGlobalPlan / transformAndTrimPlan were
// moved into PathHandler (include/nav2_teb_controller/path_handler.hpp).

double computeCurvature(const PoseSE2 &p1, const PoseSE2 &p2, const PoseSE2 &p3);

int checkFeasibility(const TimedElasticBand &teb, const ObstacleMap2D &esdf, const Footprint &fp,
                     double lookahead);

}  // namespace nav2_teb_controller
