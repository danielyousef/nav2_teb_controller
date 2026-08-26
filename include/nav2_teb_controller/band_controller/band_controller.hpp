#pragma once

#include <optional>

#include "geometry_msgs/msg/twist.hpp"
#include "nav2_teb_controller/core/pose_se2.hpp"
#include "nav2_teb_controller/core/timed_elastic_band.hpp"
#include "nav2_teb_controller/teb_controller_parameters.hpp"
#include "tf2/transform_datatypes.h"
#include "tf2_ros/buffer.h"

namespace nav2_teb_controller {

// ── Free control utilities (shared by all BandController implementations) ──────

geometry_msgs::msg::Twist extractVelocity(const PoseSE2 &pose1, const PoseSE2 &pose2, double dt,
                                          bool holonomic);

geometry_msgs::msg::Twist getVelocityCommand(const TimedElasticBand &teb, double dt_ref,
                                             int look_ahead_poses, double min_look_ahead_time,
                                             bool holonomic);

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

// ── Abstract BandController interface ──────────────────────────────────────────

// Turns the optimized band (reference trajectory) plus the current robot state into a
// velocity command. `computeCommand` is the non-virtual entry point (applies the common
// velocity/steering saturation after the implementation-specific raw control law).
class BandController {
public:
  virtual ~BandController() = default;

  // Push in the parameter struct (lifetime must outlive the controller; the struct is
  // copy-assigned in place by the ParamListener, so a stored reference stays valid).
  virtual void configure(const teb_controller::Params &params) = 0;

  // Compute the (saturated) velocity command for the current tick.
  //   teb                  — optimized band (reference trajectory)
  //   robot_pose           — current robot pose (global frame)
  //   robot_vel            — current robot velocity
  //   current_steering_angle — last commanded steering angle (for the rate limit)
  //   dt                   — time since the last command (for the steering rate limit)
  geometry_msgs::msg::Twist computeCommand(const TimedElasticBand &teb,
                                           const geometry_msgs::msg::PoseStamped &robot_pose,
                                           const geometry_msgs::msg::Twist &robot_vel,
                                           double current_steering_angle, double dt);

protected:
  // Derived classes implement the raw (pre-saturation) control law.
  [[nodiscard]] virtual geometry_msgs::msg::Twist computeRawCommand(
      const TimedElasticBand &teb, const geometry_msgs::msg::PoseStamped &robot_pose,
      const geometry_msgs::msg::Twist &robot_vel) = 0;

  // Common saturation (velocity limits + proportional option, steering rate limit).
  void applySaturation(geometry_msgs::msg::Twist &cmd, double current_steering_angle,
                       double dt) const;

  const teb_controller::Params *params_{nullptr};
};

}  // namespace nav2_teb_controller
