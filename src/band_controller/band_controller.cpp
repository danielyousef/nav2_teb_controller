#include "nav2_teb_controller/band_controller/band_controller.hpp"

#include <angles/angles.h>
#include <tf2/utils.h>

#include <cmath>

#include "nav2_util/geometry_utils.hpp"

namespace nav2_teb_controller {

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
      look_ahead_poses = counter + 1;
      break;
    }
  }
  if (dt <= 1e-9) {
    return cmd_vel;
  }
  cmd_vel = extractVelocity(teb.pose(0), teb.pose(static_cast<std::size_t>(look_ahead_poses)), dt,
                            holonomic);
  return cmd_vel;
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

void BandController::applySaturation(geometry_msgs::msg::Twist &cmd, double current_steering_angle,
                                     double dt) const {
  const auto &r = params_->FollowPath.robot;
  saturateVelocity(cmd, r.v_max_x, r.v_max_y, r.v_max_theta, r.v_max_x_backwards,
                   r.use_proportional_saturation);
  saturateSteeringAngle(cmd, current_steering_angle, r.steering_rate_max, r.wheelbase, dt);
}

geometry_msgs::msg::Twist BandController::computeCommand(
    const TimedElasticBand &teb, const geometry_msgs::msg::PoseStamped &robot_pose,
    const geometry_msgs::msg::Twist &robot_vel, double current_steering_angle, double dt) {
  geometry_msgs::msg::Twist cmd = computeRawCommand(teb, robot_pose, robot_vel);
  applySaturation(cmd, current_steering_angle, dt);
  return cmd;
}

}  // namespace nav2_teb_controller
