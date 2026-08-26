#include "nav2_teb_controller/band_controller/feed_forward_controller.hpp"

namespace nav2_teb_controller {

void FeedForwardController::configure(const teb_controller::Params &params) {
  params_ = &params;
}

geometry_msgs::msg::Twist FeedForwardController::computeRawCommand(
    const TimedElasticBand &teb, const geometry_msgs::msg::PoseStamped & /*robot_pose*/,
    const geometry_msgs::msg::Twist & /*robot_vel*/) {
  const auto &t = params_->FollowPath.trajectory;
  const bool holonomic = params_->FollowPath.robot.v_max_y > 0.0;
  return getVelocityCommand(teb, t.dt_ref, t.control_look_ahead_poses,
                            t.control_min_look_ahead_time, holonomic);
}

}  // namespace nav2_teb_controller
