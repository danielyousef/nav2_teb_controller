#pragma once

#include "nav2_teb_controller/band_controller/band_controller.hpp"

namespace nav2_teb_controller {

// Reads the velocity command straight off the optimized band: a finite-difference
// velocity over the first `control_look_ahead_poses` segments (bounded by dt_ref and
// the minimum lookahead time). This is the original TEBController step-6 behavior.
class FeedForwardController : public BandController {
public:
  void configure(const teb_controller::Params &params) override;

protected:
  [[nodiscard]] geometry_msgs::msg::Twist computeRawCommand(
      const TimedElasticBand &teb, const geometry_msgs::msg::PoseStamped &robot_pose,
      const geometry_msgs::msg::Twist &robot_vel) override;
};

}  // namespace nav2_teb_controller
