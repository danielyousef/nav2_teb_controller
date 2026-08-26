#pragma once
#include <Eigen/Core>
#include <complex>
#include <costmap_converter_msgs/msg/obstacle_array_msg.hpp>
#include <vector>

#include "nav2_teb_controller/core/pose_se2.hpp"

namespace nav2_teb_controller {

/**
 * @brief H-Signature for 2D environments
 * Encodes homotopy class of a path relative to obstacles.
 * Based on: Bhattacharya et al., "Search-Based Path Planning with Homotopy Class Constraints"
 *
 * For each obstacle k: L_k = integral along path of Re(1/(z - z_k))
 * Two paths are in the same homotopy class iff their signatures match.
 */
class HSignature {
public:
  using ObstacleArray = costmap_converter_msgs::msg::ObstacleArrayMsg;

  HSignature() = default;

  /**
   * @brief Compute H-signature for a path given obstacles
   * @param path Sequence of 2D poses
   * @param obstacles Current obstacle array
   */
  void compute(const std::vector<PoseSE2> &path, const ObstacleArray &obstacles);

  /**
   * @brief Check if two signatures belong to same homotopy class
   * @param other Other H-signature
   * @param tolerance Numerical tolerance for comparison
   */
  [[nodiscard]] bool isEqual(const HSignature &other, double tolerance = 1e-3) const;

  /**
   * @brief Check if signature is valid (no NaN/Inf)
   */
  [[nodiscard]] bool isValid() const;

  /**
   * @brief Raw signature vector (one complex value per obstacle)
   */
  [[nodiscard]] const std::vector<std::complex<double>> &signature() const { return signature_; }

private:
  std::vector<std::complex<double>> signature_;
};

}  // namespace nav2_teb_controller
