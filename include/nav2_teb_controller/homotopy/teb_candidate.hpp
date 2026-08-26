#pragma once
#include <memory>

#include "nav2_teb_controller/core/timed_elastic_band.hpp"
#include "nav2_teb_controller/homotopy/h_signature.hpp"

namespace nav2_teb_controller {

/**
 * @brief One TEB candidate with its homotopy class and optimization cost
 */
struct TebCandidate {
  using Ptr = std::shared_ptr<TebCandidate>;

  TimedElasticBand teb;
  HSignature h_signature;
  int route_id = -1;  ///< Stable geometric identity across ticks (see HomotopyClassPlanner)
  double optimization_cost = std::numeric_limits<double>::infinity();
  /// Summed chi2 over the efficiency-category edges (time optimal + shortest path +
  /// smoothness), normalized by pose count. Drives best-candidate selection/hysteresis —
  /// unlike optimization_cost it excludes obstacle-proximity noise.
  double efficiency_cost = std::numeric_limits<double>::infinity();
  /// Arc length [m] of the optimized band (robot pose → mission-goal seed). Comparable
  /// across candidates — all bands span the same endpoints — and used by the progress
  /// guard so takeovers by longer detours require an explicit slack.
  double path_length = std::numeric_limits<double>::infinity();
  bool is_feasible = false;
  bool has_diverged = false;

  /**
   * @brief Compare two candidates (for best selection)
   * Priority: feasible first, then lowest cost
   */
  bool operator<(const TebCandidate &other) const {
    if (is_feasible != other.is_feasible)
      return is_feasible > other.is_feasible;
    return optimization_cost < other.optimization_cost;
  }
};

}  // namespace nav2_teb_controller
