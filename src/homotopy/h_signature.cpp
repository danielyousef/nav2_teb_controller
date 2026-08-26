#include "nav2_teb_controller/homotopy/h_signature.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace nav2_teb_controller {

void HSignature::compute(const std::vector<PoseSE2> &path, const ObstacleArray &obstacles) {
  signature_.clear();

  if (path.size() < 2)
    return;

  // One complex component per obstacle
  const size_t n_obstacles = obstacles.obstacles.size();
  if (n_obstacles == 0)
    return;

  // Canonical obstacle ordering: the converter's array order is not stable across ticks,
  // and signature components are compared positionally (isEqual). Sorting indices by
  // centroid makes signatures identical for the same obstacle SET regardless of order —
  // per-class planners keep matching across ticks (warm starts persist).
  std::vector<Eigen::Vector2d> centers(n_obstacles);
  for (size_t k = 0; k < n_obstacles; ++k) {
    centers[k].setZero();
    const auto &poly = obstacles.obstacles[k].polygon.points;
    if (poly.empty())
      continue;
    for (const auto &pt : poly)
      centers[k] += Eigen::Vector2d(pt.x, pt.y);
    centers[k] /= static_cast<double>(poly.size());
  }

  std::vector<size_t> order(n_obstacles);
  std::iota(order.begin(), order.end(), 0);
  constexpr double kCentroidEps = 1e-6;
  std::sort(order.begin(), order.end(), [&centers](size_t a, size_t b) {
    const Eigen::Vector2d &ca = centers[a];
    const Eigen::Vector2d &cb = centers[b];
    if (std::abs(ca.x() - cb.x()) > kCentroidEps)
      return ca.x() < cb.x();
    return ca.y() < cb.y();
  });

  // Initialize signatures to zero
  std::vector<double> cumulative(n_obstacles, 0.0);

  // Net swept angle per obstacle: sum of principal-value segment rotations. For fine
  // paths this accumulates the CONTINUOUS polar angle change, including multiples of
  // 2*pi for every full encirclement.
  for (size_t seg = 0; seg + 1 < path.size(); ++seg) {
    const Eigen::Vector2d p1 = path[seg].position();
    const Eigen::Vector2d p2 = path[seg + 1].position();
    for (size_t k = 0; k < n_obstacles; ++k) {
      const Eigen::Vector2d v1 = p1 - centers[k];
      const Eigen::Vector2d v2 = p2 - centers[k];
      if (v1.squaredNorm() < 1e-12 || v2.squaredNorm() < 1e-12)
        continue;  // degenerate: path point on obstacle center
      cumulative[k] +=
          std::atan2(v1.x() * v2.y() - v1.y() * v2.x(), v1.x() * v2.x() + v1.y() * v2.y());
    }
  }

  // Homotopy invariant: the swept angle minus the principal endpoint angle leaves the
  // pure integer winding count. Raw swept angles are geometry-dependent (a path merely
  // PASSING NEAR an obstacle subtends a nonzero angle) and would not compare equal
  // within one class.
  const Eigen::Vector2d &p_first = path.front().position();
  const Eigen::Vector2d &p_last = path.back().position();

  signature_.reserve(n_obstacles);
  for (const size_t k : order) {
    const Eigen::Vector2d v0 = p_first - centers[k];
    const Eigen::Vector2d v1 = p_last - centers[k];
    double principal_delta = 0.0;
    if (v0.squaredNorm() >= 1e-12 && v1.squaredNorm() >= 1e-12) {
      // Principal-value rotation from v0 to v1 (same convention as the segment sum)
      principal_delta =
          std::atan2(v0.x() * v1.y() - v0.y() * v1.x(), v0.x() * v1.x() + v0.y() * v1.y());
    }
    const double winding = (cumulative[k] - principal_delta) / (2.0 * M_PI);
    // Snap to the nearest integer: kills numerical drift of the accumulated angles
    signature_.emplace_back(std::round(winding), 0.0);
  }
}

bool HSignature::isEqual(const HSignature &other, double tolerance) const {
  if (signature_.size() != other.signature_.size())
    return false;

  for (size_t i = 0; i < signature_.size(); ++i) {
    double diff = std::abs(signature_[i].real() - other.signature_[i].real());
    if (diff > tolerance)
      return false;
  }
  return true;
}

bool HSignature::isValid() const {
  for (const auto &s : signature_) {
    if (std::isnan(s.real()) || std::isinf(s.real()))
      return false;
  }
  return true;
}

}  // namespace nav2_teb_controller
