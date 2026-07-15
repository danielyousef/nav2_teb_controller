#include "nav2_teb_controller/homotopy/h_signature.hpp"

#include <cmath>
#include <limits>

namespace nav2_teb_controller {

void HSignature::compute(const std::vector<PoseSE2> &path, const ObstacleArray &obstacles) {
  signature_.clear();

  if (path.size() < 2)
    return;

  // One complex component per obstacle
  const size_t n_obstacles = obstacles.obstacles.size();
  if (n_obstacles == 0)
    return;

  // Initialize signatures to zero
  std::vector<double> cumulative(n_obstacles, 0.0);

  // For each segment in the path
  for (size_t seg = 0; seg < path.size() - 1; ++seg) {
    const Eigen::Vector2d p1 = path[seg].position();
    const Eigen::Vector2d p2 = path[seg + 1].position();

    // For each obstacle
    for (size_t k = 0; k < n_obstacles; ++k) {
      const auto &obs = obstacles.obstacles[k];
      // Compute obstacle center as centroid of polygon
      Eigen::Vector2d center(0, 0);
      if (obs.polygon.points.empty())
        continue;
      for (const auto &pt : obs.polygon.points) {
        center.x() += pt.x;
        center.y() += pt.y;
      }
      center /= static_cast<double>(obs.polygon.points.size());

      // Contribution = signed angle of segment as seen from obstacle center
      // This is the imaginary part of log((p2 - z_k) / (p1 - z_k))
      Eigen::Vector2d v1 = p1 - center;
      Eigen::Vector2d v2 = p2 - center;

      double cross = v1.x() * v2.y() - v1.y() * v2.x();
      double dot = v1.x() * v2.x() + v1.y() * v2.y();

      double angle = std::atan2(cross, dot);
      cumulative[k] += angle;
    }
  }

  // Normalize and store as complex numbers
  // The real part encodes the winding number, the imaginary part is 0
  // (we store only the cumulative angle in the complex component)
  signature_.reserve(n_obstacles);
  for (size_t k = 0; k < n_obstacles; ++k) {
    // Store as complex where real = cumulative angle / (2*PI)
    // This gives the winding number contribution for each obstacle
    signature_.emplace_back(cumulative[k] / (2.0 * M_PI), 0.0);
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

std::complex<double> HSignature::computeSegmentContribution(
    const Eigen::Vector2d &p1, const Eigen::Vector2d &p2,
    const Eigen::Vector2d &obs_center) const {
  Eigen::Vector2d v1 = p1 - obs_center;
  Eigen::Vector2d v2 = p2 - obs_center;

  double cross = v1.x() * v2.y() - v1.y() * v2.x();
  double dot = v1.x() * v2.x() + v1.y() * v2.y();

  double angle = std::atan2(cross, dot);
  return std::complex<double>(angle / (2.0 * M_PI), 0.0);
}

}  // namespace nav2_teb_controller
