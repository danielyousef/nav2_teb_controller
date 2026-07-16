#include "nav2_teb_controller/homotopy/visibility_graph.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <rclcpp/rclcpp.hpp>

namespace nav2_teb_controller {

void VisibilityGraph::build(const Eigen::Vector2d &start, const Eigen::Vector2d &goal,
                            const ObstacleArray &obstacles) {
  nodes_.clear();
  adj_.clear();
  start_id_ = -1;
  goal_id_ = -1;

  // --- Step 1: Collect all nodes ---
  int id = 0;

  // Start node
  nodes_.push_back({start, id, true, false});
  start_id_ = id++;

  // Goal node
  nodes_.push_back({goal, id, false, true});
  goal_id_ = id++;

  // Obstacle keypoints (raw polygon vertices, no inflation)
  for (const auto &obs : obstacles.obstacles) {
    auto kp = extractKeypoints(obs);
    for (auto &pt : kp) {
      nodes_.push_back({pt, id++, false, false});
    }
  }

  // --- Step 2: Build adjacency list using ESDF-based visibility ---
  const int n = static_cast<int>(nodes_.size());
  adj_.resize(n);

  for (int i = 0; i < n; ++i) {
    for (int j = i + 1; j < n; ++j) {
      if (isVisible(nodes_[i].pos, nodes_[j].pos)) {
        double cost = (nodes_[i].pos - nodes_[j].pos).norm();
        adj_[i].push_back({i, j, cost});
        adj_[j].push_back({j, i, cost});
      }
    }
  }

  RCLCPP_DEBUG(rclcpp::get_logger("VisibilityGraph"),
               "Built visibility graph: %zu nodes, ~%zu edges",
               nodes_.size(), adj_.size());
}

bool VisibilityGraph::isVisible(const Eigen::Vector2d &p1,
                                const Eigen::Vector2d &p2) const {
  if (!esdf_) {
    RCLCPP_WARN(rclcpp::get_logger("VisibilityGraph"),
                "No ESDF set — visibility check skipped");
    return true;
  }

  if ((p1 - p2).squaredNorm() < 1e-9)
    return false;

  // Sample along the segment at ESDF resolution
  double seg_len = (p2 - p1).norm();
  double step = esdf_->resolution();
  int n_samples = std::max(2, static_cast<int>(std::ceil(seg_len / step)));

  for (int i = 0; i <= n_samples; ++i) {
    double t = static_cast<double>(i) / n_samples;
    Eigen::Vector2d pt = p1 + t * (p2 - p1);
    double dist = esdf_->query(pt.x(), pt.y()).distance;

    // If any sample point is inside an obstacle (distance < 0), segment is blocked
    if (dist < 0.0)
      return false;
  }

  return true;
}

std::vector<Eigen::Vector2d> VisibilityGraph::extractKeypoints(
    const costmap_converter_msgs::msg::ObstacleMsg &obs) const {
  std::vector<Eigen::Vector2d> kp;
  const auto &poly = obs.polygon;

  if (poly.points.empty())
    return kp;

  kp.reserve(poly.points.size());
  for (const auto &pt : poly.points) {
    kp.emplace_back(pt.x, pt.y);
  }

  return kp;
}

}  // namespace nav2_teb_controller
