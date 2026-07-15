#include "nav2_teb_controller/homotopy/visibility_graph.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <rclcpp/rclcpp.hpp>

namespace nav2_teb_controller {

void VisibilityGraph::build(const Eigen::Vector2d &start, const Eigen::Vector2d &goal,
                            const ObstacleArray &obstacles, double inflation) {
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

  // Obstacle keypoints
  for (const auto &obs : obstacles.obstacles) {
    auto kp = extractKeypoints(obs, inflation);
    for (auto &pt : kp) {
      nodes_.push_back({pt, id++, false, false});
    }
  }

  // --- Step 2: Build adjacency list ---
  const int n = static_cast<int>(nodes_.size());
  adj_.resize(n);

  for (int i = 0; i < n; ++i) {
    for (int j = i + 1; j < n; ++j) {
      if (isVisible(nodes_[i].pos, nodes_[j].pos, obstacles, inflation)) {
        double cost = (nodes_[i].pos - nodes_[j].pos).norm();
        adj_[i].push_back({i, j, cost});
        adj_[j].push_back({j, i, cost});
      }
    }
  }

  RCLCPP_DEBUG(rclcpp::get_logger("VisibilityGraph"),
               "Built visibility graph: %zu nodes, %zu edges (approx)",
               nodes_.size(), adj_.size());
}

bool VisibilityGraph::isVisible(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2,
                                const ObstacleArray &obstacles, double inflation) const {
  // Quick self-check: same point is not visible
  if ((p1 - p2).squaredNorm() < 1e-9)
    return false;

  for (const auto &obs : obstacles.obstacles) {
    const auto &poly = obs.polygon;
    if (poly.points.size() < 3)
      continue;

    const size_t N = poly.points.size();
    std::vector<Eigen::Vector2d> verts;
    verts.reserve(N);
    for (const auto &pt : poly.points) {
      verts.emplace_back(pt.x, pt.y);
    }

    auto orient = [](const Eigen::Vector2d &p, const Eigen::Vector2d &q,
                     const Eigen::Vector2d &r) -> double {
      return (q.x() - p.x()) * (r.y() - p.y()) - (q.y() - p.y()) * (r.x() - p.x());
    };

    // Check if segment p1-p2 intersects any polygon edge
    for (size_t i = 0; i < N; ++i) {
      const size_t j = (i + 1) % N;
      const Eigen::Vector2d &a = verts[i];
      const Eigen::Vector2d &b = verts[j];

      double o1 = orient(p1, p2, a);
      double o2 = orient(p1, p2, b);
      double o3 = orient(a, b, p1);
      double o4 = orient(a, b, p2);

      if (o1 * o2 < 0.0 && o3 * o4 < 0.0)
        return false;
    }

    // Check if either endpoint is inside the polygon (point-in-polygon)
    auto pointInPolygon = [&](const Eigen::Vector2d &pt) -> bool {
      int winding = 0;
      for (size_t i = 0; i < N; ++i) {
        const size_t j = (i + 1) % N;
        const Eigen::Vector2d &a = verts[i];
        const Eigen::Vector2d &b = verts[j];

        if (a.y() <= pt.y()) {
          if (b.y() > pt.y() && orient(a, b, pt) > 0.0)
            ++winding;
        } else {
          if (b.y() <= pt.y() && orient(a, b, pt) < 0.0)
            --winding;
        }
      }
      return winding != 0;
    };

    if (pointInPolygon(p1) || pointInPolygon(p2))
      return false;

    // Check distance from segment to each polygon vertex (for inflation)
    // If segment passes too close to any vertex, consider it blocked
    double inflate_sq = inflation * inflation;
    Eigen::Vector2d seg_dir = p2 - p1;
    double seg_len_sq = seg_dir.squaredNorm();
    if (seg_len_sq < 1e-12)
      continue;

    for (size_t i = 0; i < N; ++i) {
      const Eigen::Vector2d &v = verts[i];
      double t = ((v - p1).dot(seg_dir)) / seg_len_sq;
      t = std::max(0.0, std::min(1.0, t));
      Eigen::Vector2d closest = p1 + t * seg_dir;
      double dist_sq = (v - closest).squaredNorm();
      if (dist_sq < inflate_sq)
        return false;
    }
  }

  return true;
}

std::vector<Eigen::Vector2d> VisibilityGraph::extractKeypoints(
    const costmap_converter_msgs::msg::ObstacleMsg &obs, double inflation) const {
  std::vector<Eigen::Vector2d> kp;
  const auto &poly = obs.polygon;

  if (poly.points.empty())
    return kp;

  // Compute centroid
  Eigen::Vector2d centroid(0, 0);
  for (const auto &pt : poly.points) {
    centroid.x() += pt.x;
    centroid.y() += pt.y;
  }
  centroid /= static_cast<double>(poly.points.size());

  // Inflate vertices outward from centroid
  kp.reserve(poly.points.size());
  for (const auto &pt : poly.points) {
    Eigen::Vector2d vertex(pt.x, pt.y);
    Eigen::Vector2d dir = vertex - centroid;
    double len = dir.norm();
    if (len > 1e-6) {
      dir /= len;
    } else {
      dir = Eigen::Vector2d(1.0, 0.0);
    }
    Eigen::Vector2d inflated = vertex + dir * inflation;
    kp.push_back(inflated);
  }

  return kp;
}

}  // namespace nav2_teb_controller
