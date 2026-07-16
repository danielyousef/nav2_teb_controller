#include "nav2_teb_controller/homotopy/visibility_graph_search.hpp"

#include <algorithm>
#include <limits>
#include <queue>
#include <set>
#include <rclcpp/rclcpp.hpp>
#include <cmath>

namespace nav2_teb_controller {

bool VisibilityGraphSearch::search(const PoseSE2 &start, const PoseSE2 &goal,
                                   const ObstacleArray &obstacles, int max_classes,
                                   std::vector<GraphSearchResult> &results) {
  results.clear();

  // Pass ESDF to the visibility graph for collision-free checks
  vis_graph_.setObstacleMap(esdf_);

  // Step 1: Build visibility graph (using ESDF instead of inflation)
  vis_graph_.build(start.position(), goal.position(), obstacles);

  if (vis_graph_.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("VisibilityGraphSearch"), "Empty visibility graph");
    return false;
  }

  // Step 2: Dijkstra for the trivial (shortest) path
  auto shortest = dijkstra(vis_graph_, vis_graph_.startId(), vis_graph_.goalId());
  if (shortest.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("VisibilityGraphSearch"), "No path found by Dijkstra");
    return false;
  }

  // Step 3: Yen's K-Shortest Paths to get alternative homotopy classes
  auto all_paths = yenKShortestPaths(vis_graph_, vis_graph_.startId(),
                                      vis_graph_.goalId(), max_classes);

  // Ensure we include the shortest path at the front
  if (all_paths.empty() || all_paths[0] != shortest) {
    all_paths.insert(all_paths.begin(), shortest);
  }

  // Step 4: Convert node paths to PoseSE2 paths and compute H-signatures
  for (const auto &node_path : all_paths) {
    GraphSearchResult result;
    result.path = nodePathToPoses(vis_graph_, node_path);

    // Compute H-signature for this path
    result.h_signature.compute(result.path, obstacles);

    // Compute path cost as sum of Euclidean distances
    double cost = 0.0;
    for (size_t i = 1; i < result.path.size(); ++i) {
      cost += (result.path[i].position() - result.path[i - 1].position()).norm();
    }
    result.cost = cost;

    results.push_back(std::move(result));
  }

  // Step 5: Filter duplicate homotopy classes
  filterDuplicateClasses(results, 1e-3);

  RCLCPP_DEBUG(rclcpp::get_logger("VisibilityGraphSearch"),
               "Found %zu distinct homotopy classes", results.size());

  return !results.empty();
}

void VisibilityGraphSearch::updateObstacles(const ObstacleArray &obstacles) {
  (void)obstacles;
}

std::vector<int> VisibilityGraphSearch::dijkstra(const VisibilityGraph &graph,
                                                  int start_id, int goal_id) {
  const int n = static_cast<int>(graph.nodes().size());
  if (n == 0 || start_id < 0 || goal_id < 0)
    return {};

  std::vector<double> dist(n, std::numeric_limits<double>::infinity());
  std::vector<int> prev(n, -1);
  std::vector<bool> visited(n, false);

  using PrioPair = std::pair<double, int>;
  std::priority_queue<PrioPair, std::vector<PrioPair>, std::greater<PrioPair>> pq;

  dist[start_id] = 0.0;
  pq.push({0.0, start_id});

  while (!pq.empty()) {
    auto [d, u] = pq.top();
    pq.pop();

    if (visited[u])
      continue;
    visited[u] = true;

    if (u == goal_id)
      break;

    for (const auto &edge : graph.edges(u)) {
      int v = edge.to_id;
      double new_dist = dist[u] + edge.cost;
      if (new_dist < dist[v]) {
        dist[v] = new_dist;
        prev[v] = u;
        pq.push({new_dist, v});
      }
    }
  }

  if (std::isinf(dist[goal_id]))
    return {};

  std::vector<int> path;
  for (int v = goal_id; v != -1; v = prev[v])
    path.push_back(v);
  std::reverse(path.begin(), path.end());

  return path;
}

std::vector<std::vector<int>> VisibilityGraphSearch::yenKShortestPaths(
    const VisibilityGraph &graph, int start_id, int goal_id, int k) {
  std::vector<std::vector<int>> ksp;

  auto first_path = dijkstra(graph, start_id, goal_id);
  if (first_path.empty())
    return ksp;

  ksp.push_back(first_path);

  using PathCandidate = std::pair<double, std::vector<int>>;
  auto cmp = [](const PathCandidate &a, const PathCandidate &b) {
    return a.first > b.first;
  };
  std::priority_queue<PathCandidate, std::vector<PathCandidate>, decltype(cmp)> candidates(cmp);

  const auto &nodes = graph.nodes();
  auto pathCost = [&](const std::vector<int> &p) -> double {
    double cost = 0.0;
    for (size_t i = 1; i < p.size(); ++i) {
      for (const auto &edge : graph.edges(p[i - 1])) {
        if (edge.to_id == p[i]) {
          cost += edge.cost;
          break;
        }
      }
    }
    return cost;
  };

  for (int iter = 1; iter < k; ++iter) {
    const auto &prev_path = ksp[iter - 1];

    for (size_t spur_idx = 0; spur_idx < prev_path.size() - 1; ++spur_idx) {
      int spur_node = prev_path[spur_idx];

      std::vector<int> root_path(prev_path.begin(), prev_path.begin() + spur_idx + 1);

      std::set<std::pair<int, int>> blocked_edges;

      for (const auto &p : ksp) {
        if (p.size() <= spur_idx)
          continue;
        bool same_root = true;
        for (size_t i = 0; i <= spur_idx; ++i) {
          if (p[i] != prev_path[i]) {
            same_root = false;
            break;
          }
        }
        if (same_root && p.size() > spur_idx + 1) {
          blocked_edges.insert({p[spur_idx], p[spur_idx + 1]});
        }
      }

      const int n = static_cast<int>(nodes.size());
      std::vector<double> dist(n, std::numeric_limits<double>::infinity());
      std::vector<int> prev(n, -1);
      std::vector<bool> visited(n, false);

      std::set<int> removed_nodes(root_path.begin(), root_path.end() - 1);

      using PrioPair = std::pair<double, int>;
      std::priority_queue<PrioPair, std::vector<PrioPair>, std::greater<PrioPair>> pq;

      dist[spur_node] = 0.0;
      pq.push({0.0, spur_node});

      while (!pq.empty()) {
        auto [d, u] = pq.top();
        pq.pop();

        if (visited[u])
          continue;
        visited[u] = true;

        if (u == goal_id)
          break;

        for (const auto &edge : graph.edges(u)) {
          int v = edge.to_id;

          if (blocked_edges.count({u, v}))
            continue;

          if (removed_nodes.count(v))
            continue;

          double new_dist = dist[u] + edge.cost;
          if (new_dist < dist[v]) {
            dist[v] = new_dist;
            prev[v] = u;
            pq.push({new_dist, v});
          }
        }
      }

      if (!std::isinf(dist[goal_id])) {
        std::vector<int> spur_path;
        for (int v = goal_id; v != -1; v = prev[v])
          spur_path.push_back(v);
        std::reverse(spur_path.begin(), spur_path.end());

        std::vector<int> total_path = root_path;
        total_path.insert(total_path.end(), spur_path.begin(), spur_path.end());

        double total_cost = pathCost(total_path);
        candidates.push({total_cost, std::move(total_path)});
      }
    }

    while (!candidates.empty()) {
      auto [cost, path] = candidates.top();
      candidates.pop();

      bool already_found = false;
      for (const auto &existing : ksp) {
        if (existing == path) {
          already_found = true;
          break;
        }
      }

      if (!already_found) {
        ksp.push_back(std::move(path));
        break;
      }
    }

    if (candidates.empty())
      break;
  }

  return ksp;
}

std::vector<PoseSE2> VisibilityGraphSearch::nodePathToPoses(
    const VisibilityGraph &graph, const std::vector<int> &node_path) {
  std::vector<PoseSE2> poses;
  poses.reserve(node_path.size());

  const auto &nodes = graph.nodes();
  for (int id : node_path) {
    if (id >= 0 && id < static_cast<int>(nodes.size())) {
      poses.emplace_back(nodes[id].pos.x(), nodes[id].pos.y(), 0.0);
    }
  }

  for (size_t i = 0; i + 1 < poses.size(); ++i) {
    Eigen::Vector2d dir = poses[i + 1].position() - poses[i].position();
    double theta = std::atan2(dir.y(), dir.x());
    poses[i].theta() = theta;
  }
  if (poses.size() >= 2) {
    poses.back().theta() = poses[poses.size() - 2].theta();
  }

  return poses;
}

void VisibilityGraphSearch::filterDuplicateClasses(
    std::vector<GraphSearchResult> &results, double h_sig_tolerance) {
  if (results.size() <= 1)
    return;

  std::vector<GraphSearchResult> unique;
  unique.reserve(results.size());

  for (auto &result : results) {
    bool is_duplicate = false;
    for (const auto &existing : unique) {
      if (result.h_signature.isEqual(existing.h_signature, h_sig_tolerance)) {
        is_duplicate = true;
        break;
      }
    }
    if (!is_duplicate)
      unique.push_back(std::move(result));
  }

  results = std::move(unique);
}

}  // namespace nav2_teb_controller
