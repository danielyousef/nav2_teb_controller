#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <set>
#include <utility>
#include <vector>

#include "nav2_teb_controller/core/pose_se2.hpp"
#include "nav2_teb_controller/homotopy/graph_search_interface.hpp"

namespace nav2_teb_controller::graph_algo {

/// Shared graph-search algorithms for homotopy exploration. Templated on the graph type so the
/// visibility-graph and Voronoi-graph searches share one implementation.
///
/// Requirements on @p GraphT:
///   - `nodes()` → sequence of node objects exposing `.pos` (Eigen::Vector2d)
///   - `edges(int id)` → sequence of edge objects exposing `.to_id` (int) and `.cost` (double)

/// @brief Dijkstra shortest path.
/// @return Node-ID path from @p start_id to @p goal_id, empty if unreachable.
template <typename GraphT>
std::vector<int> shortestPath(const GraphT &graph, int start_id, int goal_id) {
  const int n = static_cast<int>(graph.nodes().size());
  if (n == 0 || start_id < 0 || goal_id < 0 || start_id >= n || goal_id >= n)
    return {};

  std::vector<double> dist(n, std::numeric_limits<double>::infinity());
  std::vector<int> prev(n, -1);
  std::vector<bool> visited(n, false);

  using PrioPair = std::pair<double, int>;
  std::priority_queue<PrioPair, std::vector<PrioPair>, std::greater<>> pq;

  dist[start_id] = 0.0;
  pq.emplace(0.0, start_id);

  while (!pq.empty()) {
    auto [d, u] = pq.top();
    pq.pop();

    if (visited[u])
      continue;
    visited[u] = true;

    if (u == goal_id)
      break;

    for (const auto &edge : graph.edges(u)) {
      const int v = edge.to_id;
      const double new_dist = dist[u] + edge.cost;
      if (new_dist < dist[v]) {
        dist[v] = new_dist;
        prev[v] = u;
        pq.emplace(new_dist, v);
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

/// @brief Summed edge cost of a node path (infinity if any edge is missing).
template <typename GraphT>
double pathCost(const GraphT &graph, const std::vector<int> &path) {
  double cost = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    bool found = false;
    for (const auto &edge : graph.edges(path[i - 1])) {
      if (edge.to_id == path[i]) {
        cost += edge.cost;
        found = true;
        break;
      }
    }
    if (!found)
      return std::numeric_limits<double>::infinity();
  }
  return cost;
}

/// @brief Yen's K-shortest loopless paths for alternative homotopy classes.
///
/// Returns up to @p k distinct node-ID paths ordered by cost. Note: distinct paths are not
/// necessarily distinct homotopy classes — callers must filter by H-signature and re-request
/// with a larger @p k until enough classes are found.
template <typename GraphT>
std::vector<std::vector<int>> kShortestPaths(const GraphT &graph, int start_id, int goal_id,
                                             int k) {
  std::vector<std::vector<int>> ksp;

  auto first_path = shortestPath(graph, start_id, goal_id);
  if (first_path.empty())
    return ksp;

  ksp.push_back(first_path);

  using PathCandidate = std::pair<double, std::vector<int>>;
  auto cmp = [](const PathCandidate &a, const PathCandidate &b) { return a.first > b.first; };
  std::priority_queue<PathCandidate, std::vector<PathCandidate>, decltype(cmp)> candidates(cmp);

  for (int iter = 1; iter < k; ++iter) {
    const auto &prev_path = ksp[iter - 1];

    for (size_t spur_idx = 0; spur_idx + 1 < prev_path.size(); ++spur_idx) {
      const int spur_node = prev_path[spur_idx];

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
        if (same_root && p.size() > spur_idx + 1)
          blocked_edges.insert({p[spur_idx], p[spur_idx + 1]});
      }

      const int n = static_cast<int>(graph.nodes().size());
      std::vector<double> dist(n, std::numeric_limits<double>::infinity());
      std::vector<int> prev(n, -1);
      std::vector<bool> visited(n, false);

      const std::set<int> removed_nodes(root_path.begin(), root_path.end() - 1);

      using PrioPair = std::pair<double, int>;
      std::priority_queue<PrioPair, std::vector<PrioPair>, std::greater<>> pq;

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
          const int v = edge.to_id;

          if (blocked_edges.contains({u, v}))
            continue;

          if (removed_nodes.contains(v))
            continue;

          const double new_dist = dist[u] + edge.cost;
          if (new_dist < dist[v]) {
            dist[v] = new_dist;
            prev[v] = u;
            pq.emplace(new_dist, v);
          }
        }
      }

      if (!std::isinf(dist[goal_id])) {
        std::vector<int> spur_path;
        for (int v = goal_id; v != -1; v = prev[v])
          spur_path.push_back(v);
        std::reverse(spur_path.begin(), spur_path.end());
        // The reconstruction starts at the spur node, which is already the last element
        // of root_path — drop it to avoid duplicating the vertex.
        if (!spur_path.empty() && spur_path.front() == spur_node)
          spur_path.erase(spur_path.begin());

        std::vector<int> total_path = root_path;
        total_path.insert(total_path.end(), spur_path.begin(), spur_path.end());

        candidates.push({pathCost(graph, total_path), std::move(total_path)});
      }
    }

    bool added = false;
    while (!candidates.empty() && !added) {
      auto [cost, path] = candidates.top();
      candidates.pop();

      const bool already_found = std::any_of(
          ksp.begin(), ksp.end(), [&path](const auto &existing) { return existing == path; });

      if (!already_found) {
        ksp.push_back(std::move(path));
        added = true;
      }
    }

    if (!added)
      break;  // candidate queue exhausted without a new path
  }

  return ksp;
}

/// @brief Convert a node-ID path into a PoseSE2 polyline (heading from segment direction).
template <typename GraphT>
std::vector<PoseSE2> pathToPoses(const GraphT &graph, const std::vector<int> &node_path) {
  std::vector<PoseSE2> poses;
  poses.reserve(node_path.size());

  const auto &nodes = graph.nodes();
  for (int id : node_path) {
    if (id >= 0 && id < static_cast<int>(nodes.size()))
      poses.emplace_back(nodes[id].pos.x(), nodes[id].pos.y(), 0.0);
  }

  for (size_t i = 0; i + 1 < poses.size(); ++i) {
    const Eigen::Vector2d dir = poses[i + 1].position() - poses[i].position();
    poses[i].theta() = std::atan2(dir.y(), dir.x());
  }
  if (poses.size() >= 2)
    poses.back().theta() = poses[poses.size() - 2].theta();

  return poses;
}

/// @brief Remove results whose H-signatures duplicate an earlier (cheaper) result.
inline void filterDuplicateClasses(std::vector<GraphSearchResult> &results,
                                   double h_sig_tolerance = 1e-3) {
  if (results.size() <= 1)
    return;

  std::vector<GraphSearchResult> unique;
  unique.reserve(results.size());

  for (auto &result : results) {
    const bool is_duplicate = std::any_of(
        unique.begin(), unique.end(), [&result, h_sig_tolerance](const auto &existing) {
          return result.h_signature.isEqual(existing.h_signature, h_sig_tolerance);
        });
    if (!is_duplicate)
      unique.push_back(std::move(result));
  }

  results = std::move(unique);
}

}  // namespace nav2_teb_controller::graph_algo
