#include "nav2_teb_controller/homotopy/voronoi_graph_search.hpp"

#include <rclcpp/rclcpp.hpp>

#include "nav2_teb_controller/homotopy/graph_algorithms.hpp"
#include "nav2_teb_controller/teb_profiler.hpp"

namespace nav2_teb_controller {

bool VoronoiGraphSearch::search(const PoseSE2 &start, const PoseSE2 &goal,
                                const ObstacleArray &obstacles, int max_classes,
                                std::vector<GraphSearchResult> &results) {
  results.clear();

  if (!esdf_ || !esdf_->isInitialized()) {
    RCLCPP_WARN(rclcpp::get_logger("VoronoiGraphSearch"), "No ESDF available");
    return false;
  }

  {
    PROFILE_BLOCK("gvd_build");
    graph_.build(*esdf_, start.position(), goal.position(), min_clearance_);
  }
  if (graph_.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("VoronoiGraphSearch"), "Empty Voronoi graph");
    return false;
  }

  // Growing-K loop: Yen returns distinct NODE paths, not distinct homotopy classes — keep
  // widening the search until enough distinct H-signature classes exist or paths run out.
  results.clear();
  int k = std::max(2 * max_classes, 4);
  // Yen is the dominant cost on dense GVDs — keep the path budget tight. We only ever
  // need max_classes DISTINCT signatures; anything beyond that is wasted Dijkstra work
  // (benchmark_test_18: refresh ticks cost 120-550 ms in rack-dense regions).
  const int k_max = std::max(8, 4 * max_classes);
  for (int attempt = 0; attempt < 4; ++attempt) {
    PROFILE_BLOCK("yen_ksp");
    const auto node_paths =
        graph_algo::kShortestPaths(graph_, graph_.startId(), graph_.goalId(), k);

    results.clear();
    for (const auto &node_path : node_paths) {
      GraphSearchResult result;
      result.path = graph_algo::pathToPoses(graph_, node_path);

      result.h_signature.compute(result.path, obstacles);

      double cost = 0.0;
      for (size_t i = 1; i < result.path.size(); ++i)
        cost += (result.path[i].position() - result.path[i - 1].position()).norm();
      result.cost = cost;

      results.push_back(std::move(result));
    }
    {
      PROFILE_BLOCK("convert_dedup");
      graph_algo::filterDuplicateClasses(results);
    }

    if (static_cast<int>(results.size()) >= max_classes || static_cast<int>(node_paths.size()) < k)
      break;  // enough classes found, or the graph has no more loopless paths
    k = std::min(k * 2, k_max);
  }

  if (static_cast<int>(results.size()) > max_classes)
    results.resize(max_classes);  // keep the cheapest classes (Yen emits cost-ordered)

  RCLCPP_DEBUG(rclcpp::get_logger("VoronoiGraphSearch"), "Found %zu distinct homotopy classes",
               results.size());

  return !results.empty();
}

}  // namespace nav2_teb_controller
