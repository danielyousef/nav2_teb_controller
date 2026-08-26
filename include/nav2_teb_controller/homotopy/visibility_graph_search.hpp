#pragma once

#include "nav2_teb_controller/homotopy/graph_search_interface.hpp"
#include "nav2_teb_controller/homotopy/h_signature.hpp"
#include "nav2_teb_controller/homotopy/visibility_graph.hpp"
#include "nav2_teb_controller/obstacles/esdf.hpp"

namespace nav2_teb_controller {

/**
 * @brief HCP path search using Visibility Graph (alternative to VoronoiGraphSearch)
 *
 * Finds homotopy-distinct paths by:
 * 1) Building visibility graph from clearance-offset obstacle keypoints
 * 2) Dijkstra shortest path (trivial class)
 * 3) Yen's K-Shortest Paths with a growing-K loop until enough DISTINCT H-signature
 *    classes are found
 *
 * Kept as a fallback/experimental search — the Voronoi-based search is the default.
 */
class VisibilityGraphSearch : public GraphSearchInterface {
public:
  /// @param min_clearance Required obstacle clearance for edges and keypoints [m]
  explicit VisibilityGraphSearch(double min_clearance = 0.2) : min_clearance_(min_clearance) {}

  bool search(const PoseSE2 &start, const PoseSE2 &goal, const ObstacleArray &obstacles,
              int max_classes, std::vector<GraphSearchResult> &results) override;

  void updateObstacles(const ObstacleArray & /*obstacles*/) override {}

  void setObstacleMap(const ObstacleMap2D *esdf) override { esdf_ = esdf; }

  [[nodiscard]] const VisibilityGraph &getVisibilityGraph() const { return vis_graph_; }

private:
  double min_clearance_;
  const ObstacleMap2D *esdf_{nullptr};
  VisibilityGraph vis_graph_;
};

}  // namespace nav2_teb_controller
