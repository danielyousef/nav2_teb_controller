#pragma once
#include "nav2_teb_controller/homotopy/graph_search_interface.hpp"
#include "nav2_teb_controller/homotopy/h_signature.hpp"
#include "nav2_teb_controller/homotopy/visibility_graph.hpp"
#include "nav2_teb_controller/obstacles/esdf.hpp"

namespace nav2_teb_controller {

/**
 * @brief HCP path search using Visibility Graph + Dijkstra
 *
 * Finds homotopy-distinct paths by:
 * 1) Building visibility graph from obstacle keypoints
 * 2) Dijkstra shortest path (trivial class)
 * 3) Yen's K-Shortest Paths (alternative classes)
 * 4) H-Signature filter (remove duplicates)
 *
 * Visibility checks use the ESDF layer instead of polygon intersection + inflation.
 */
class VisibilityGraphSearch : public GraphSearchInterface {
public:
  VisibilityGraphSearch() = default;

  bool search(const PoseSE2 &start, const PoseSE2 &goal, const ObstacleArray &obstacles,
              int max_classes, std::vector<GraphSearchResult> &results) override;

  void updateObstacles(const ObstacleArray &obstacles) override;

  void setObstacleMap(const ObstacleMap2D *esdf) override { esdf_ = esdf; }

  [[nodiscard]] const VisibilityGraph &getVisibilityGraph() const { return vis_graph_; }

private:
  /**
   * @brief Dijkstra on visibility graph
   * @return Node-ID path from start to goal, empty if no path
   */
  [[nodiscard]] static std::vector<int> dijkstra(const VisibilityGraph &graph, int start_id, int goal_id) ;

  /**
   * @brief Yen's K-Shortest Paths for alternative homotopy classes
   */
  [[nodiscard]] static std::vector<std::vector<int>> yenKShortestPaths(const VisibilityGraph &graph, int start_id,
                                                  int goal_id, int k) ;

  /**
   * @brief Convert node-ID path to PoseSE2 path
   */
  [[nodiscard]] static std::vector<PoseSE2> nodePathToPoses(const VisibilityGraph &graph,
                                       const std::vector<int> &node_path) ;

  /**
   * @brief Remove paths with duplicate H-signatures
   */
  static void filterDuplicateClasses(std::vector<GraphSearchResult> &results,
                              double h_sig_tolerance = 1e-3) ;

  const ObstacleMap2D *esdf_{nullptr};
  VisibilityGraph vis_graph_;
};

}  // namespace nav2_teb_controller
