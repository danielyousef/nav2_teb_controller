#pragma once

#include "nav2_teb_controller/homotopy/graph_search_interface.hpp"
#include "nav2_teb_controller/homotopy/h_signature.hpp"
#include "nav2_teb_controller/homotopy/voronoi_graph.hpp"
#include "nav2_teb_controller/obstacles/esdf.hpp"

namespace nav2_teb_controller {

/**
 * @brief HCP path search on the reduced Generalized Voronoi Diagram of the ESDF.
 *
 * Finds homotopy-distinct paths by:
 * 1) Extracting the reduced GVD graph (max-clearance corridors) from the ESDF
 * 2) Dijkstra shortest path (trivial class)
 * 3) Yen's K-Shortest Paths with a growing-K loop until enough DISTINCT H-signature
 *    classes are found (distinct node paths are not necessarily distinct classes)
 *
 * All returned paths keep at least `min_clearance` from obstacles by construction.
 */
class VoronoiGraphSearch : public GraphSearchInterface {
public:
  /// @param min_clearance Minimum obstacle clearance for GVD cells and connectors [m].
  ///        Should cover the robot footprint (e.g. circumradius).
  explicit VoronoiGraphSearch(double min_clearance = 0.15) : min_clearance_(min_clearance) {}

  bool search(const PoseSE2 &start, const PoseSE2 &goal, const ObstacleArray &obstacles,
              int max_classes, std::vector<GraphSearchResult> &results) override;

  /// Grid data comes from the ESDF; obstacle-array updates are not needed.
  void updateObstacles(const ObstacleArray & /*obstacles*/) override {}

  void setObstacleMap(const ObstacleMap2D *esdf) override { esdf_ = esdf; }

  [[nodiscard]] const VoronoiGraph &getVoronoiGraph() const { return graph_; }

private:
  double min_clearance_;
  const ObstacleMap2D *esdf_{nullptr};
  VoronoiGraph graph_;
};

}  // namespace nav2_teb_controller
