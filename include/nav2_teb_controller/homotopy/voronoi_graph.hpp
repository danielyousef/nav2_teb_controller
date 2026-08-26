#pragma once

#include <Eigen/Core>
#include <vector>

#include "nav2_teb_controller/obstacles/esdf.hpp"

namespace nav2_teb_controller {

/** @brief Single node in the reduced Voronoi (GVD) graph */
struct VoronoiNode {
  Eigen::Vector2d pos;
  int id = -1;
  bool is_start = false;
  bool is_goal = false;
};

/** @brief Edge between two connected GVD nodes (cost = chain arc length) */
struct VoronoiEdge {
  int from_id;
  int to_id;
  double cost;
};

/**
 * @brief Reduced Generalized Voronoi Diagram graph extracted from the ESDF.
 *
 * Build pipeline:
 *  1. Ridge extraction: cells with `distance >= min_clearance` that are local maxima across at
 *     least one axis-aligned neighbor pair (medial-axis approximation).
 *  2. Spur pruning (bounded iterations) to remove skeleton noise.
 *  3. Reduction to junction (degree >= 3) / endpoint (degree 1) nodes; degree-2 chains become
 *     weighted edges, subdivided with Steiner nodes so no edge exceeds ~max_edge_len.
 *  4. Closed loops without junctions (isolated blobs) are split into two half-perimeter arcs so
 *     both directions around the blob remain representable.
 *  5. Start/goal connector edges via clearance-checked straight segments.
 *
 * All paths on this graph keep at least `min_clearance` distance to the nearest obstacle by
 * construction — the homotopy classes follow the corridor topology of the environment.
 */
class VoronoiGraph {
public:
  /**
   * @brief Build the reduced GVD graph from the ESDF.
   *
   * @param esdf          Distance field (must be initialized)
   * @param start         Start position [m]
   * @param goal          Goal position [m]
   * @param min_clearance Minimum obstacle clearance for GVD cells and connectors [m]
   */
  void build(const ObstacleMap2D &esdf, const Eigen::Vector2d &start, const Eigen::Vector2d &goal,
             double min_clearance);

  [[nodiscard]] const std::vector<VoronoiNode> &nodes() const { return nodes_; }
  [[nodiscard]] const std::vector<VoronoiEdge> &edges(int node_id) const { return adj_[node_id]; }
  [[nodiscard]] int startId() const { return start_id_; }
  [[nodiscard]] int goalId() const { return goal_id_; }
  [[nodiscard]] bool empty() const { return nodes_.empty(); }

private:
  /// True iff the straight segment stays at least @p clearance away from obstacles.
  [[nodiscard]] bool segmentClear(const Eigen::Vector2d &a, const Eigen::Vector2d &b,
                                  double clearance) const;

  /// Nearest graph node reachable from @p p via a clearance-checked straight segment.
  /// @return node id or -1
  [[nodiscard]] int nearestVisibleNode(const Eigen::Vector2d &p, double clearance) const;

  void addEdge(int u, int v, double cost);

  const ObstacleMap2D *esdf_{nullptr};
  double res_{0.05};

  std::vector<VoronoiNode> nodes_;
  std::vector<std::vector<VoronoiEdge>> adj_;
  int start_id_ = -1;
  int goal_id_ = -1;

  static constexpr double kMaxEdgeLen = 1.0;      ///< Steiner-subdivision target [m]
  static constexpr int kSpurPruneIterations = 3;  ///< Skeleton noise-trimming passes
};

}  // namespace nav2_teb_controller
