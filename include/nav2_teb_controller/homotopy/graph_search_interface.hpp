#pragma once
#include <costmap_converter_msgs/msg/obstacle_array_msg.hpp>
#include <memory>
#include <vector>

#include "nav2_teb_controller/core/pose_se2.hpp"
#include "nav2_teb_controller/homotopy/h_signature.hpp"
#include "nav2_teb_controller/obstacles/esdf.hpp"

namespace nav2_teb_controller {

/** @brief One HCP candidate path from graph search */
struct GraphSearchResult {
  std::vector<PoseSE2> path;  // Initial path for TEB warm-start
  HSignature h_signature;     // Homotopy class identifier
  double cost = 0.0;          // Estimated path cost
};

/**
 * @brief Abstract interface for HCP graph search
 *
 * Implementations: VisibilityGraphSearch, ProAStarSearch (future)
 */
class GraphSearchInterface {
public:
  using ObstacleArray = costmap_converter_msgs::msg::ObstacleArrayMsg;

  virtual ~GraphSearchInterface() = default;

  /**
   * @brief Find multiple homotopy-distinct paths from start to goal
   * @param start Start pose
   * @param goal Goal pose
   * @param obstacles Current obstacle array
   * @param max_classes Maximum number of homotopy classes to return
   * @param results Output: homotopy-distinct paths
   * @return true if at least one path found
   */
  virtual bool search(const PoseSE2 &start, const PoseSE2 &goal, const ObstacleArray &obstacles,
                      int max_classes, std::vector<GraphSearchResult> &results) = 0;

  /**
   * @brief Update obstacle data (e.g. dynamic obstacles)
   */
  virtual void updateObstacles(const ObstacleArray &obstacles) = 0;

  /**
   * @brief Set the ESDF for collision-free visibility checks
   */
  virtual void setObstacleMap(const ObstacleMap2D *esdf) = 0;
};

}  // namespace nav2_teb_controller
