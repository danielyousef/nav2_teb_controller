#pragma once
#include <Eigen/Core>
#include <costmap_converter_msgs/msg/obstacle_array_msg.hpp>
#include <vector>

#include "nav2_teb_controller/obstacles/esdf.hpp"

namespace nav2_teb_controller {

/** @brief Single node in Visibility Graph */
struct VisibilityNode {
  Eigen::Vector2d pos;
  int id = -1;
  bool is_start = false;
  bool is_goal = false;
};

/** @brief Edge between two visible nodes */
struct VisibilityEdge {
  int from_id;
  int to_id;
  double cost;  // Euclidean distance
};

/**
 * @brief Visibility Graph data structure
 *
 * Nodes: start, goal, obstacle keypoints
 * Edges: direct line-of-sight between nodes (checked via ESDF)
 */
class VisibilityGraph {
public:
  using ObstacleArray = costmap_converter_msgs::msg::ObstacleArrayMsg;

  /**
   * @brief Build visibility graph from obstacles + start + goal
   * @param start Start position
   * @param goal Goal position
   * @param obstacles Current obstacle array
   */
  void build(const Eigen::Vector2d &start, const Eigen::Vector2d &goal,
             const ObstacleArray &obstacles);

  /** @brief Set the ESDF for collision-free visibility checks */
  void setObstacleMap(const ObstacleMap2D *esdf) { esdf_ = esdf; }

  /** @brief Get all nodes */
  [[nodiscard]] const std::vector<VisibilityNode> &nodes() const { return nodes_; }

  /** @brief Get adjacency list for a node */
  [[nodiscard]] const std::vector<VisibilityEdge> &edges(int node_id) const { return adj_[node_id]; }

  [[nodiscard]] int startId() const { return start_id_; }
  [[nodiscard]] int goalId() const { return goal_id_; }
  [[nodiscard]] bool empty() const { return nodes_.empty(); }

  [[nodiscard]] const ObstacleMap2D *obstacleMap() const { return esdf_; }

private:
  /**
   * @brief Check if line p1→p2 is free using ESDF sampling
   */
  [[nodiscard]] bool isVisible(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2) const;

  /**
   * @brief Extract raw polygon vertices (no inflation needed with ESDF)
   */
  [[nodiscard]] std::vector<Eigen::Vector2d> extractKeypoints(
      const costmap_converter_msgs::msg::ObstacleMsg &obs) const;

  const ObstacleMap2D *esdf_{nullptr};
  std::vector<VisibilityNode> nodes_;
  std::vector<std::vector<VisibilityEdge>> adj_;
  int start_id_ = -1;
  int goal_id_ = -1;
};

}  // namespace nav2_teb_controller
