// include/teb_k/obstacle_map_2d.hpp
#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

// Thin ROS2 include — nur für den Wrapper, nicht für den Core-Algorithmus
#include <nav2_costmap_2d/costmap_2d.hpp>

namespace nav2_teb_controller {

/// @brief 2D Euclidean Distance Transform (EDT) computed from a binary
///        occupancy grid. Provides metrically correct obstacle distances
///        and C¹-continuous gradients for gradient-based optimizers (g2o).
///
/// ## Usage
/// @code
/// ObstacleMap2D esdf;
/// esdf.update(*costmap_ros->getCostmap());
///
/// auto [dist, grad] = esdf.query(robot_x, robot_y);
/// // dist [m]: distance to nearest obstacle surface
/// // grad:     unit vector pointing AWAY from nearest obstacle
/// @endcode
///
/// ## Thread Safety
/// update() and query() must not be called concurrently.
/// query() is safe to call concurrently from multiple threads
/// (read-only after update).
class ObstacleMap2D {
public:
  // ──────────────────────────────────────────────
  // Types
  // ──────────────────────────────────────────────

  struct QueryResult {
    double distance;           ///< Euclidean distance to nearest obstacle [m]
    Eigen::Vector2d gradient;  ///< Unit vector pointing away from obstacle
                               ///< (satisfies |gradient| == 1 everywhere)
  };

  // ──────────────────────────────────────────────
  // Core API (ROS2-agnostic)
  // ──────────────────────────────────────────────

  /// @brief Recompute the EDT from a raw occupancy grid.
  ///
  /// @param data     Row-major uint8 array; cell (x,y) at data[y*nx + x]
  /// @param nx       Grid width  (cells)
  /// @param ny       Grid height (cells)
  /// @param res      Cell size   [m/cell]
  /// @param origin_x World X of cell (0,0) center [m]
  /// @param origin_y World Y of cell (0,0) center [m]
  /// @param lethal_threshold  Cells >= this value are treated as obstacle
  void update(const uint8_t *data, unsigned nx, unsigned ny, double res, double origin_x,
              double origin_y, uint8_t lethal_threshold = 253);

  /// @brief Convenience wrapper for nav2_costmap_2d::Costmap2D.
  ///        Internally calls update(data, nx, ny, res, ox, oy).
  void update(const nav2_costmap_2d::Costmap2D &costmap);

  /// @brief Query distance and gradient at world coordinates (wx, wy).
  ///
  /// Uses bilinear interpolation → C¹-continuous result.
  /// Clamps to grid bounds; returns large distance for out-of-bounds queries.
  ///
  /// @param wx  World X coordinate [m]
  /// @param wy  World Y coordinate [m]
  /// @return    {distance [m], unit gradient vector}
  [[nodiscard]] QueryResult query(double wx, double wy) const;

  /// @brief True derivative of the interpolated distance w.r.t. the query
  /// position (unnormalized gradient of the bilinear interpolation).
  ///
  /// Required by gradient-based edges (the unit gradient returned by query()
  /// is only correct up to the scale of the local distance gradient).
  [[nodiscard]] Eigen::Vector2d gradient(double wx, double wy) const;

  /// @brief Check whether the map has been initialized.
  [[nodiscard]] bool isInitialized() const { return nx_ > 0 && ny_ > 0; }

  // ──────────────────────────────────────────────
  // Accessors (useful for visualization / Layer export)
  // ──────────────────────────────────────────────

  [[nodiscard]] unsigned sizeX() const { return nx_; }
  [[nodiscard]] unsigned sizeY() const { return ny_; }
  [[nodiscard]] double resolution() const { return res_; }
  [[nodiscard]] double originX() const { return origin_x_; }
  [[nodiscard]] double originY() const { return origin_y_; }

  /// @brief Raw EDT grid (float, meters), row-major. Useful for publishing.
  [[nodiscard]] const std::vector<float> &rawGrid() const { return edt_; }

private:
  // ──────────────────────────────────────────────
  // EDT computation (Meijster algorithm, O(n))
  // ──────────────────────────────────────────────

  /// Phase 1: 1D column scan → squared distances in Y direction
  void phase1_columnScan(const std::vector<bool> &occ);

  /// Phase 2: 1D parabola envelope per row → final 2D squared distances
  void phase2_rowScan();

  // ──────────────────────────────────────────────
  // Internal helpers
  // ──────────────────────────────────────────────

  [[nodiscard]] size_t idx(unsigned x, unsigned y) const {
    return static_cast<size_t>(y) * nx_ + x;
  }

  [[nodiscard]] bool inBounds(int x, int y) const {
    return x >= 0 && y >= 0 && static_cast<unsigned>(x) < nx_ && static_cast<unsigned>(y) < ny_;
  }

  // ──────────────────────────────────────────────
  // State
  // ──────────────────────────────────────────────

  std::vector<float> edt_;  ///< EDT in meters, row-major [ny × nx]
  std::vector<double> g_;   ///< Intermediate column-scan buffer (squared cells)

  unsigned nx_{0}, ny_{0};
  double res_{0.05};
  double origin_x_{0.0}, origin_y_{0.0};

  static constexpr double kOutOfBoundsDist = 1e6;
};

}  // namespace nav2_teb_controller
