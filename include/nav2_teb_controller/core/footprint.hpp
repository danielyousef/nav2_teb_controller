// include/teb_k/footprint.hpp
#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <limits>
#include <nav2_teb_controller/obstacles/esdf.hpp>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace nav2_teb_controller {

struct FootprintCircle {
  Eigen::Vector2d offset;  ///< Center in robot body frame [m]
  double radius;           ///< [m]
};

struct FootprintCheckResult {
  double min_clearance;  ///< Minimum dist to obstacle across all circles [m]
  int worst_circle;      ///< Index of circle with min clearance
  bool is_colliding;     ///< min_clearance < d_min
};

class Footprint {
public:
  enum class Model { Circles, Polygon };

  // ── Construction ──────────────────────────────────────────────────────────

  /// @brief Build from already-declared generate_parameter_library values.
  ///
  /// @param use_local_costmap  Use nav2 costmap footprint instead
  /// @param model              "circles" | "polygon"
  /// @param points             Encoded string or flat values — see parsePoints()
  Footprint(bool use_local_costmap, const std::string &model, const std::string &points)
      : use_local_costmap_(use_local_costmap) {
    if (model == "circles")
      model_ = Model::Circles;
    else if (model == "polygon")
      model_ = Model::Polygon;
    else
      throw std::invalid_argument("[Footprint] Unknown model '" + model +
                                  "'. Use 'circles' or 'polygon'.");

    parsePoints(points);
  }
  // Default
  Footprint() = default;

  // ── Check ─────────────────────────────────────────────────────────────────

  /// @brief Check obstacle clearance for a given robot pose.
  ///
  /// Transforms each footprint circle into world frame, queries the ESDF,
  /// and returns the minimum effective clearance across all circles:
  ///   clearance_k = esdf(center_k_world) − circle_radius_k
  ///
  /// @param x       Robot pose world-x [m]
  /// @param y       Robot pose world-y [m]
  /// @param theta   Robot heading [rad]
  /// @param esdf    Precomputed ESDF (updated externally, not owned here)
  /// @param d_min   Collision threshold [m]
  [[nodiscard]] FootprintCheckResult check(double x, double y, double theta,
                                           const ObstacleMap2D &esdf, double d_min = 0.0) const {
    const double ct = std::cos(theta), st = std::sin(theta);

    FootprintCheckResult result{};
    result.min_clearance = std::numeric_limits<double>::max();
    result.worst_circle = -1;
    result.is_colliding = false;

    for (int k = 0; k < static_cast<int>(circles_.size()); ++k) {
      const auto &c = circles_[k];

      // Transform circle center to world frame
      const double wx = x + ct * c.offset.x() - st * c.offset.y();
      const double wy = y + st * c.offset.x() + ct * c.offset.y();

      const double dist_to_obs = esdf.queryDistance(wx, wy);
      const double clearance = dist_to_obs - c.radius;

      if (clearance < result.min_clearance) {
        result.min_clearance = clearance;
        result.worst_circle = k;
      }
    }

    result.is_colliding = (result.min_clearance < d_min);
    return result;
  }

  // ── Accessors ─────────────────────────────────────────────────────────────

  [[nodiscard]] const std::vector<FootprintCircle> &circles() const { return circles_; }
  [[nodiscard]] Model model() const { return model_; }
  [[nodiscard]] bool useCostmap() const { return use_local_costmap_; }
  [[nodiscard]] size_t size() const { return circles_.size(); }

  [[nodiscard]] double circumscribedRadius() const {
    double r = 0.0;
    for (const auto &c : circles_)
      r = std::max(r, c.offset.norm() + c.radius);
    return r;
  }

  [[nodiscard]] std::string toString() const {
    std::ostringstream ss;
    ss << "Footprint{model=" << (model_ == Model::Circles ? "circles" : "polygon")
       << ", n=" << circles_.size() << ", r_circ=" << circumscribedRadius() << "m}";
    return ss.str();
  }

private:
  // ── Point string parser ───────────────────────────────────────────────────
  //
  // Supports two formats:
  //
  //   circles — groups of 3: "[ [cx, cy, r], [cx, cy, r], ... ]"
  //   polygon — groups of 2: "[ [x, y], [x, y], ... ]"
  //
  // Just strips all non-numeric characters except [.,\-] and reads flat.

  void parsePoints(const std::string &raw) {
    // Replace brackets, commas → spaces, then read as flat double stream
    std::string cleaned = raw;
    for (char &c : cleaned)
      if (c == '[' || c == ']' || c == ',')
        c = ' ';

    std::istringstream ss(cleaned);
    std::vector<double> vals;
    double v;
    while (ss >> v)
      vals.push_back(v);

    if (model_ == Model::Circles) {
      if (vals.size() % 3 != 0)
        throw std::invalid_argument("[Footprint] model=circles: need N×3 values (cx,cy,r), got " +
                                    std::to_string(vals.size()));

      circles_.reserve(vals.size() / 3);
      for (size_t i = 0; i + 2 < vals.size(); i += 3)
        circles_.push_back({Eigen::Vector2d(vals[i], vals[i + 1]), vals[i + 2]});

    } else {  // Polygon
      if (vals.size() % 2 != 0)
        throw std::invalid_argument("[Footprint] model=polygon: need N×2 values (x,y), got " +
                                    std::to_string(vals.size()));

      // Store as zero-radius circles for uniform downstream handling
      circles_.reserve(vals.size() / 2);
      for (size_t i = 0; i + 1 < vals.size(); i += 2)
        circles_.push_back({Eigen::Vector2d(vals[i], vals[i + 1]), 0.0});
    }

    if (circles_.empty())
      throw std::invalid_argument("[Footprint] No points parsed from: " + raw);
  }

  Model model_;
  std::vector<FootprintCircle> circles_;
  bool use_local_costmap_{};
};

}  // namespace nav2_teb_controller