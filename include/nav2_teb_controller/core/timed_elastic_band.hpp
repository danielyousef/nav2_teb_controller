#pragma once

#include <Eigen/Core>
#include <cassert>
#include <cmath>
#include <numeric>
#include <optional>
#include <vector>

#include "nav2_teb_controller/core/pose_se2.hpp"

namespace nav2_teb_controller {

/**
 * @brief Pure data container for a Timed Elastic Band.
 *
 * Stores a sequence of poses (PoseSE2) and time differences (dt) between them.
 *
 * Invariant: timediffs_.size() == poses_.size() - 1
 */
class TimedElasticBand {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  TimedElasticBand() = default;
  ~TimedElasticBand() = default;

  // Copyable (needed for HCP candidate management)
  TimedElasticBand(const TimedElasticBand &) = default;
  TimedElasticBand &operator=(const TimedElasticBand &) = default;
  TimedElasticBand(TimedElasticBand &&) = default;
  TimedElasticBand &operator=(TimedElasticBand &&) = default;

  // ── Access Sequences ─────────────────────────────────────────────────────
  std::vector<PoseSE2> &poses() { return poses_; }
  [[nodiscard]] const std::vector<PoseSE2> &poses() const { return poses_; }

  std::vector<double> &timediffs() { return timediffs_; }
  [[nodiscard]] const std::vector<double> &timediffs() const { return timediffs_; }

  // ── Access Single Elements ────────────────────────────────────────────────
  PoseSE2 &pose(std::size_t i) { return poses_.at(i); }
  [[nodiscard]] const PoseSE2 &pose(std::size_t i) const { return poses_.at(i); }

  double &timeDiff(std::size_t i) { return timediffs_.at(i); }
  [[nodiscard]] const double &timeDiff(std::size_t i) const { return timediffs_.at(i); }

  PoseSE2 &backPose() { return poses_.back(); }
  [[nodiscard]] const PoseSE2 &backPose() const { return poses_.back(); }

  double &backTimeDiff() { return timediffs_.back(); }
  [[nodiscard]] const double &backTimeDiff() const { return timediffs_.back(); }

  // ── Size & Status ─────────────────────────────────────────────────────────
  [[nodiscard]] std::size_t sizePoses() const { return poses_.size(); }
  [[nodiscard]] std::size_t sizeTimeDiffs() const { return timediffs_.size(); }

  [[nodiscard]] bool isInit() const { return !poses_.empty() && (timediffs_.size() == poses_.size() - 1); }

  // ── Add / Insert / Delete ─────────────────────────────────────────────────
  void addPose(const PoseSE2 &pose) { poses_.push_back(pose); }
  void addPose(double x, double y, double theta) { poses_.emplace_back(x, y, theta); }
  void addPose(const Eigen::Vector2d &position, double theta) {
    poses_.emplace_back(position, theta);
  }

  void addTimeDiff(double dt) {
    assert(dt > 0.0 && "Adding a timediff requires a positive dt");
    timediffs_.push_back(dt);
  }

  // Add pose + associated dt in one call (maintains invariant)
  void addPoseAndTimeDiff(const PoseSE2 &pose, double dt) {
    assert(sizePoses() == sizeTimeDiffs() + 1 &&
           "Add first pose via addPose() before calling addPoseAndTimeDiff");
    addPose(pose);
    addTimeDiff(dt);
  }

  void insertPose(std::size_t index, const PoseSE2 &pose) {
    assert(index <= poses_.size());
    poses_.insert(poses_.begin() + index, pose);
  }

  void insertTimeDiff(std::size_t index, double dt) {
    assert(index <= timediffs_.size());
    timediffs_.insert(timediffs_.begin() + index, dt);
  }

  void deletePose(std::size_t index) {
    assert(index < poses_.size());
    poses_.erase(poses_.begin() + index);
  }

  void deletePoses(std::size_t index, std::size_t number) {
    assert(index + number <= poses_.size());
    poses_.erase(poses_.begin() + index, poses_.begin() + index + number);
  }

  void deleteTimeDiff(std::size_t index) {
    assert(index < timediffs_.size());
    timediffs_.erase(timediffs_.begin() + index);
  }

  void deleteTimeDiffs(std::size_t index, std::size_t number) {
    assert(index + number <= timediffs_.size());
    timediffs_.erase(timediffs_.begin() + index, timediffs_.begin() + index + number);
  }

  // ── Utility ───────────────────────────────────────────────────────────────
  [[nodiscard]] double sumTimeDiffs() const {
    return std::accumulate(timediffs_.begin(), timediffs_.end(), 0.0);
  }

  [[nodiscard]] double sumTimeDiffsUpToIdx(std::size_t i) const {
    return std::accumulate(timediffs_.begin(), timediffs_.begin() + i, 0.0);
  }

  [[nodiscard]] double accumulatedDistance() const {
    double dist = 0.0;
    for (std::size_t i = 1; i < poses_.size(); ++i) {
      dist += (poses_[i].position() - poses_[i - 1].position()).norm();
    }
    return dist;
  }

  void clear() {
    poses_.clear();
    timediffs_.clear();
  }

private:
  // Trajectory poses
  std::vector<PoseSE2> poses_;
  // Time diffs: timediffs_[i] = dt between pose i and i+1
  std::vector<double> timediffs_;
};

}  // namespace nav2_teb_controller
