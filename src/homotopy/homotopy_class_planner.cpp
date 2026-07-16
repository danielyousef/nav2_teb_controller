#include "nav2_teb_controller/homotopy/homotopy_class_planner.hpp"

#include <algorithm>
#include <limits>
#include <rclcpp/rclcpp.hpp>

namespace nav2_teb_controller {

HomotopyClassPlanner::HomotopyClassPlanner(const teb_controller::Params &params,
                                           const Footprint &footprint,
                                           nav2_costmap_2d::Costmap2DROS *costmap_ros)
    : params_(params), footprint_(footprint), costmap_ros_(costmap_ros) {
  RCLCPP_INFO(rclcpp::get_logger("HomotopyClassPlanner"),
              "HomotopyClassPlanner constructed.");
}

bool HomotopyClassPlanner::plan(const nav_msgs::msg::Path &global_plan,
                                 const geometry_msgs::msg::Twist &start_vel) {
  if (!graph_search_ || !base_planner_) {
    RCLCPP_ERROR(rclcpp::get_logger("HomotopyClassPlanner"),
                 "GraphSearch or BasePlanner not set!");
    return false;
  }

  if (global_plan.poses.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("HomotopyClassPlanner"), "Empty global plan");
    return false;
  }

  // Convert start/goal from the global plan
  PoseSE2 start(global_plan.poses.front().pose);
  PoseSE2 goal(global_plan.poses.back().pose);

  if (!obstacles_) {
    RCLCPP_WARN(rclcpp::get_logger("HomotopyClassPlanner"),
                "No obstacle data available. Falling back to base planner.");
    bool success = base_planner_->plan(global_plan, start_vel);
    if (success)
      storeFallbackCandidate();
    return success;
  }

  // --- Step 1: Find homotopy-distinct paths via graph search ---
  std::vector<GraphSearchResult> classes;
  int max_classes = std::max<int>(1, static_cast<int>(params_.FollowPath.hcp.max_classes));
  bool found = graph_search_->search(start, goal, *obstacles_, max_classes, classes);

  if (!found || classes.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("HomotopyClassPlanner"),
                "No homotopy classes found. Falling back to base planner.");
    bool success = base_planner_->plan(global_plan, start_vel);
    if (success)
      storeFallbackCandidate();
    return success;
  }

  RCLCPP_DEBUG(rclcpp::get_logger("HomotopyClassPlanner"),
               "Found %zu homotopy classes. Optimizing candidates...", classes.size());

  // --- Step 2: Optimize each candidate TEB ---
  optimizeCandidates(classes, start_vel);

  // --- Step 3: Select best feasible candidate ---
  best_candidate_idx_ = selectBestCandidate();

  if (best_candidate_idx_ < 0) {
    RCLCPP_WARN(rclcpp::get_logger("HomotopyClassPlanner"),
                "No feasible candidate found. Falling back to base planner.");
    bool success = base_planner_->plan(global_plan, start_vel);
    if (success)
      storeFallbackCandidate();
    return success;
  }

  RCLCPP_DEBUG(rclcpp::get_logger("HomotopyClassPlanner"),
               "Selected candidate %d with cost %.3f",
               best_candidate_idx_,
               candidates_[best_candidate_idx_]->optimization_cost);

  // Save best candidate for next cycle's warm-start
  prev_best_candidate_ = candidates_[best_candidate_idx_];

  return true;
}

bool HomotopyClassPlanner::hasDiverged() {
  if (candidates_.empty() || best_candidate_idx_ < 0)
    return false;

  auto &best = candidates_[best_candidate_idx_];
  if (best->has_diverged)
    return true;

  if (base_planner_)
    return base_planner_->hasDiverged();

  return best->has_diverged;
}

void HomotopyClassPlanner::clear() {
  candidates_.clear();
  best_candidate_idx_ = -1;
  prev_best_candidate_.reset();
  if (base_planner_)
    base_planner_->clear();
}

const TimedElasticBand &HomotopyClassPlanner::getTEB() const {
  return getBestCandidate().teb;
}

double HomotopyClassPlanner::getCost() const {
  if (candidates_.empty() || best_candidate_idx_ < 0)
    return std::numeric_limits<double>::infinity();
  return candidates_[best_candidate_idx_]->optimization_cost;
}

void HomotopyClassPlanner::setFeedback(
    const ackermann_msgs::msg::AckermannDrive &feedback) {
  if (base_planner_)
    base_planner_->setFeedback(feedback);
}

void HomotopyClassPlanner::updateObstacleContainer(
    costmap_converter_msgs::msg::ObstacleArrayMsg::ConstSharedPtr obstacle_array) {
  obstacles_ = obstacle_array;
  if (base_planner_)
    base_planner_->updateObstacleContainer(obstacle_array);
  if (graph_search_)
    graph_search_->updateObstacles(*obstacle_array);
}

void HomotopyClassPlanner::setObstacleMap(const ObstacleMap2D *esdf) {
  if (base_planner_)
    base_planner_->setObstacleMap(esdf);
  if (graph_search_)
    graph_search_->setObstacleMap(esdf);
}

void HomotopyClassPlanner::setFixedGoal(bool fix) {
  if (base_planner_)
    base_planner_->setFixedGoal(fix);
}

void HomotopyClassPlanner::setGraphSearch(
    std::shared_ptr<GraphSearchInterface> graph_search) {
  graph_search_ = std::move(graph_search);
}

void HomotopyClassPlanner::setBasePlanner(
    std::shared_ptr<PlannerInterface<TimedElasticBand>> base_planner) {
  base_planner_ = std::move(base_planner);
}

const TebCandidate &HomotopyClassPlanner::getBestCandidate() const {
  if (candidates_.empty() || best_candidate_idx_ < 0)
    throw std::runtime_error("HCP: No valid candidate!");
  return *candidates_[best_candidate_idx_];
}

bool HomotopyClassPlanner::findHomotopyClasses(
    const PoseSE2 &start, const PoseSE2 &goal,
    std::vector<GraphSearchResult> &classes) {
  if (!graph_search_ || !obstacles_)
    return false;

  int max_classes = std::max<int>(1, static_cast<int>(params_.FollowPath.hcp.max_classes));
  return graph_search_->search(start, goal, *obstacles_, max_classes, classes);
}

void HomotopyClassPlanner::optimizeCandidates(
    const std::vector<GraphSearchResult> &classes,
    const geometry_msgs::msg::Twist &start_vel) {
  updateCandidates(classes);

  for (auto &candidate : candidates_) {
    if (candidate->teb.sizePoses() < 2) {
      candidate->is_feasible = false;
      candidate->optimization_cost = std::numeric_limits<double>::infinity();
      continue;
    }

    if (!base_planner_) {
      candidate->optimization_cost = 0.0;
      candidate->is_feasible = true;
      continue;
    }

    // Build a nav_msgs::Path from the candidate poses
    nav_msgs::msg::Path path;
    path.poses.reserve(candidate->teb.sizePoses());
    for (std::size_t j = 0; j < candidate->teb.sizePoses(); ++j) {
      geometry_msgs::msg::PoseStamped ps;
      ps.pose = candidate->teb.pose(j).toPoseMsg();
      path.poses.push_back(std::move(ps));
    }

    // Warm-start: if this candidate's H-signature matches the previous best,
    // don't clear the base planner so plan() can warm-start from the existing TEB
    bool warm_start = matchesPreviousBest(*candidate);

    if (!warm_start) {
      base_planner_->clear();
    }
    base_planner_->setFixedGoal(true);
    bool success = base_planner_->plan(path, start_vel);
    bool diverged = base_planner_->hasDiverged();

    if (!success || diverged) {
      candidate->is_feasible = false;
      candidate->has_diverged = diverged;
      candidate->optimization_cost = std::numeric_limits<double>::infinity();
      RCLCPP_DEBUG(rclcpp::get_logger("HomotopyClassPlanner"),
                   "Candidate optimization failed (success=%d diverged=%d)", success, diverged);
    } else {
      const auto &new_teb = base_planner_->getTEB();
      candidate->teb = new_teb;
      candidate->is_feasible = true;
      candidate->optimization_cost = base_planner_->getCost();
      RCLCPP_DEBUG(rclcpp::get_logger("HomotopyClassPlanner"),
                   "Candidate optimized cost=%.3f poses=%zu",
                   candidate->optimization_cost, candidate->teb.sizePoses());
    }
  }

  pruneCandidates();
}

int HomotopyClassPlanner::selectBestCandidate() const {
  if (candidates_.empty())
    return -1;

  int best_idx = -1;
  double best_cost = std::numeric_limits<double>::infinity();

  for (size_t i = 0; i < candidates_.size(); ++i) {
    const auto &candidate = candidates_[i];
    if (!candidate->is_feasible)
      continue;

    if (candidate->optimization_cost < best_cost) {
      best_cost = candidate->optimization_cost;
      best_idx = static_cast<int>(i);
    }
  }

  return best_idx;
}

void HomotopyClassPlanner::updateCandidates(
    const std::vector<GraphSearchResult> &classes) {
  candidates_.clear();
  candidates_.reserve(classes.size());

  for (const auto &cls : classes) {
    auto candidate = std::make_shared<TebCandidate>();
    candidate->h_signature = cls.h_signature;

    // Initialize TEB from the path returned by graph search
    candidate->teb = TimedElasticBand();
    for (const auto &pose : cls.path) {
      candidate->teb.addPose(pose);
    }

    // Initialize time diffs with a default value
    double dt_default = 0.3; // seconds
    for (size_t i = 0; i + 1 < cls.path.size(); ++i) {
      candidate->teb.addTimeDiff(dt_default);
    }

    candidate->optimization_cost = cls.cost;
    candidate->is_feasible = true;
    candidate->has_diverged = false;

    candidates_.push_back(std::move(candidate));
  }
}

void HomotopyClassPlanner::pruneCandidates() {
  auto it = std::remove_if(candidates_.begin(), candidates_.end(),
      [](const TebCandidate::Ptr &c) {
        return c->has_diverged || !c->is_feasible ||
               std::isinf(c->optimization_cost);
      });
  candidates_.erase(it, candidates_.end());
}

void HomotopyClassPlanner::storeFallbackCandidate() {
  auto candidate = std::make_shared<TebCandidate>();
  candidate->teb = base_planner_->getTEB();
  candidate->is_feasible = true;
  candidate->optimization_cost = base_planner_->getCost();
  candidate->has_diverged = false;

  candidates_.clear();
  candidates_.push_back(candidate);
  best_candidate_idx_ = 0;
  prev_best_candidate_ = candidate;
}

bool HomotopyClassPlanner::matchesPreviousBest(const TebCandidate &candidate) const {
  if (!prev_best_candidate_)
    return false;
  return candidate.h_signature.isEqual(prev_best_candidate_->h_signature, 1e-3);
}

}  // namespace nav2_teb_controller
