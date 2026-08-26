#include "nav2_teb_controller/homotopy/homotopy_class_planner.hpp"

#include <algorithm>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <set>
#include <thread>

#include "nav2_teb_controller/core/teb_utils.hpp"

namespace nav2_teb_controller {

namespace {
/// Two polylines count as the same route when the GOAL endpoints agree (within
/// @p endpoint_dist) and the mean nearest-point distance is below @p mean_dist.
/// Deliberately NO start-endpoint check: the start is the moving robot, and during
/// slow control ticks it legitimately advances far between two searches.
bool polylinesMatch(const std::vector<PoseSE2> &a, const std::vector<PoseSE2> &b,
                    double endpoint_dist, double mean_dist) {
  if (a.size() < 2 || b.size() < 2)
    return false;
  if ((a.back().position() - b.back().position()).norm() > endpoint_dist)
    return false;

  // Shape comparison SKIPS both polylines' endpoint poses: HCP seeds every band's front
  // with the live robot pose and its back with the mission goal, so those points are
  // IDENTICAL across homotopy classes and drag the mean nearest-distance down by up to
  // half the corridor separation — distinct routes then falsely merge (observed as GVD
  // churn in benchmark_test_19: mean 0.976 < 1.0 for corridors 5 m apart!).
  const size_t i0 = (a.size() >= 3) ? 1u : 0u;
  const size_t i1 = (a.size() >= 3) ? a.size() - 1 : a.size();
  double sum = 0.0;
  size_t count = 0;
  for (size_t idx = i0; idx < i1; ++idx) {
    double best = std::numeric_limits<double>::infinity();
    for (const auto &pb : b) {
      const double d = (a[idx].position() - pb.position()).norm();
      if (d < best)
        best = d;
    }
    sum += best;
    ++count;
  }
  return count == 0 || sum / static_cast<double>(count) < mean_dist;
}
}  // namespace

HomotopyClassPlanner::HomotopyClassPlanner(const teb_controller::Params &params,
                                           const Footprint &footprint,
                                           nav2_costmap_2d::Costmap2DROS *costmap_ros)
    : params_(params), footprint_(footprint), costmap_ros_(costmap_ros) {
  RCLCPP_INFO(rclcpp::get_logger("HomotopyClassPlanner"), "HomotopyClassPlanner constructed.");
}

bool HomotopyClassPlanner::plan(const nav_msgs::msg::Path &global_plan,
                                const geometry_msgs::msg::Twist &start_vel) {
  ++tick_;

  // Record full-plan wall time on every exit path (feeds the adaptive refresh interval).
  struct PlanTimer {
    std::chrono::duration<double> &out;
    std::chrono::steady_clock::time_point t0;
    explicit PlanTimer(std::chrono::duration<double> &out_)
        : out(out_), t0(std::chrono::steady_clock::now()) {}
    ~PlanTimer() { out = std::chrono::steady_clock::now() - t0; }
  } plan_timer{last_plan_elapsed_};

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

  // --- Step 1: Find homotopy-distinct paths via graph search (rate-limited) ---
  std::vector<GraphSearchResult> classes;
  const bool found = refreshOrReuseClasses(start, goal, classes);

  if (!found || classes.empty()) {
    // Serve a fresh-enough previous selection instead of double-solving through the
    // base planner in the same tick (divergence cascades otherwise cold-start it).
    if (haveRecentCandidate())
      return true;

    warnThrottled("No homotopy classes found. Falling back to base planner.");
    bool success = base_planner_->plan(global_plan, start_vel);
    if (success) {
      storeFallbackCandidate();
      last_success_time_ = std::chrono::steady_clock::now();
    }
    return success;
  }

  RCLCPP_DEBUG(rclcpp::get_logger("HomotopyClassPlanner"),
               "Found %zu homotopy classes. Optimizing candidates...", classes.size());

  // --- Step 2: Optimize each candidate TEB ---
  // The controller overwrites the plan front with the live robot pose before calling us —
  // use it as the authoritative band start (position AND heading). The plan back carries
  // the mission goal pose (Smac hybrid emits the goal orientation) — equally authoritative.
  const PoseSE2 robot_pose(global_plan.poses.front().pose);
  const PoseSE2 goal_pose(global_plan.poses.back().pose);
  // Grace-period continuity FIRST: if the previous best route's class blinked out of this
  // search round, resurrect it from its RouteEntry so it competes with a fresh cost.
  synthesizePrevRouteCandidate(classes, robot_pose, goal_pose);
  optimizeCandidates(classes, start_vel, robot_pose, goal_pose);

  // --- Step 3: Select best feasible candidate ---
  best_candidate_idx_ = selectBestCandidate();

  if (best_candidate_idx_ < 0) {
    // Serve a fresh-enough previous selection instead of re-solving through the base
    // planner in the same tick.
    if (haveRecentCandidate())
      return true;

    warnThrottled("No feasible candidate found. Falling back to base planner.");
    bool success = base_planner_->plan(global_plan, start_vel);
    if (success) {
      storeFallbackCandidate();
      last_success_time_ = std::chrono::steady_clock::now();
    }
    return success;
  }

  RCLCPP_DEBUG(rclcpp::get_logger("HomotopyClassPlanner"),
               "Selected candidate %d with cost %.3f (eff %.3f)", best_candidate_idx_,
               candidates_[best_candidate_idx_]->optimization_cost,
               candidates_[best_candidate_idx_]->efficiency_cost);

  // Route continuity bookkeeping: hysteresis anchor + switch blocking + telemetry. The
  // anchor cost is the EFFICIENCY category — obstacle-proximity chi2 noise must not
  // gate route switches.
  const int chosen_route = candidates_[best_candidate_idx_]->route_id;
  const double chosen_cost = candidates_[best_candidate_idx_]->efficiency_cost;
  const double chosen_len = candidates_[best_candidate_idx_]->path_length;
  if (has_prev_best_ && chosen_route != prev_best_route_id_) {
    RCLCPP_INFO(rclcpp::get_logger("HomotopyClassPlanner"),
                "HCP: best route %d -> %d (eff %.3f -> %.3f, len %.2f -> %.2f)",
                prev_best_route_id_, chosen_route, prev_best_route_cost_, chosen_cost,
                prev_best_route_len_, chosen_len);
    last_switch_time_ = std::chrono::steady_clock::now();
  } else if (!has_prev_best_) {
    last_switch_time_ = std::chrono::steady_clock::now();  // first selection: start blocking
  }
  prev_best_route_id_ = chosen_route;
  prev_best_route_cost_ = chosen_cost;
  prev_best_route_len_ = chosen_len;
  last_prev_seen_time_ = std::chrono::steady_clock::now();
  has_prev_best_ = true;
  last_success_time_ = std::chrono::steady_clock::now();

  pruneRoutes();
  return true;
}

bool HomotopyClassPlanner::hasDiverged() {
  if (candidates_.empty() || best_candidate_idx_ < 0)
    return false;

  // Only the SELECTED candidate's divergence matters — the per-class planners of other
  // candidates are irrelevant for the emitted command.
  return candidates_[best_candidate_idx_]->has_diverged;
}

void HomotopyClassPlanner::clear() {
  candidates_.clear();
  best_candidate_idx_ = -1;
  cached_classes_.clear();
  has_cached_search_ = false;
  // Route identity survives clears: only the bands are poisoned. The next plan
  // re-initializes into the SAME route ids, so warm-start anchors and the switch
  // block stay intact across divergence cascades.
  for (auto &route : routes_)
    route.planner->clear();
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

void HomotopyClassPlanner::setFeedback(const ackermann_msgs::msg::AckermannDrive &feedback) {
  last_ackermann_feedback_ = feedback;
  if (base_planner_)
    base_planner_->setFeedback(feedback);
  for (auto &route : routes_)
    route.planner->setFeedback(feedback);
}

void HomotopyClassPlanner::updateObstacleContainer(
    costmap_converter_msgs::msg::ObstacleArrayMsg::ConstSharedPtr obstacle_array) {
  obstacles_ = obstacle_array;
  if (base_planner_)
    base_planner_->updateObstacleContainer(obstacle_array);
  for (auto &route : routes_)
    route.planner->updateObstacleContainer(obstacle_array);
  if (graph_search_)
    graph_search_->updateObstacles(*obstacle_array);
}

void HomotopyClassPlanner::setObstacleMap(const ObstacleMap2D *esdf) {
  esdf_ = esdf;
  if (base_planner_)
    base_planner_->setObstacleMap(esdf);
  for (auto &route : routes_)
    route.planner->setObstacleMap(esdf);
  if (graph_search_)
    graph_search_->setObstacleMap(esdf);
}

void HomotopyClassPlanner::setFixedGoal(bool fix) {
  fixed_goal_ = fix;
  if (base_planner_)
    base_planner_->setFixedGoal(fix);
}

void HomotopyClassPlanner::setGraphSearch(std::shared_ptr<GraphSearchInterface> graph_search) {
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

int HomotopyClassPlanner::matchOrCreateRoute(const std::vector<PoseSE2> &path,
                                             const HSignature &h_signature,
                                             std::set<int> &consumed_ids) {
  const double reinit_dist = params_.FollowPath.trajectory.reinit_dist;
  // Pass 1 — topological identity: the same corridor keeps its route across GVD
  // re-extractions even when the polyline geometry wobbles (Steiner nodes shift by up to
  // the subdivision spacing). This is the churn fix for benchmark_test_19's LRU storm.
  // Only computed (non-empty, finite) signatures participate — two default-constructed
  // signatures are vacuously "equal" (isEqual compares sizes first).
  const bool sig_usable = !h_signature.signature().empty() && h_signature.isValid();
  if (sig_usable) {
    for (auto &route : routes_) {
      if (consumed_ids.count(route.route_id))
        continue;
      if (!route.h_signature.signature().empty() && route.h_signature.isEqual(h_signature) &&
          (path.back().position() - route.path.back().position()).norm() <= reinit_dist) {
        route.path = path;
        route.h_signature = h_signature;
        route.last_seen_tick = tick_;
        consumed_ids.insert(route.route_id);
        return route.route_id;
      }
    }
  }

  // Pass 2 — geometric similarity: signatures change when the sliding window's obstacle
  // set changes; the corridor geometry usually does not.
  for (auto &route : routes_) {
    if (consumed_ids.count(route.route_id))
      continue;  // already claimed by another candidate this tick — never share a planner
    if (polylinesMatch(path, route.path, reinit_dist, kRouteMatchDist)) {
      route.path = path;
      route.h_signature = h_signature;
      route.last_seen_tick = tick_;
      consumed_ids.insert(route.route_id);
      return route.route_id;
    }
  }

  RouteEntry entry;
  entry.route_id = next_route_id_++;
  entry.path = path;
  entry.h_signature = h_signature;
  entry.planner = std::make_shared<DiscreteTEBPlanner>(params_, footprint_, costmap_ros_);
  entry.planner->setObstacleMap(esdf_);
  entry.planner->updateObstacleContainer(obstacles_);
  entry.planner->setFeedback(last_ackermann_feedback_);
  entry.last_seen_tick = tick_;
  routes_.push_back(std::move(entry));
  consumed_ids.insert(routes_.back().route_id);

  // Hard bound on memory: drop the least recently seen route when over capacity
  if (routes_.size() > kMaxRoutes) {
    auto lru = std::min_element(routes_.begin(), routes_.end(),
                                [](const RouteEntry &a, const RouteEntry &b) {
                                  return a.last_seen_tick < b.last_seen_tick;
                                });
    RCLCPP_INFO(rclcpp::get_logger("HomotopyClassPlanner"), "HCP: evicting route %d (LRU)",
                lru->route_id);
    routes_.erase(lru);
  }
  return routes_.back().route_id;
}

DiscreteTEBPlanner &HomotopyClassPlanner::plannerForRoute(int route_id) {
  for (auto &route : routes_) {
    if (route.route_id == route_id)
      return *route.planner;
  }
  throw std::runtime_error("HCP: no planner for route id!");
}

void HomotopyClassPlanner::pruneRoutes() {
  routes_.erase(
      std::remove_if(routes_.begin(), routes_.end(),
                     [this](const RouteEntry &e) { return tick_ - e.last_seen_tick > kRouteTTL; }),
      routes_.end());
}

bool HomotopyClassPlanner::haveRecentCandidate() const {
  if (candidates_.empty() || best_candidate_idx_ < 0)
    return false;
  const auto elapsed =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - last_success_time_).count();
  return elapsed < kRecentCandidateWindow;
}

void HomotopyClassPlanner::warnThrottled(const char *message) {
  const auto now = std::chrono::steady_clock::now();
  const auto logger = rclcpp::get_logger("HomotopyClassPlanner");
  if (std::chrono::duration<double>(now - last_fallback_warn_).count() >= kWarnInterval) {
    last_fallback_warn_ = now;
    RCLCPP_WARN(logger, "%s", message);
  } else {
    RCLCPP_DEBUG(logger, "%s", message);
  }
}

bool HomotopyClassPlanner::refreshOrReuseClasses(const PoseSE2 &start, const PoseSE2 &goal,
                                                 std::vector<GraphSearchResult> &classes) {
  using SteadyClock = std::chrono::steady_clock;

  // Reuse the cached classes while inside the search interval AND the endpoints stayed
  // put (within reinit_dist). Only the per-class TEB optimization runs on reuse ticks.
  const double min_interval = 1.0 / std::max<double>(0.1, params_.FollowPath.hcp.search_rate);
  const double reinit_dist = params_.FollowPath.trajectory.reinit_dist;
  const bool endpoint_moved =
      has_cached_search_ &&
      ((start.position() - last_search_start_.position()).norm() > reinit_dist ||
       (goal.position() - last_search_goal_.position()).norm() > reinit_dist);
  if (!endpoint_moved && has_cached_search_) {
    const bool due =
        std::chrono::duration<double>(SteadyClock::now() - last_search_time_).count() >=
        min_interval;
    if (!due) {
      classes = cached_classes_;
      return !classes.empty();
    }
  }

  // Refresh: run the search even without converter obstacles (empty array → trivial
  // direct class) so the graph — and its RViz visualization — never goes stale.
  static const costmap_converter_msgs::msg::ObstacleArrayMsg kEmptyObstacles;
  const costmap_converter_msgs::msg::ObstacleArrayMsg &obstacles =
      obstacles_ ? *obstacles_ : kEmptyObstacles;

  const int max_classes = std::max<int>(1, static_cast<int>(params_.FollowPath.hcp.max_classes));
  ++search_count_;
  last_search_time_ = SteadyClock::now();
  last_search_start_ = start;
  last_search_goal_ = goal;

  classes.clear();
  bool found;
  {
    PROFILE_BLOCK("hcp_search");
    found = graph_search_->search(start, goal, obstacles, max_classes, classes);
  }
  if (found && !classes.empty()) {
    cached_classes_ = classes;
    has_cached_search_ = true;
    return true;
  }

  // A failed search must not serve stale classes and must retry on the next tick
  cached_classes_.clear();
  has_cached_search_ = false;
  return false;
}

void HomotopyClassPlanner::optimizeCandidates(const std::vector<GraphSearchResult> &classes,
                                              const geometry_msgs::msg::Twist &start_vel,
                                              const PoseSE2 &robot_pose,
                                              const PoseSE2 &goal_pose) {
  updateCandidates(classes, robot_pose, goal_pose);

  // ── Phase A (sequential): identity + planner wiring ──
  // matchOrCreateRoute mutates routes_/next_route_id_ — not thread-safe, so route
  // assignment happens before any parallel solve.
  struct WorkItem {
    TebCandidate *candidate;
    DiscreteTEBPlanner *planner;
    nav_msgs::msg::Path path;
  };
  std::vector<WorkItem> work;
  work.reserve(candidates_.size());
  std::set<int> consumed_route_ids;

  for (auto &candidate : candidates_) {
    if (candidate->teb.sizePoses() < 2) {
      candidate->is_feasible = false;
      candidate->optimization_cost = std::numeric_limits<double>::infinity();
      continue;
    }

    // Geometric route identity: match against last tick's routes or register new. This —
    // not the H-signature — is the cross-tick anchor; it survives obstacle-set changes in
    // the sliding window, so planners keep warm-starting and hysteresis keeps applying.
    std::vector<PoseSE2> init_poses;
    init_poses.reserve(candidate->teb.sizePoses());
    for (std::size_t j = 0; j < candidate->teb.sizePoses(); ++j)
      init_poses.push_back(candidate->teb.pose(j));
    candidate->route_id =
        matchOrCreateRoute(init_poses, candidate->h_signature, consumed_route_ids);

    // Each route owns a persistent optimizer — no cross-route contamination.
    auto &route_planner = plannerForRoute(candidate->route_id);
    route_planner.setFixedGoal(fixed_goal_);

    // Plan message from the seeded band: pose 0 = live robot pose, last pose = mission
    // goal pose (both injected by updateCandidates) — never stale GVD endpoint states.
    nav_msgs::msg::Path path;
    path.poses.reserve(candidate->teb.sizePoses());
    for (std::size_t j = 0; j < candidate->teb.sizePoses(); ++j) {
      geometry_msgs::msg::PoseStamped ps;
      ps.pose = candidate->teb.pose(j).toPoseMsg();
      path.poses.push_back(std::move(ps));
    }
    path.poses.back().pose = goal_pose.toPoseMsg();

    work.push_back({candidate.get(), &route_planner, std::move(path)});
  }

  // ── Phase B (optionally parallel): independent per-route solves ──
  const bool run_parallel = params_.FollowPath.hcp.parallel_optimization && work.size() > 1;
  auto solve_one = [this, &start_vel](WorkItem &item) {
    TebCandidate &candidate = *item.candidate;
    DiscreteTEBPlanner &route_planner = *item.planner;

    bool success = false;
    bool diverged = false;
    try {
      success = route_planner.plan(item.path, start_vel);
      diverged = route_planner.hasDiverged();
    } catch (const std::exception &e) {
      RCLCPP_ERROR(rclcpp::get_logger("HomotopyClassPlanner"), "Route %d optimization threw: %s",
                   candidate.route_id, e.what());
      success = false;
    }

    if (!success || diverged) {
      candidate.is_feasible = false;
      candidate.has_diverged = diverged;
      candidate.optimization_cost = std::numeric_limits<double>::infinity();
      RCLCPP_DEBUG(rclcpp::get_logger("HomotopyClassPlanner"),
                   "Candidate optimization failed (success=%d diverged=%d)", success, diverged);
      return;
    }

    const auto &new_teb = route_planner.getTEB();
    candidate.teb = new_teb;
    candidate.is_feasible = true;
    // Normalize by pose count so chi2 totals are comparable across bands of
    // different length/edge count.
    const double norm = static_cast<double>(std::max<size_t>(1, new_teb.sizePoses()));
    candidate.optimization_cost = route_planner.getCost() / norm;
    candidate.efficiency_cost = route_planner.getEfficiencyCost() / norm;
    double len = 0.0;
    for (std::size_t i = 1; i < new_teb.sizePoses(); ++i)
      len += (new_teb.pose(i).position() - new_teb.pose(i - 1).position()).norm();
    candidate.path_length = len;

    // Post-optimization collision gate: a band that still intersects lethal cells is
    // rejected regardless of cost (its obstacle-edge penalties alone don't guarantee
    // footprint clearance — benchmark_test_19 showed through-rack bands winning).
    if (!passesFeasibilityGate(candidate.teb) || !staysInWindow(candidate.teb)) {
      candidate.is_feasible = false;
      candidate.optimization_cost = std::numeric_limits<double>::infinity();
      candidate.efficiency_cost = std::numeric_limits<double>::infinity();
      RCLCPP_DEBUG(rclcpp::get_logger("HomotopyClassPlanner"),
                   "Route %d rejected: band collides or leaves the window after optimization",
                   candidate.route_id);
      return;
    }

    RCLCPP_DEBUG(rclcpp::get_logger("HomotopyClassPlanner"),
                 "Candidate optimized cost=%.3f eff=%.3f poses=%zu", candidate.optimization_cost,
                 candidate.efficiency_cost, candidate.teb.sizePoses());
  };

  if (run_parallel) {
    std::vector<std::thread> threads;
    threads.reserve(work.size());
    for (auto &item : work)
      threads.emplace_back([&item, &solve_one]() { solve_one(item); });
    for (auto &t : threads)
      t.join();
  } else {
    for (auto &item : work)
      solve_one(item);
  }

  // ── Phase C (sequential): failure handling on shared state + pruning ──
  for (auto &item : work) {
    if (!item.candidate->is_feasible)
      item.planner->clear();  // poisoned band must not warm-start the next attempt
  }

  pruneCandidates();
}

int HomotopyClassPlanner::selectBestCandidateIndex(
    const std::vector<TebCandidate::Ptr> &candidates, bool has_prev_best, int prev_best_route_id,
    double prev_best_route_cost, bool switch_blocked, double hysteresis,
    double prev_best_route_len, double progress_slack, double anchor_floor) {
  if (candidates.empty())
    return -1;

  // Is the previous best route still offered this tick? If not, blocking cannot apply
  // (dead end) and the margin rule has nothing to anchor on. (Route-level continuity for
  // that case is handled upstream by synthesizePrevRouteCandidate.)
  const bool prev_feasible_present =
      has_prev_best &&
      std::any_of(candidates.begin(), candidates.end(), [prev_best_route_id](const auto &c) {
        return c->is_feasible && c->route_id == prev_best_route_id;
      });
  // Anchor-floor distrust (benchmark_test_20): efficiency anchors below the floor come
  // from bands reading free space beyond the mapped window — their cost is meaningless.
  // Such a previous best loses its hysteresis/progress protection AND its ranking
  // privilege (realistic competitors decided by plain min-cost); it remains available as
  // a last-resort fallback so the tick never collapses.
  const bool anchor_trusted = !has_prev_best || prev_best_route_cost >= anchor_floor;
  const bool prev_route_offered = prev_feasible_present && anchor_trusted;

  int best_idx = -1;
  double best_cost = std::numeric_limits<double>::infinity();

  for (size_t i = 0; i < candidates.size(); ++i) {
    const auto &candidate = candidates[i];
    if (!candidate->is_feasible)
      continue;
    // Untrusted previous best: out of the normal ranking (fallback-only)
    if (has_prev_best && !anchor_trusted && candidate->route_id == prev_best_route_id)
      continue;

    // Continuity rules apply only to candidates of OTHER routes
    if (has_prev_best && candidate->route_id != prev_best_route_id) {
      if (switch_blocked && prev_route_offered)
        continue;
      if (prev_route_offered) {
        // Hysteresis: switching away requires beating the previous best by a margin. The
        // margin only applies while the previous best route is actually offered this tick
        // — once it slid out of the window its stale cost must not veto new routes.
        if (candidate->efficiency_cost >= prev_best_route_cost * hysteresis)
          continue;
        // Progress guard: a takeover must also not be a detour — all bands span robot →
        // mission-goal, so arc length is a comparable progress estimate (ΣΔt alone barely
        // rewards shorter geometry). Only while prev is offered: dead-end escapes may be
        // arbitrarily longer.
        if (progress_slack < 1.0 && std::isfinite(prev_best_route_len) &&
            candidate->path_length > prev_best_route_len * (1.0 + progress_slack))
          continue;
      }
    }

    if (candidate->efficiency_cost < best_cost) {
      best_cost = candidate->efficiency_cost;
      best_idx = static_cast<int>(i);
    }
  }

  // Fallback: continuity rules rejected everything, but the previous best route has a
  // feasible member → stick with it rather than reporting failure (also serves the
  // distrusted-anchor case when no competitor exists).
  if (best_idx == -1 && prev_feasible_present) {
    for (size_t i = 0; i < candidates.size(); ++i) {
      const auto &candidate = candidates[i];
      if (!candidate->is_feasible || candidate->route_id != prev_best_route_id)
        continue;
      if (best_idx == -1 || candidate->efficiency_cost < best_cost) {
        best_cost = candidate->efficiency_cost;
        best_idx = static_cast<int>(i);
      }
    }
  }

  return best_idx;
}

int HomotopyClassPlanner::selectBestCandidate() const {
  if (candidates_.empty())
    return -1;

  // Switch blocking: within the block window after a switch, the previous best route may
  // not be abandoned.
  bool switch_blocked = false;
  if (has_prev_best_ && prev_best_route_id_ >= 0) {
    const auto elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - last_switch_time_)
            .count();
    switch_blocked = elapsed < params_.FollowPath.hcp.switch_block_time;
  }

  return selectBestCandidateIndex(
      candidates_, has_prev_best_, prev_best_route_id_, prev_best_route_cost_, switch_blocked,
      params_.FollowPath.hcp.selection_hysteresis, prev_best_route_len_,
      params_.FollowPath.hcp.progress_slack, params_.FollowPath.hcp.efficiency_anchor_floor);
}

void HomotopyClassPlanner::updateCandidates(const std::vector<GraphSearchResult> &classes,
                                            const PoseSE2 &robot_pose, const PoseSE2 &goal_pose) {
  candidates_.clear();
  candidates_.reserve(classes.size());

  double dt_default = params_.FollowPath.trajectory.dt_ref;
  for (const auto &cls : classes) {
    auto candidate = std::make_shared<TebCandidate>();
    candidate->h_signature = cls.h_signature;

    // Initialize TEB from the path returned by graph search
    candidate->teb = TimedElasticBand();
    for (const auto &pose : cls.path) {
      candidate->teb.addPose(pose);
    }

    // Seed band pose 0 with the LIVE robot pose (position AND heading). The GVD polyline
    // starts at the search-time start node — stale by up to 1/search_rate (worse during
    // loop-rate dips) and oriented along the connector, not the robot. extractVelocity()
    // derives the entire command from pose(0), so this seed is what makes the band
    // followable.
    if (candidate->teb.sizePoses() >= 1) {
      auto &p0 = candidate->teb.pose(0);
      p0.x() = robot_pose.x();
      p0.y() = robot_pose.y();
      p0.theta() = robot_pose.theta();
    }

    // Seed band LAST pose with the mission goal pose. The GVD polyline's terminal heading
    // is derived from the connector segment; with fix_goal that vertex is PINNED, so an
    // unseeded band converges to the goal position while ignoring the desired heading.
    if (candidate->teb.sizePoses() >= 2) {
      auto &pn = candidate->teb.pose(candidate->teb.sizePoses() - 1);
      pn.x() = goal_pose.x();
      pn.y() = goal_pose.y();
      pn.theta() = goal_pose.theta();
    }

    // Initialize time diffs from the configured reference dt
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
  auto it = std::remove_if(candidates_.begin(), candidates_.end(), [](const TebCandidate::Ptr &c) {
    return c->has_diverged || !c->is_feasible || std::isinf(c->optimization_cost);
  });
  candidates_.erase(it, candidates_.end());
}

bool HomotopyClassPlanner::passesFeasibilityGate(const TimedElasticBand &teb) const {
  if (esdf_ == nullptr)
    return true;  // no obstacle data — nothing to gate on
  if (!params_.FollowPath.hcp.feasibility_gate)
    return true;
  return checkFeasibility(teb, *esdf_, footprint_,
                          params_.FollowPath.obstacles.feasibility_check) < 0;
}

bool HomotopyClassPlanner::staysInWindow(const TimedElasticBand &teb) const {
  if (esdf_ == nullptr || !esdf_->isInitialized())
    return true;
  if (!params_.FollowPath.hcp.window_containment)
    return true;

  const double margin = kWindowContainmentMargin;

  // Self-disable: if even the mission-goal seed lies outside the window, every band ends
  // beyond it on this leg — rejecting them all would collapse HCP to its fallback.
  if (!esdf_->contains(teb.backPose().x(), teb.backPose().y(), margin))
    return true;

  for (std::size_t i = 0; i < teb.sizePoses(); ++i) {
    if (!esdf_->contains(teb.pose(i).x(), teb.pose(i).y(), margin)) {
      RCLCPP_DEBUG(rclcpp::get_logger("HomotopyClassPlanner"),
                   "Route gate: band pose %zu leaves the mapped window", i);
      return false;
    }
  }
  return true;
}

void HomotopyClassPlanner::synthesizePrevRouteCandidate(std::vector<GraphSearchResult> &classes,
                                                        const PoseSE2 &robot_pose,
                                                        const PoseSE2 &goal_pose) {
  if (!has_prev_best_ || prev_best_route_id_ < 0 || params_.FollowPath.hcp.route_grace_time <= 0.0)
    return;

  // Only while the grace window is open
  const auto since_seen =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - last_prev_seen_time_)
          .count();
  if (since_seen >= params_.FollowPath.hcp.route_grace_time)
    return;

  const RouteEntry *entry = nullptr;
  for (const auto &r : routes_) {
    if (r.route_id == prev_best_route_id_) {
      entry = &r;
      break;
    }
  }
  if (entry == nullptr || entry->path.size() < 2)
    return;  // route was evicted — no continuity anchor left

  // Already offered this round? Compare against the ENTRY (classes carry no route ids
  // yet — those are assigned during Phase A matching).
  const bool prev_present =
      std::any_of(classes.begin(), classes.end(), [this, entry](const GraphSearchResult &c) {
        if (!entry->h_signature.signature().empty() && !c.h_signature.signature().empty() &&
            c.h_signature.isEqual(entry->h_signature))
          return true;
        return polylinesMatch(c.path, entry->path, params_.FollowPath.trajectory.reinit_dist,
                              kRouteMatchDist);
      });
  if (prev_present)
    return;

  GraphSearchResult resynth;
  resynth.h_signature = entry->h_signature;
  resynth.path = entry->path;
  // Same seeding contract as updateCandidates: live robot pose in front, mission goal at back.
  resynth.path.front() = robot_pose;
  resynth.path.back() = goal_pose;
  classes.push_back(std::move(resynth));
  RCLCPP_DEBUG(rclcpp::get_logger("HomotopyClassPlanner"),
               "HCP: prev route %d missing from search round — synthesized from route entry "
               "(grace %.3f/%.3f s)",
               entry->route_id, since_seen, params_.FollowPath.hcp.route_grace_time);
}

void HomotopyClassPlanner::storeFallbackCandidate() {
  auto candidate = std::make_shared<TebCandidate>();
  candidate->teb = base_planner_->getTEB();
  candidate->is_feasible = true;
  candidate->optimization_cost = base_planner_->getCost();
  candidate->efficiency_cost = base_planner_->getEfficiencyCost();
  candidate->has_diverged = false;

  // Register the fallback band under a geometrically matching route (or a new one) so
  // identity, hysteresis and visualization survive fallback periods.
  std::vector<PoseSE2> poses;
  poses.reserve(candidate->teb.sizePoses());
  for (std::size_t j = 0; j < candidate->teb.sizePoses(); ++j)
    poses.push_back(candidate->teb.pose(j));
  if (poses.size() >= 2) {
    std::set<int> consumed;
    candidate->route_id = matchOrCreateRoute(poses, candidate->h_signature, consumed);
  }

  candidates_.clear();
  candidates_.push_back(candidate);
  best_candidate_idx_ = 0;
}

}  // namespace nav2_teb_controller
