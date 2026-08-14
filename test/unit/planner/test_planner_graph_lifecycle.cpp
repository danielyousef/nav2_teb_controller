// Graph-lifecycle regression tests for DiscreteTEBPlanner.
//
// Exercises the exact crash scenarios from the field: cold start + warm-start
// ticks, 3-phase stepwise optimization with vertex reuse across phases and
// ticks, band growth/shrink between ticks, reinit, and ESDF obstacle edges
// (robust kernels) across consecutive plans. Most tests zero the obstacle
// weights so addEdgesESDFObstacles() returns early and no costmap pointer is
// dereferenced; EsdfObstacleEdgesAcrossConsecutivePlans builds a synthetic
// costmap/ESDF instead.
//
// NOTE: these tests segfault (or crash) if the graph lifecycle is broken, e.g.
//   - vertex pools grown via vector::resize on a unique_ptr vector (null pool)
//   - pooled vertices still owned by the optimizer when SparseOptimizer::clear()
//     is called (HyperGraph::clear deletes every owned vertex -> use-after-free)

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>

#include <geometry_msgs/msg/twist.hpp>
#include <g2o/core/robust_kernel_impl.h>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav_msgs/msg/path.hpp>

#include "nav2_teb_controller/core/footprint.hpp"
#include "nav2_teb_controller/g2o_types/edge_esdf_obstacle.h"
#include "nav2_teb_controller/obstacles/esdf.hpp"
#include "nav2_teb_controller/planner/optimal_planner.hpp"

namespace nav2_teb_controller {
namespace {

nav_msgs::msg::Path makePath(size_t n_poses, double spacing, double y_offset) {
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.poses.reserve(n_poses);
  for (size_t i = 0; i < n_poses; ++i) {
    geometry_msgs::msg::PoseStamped ps;
    ps.pose.position.x = static_cast<double>(i) * spacing;
    ps.pose.position.y = y_offset + 0.1 * std::sin(0.2 * static_cast<double>(i));
    ps.pose.orientation.w = 1.0;
    path.poses.push_back(ps);
  }
  return path;
}

teb_controller::Params makeParams() {
  teb_controller::Params p;
  p.FollowPath.optimizer.algorithm = "levenberg_marquardt";  // matches the runtime config
  p.FollowPath.optimizer.no_inner_iterations = 8;
  p.FollowPath.optimizer.no_outer_iterations = 4;
  p.FollowPath.optimizer.stepwise_optimization = true;
  p.FollowPath.optimizer.divergence_detection_enable = false;
  p.FollowPath.optimizer.fix_goal = true;
  p.FollowPath.robot.robot_model = "diff_drive";
  p.FollowPath.weights.weight_obstacle = 0.0;  // no costmap in unit test
  p.FollowPath.weights.weight_inflation = 0.0;
  return p;
}

// Cold start, then warm-start ticks with the plan front advancing (band shrinks
// via updateAndPrune + autoResize), then a longer plan (pool growth + reinit).
TEST(PlannerGraphLifecycle, ColdStartWarmTicksGrowthAndReinit) {
  teb_controller::Params params = makeParams();
  Footprint footprint;
  DiscreteTEBPlanner planner(params, footprint, nullptr);

  const geometry_msgs::msg::Twist start_vel;
  nav_msgs::msg::Path path = makePath(40, 0.4, 0.0);

  // Tick 1: cold start (band initialized from the path, all three phases run)
  ASSERT_TRUE(planner.plan(path, start_vel));
  ASSERT_GT(planner.getTEB().sizePoses(), 1u);

  // Ticks 2..9: warm start — robot advances, plan front is pruned, same goal.
  // Exercises vertex/kernel reuse across phases AND across ticks, plus band
  // shrink.
  for (size_t tick = 1; tick <= 8; ++tick) {
    const size_t front = 2 * tick;
    if (path.poses.size() < front + 5)
      break;
    path.poses.erase(path.poses.begin(), path.poses.begin() + front);
    ASSERT_TRUE(planner.plan(path, start_vel));
    ASSERT_GT(planner.getTEB().sizePoses(), 1u);
  }

  // Longer plan with a far goal: forces teb reinit + vertex pool growth.
  const nav_msgs::msg::Path long_path = makePath(60, 0.4, 5.0);
  ASSERT_TRUE(planner.plan(long_path, start_vel));
  ASSERT_GT(planner.getTEB().sizePoses(), 1u);

  // Second run scenario (sim benchmark): 275-pose cold start after a completed
  // run — the band + pools must grow far beyond the previous sizes.
  const nav_msgs::msg::Path big_path = makePath(275, 0.4, 8.0);
  ASSERT_TRUE(planner.plan(big_path, start_vel));
  ASSERT_GT(planner.getTEB().sizePoses(), 1u);

  planner.clear();
}

// ESDF obstacle edges with robust kernels across consecutive plans — regression
// for the field crash. g2o's OptimizableGraph::Edge::~Edge() DELETES its robust
// kernel (the dtor calls the virtual deleting dtor -> operator delete, verified
// against the installed libg2o 2020.5). The previous pooled-kernel code reused
// kernel objects that clearGraph() had already freed, so the next plan() crashed
// in addEdgesESDFObstacles() at rk->setDelta(1.0). Requires a real ESDF +
// obstacle weights > 0 (the other tests zero the weights to avoid a costmap).
TEST(PlannerGraphLifecycle, EsdfObstacleEdgesAcrossConsecutivePlans) {
  teb_controller::Params params = makeParams();
  params.FollowPath.weights.weight_obstacle = 50.0;
  params.FollowPath.weights.weight_inflation = 5.0;
  Footprint footprint(false, "circles", "[ [0.0, 0.0, 0.15], [0.35, 0.0, 0.12] ]");
  DiscreteTEBPlanner planner(params, footprint, nullptr);

  // Synthetic 10x10 m costmap with a LETHAL cell cluster at (3.0, 0.0).
  nav2_costmap_2d::Costmap2D costmap(200, 200, 0.05, -5.0, -5.0);
  const int cx = static_cast<int>((3.0 + 5.0) / 0.05);
  const int cy = static_cast<int>((0.0 + 5.0) / 0.05);
  for (int dy = -3; dy <= 3; ++dy)
    for (int dx = -3; dx <= 3; ++dx)
      costmap.setCost(static_cast<unsigned int>(cx + dx), static_cast<unsigned int>(cy + dy),
                      nav2_costmap_2d::LETHAL_OBSTACLE);

  ObstacleMap2D esdf;
  esdf.update(costmap);
  ASSERT_TRUE(esdf.isInitialized());
  planner.setObstacleMap(&esdf);

  const geometry_msgs::msg::Twist start_vel;
  const nav_msgs::msg::Path path = makePath(40, 0.4, 0.0);  // passes through the obstacle

  // Plan 1: cold start — phases 0+1 build ESDF edges with robust kernels.
  ASSERT_TRUE(planner.plan(path, start_vel));
  // Plan 2: warm start — with the pooled-kernel code, kernels freed by the edge
  // dtor in clearGraph() were reused here -> SIGSEGV in rk->setDelta().
  ASSERT_TRUE(planner.plan(path, start_vel));
  // Plan 3: new goal — teb reinit + fresh ESDF edges (same kernel code).
  const nav_msgs::msg::Path path2 = makePath(55, 0.4, -3.0);
  ASSERT_TRUE(planner.plan(path2, start_vel));
  // Plan 4: cold reinit back through the obstacle.
  ASSERT_TRUE(planner.plan(path, start_vel));

  planner.clear();
}

// Pins the g2o dependency the ESDF kernel ownership relies on: an edge's
// destructor DELETES its robust kernel (OptimizableGraph::Edge dtor calls the
// virtual deleting dtor, verified against the installed libg2o 2020.5). The
// earlier pooled-kernel design violated this and reused freed kernel objects in
// the next plan().
TEST(PlannerGraphLifecycle, EdgeDestructorDeletesRobustKernel) {
  bool deleted = false;
  struct KernelProbe : g2o::RobustKernelHuber {
    explicit KernelProbe(bool *flag) : flag_(flag) {}
    ~KernelProbe() override { *flag_ = true; }
    bool *flag_;
  };
  auto *probe = new KernelProbe(&deleted);
  auto *edge = new EdgeESDFObstacle();
  edge->setRobustKernel(probe);
  delete edge;
  EXPECT_TRUE(deleted);
}

// Stress test for pooled graph objects across many ticks: alternating long/short
// plans (band growth/shrink -> varying edge counts), ESDF obstacle edges with
// robust kernels (kernel lifecycle on pooled edges), and periodic reinit.
// Exercises the edge pool's growth, reuse and per-phase reset paths.
TEST(PlannerGraphLifecycle, PooledEdgesAcrossManyPlans) {
  teb_controller::Params params = makeParams();
  params.FollowPath.weights.weight_obstacle = 50.0;
  params.FollowPath.weights.weight_inflation = 5.0;
  Footprint footprint(false, "circles", "[ [0.0, 0.0, 0.15], [0.35, 0.0, 0.12] ]");
  DiscreteTEBPlanner planner(params, footprint, nullptr);

  nav2_costmap_2d::Costmap2D costmap(200, 200, 0.05, -5.0, -5.0);
  const int cx = static_cast<int>((3.0 + 5.0) / 0.05);
  const int cy = static_cast<int>((0.0 + 5.0) / 0.05);
  for (int dy = -3; dy <= 3; ++dy)
    for (int dx = -3; dx <= 3; ++dx)
      costmap.setCost(static_cast<unsigned int>(cx + dx), static_cast<unsigned int>(cy + dy),
                      nav2_costmap_2d::LETHAL_OBSTACLE);

  ObstacleMap2D esdf;
  esdf.update(costmap);
  planner.setObstacleMap(&esdf);

  const geometry_msgs::msg::Twist start_vel;
  for (size_t iter = 0; iter < 30; ++iter) {
    // Alternate long/short plans (different band sizes -> different edge counts)
    // and reinit every 4th tick (different goal -> teb_.clear() + cold start).
    const size_t n = (iter % 2 == 0) ? 45u : 25u;
    const double y = (iter % 4 == 0) ? 2.0 : 0.0;
    nav_msgs::msg::Path path = makePath(n, 0.4, y);
    if (iter > 0 && path.poses.size() >= 6) {
      // Advance the front so warm-start pruning shrinks the band each tick.
      const size_t erase_n = std::min<size_t>(iter % 7, path.poses.size() - 5);
      path.poses.erase(path.poses.begin(), path.poses.begin() + erase_n);
    }
    ASSERT_TRUE(planner.plan(path, start_vel));

    const auto &teb = planner.getTEB();
    ASSERT_GT(teb.sizePoses(), 1u);
    for (size_t i = 0; i < teb.sizePoses(); ++i) {
      EXPECT_TRUE(std::isfinite(teb.pose(i).x()));
      EXPECT_TRUE(std::isfinite(teb.pose(i).y()));
      EXPECT_TRUE(std::isfinite(teb.pose(i).theta()));
    }
    for (size_t i = 0; i < teb.sizeTimeDiffs(); ++i)
      EXPECT_TRUE(std::isfinite(teb.timeDiff(i)));
  }
  planner.clear();
}

// Non-fixed goal variant (goal vertex not anchored; different edge set).
TEST(PlannerGraphLifecycle, GoalNotFixedMultiTick) {
  teb_controller::Params params = makeParams();
  params.FollowPath.optimizer.fix_goal = false;
  Footprint footprint;
  DiscreteTEBPlanner planner(params, footprint, nullptr);

  const geometry_msgs::msg::Twist start_vel;
  for (size_t tick = 0; tick < 6; ++tick) {
    const nav_msgs::msg::Path path = makePath(35, 0.35, 0.25 * static_cast<double>(tick));
    ASSERT_TRUE(planner.plan(path, start_vel));
    ASSERT_GT(planner.getTEB().sizePoses(), 1u);
  }
  planner.clear();
}

// Ackermann model variant (car-like kinematics + steering edges).
TEST(PlannerGraphLifecycle, AckermannMultiTick) {
  teb_controller::Params params = makeParams();
  params.FollowPath.robot.robot_model = "ackermann";
  params.FollowPath.robot.v_max_y = 0.0;
  Footprint footprint;
  DiscreteTEBPlanner planner(params, footprint, nullptr);

  const geometry_msgs::msg::Twist start_vel;
  for (size_t tick = 0; tick < 6; ++tick) {
    const nav_msgs::msg::Path path = makePath(35, 0.35, 0.0);
    ASSERT_TRUE(planner.plan(path, start_vel));
    ASSERT_GT(planner.getTEB().sizePoses(), 1u);
  }
  planner.clear();
}

}  // namespace
}  // namespace nav2_teb_controller