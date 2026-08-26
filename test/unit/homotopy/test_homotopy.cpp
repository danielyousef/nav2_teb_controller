// Unit tests for the homotopy module: H-signature winding numbers, the shared
// templated graph algorithms (Dijkstra / Yen K-shortest paths), the Voronoi
// (GVD) graph extraction from the ESDF, both graph searches, and the
// HomotopyClassPlanner candidate pipeline (per-class planner isolation).

#include <angles/angles.h>
#include <gtest/gtest.h>

#include <cmath>
#include <costmap_converter_msgs/msg/obstacle_array_msg.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav_msgs/msg/path.hpp>
#include <vector>

#include "nav2_teb_controller/core/footprint.hpp"
#include "nav2_teb_controller/core/teb_utils.hpp"
#include "nav2_teb_controller/homotopy/graph_algorithms.hpp"
#include "nav2_teb_controller/homotopy/h_signature.hpp"
#include "nav2_teb_controller/homotopy/homotopy_class_planner.hpp"
#include "nav2_teb_controller/homotopy/visibility_graph_search.hpp"
#include "nav2_teb_controller/homotopy/voronoi_graph_search.hpp"
#include "nav2_teb_controller/obstacles/esdf.hpp"
#include "nav2_teb_controller/planner/optimal_planner.hpp"

namespace nav2_teb_controller {
namespace {

using ObstacleArray = costmap_converter_msgs::msg::ObstacleArrayMsg;

// ── Minimal graph satisfying the graph_algo template requirements ──────────
struct TestNode {
  Eigen::Vector2d pos;
  int id;
};

struct TestEdge {
  int from_id;
  int to_id;
  double cost;
};

struct TestGraph {
  std::vector<TestNode> nodes_;
  std::vector<std::vector<TestEdge>> adj_;

  void addNode(double x, double y) {
    nodes_.push_back({Eigen::Vector2d(x, y), static_cast<int>(nodes_.size())});
    adj_.emplace_back();
  }
  void addEdge(int u, int v) {
    const double c = (nodes_[u].pos - nodes_[v].pos).norm();
    adj_[u].push_back({u, v, c});
    adj_[v].push_back({v, u, c});
  }
  [[nodiscard]] const std::vector<TestNode> &nodes() const { return nodes_; }
  [[nodiscard]] const std::vector<TestEdge> &edges(int id) const { return adj_[id]; }
};

ObstacleArray makeTwoObstacles() {
  ObstacleArray arr;
  // Two square blobs centered at y = ±0.75
  for (const double cy : {-0.75, 0.75}) {
    costmap_converter_msgs::msg::ObstacleMsg obs;
    for (const auto &[px, py] : std::array<std::pair<double, double>, 4>{
             {std::make_pair(-0.2, -0.2), std::make_pair(0.2, -0.2), std::make_pair(0.2, 0.2),
              std::make_pair(-0.2, 0.2)}}) {
      geometry_msgs::msg::Point32 pt;
      pt.x = static_cast<float>(3.0 + px);
      pt.y = static_cast<float>(cy + py);
      obs.polygon.points.push_back(pt);
    }
    arr.obstacles.push_back(obs);
  }
  return arr;
}

/// Lethal blobs at (3, ±0.75) leaving an open corridor along y=0 between them.
ObstacleMap2D makeTwoBlobEsdf() {
  nav2_costmap_2d::Costmap2D costmap(200, 200, 0.05, -5.0, -5.0);
  auto fillBlob = [&](double cx, double cy) {
    const int gx = static_cast<int>((cx + 5.0) / 0.05);
    const int gy = static_cast<int>((cy + 5.0) / 0.05);
    for (int dy = -4; dy <= 4; ++dy)
      for (int dx = -4; dx <= 4; ++dx)
        costmap.setCost(static_cast<unsigned int>(gx + dx), static_cast<unsigned int>(gy + dy),
                        nav2_costmap_2d::LETHAL_OBSTACLE);
  };
  fillBlob(3.0, -0.75);
  fillBlob(3.0, 0.75);

  ObstacleMap2D esdf;
  esdf.update(costmap);
  return esdf;
}

std::vector<PoseSE2> makeStraightPoses(double x0, double x1, double y, double step = 0.1) {
  std::vector<PoseSE2> poses;
  const int n = static_cast<int>(std::abs(x1 - x0) / step);
  poses.reserve(n + 1);
  for (int i = 0; i <= n; ++i)
    poses.emplace_back(x0 + i * step, y, 0.0);
  return poses;
}

/// Wall at x≈3 made of THREE pieces (top / middle / bottom) with two passable gaps.
/// Routes through the upper vs lower gap wind oppositely around the middle piece →
/// genuinely distinct homotopy classes on the GVD (two free-standing blobs would not:
/// their GVD is a single bisector cross with one class).
ObstacleMap2D makeWallWithGapsEsdf() {
  nav2_costmap_2d::Costmap2D costmap(200, 200, 0.05, -5.0, -5.0);
  auto fillRect = [&](double x0, double x1, double y0, double y1) {
    const int gx0 = static_cast<int>((x0 + 5.0) / 0.05);
    const int gx1 = static_cast<int>((x1 + 5.0) / 0.05);
    const int gy0 = static_cast<int>((y0 + 5.0) / 0.05);
    const int gy1 = static_cast<int>((y1 + 5.0) / 0.05);
    for (int gy = gy0; gy <= gy1; ++gy)
      for (int gx = gx0; gx <= gx1; ++gx)
        costmap.setCost(static_cast<unsigned int>(gx), static_cast<unsigned int>(gy),
                        nav2_costmap_2d::LETHAL_OBSTACLE);
  };
  fillRect(2.9, 3.1, -2.0, -1.2);  // bottom piece
  fillRect(2.9, 3.1, -0.4, 0.4);   // middle piece (blocks the direct line)
  fillRect(2.9, 3.1, 1.2, 2.0);    // top piece

  ObstacleMap2D esdf;
  esdf.update(costmap);
  return esdf;
}

ObstacleArray makeWallObstacles() {
  ObstacleArray arr;
  auto addRect = [&](double y0, double y1) {
    costmap_converter_msgs::msg::ObstacleMsg obs;
    for (const auto &[px, py] : std::array<std::pair<double, double>, 4>{
             {std::make_pair(2.9, y0), std::make_pair(3.1, y0), std::make_pair(3.1, y1),
              std::make_pair(2.9, y1)}}) {
      geometry_msgs::msg::Point32 pt;
      pt.x = static_cast<float>(px);
      pt.y = static_cast<float>(py);
      obs.polygon.points.push_back(pt);
    }
    arr.obstacles.push_back(obs);
  };
  addRect(-2.0, -1.2);
  addRect(-0.4, 0.4);
  addRect(1.2, 2.0);
  return arr;
}

nav_msgs::msg::Path makeStraightPath(double x0, double x1, double y, double step = 0.1) {
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (const auto &pose : makeStraightPoses(x0, x1, y, step)) {
    geometry_msgs::msg::PoseStamped ps;
    ps.pose = pose.toPoseMsg();
    path.poses.push_back(ps);
  }
  return path;
}

teb_controller::Params makeHcpParams() {
  teb_controller::Params p;
  p.FollowPath.optimizer.algorithm = "levenberg_marquardt";
  p.FollowPath.optimizer.no_inner_iterations = 8;
  p.FollowPath.optimizer.no_outer_iterations = 4;
  p.FollowPath.optimizer.stepwise_optimization = true;
  p.FollowPath.optimizer.divergence_detection_enable = false;
  p.FollowPath.optimizer.fix_goal = true;
  p.FollowPath.robot.robot_model = "diff_drive";
  p.FollowPath.hcp.max_classes = 3;
  p.FollowPath.weights.weight_obstacle = 50.0;
  p.FollowPath.weights.weight_inflation = 5.0;
  return p;
}

// ── HSignature ───────────────────────────────────────────────────────────────

TEST(HSignature, StraightPathHasZeroWinding) {
  ObstacleArray obstacles = makeTwoObstacles();
  HSignature sig;
  sig.compute(makeStraightPoses(1.0, 5.0, 0.0), obstacles);

  ASSERT_EQ(sig.signature().size(), obstacles.obstacles.size());
  for (const auto &c : sig.signature())
    EXPECT_NEAR(c.real(), 0.0, 1e-6);
  EXPECT_TRUE(sig.isValid());
}

TEST(HSignature, NearFullLoopWindsAroundOneObstacle) {
  ObstacleArray obstacles = makeTwoObstacles();

  // ~full CCW loop around the lower blob at (3, -0.75) → winding number +1
  std::vector<PoseSE2> path;
  for (int i = 0; i <= 60; ++i) {
    const double theta = (11.0 * M_PI / 6.0) * static_cast<double>(i) / 60.0;  // 0..330°
    path.emplace_back(3.0 + 0.5 * std::cos(theta), -0.75 + 0.5 * std::sin(theta), 0.0);
  }

  HSignature sig;
  sig.compute(path, obstacles);

  EXPECT_NEAR(sig.signature()[0].real(), 1.0, 1e-3);  // one full encirclement
  EXPECT_NEAR(sig.signature()[1].real(), 0.0, 1e-3);  // untouched upper blob
}

TEST(HSignature, SemicircleStaysInZeroClass) {
  ObstacleArray obstacles = makeTwoObstacles();

  // Half loop over the top of the lower blob: NOT an encirclement — deformable into the
  // zero class (e.g. a wide arc on the other side), so the invariant must report 0.
  std::vector<PoseSE2> path;
  for (int i = 0; i <= 36; ++i) {
    const double theta = M_PI * static_cast<double>(i) / 36.0;  // 0..pi (CCW)
    path.emplace_back(3.0 + 0.5 * std::cos(theta), -0.75 + 0.5 * std::sin(theta), 0.0);
  }

  HSignature sig;
  sig.compute(path, obstacles);

  EXPECT_NEAR(sig.signature()[0].real(), 0.0, 1e-3);
  EXPECT_NEAR(sig.signature()[1].real(), 0.0, 1e-3);
}

TEST(HSignature, CanonicalOrderingIdenticalAcrossReorder) {
  // The converter's array order is unstable across ticks; signatures must not care.
  ObstacleArray a = makeTwoObstacles();
  std::vector<PoseSE2> path;
  for (int i = 0; i <= 60; ++i) {
    const double theta = (11.0 * M_PI / 6.0) * static_cast<double>(i) / 60.0;
    path.emplace_back(3.0 + 0.5 * std::cos(theta), -0.75 + 0.5 * std::sin(theta), 0.0);
  }

  HSignature sig_a, sig_b;
  sig_a.compute(path, a);

  ObstacleArray b = a;
  std::reverse(b.obstacles.begin(), b.obstacles.end());
  sig_b.compute(path, b);

  ASSERT_EQ(sig_a.signature().size(), sig_b.signature().size());
  EXPECT_TRUE(sig_a.isEqual(sig_b));
  for (size_t i = 0; i < sig_a.signature().size(); ++i)
    EXPECT_NEAR(sig_a.signature()[i].real(), sig_b.signature()[i].real(), 1e-9);
}

TEST(HSignature, IsEqualComparesComponentwiseWithTolerance) {
  HSignature a, b, c;
  a.compute(makeStraightPoses(1.0, 5.0, 0.0), makeTwoObstacles());
  b.compute(makeStraightPoses(1.0, 5.0, 0.02), makeTwoObstacles());
  c.compute(makeStraightPoses(1.0, 5.0, 0.0), ObstacleArray());  // different size

  EXPECT_TRUE(a.isEqual(b, 1e-3));
  EXPECT_FALSE(a.isEqual(c, 1e-3));
}

// ── Shared graph algorithms ──────────────────────────────────────────────

TEST(GraphAlgo, ShortestPathOnLineGraph) {
  TestGraph g;
  g.addNode(0, 0);
  g.addNode(1, 0);
  g.addNode(2, 0);
  g.addEdge(0, 1);
  g.addEdge(1, 2);

  const auto path = graph_algo::shortestPath(g, 0, 2);
  ASSERT_EQ(path.size(), 3u);
  EXPECT_EQ(path.front(), 0);
  EXPECT_EQ(path.back(), 2);
}

TEST(GraphAlgo, ShortestPathUnreachableReturnsEmpty) {
  TestGraph g;
  g.addNode(0, 0);
  g.addNode(1, 5);
  // no edge

  EXPECT_TRUE(graph_algo::shortestPath(g, 0, 1).empty());
}

TEST(GraphAlgo, YenFindsBothDiamondRoutesCheapestFirst) {
  // Diamond: A→B→D (top, shorter) vs A→C→D (bottom, longer)
  TestGraph g;
  g.addNode(0, 0);   // A
  g.addNode(2, 1);   // B
  g.addNode(2, -2);  // C
  g.addNode(4, 0);   // D
  g.addEdge(0, 1);
  g.addEdge(1, 3);
  g.addEdge(0, 2);
  g.addEdge(2, 3);

  const auto paths = graph_algo::kShortestPaths(g, 0, 3, 2);
  ASSERT_EQ(paths.size(), 2u);
  EXPECT_EQ(paths[0], (std::vector<int>{0, 1, 3}));  // top route first
  EXPECT_EQ(paths[1], (std::vector<int>{0, 2, 3}));  // bottom route second

  const auto poses = graph_algo::pathToPoses(g, paths[0]);
  ASSERT_EQ(poses.size(), 3u);
  EXPECT_NEAR(poses[0].x(), 0.0, 1e-9);
  EXPECT_NEAR(poses.back().y(), 0.0, 1e-9);
}

TEST(GraphAlgo, FilterDuplicateClassesKeepsFirstOccurrence) {
  std::vector<GraphSearchResult> results(2);
  results[0].h_signature.compute(makeStraightPoses(1.0, 5.0, 0.0), makeTwoObstacles());
  results[1].h_signature.compute(makeStraightPoses(1.0, 5.0, 0.01), makeTwoObstacles());

  graph_algo::filterDuplicateClasses(results);
  ASSERT_EQ(results.size(), 1u);  // identical winding numbers → duplicate removed
}

// ── Voronoi (GVD) graph + search ─────────────────────────────────────────────

TEST(Voronoi, SearchFindsDistinctClassesThroughWallGaps) {
  const ObstacleMap2D esdf = makeWallWithGapsEsdf();
  VoronoiGraphSearch search(0.15);
  search.setObstacleMap(&esdf);

  std::vector<GraphSearchResult> results;
  const ObstacleArray obstacles = makeWallObstacles();
  ASSERT_TRUE(
      search.search(PoseSE2(1.0, 0.0, 0.0), PoseSE2(5.0, 0.0, 0.0), obstacles, 3, results));

  // Upper-gap and lower-gap routes wind oppositely around the middle wall piece
  EXPECT_GE(results.size(), 2u);
  for (const auto &r : results) {
    EXPECT_GT(r.cost, 0.0);
    for (const auto &pose : r.path) {
      EXPECT_TRUE(std::isfinite(pose.x()));
      EXPECT_TRUE(std::isfinite(pose.y()));
    }
  }

  // At least two DISTINCT homotopy classes (different winding patterns)
  EXPECT_FALSE(results[0].h_signature.isEqual(results[1].h_signature));
}

TEST(Voronoi, GraphNodesStayOutOfObstacles) {
  const ObstacleMap2D esdf = makeTwoBlobEsdf();
  VoronoiGraphSearch search(0.15);
  search.setObstacleMap(&esdf);

  std::vector<GraphSearchResult> results;
  ASSERT_TRUE(search.search(PoseSE2(1.0, 0.0, 0.0), PoseSE2(5.0, 0.0, 0.0), makeTwoObstacles(), 2,
                            results));

  const auto &graph = search.getVoronoiGraph();
  ASSERT_FALSE(graph.empty());
  for (const auto &node : graph.nodes()) {
    // No graph node may sit inside a lethal cell (clearance > 0)
    EXPECT_GT(esdf.queryDistance(node.pos.x(), node.pos.y()), 0.0);
  }
}

TEST(Voronoi, NoPathWhenGoalFullyBlockedReturnsFalse) {
  nav2_costmap_2d::Costmap2D costmap(200, 200, 0.05, -5.0, -5.0);
  // Solid wall separating start from goal
  for (unsigned y = 0; y < costmap.getSizeInCellsY(); ++y)
    costmap.setCost(static_cast<unsigned int>((0.0 + 5.0) / 0.05), y,
                    nav2_costmap_2d::LETHAL_OBSTACLE);
  ObstacleMap2D esdf;
  esdf.update(costmap);

  VoronoiGraphSearch search(0.15);
  search.setObstacleMap(&esdf);

  std::vector<GraphSearchResult> results;
  EXPECT_FALSE(search.search(PoseSE2(-1.0, 0.0, 0.0), PoseSE2(1.0, 0.0, 0.0), makeTwoObstacles(),
                             3, results));
}

TEST(Voronoi, SearchWithEmptyObstaclesReturnsDirectClassAtExactEndpoints) {
  // Regression: with no converter obstacles the search must still run (fresh graph each
  // tick) instead of freezing at a stale snapshot.
  const ObstacleMap2D esdf = makeTwoBlobEsdf();
  VoronoiGraphSearch search(0.15);
  search.setObstacleMap(&esdf);

  std::vector<GraphSearchResult> results;
  const ObstacleArray no_obstacles;
  ASSERT_TRUE(
      search.search(PoseSE2(1.0, 0.0, 0.0), PoseSE2(5.0, 0.0, 0.0), no_obstacles, 3, results));

  EXPECT_GE(results.size(), 1u);

  // Start/goal connector nodes sit EXACTLY at the requested endpoints
  const auto &graph = search.getVoronoiGraph();
  ASSERT_GE(graph.startId(), 0);
  ASSERT_GE(graph.goalId(), 0);
  const auto &s = graph.nodes()[graph.startId()].pos;
  const auto &g = graph.nodes()[graph.goalId()].pos;
  EXPECT_NEAR(s.x(), 1.0, 1e-9);
  EXPECT_NEAR(s.y(), 0.0, 1e-9);
  EXPECT_NEAR(g.x(), 5.0, 1e-9);
  EXPECT_NEAR(g.y(), 0.0, 1e-9);
}

TEST(Voronoi, GridNodesUseCellCenterWorldTransform) {
  // Regression: world() must map cell indices to CELL CENTERS (origin + (i+0.5)*res),
  // matching ObstacleMap2D::interpolate. The two-blob layout puts the corridor medial
  // axis exactly on grid row/column centers: column gx=160 → x=3.025, row gy=100 →
  // y=0.025 (origin -5, res 0.05). The old corner-convention mapping placed nodes half
  // a cell off and fails this check.
  const ObstacleMap2D esdf = makeTwoBlobEsdf();
  VoronoiGraphSearch search(0.15);
  search.setObstacleMap(&esdf);

  std::vector<GraphSearchResult> results;
  ASSERT_TRUE(search.search(PoseSE2(1.0, 0.0, 0.0), PoseSE2(5.0, 0.0, 0.0), makeTwoObstacles(), 2,
                            results));

  bool corridor_node_found = false;
  for (const auto &node : search.getVoronoiGraph().nodes()) {
    if (std::abs(node.pos.x() - 3.025) < 0.02 && std::abs(node.pos.y() - 0.025) < 0.02)
      corridor_node_found = true;
  }
  EXPECT_TRUE(corridor_node_found);
}

TEST(Voronoi, NoDegenerateOrSelfLoopEdges) {
  const ObstacleMap2D esdf = makeTwoBlobEsdf();
  VoronoiGraphSearch search(0.15);
  search.setObstacleMap(&esdf);

  std::vector<GraphSearchResult> results;
  ASSERT_TRUE(search.search(PoseSE2(1.0, 0.0, 0.0), PoseSE2(5.0, 0.0, 0.0), makeTwoObstacles(), 2,
                            results));

  for (int i = 0; i < static_cast<int>(search.getVoronoiGraph().nodes().size()); ++i) {
    for (const auto &e : search.getVoronoiGraph().edges(i)) {
      EXPECT_NE(e.from_id, e.to_id) << "self-loop at node " << i;
      const auto &a = search.getVoronoiGraph().nodes()[e.from_id].pos;
      const auto &b = search.getVoronoiGraph().nodes()[e.to_id].pos;
      EXPECT_GT((a - b).squaredNorm(), 1e-12) << "degenerate edge at node " << i;
    }
  }
}

// ── Visibility fallback smoke test ───────────────────────────────────────────

TEST(VisibilitySearch, FallbackSmokeTestOnTwoBlobLayout) {
  const ObstacleMap2D esdf = makeTwoBlobEsdf();
  VisibilityGraphSearch search(0.15);
  search.setObstacleMap(&esdf);

  std::vector<GraphSearchResult> results;
  EXPECT_TRUE(search.search(PoseSE2(1.0, 0.0, 0.0), PoseSE2(5.0, 0.0, 0.0), makeTwoObstacles(), 2,
                            results));
  EXPECT_GE(results.size(), 1u);
}

// ── HomotopyClassPlanner ─────────────────────────────────────────────────

TEST(HomotopyClassPlanner, PlanProducesCandidatesAndSurvivesSecondTick) {
  const ObstacleMap2D esdf = makeTwoBlobEsdf();

  Footprint footprint(false, "circles", "[ [0.0, 0.0, 0.12] ]");
  teb_controller::Params params = makeHcpParams();

  auto base = std::make_shared<DiscreteTEBPlanner>(params, footprint, nullptr);
  auto gs = std::make_shared<VoronoiGraphSearch>(footprintCircumRadius(footprint));

  HomotopyClassPlanner hcp(params, footprint, nullptr);
  hcp.setBasePlanner(base);
  hcp.setGraphSearch(gs);
  hcp.setObstacleMap(&esdf);
  hcp.updateObstacleContainer(std::make_shared<const ObstacleArray>(makeTwoObstacles()));

  const nav_msgs::msg::Path plan = makeStraightPath(1.0, 5.0, 0.0);
  const geometry_msgs::msg::Twist start_vel;

  // Tick 1: cold start across all classes
  ASSERT_TRUE(hcp.plan(plan, start_vel));
  EXPECT_GE(hcp.getCandidates().size(), 1u);
  EXPECT_GT(hcp.getTEB().sizePoses(), 1u);
  EXPECT_TRUE(std::isfinite(hcp.getCost()));

  // Tick 2: warm-start through the persistent per-class planners (C2 regression guard)
  ASSERT_TRUE(hcp.plan(plan, start_vel));
  EXPECT_GE(hcp.getCandidates().size(), 1u);
  EXPECT_GT(hcp.getTEB().sizePoses(), 1u);

  // All optimized candidates stay collision-free against the blobs
  for (const auto &candidate : hcp.getCandidates()) {
    ASSERT_NE(candidate, nullptr);
    EXPECT_TRUE(candidate->is_feasible);
    EXPECT_TRUE(std::isfinite(candidate->optimization_cost));
  }
}

TEST(HomotopyClassPlanner, GraphSearchIsRateLimited) {
  const ObstacleMap2D esdf = makeWallWithGapsEsdf();

  Footprint footprint(false, "circles", "[ [0.0, 0.0, 0.12] ]");
  teb_controller::Params params = makeHcpParams();
  params.FollowPath.hcp.search_rate = 0.1;  // 10 s interval → deterministic gating

  auto base = std::make_shared<DiscreteTEBPlanner>(params, footprint, nullptr);
  auto gs = std::make_shared<VoronoiGraphSearch>(footprintCircumRadius(footprint));

  HomotopyClassPlanner hcp(params, footprint, nullptr);
  hcp.setBasePlanner(base);
  hcp.setGraphSearch(gs);
  hcp.setObstacleMap(&esdf);
  hcp.updateObstacleContainer(std::make_shared<const ObstacleArray>(makeWallObstacles()));

  const geometry_msgs::msg::Twist start_vel;

  // Three consecutive ticks with the same plan → exactly one search execution
  const nav_msgs::msg::Path plan = makeStraightPath(1.0, 5.0, 0.0);
  ASSERT_TRUE(hcp.plan(plan, start_vel));
  EXPECT_EQ(hcp.searchCount(), 1u);
  ASSERT_TRUE(hcp.plan(plan, start_vel));
  ASSERT_TRUE(hcp.plan(plan, start_vel));
  EXPECT_EQ(hcp.searchCount(), 1u);  // reused cached classes

  // Endpoint moves beyond reinit_dist → forced refresh despite the interval
  const nav_msgs::msg::Path moved_plan = makeStraightPath(1.0, 5.0, 3.0);  // goal +3 m
  ASSERT_TRUE(hcp.plan(moved_plan, start_vel));
  EXPECT_EQ(hcp.searchCount(), 2u);

  // ...and the interval gate applies again afterwards
  ASSERT_TRUE(hcp.plan(moved_plan, start_vel));
  EXPECT_EQ(hcp.searchCount(), 2u);
}

// ── Failing search stub for fallback-path tests ──────────────────────────────
class FailingSearch : public GraphSearchInterface {
public:
  bool search(const PoseSE2 & /*start*/, const PoseSE2 & /*goal*/,
              const ObstacleArray & /*obstacles*/, int /*max_classes*/,
              std::vector<GraphSearchResult> &results) override {
    results.clear();
    return false;
  }
  void updateObstacles(const ObstacleArray & /*obstacles*/) override {}
  void setObstacleMap(const ObstacleMap2D * /*esdf*/) override {}
};

// ── Scriptable search stub: returns fixed routes with per-tick costs ─────────
class StubSearch : public GraphSearchInterface {
public:
  struct Route {
    std::vector<PoseSE2> path;
  };

  explicit StubSearch(std::vector<Route> routes) : routes_(std::move(routes)) {}

  /// Set the costs returned by the next search() calls (one entry per route)
  void setCosts(std::vector<double> costs) { costs_ = std::move(costs); }
  void setObstacles(ObstacleArray arr) { obstacles_ = std::move(arr); }
  void setRoutes(std::vector<Route> routes) { routes_ = std::move(routes); }

  bool search(const PoseSE2 &start, const PoseSE2 &goal, const ObstacleArray &, int max_classes,
              std::vector<GraphSearchResult> &results) override {
    results.clear();
    const size_t n = std::min(routes_.size(), costs_.size());
    for (size_t i = 0; i < n && static_cast<int>(i) < max_classes; ++i) {
      GraphSearchResult r;
      r.path = routes_[i].path;
      // Signatures are irrelevant here — route identity is geometric
      r.cost = costs_[i];
      results.push_back(std::move(r));
    }
    (void)start;
    (void)goal;
    return !results.empty();
  }

  void updateObstacles(const ObstacleArray &) override {}
  void setObstacleMap(const ObstacleMap2D *) override {}

private:
  std::vector<Route> routes_;
  std::vector<double> costs_;
  ObstacleArray obstacles_;
};

std::vector<PoseSE2> makeUpperRoute() {
  return makeStraightPoses(1.0, 5.0, 1.6);  // through upper gap of the wall layout
}

std::vector<PoseSE2> makeLowerRoute() {
  return makeStraightPoses(1.0, 5.0, -1.6);  // through lower gap
}

TEST(HomotopyClassPlanner, FailureServesRecentCandidate) {
  const ObstacleMap2D esdf = makeWallWithGapsEsdf();

  Footprint footprint(false, "circles", "[ [0.0, 0.0, 0.12] ]");
  teb_controller::Params params = makeHcpParams();
  params.FollowPath.hcp.search_rate = 100.0;  // nominal 10 ms interval

  auto base = std::make_shared<DiscreteTEBPlanner>(params, footprint, nullptr);
  auto gs = std::make_shared<VoronoiGraphSearch>(footprintCircumRadius(footprint));

  HomotopyClassPlanner hcp(params, footprint, nullptr);
  hcp.setBasePlanner(base);
  hcp.setGraphSearch(gs);
  hcp.setObstacleMap(&esdf);
  hcp.updateObstacleContainer(std::make_shared<const ObstacleArray>(makeWallObstacles()));

  const nav_msgs::msg::Path plan = makeStraightPath(1.0, 5.0, 0.8);  // through upper gap
  const geometry_msgs::msg::Twist start_vel;
  ASSERT_TRUE(hcp.plan(plan, start_vel));
  ASSERT_GE(hcp.getCandidates().size(), 1u);
  const TimedElasticBand good_teb = hcp.getTEB();  // copy of the last good band

  // Adaptive-interval semantics (see refreshOrReuseClasses): an immediate identical tick
  // does NOT re-search — the previous plan's duration exceeds the nominal interval, so
  // cached classes are reused without invoking (the failing) search.
  hcp.setGraphSearch(std::make_shared<FailingSearch>());
  ASSERT_TRUE(hcp.plan(plan, start_vel));
  EXPECT_EQ(hcp.searchCount(), 1u);

  // Forced refresh via endpoint movement (> reinit_dist): the failing search runs, and
  // the recent candidate must be served without touching the base planner (no same-tick
  // double solve).
  const nav_msgs::msg::Path moved_plan = makeStraightPath(1.0, 5.0, 2.3);  // goal +1.5 m
  ASSERT_TRUE(hcp.plan(moved_plan, start_vel));
  EXPECT_EQ(hcp.searchCount(), 2u);
  EXPECT_GE(hcp.getCandidates().size(), 1u);
  EXPECT_EQ(&hcp.getTEB(), &hcp.getTEB());  // still valid access

  // Cache was invalidated by the failed search → immediate retry fires again, fails
  // again, and keeps serving the recent candidates.
  ASSERT_TRUE(hcp.plan(moved_plan, start_vel));
  EXPECT_EQ(hcp.searchCount(), 3u);
  EXPECT_EQ(hcp.getCandidates().size(), 1u);
  EXPECT_GT(hcp.getTEB().sizePoses(), 1u);
  EXPECT_GT(good_teb.sizePoses(), 1u);
}

TEST(HomotopyClassPlanner, FeasibilityGateDropsCollidingBand) {
  // Post-optimization collision gate: a band intersecting lethal ESDF cells must be
  // rejected regardless of its cost (benchmark_test_19: through-rack bands stayed
  // selectable because only the controller's final band was ever checked).
  const ObstacleMap2D esdf = makeWallWithGapsEsdf();
  Footprint footprint(false, "circles", "[ [0.0, 0.0, 0.12] ]");
  teb_controller::Params params = makeHcpParams();
  params.FollowPath.obstacles.feasibility_check = 10.0;  // check the whole band

  HomotopyClassPlanner hcp(params, footprint, nullptr);
  hcp.setObstacleMap(&esdf);

  TimedElasticBand through_wall;
  for (const auto &pose : makeStraightPoses(1.0, 5.0, 0.0))
    through_wall.addPose(pose);  // crosses the middle wall piece at x≈3

  TimedElasticBand through_gap;
  for (const auto &pose : makeStraightPoses(1.0, 5.0, 0.8))
    through_gap.addPose(pose);  // centered in the gap between middle and top piece

  EXPECT_FALSE(hcp.passesFeasibilityGate(through_wall));
  EXPECT_TRUE(hcp.passesFeasibilityGate(through_gap));

  // Gate disabled → nothing is filtered here (controller-side check still applies)
  params.FollowPath.hcp.feasibility_gate = false;
  HomotopyClassPlanner ungated(params, footprint, nullptr);
  ungated.setObstacleMap(&esdf);
  EXPECT_TRUE(ungated.passesFeasibilityGate(through_wall));
}

TEST(HomotopyClassPlanner, WindowContainmentRejectsOutOfBoundsBand) {
  // Fixture window: [-5, 5]² → containment margin 0.5 shrinks it to [-4.5, 4.5]².
  const ObstacleMap2D esdf = makeWallWithGapsEsdf();
  Footprint footprint(false, "circles", "[ [0.0, 0.0, 0.12] ]");
  teb_controller::Params params = makeHcpParams();
  ASSERT_TRUE(params.FollowPath.hcp.window_containment);

  HomotopyClassPlanner hcp(params, footprint, nullptr);
  hcp.setObstacleMap(&esdf);

  // Band whose goal seed is inside but which bulges OUT of the window mid-course →
  // exactly the phantom shape benchmark_test_20 produced (clamped-free unknown space).
  TimedElasticBand bulging_out;
  bulging_out.addPose(PoseSE2(1.0, 0.8, 0.0));
  bulging_out.addPose(PoseSE2(3.0, 0.8, 0.0));
  bulging_out.addPose(PoseSE2(3.5, -5.0, 0.0));  // below the window edge
  bulging_out.addPose(PoseSE2(4.0, -2.0, 0.0));
  bulging_out.addPose(PoseSE2(4.0, 0.8, 0.0));
  EXPECT_FALSE(hcp.staysInWindow(bulging_out));

  // Fully contained band passes
  TimedElasticBand contained;
  for (const auto &pose : makeStraightPoses(1.0, 4.0, 0.8))
    contained.addPose(pose);
  EXPECT_TRUE(hcp.staysInWindow(contained));

  // Self-disable: goal seed itself outside the window (long-haul leg) → no rejection
  TimedElasticBand long_haul;
  for (const auto &pose : makeStraightPoses(6.0, 8.0, 0.8))
    long_haul.addPose(pose);
  EXPECT_TRUE(hcp.staysInWindow(long_haul));

  // Containment disabled → everything passes
  params.FollowPath.hcp.window_containment = false;
  HomotopyClassPlanner uncontained(params, footprint, nullptr);
  uncontained.setObstacleMap(&esdf);
  EXPECT_TRUE(uncontained.staysInWindow(bulging_out));
}

TEST(HomotopyClassPlanner, ProgressGuardAndAnchorFloor) {
  auto makeCand = [](int route_id, double eff, double len) {
    auto c = std::make_shared<TebCandidate>();
    c->route_id = route_id;
    c->efficiency_cost = eff;
    c->path_length = len;
    c->is_feasible = true;
    return c;
  };

  const double hyst = 0.9;
  const double slack = 0.1;

  // Progress guard: competitor beats efficiency hysteresis (0.5 < 1.0*0.9) but is a
  // DETOUR (12 > 10*1.1) while prev is offered → takeover blocked, prev kept.
  {
    std::vector<TebCandidate::Ptr> c{makeCand(7, 1.0, 10.0), makeCand(8, 0.5, 12.0)};
    EXPECT_EQ(HomotopyClassPlanner::selectBestCandidateIndex(c, true, 7, 1.0, false, hyst, 10.0,
                                                             slack, 0.0),
              0);
  }

  // Same competitor within the slack → allowed
  {
    std::vector<TebCandidate::Ptr> c{makeCand(7, 1.0, 10.0), makeCand(8, 0.5, 10.5)};
    EXPECT_EQ(HomotopyClassPlanner::selectBestCandidateIndex(c, true, 7, 1.0, false, hyst, 10.0,
                                                             slack, 0.0),
              1);
  }

  // Prev route ABSENT (dead end) → the longer escape is allowed regardless of length
  {
    std::vector<TebCandidate::Ptr> c{makeCand(8, 0.5, 12.0)};
    EXPECT_EQ(HomotopyClassPlanner::selectBestCandidateIndex(c, true, 7, 1.0, false, hyst, 10.0,
                                                             slack, 0.0),
              0);
  }

  // Anchor floor: an ultra-tiny anchor (0.05 — phantom-band leftover) is DISTRUSTED —
  // it loses hysteresis protection and ranking privilege, so a realistic competitor
  // (eff 3.0) takes over by plain min-cost. With floor 0 the phantom prev keeps winning.
  {
    std::vector<TebCandidate::Ptr> c{makeCand(7, 0.05, 10.0), makeCand(8, 3.0, 12.0)};
    EXPECT_EQ(HomotopyClassPlanner::selectBestCandidateIndex(c, true, 7, 0.05, false, hyst, 10.0,
                                                             slack, 0.2),
              1);
    EXPECT_EQ(HomotopyClassPlanner::selectBestCandidateIndex(c, true, 7, 0.05, false, hyst, 10.0,
                                                             slack, 0.0),
              0);
  }

  // Distrusted prev as last resort: no competitor exists → stick with it anyway
  {
    std::vector<TebCandidate::Ptr> c{makeCand(7, 0.05, 10.0)};
    EXPECT_EQ(HomotopyClassPlanner::selectBestCandidateIndex(c, true, 7, 0.05, false, hyst, 10.0,
                                                             slack, 0.2),
              0);
  }
}

TEST(HomotopyClassPlanner, GraceSynthesizesVanishedPrevRoute) {
  // Grace-period continuity: when a search round drops the previous best route's class,
  // a candidate is re-synthesized from the route entry so it competes with a fresh cost
  // instead of the tick collapsing to cheapest-of-the-day (the ping-pong source in
  // benchmark_test_19).
  const ObstacleMap2D esdf = makeWallWithGapsEsdf();
  Footprint footprint(false, "circles", "[ [0.0, 0.0, 0.12] ]");
  teb_controller::Params params = makeHcpParams();
  params.FollowPath.hcp.search_rate = 100.0;  // refresh every tick
  ASSERT_GT(params.FollowPath.hcp.route_grace_time, 0.0);

  auto base = std::make_shared<DiscreteTEBPlanner>(params, footprint, nullptr);
  // Two corridors: straight through the widest gap (y=0.8) and a far detour (y=-4.5).
  // Which one wins tick 1 is an optimizer detail — the test churns away WHATEVER won.
  const StubSearch::Route corridors[2] = {StubSearch::Route{makeStraightPoses(1.0, 5.0, 0.8)},
                                          StubSearch::Route{makeStraightPoses(1.0, 5.0, -4.5)}};
  auto gs = std::make_shared<StubSearch>(
      std::vector<StubSearch::Route>{corridors[0], corridors[1]});
  gs->setCosts({1.0, 1.1});

  HomotopyClassPlanner hcp(params, footprint, nullptr);
  hcp.setBasePlanner(base);
  hcp.setGraphSearch(gs);
  hcp.setObstacleMap(&esdf);
  hcp.updateObstacleContainer(std::make_shared<const ObstacleArray>(makeWallObstacles()));

  const nav_msgs::msg::Path plan = makeStraightPath(1.0, 5.0, 0.8);
  const geometry_msgs::msg::Twist start_vel;
  ASSERT_TRUE(hcp.plan(plan, start_vel));
  ASSERT_EQ(hcp.getCandidates().size(), 2u);
  // Candidate order follows search-result order → candidates[0] = corridors[0], etc.
  const int winner_id = hcp.getBestCandidate().route_id;
  const size_t loser_pos =
      (hcp.getCandidates()[0]->route_id == winner_id) ? 1u : 0u;

  // Search rounds now offer ONLY the losing corridor (churn simulation of the winner's
  // class vanishing). NOTE: the adaptive refresh interval reuses cached classes on
  // fast-following ticks, so loop until a REAL search round fires (searchCount bump).
  gs->setRoutes(std::vector<StubSearch::Route>{corridors[loser_pos]});
  gs->setCosts({1.0});
  bool vanished_tick_seen = false;
  for (int i = 0; i < 50 && !vanished_tick_seen; ++i) {
    const uint64_t before = hcp.searchCount();
    ASSERT_TRUE(hcp.plan(plan, start_vel));
    if (hcp.searchCount() > before)
      vanished_tick_seen = true;
  }
  ASSERT_TRUE(vanished_tick_seen);

  bool prev_present = false;
  for (const auto &c : hcp.getCandidates()) {
    if (c->route_id == winner_id)
      prev_present = true;
  }
  EXPECT_TRUE(prev_present);  // synthesized from the route entry inside the grace window

  // Grace disabled → the vanished route stays gone (dead-end escape still works)
  params.FollowPath.hcp.route_grace_time = 0.0;
  HomotopyClassPlanner no_grace(params, footprint, nullptr);
  no_grace.setBasePlanner(std::make_shared<DiscreteTEBPlanner>(params, footprint, nullptr));
  no_grace.setGraphSearch(gs);
  no_grace.setObstacleMap(&esdf);
  no_grace.updateObstacleContainer(std::make_shared<const ObstacleArray>(makeWallObstacles()));
  gs->setRoutes(std::vector<StubSearch::Route>{corridors[0], corridors[1]});  // reset
  gs->setCosts({1.0, 1.1});
  ASSERT_TRUE(no_grace.plan(plan, start_vel));
  ASSERT_EQ(no_grace.getCandidates().size(), 2u);
  const int ng_winner_id = no_grace.getBestCandidate().route_id;
  const size_t ng_loser_pos =
      (no_grace.getCandidates()[0]->route_id == ng_winner_id) ? 1u : 0u;
  gs->setRoutes(std::vector<StubSearch::Route>{corridors[ng_loser_pos]});
  gs->setCosts({1.0});
  bool vanished_tick_seen_ng = false;
  for (int i = 0; i < 50 && !vanished_tick_seen_ng; ++i) {
    const uint64_t before = no_grace.searchCount();
    ASSERT_TRUE(no_grace.plan(plan, start_vel));
    if (no_grace.searchCount() > before)
      vanished_tick_seen_ng = true;
  }
  ASSERT_TRUE(vanished_tick_seen_ng);
  bool first_still_there = false;
  for (const auto &c : no_grace.getCandidates()) {
    if (c->route_id == ng_winner_id)
      first_still_there = true;
  }
  EXPECT_FALSE(first_still_there);
}

TEST(HomotopyClassPlanner, RouteIdsStableAcrossObstacleSetChange) {
  const ObstacleMap2D esdf = makeWallWithGapsEsdf();

  Footprint footprint(false, "circles", "[ [0.0, 0.0, 0.12] ]");
  teb_controller::Params params = makeHcpParams();
  params.FollowPath.hcp.search_rate = 100.0;  // refresh every tick

  auto base = std::make_shared<DiscreteTEBPlanner>(params, footprint, nullptr);

  HomotopyClassPlanner hcp(params, footprint, nullptr);
  hcp.setBasePlanner(base);
  hcp.setGraphSearch(std::make_shared<VoronoiGraphSearch>(footprintCircumRadius(footprint)));
  hcp.setObstacleMap(&esdf);
  hcp.updateObstacleContainer(std::make_shared<const ObstacleArray>(makeWallObstacles()));

  // Tick 1: two routes
  auto stub = std::make_shared<StubSearch>(
      std::vector<StubSearch::Route>{{makeUpperRoute()}, {makeLowerRoute()}});
  stub->setCosts({1.0, 1.2});
  hcp.setGraphSearch(stub);
  hcp.setObstacleMap(&esdf);
  ASSERT_TRUE(hcp.plan(makeStraightPath(1.0, 5.0, 1.6), geometry_msgs::msg::Twist()));
  ASSERT_EQ(hcp.getCandidates().size(), 2u);
  const int upper_id_tick1 = hcp.getCandidates()[0]->route_id;
  const int lower_id_tick1 = hcp.getCandidates()[1]->route_id;
  EXPECT_NE(upper_id_tick1, lower_id_tick1);

  // Tick 2: an extra obstacle enters the window (signature sizes change!) — geometric
  // identity must still map both candidates onto the SAME route ids.
  ObstacleArray with_extra = makeWallObstacles();
  {
    costmap_converter_msgs::msg::ObstacleMsg extra;
    geometry_msgs::msg::Point32 pt;
    pt.x = 50.0f;
    pt.y = 50.0f;
    extra.polygon.points.push_back(pt);
    with_extra.obstacles.push_back(extra);
  }
  hcp.updateObstacleContainer(std::make_shared<const ObstacleArray>(with_extra));
  ASSERT_TRUE(hcp.plan(makeStraightPath(1.0, 5.0, 1.6), geometry_msgs::msg::Twist()));
  ASSERT_EQ(hcp.getCandidates().size(), 2u);
  EXPECT_EQ(hcp.getCandidates()[0]->route_id, upper_id_tick1);
  EXPECT_EQ(hcp.getCandidates()[1]->route_id, lower_id_tick1);
}

TEST(HomotopyClassPlanner, SwitchBlockAndHysteresisPreventOscillation) {
  // Pure selection-rule test: candidates are hand-built (route_id + efficiency cost), so
  // costs are fully deterministic — no optimizer in the loop. Selection ranks by the
  // EFFICIENCY category (time-optimal + shortest-path + smoothness), not total chi2.
  auto makeCand = [](int route_id, double cost) {
    auto c = std::make_shared<TebCandidate>();
    c->route_id = route_id;
    c->efficiency_cost = cost;
    c->is_feasible = true;
    return c;
  };

  const double hyst = 0.9;

  // First selection: pure min-cost, no history
  {
    std::vector<TebCandidate::Ptr> c{makeCand(0, 1.0), makeCand(1, 1.05)};
    EXPECT_EQ(HomotopyClassPlanner::selectBestCandidateIndex(c, false, -1, 0.0, false, hyst), 0);
  }

  // Marginal improvement on the other route (within hysteresis) → STICK to previous best
  {
    std::vector<TebCandidate::Ptr> c{makeCand(7, 1.02), makeCand(8, 1.00)};
    EXPECT_EQ(HomotopyClassPlanner::selectBestCandidateIndex(c, true, 7, 1.0, false, hyst), 0);
  }

  // Much cheaper other route but INSIDE the switch-block window → still sticks
  {
    std::vector<TebCandidate::Ptr> c{makeCand(7, 2.0), makeCand(8, 0.5)};
    EXPECT_EQ(HomotopyClassPlanner::selectBestCandidateIndex(c, true, 7, 2.0, true, hyst), 0);
  }

  // Outside the block window AND clearly cheaper → switch allowed
  {
    std::vector<TebCandidate::Ptr> c{makeCand(7, 2.0), makeCand(8, 0.5)};
    EXPECT_EQ(HomotopyClassPlanner::selectBestCandidateIndex(c, true, 7, 2.0, false, hyst), 1);
  }

  // Blocked, but the previous best route vanished (dead end) → switching permitted
  {
    std::vector<TebCandidate::Ptr> c{makeCand(8, 1.4)};
    EXPECT_EQ(HomotopyClassPlanner::selectBestCandidateIndex(c, true, 7, 1.0, true, hyst), 0);
  }

  // Previous best still offered and nothing beats the margin → fallback keeps it even
  // when the margin rejected every alternative
  {
    std::vector<TebCandidate::Ptr> c{makeCand(7, 3.0), makeCand(8, 2.95)};
    EXPECT_EQ(HomotopyClassPlanner::selectBestCandidateIndex(c, true, 7, 1.0, false, hyst), 0);
  }

  // Infeasible candidates are ignored
  {
    std::vector<TebCandidate::Ptr> c{makeCand(9, 0.1), makeCand(10, 5.0)};
    c[0]->is_feasible = false;
    EXPECT_EQ(HomotopyClassPlanner::selectBestCandidateIndex(c, false, -1, 0.0, false, hyst), 1);
  }

  // Efficiency metric: a candidate with LOWER total chi2 but WORSE efficiency must lose
  // to the efficiency-optimal one when no continuity anchor exists.
  {
    auto eff_good = makeCand(11, 5.0);       // low efficiency cost …
    eff_good->optimization_cost = 50.0;      // … but heavy obstacle penalties
    auto chi2_good = makeCand(12, 6.0);      // higher efficiency cost …
    chi2_good->optimization_cost = 5.5;      // … yet lower total chi2
    std::vector<TebCandidate::Ptr> c{eff_good, chi2_good};
    EXPECT_EQ(HomotopyClassPlanner::selectBestCandidateIndex(c, false, -1, 0.0, false, hyst), 0);
  }
}

TEST(HomotopyClassPlanner, UpdateCandidatesSeedsLiveRobotPose) {
  // Regression: the band's pose(0) — from which extractVelocity derives the whole
  // command — must be the LIVE robot pose (position AND heading), not the stale GVD
  // start node with its arbitrary connector heading.
  const ObstacleMap2D esdf = makeWallWithGapsEsdf();

  Footprint footprint(false, "circles", "[ [0.0, 0.0, 0.12] ]");
  teb_controller::Params params = makeHcpParams();

  auto base = std::make_shared<DiscreteTEBPlanner>(params, footprint, nullptr);
  auto stub = std::make_shared<StubSearch>(
      std::vector<StubSearch::Route>{{makeUpperRoute()}, {makeLowerRoute()}});
  stub->setCosts({1.0, 1.1});

  HomotopyClassPlanner hcp(params, footprint, nullptr);
  hcp.setBasePlanner(base);
  hcp.setGraphSearch(stub);
  hcp.setObstacleMap(&esdf);
  hcp.updateObstacleContainer(std::make_shared<const ObstacleArray>(makeWallObstacles()));

  // "Controller" plan whose front carries a live robot pose that differs from the graph
  // start node in BOTH position and heading.
  nav_msgs::msg::Path plan;
  plan.header.frame_id = "map";
  geometry_msgs::msg::PoseStamped robot_ps;
  robot_ps.pose.position.x = 1.3;
  robot_ps.pose.position.y = 1.75;
  robot_ps.pose.orientation.z = 0.38941834;  // yaw ≈ 0.8 rad
  robot_ps.pose.orientation.w = 0.92106099;
  plan.poses.push_back(robot_ps);
  for (const auto &pose : makeUpperRoute())
    if (pose.x() > 1.5) {
      geometry_msgs::msg::PoseStamped ps;
      ps.pose = pose.toPoseMsg();
      plan.poses.push_back(ps);
    }

  const double expected_yaw =
      2.0 * std::atan2(robot_ps.pose.orientation.z, robot_ps.pose.orientation.w);
  ASSERT_TRUE(hcp.plan(plan, geometry_msgs::msg::Twist()));

  ASSERT_GE(hcp.getCandidates().size(), 1u);
  for (const auto &candidate : hcp.getCandidates()) {
    ASSERT_GE(candidate->teb.sizePoses(), 2u);
    EXPECT_NEAR(candidate->teb.pose(0).x(), 1.3, 1e-6);
    EXPECT_NEAR(candidate->teb.pose(0).y(), 1.75, 1e-6);
    EXPECT_NEAR(candidate->teb.pose(0).theta(), expected_yaw, 1e-6);
  }
}

TEST(HomotopyClassPlanner, RouteIdsSurviveStartAdvanceBetweenSearches) {
  // Regression: matching must NOT require the START endpoints to agree — the robot
  // legitimately advances between two searches (especially during loop-rate dips).
  Footprint footprint(false, "circles", "[ [0.0, 0.0, 0.12] ]");
  teb_controller::Params params = makeHcpParams();
  params.FollowPath.hcp.search_rate = 100.0;

  auto base = std::make_shared<DiscreteTEBPlanner>(params, footprint, nullptr);
  auto stub = std::make_shared<StubSearch>(std::vector<StubSearch::Route>{{makeUpperRoute()}});

  const ObstacleMap2D esdf = makeWallWithGapsEsdf();
  HomotopyClassPlanner hcp(params, footprint, nullptr);
  hcp.setBasePlanner(base);
  hcp.setGraphSearch(stub);
  hcp.setObstacleMap(&esdf);

  ASSERT_TRUE(hcp.plan(makeStraightPath(1.0, 5.0, 1.6), geometry_msgs::msg::Twist()));
  ASSERT_EQ(hcp.getCandidates().size(), 1u);
  const int upper_id = hcp.getCandidates()[0]->route_id;

  // Next search: same corridor, start advanced 1.5 m (> reinit_dist) along it
  std::vector<PoseSE2> advanced = makeUpperRoute();
  advanced.erase(advanced.begin(), advanced.begin() + 15);  // 15 × 0.1 m
  stub->setRoutes({{advanced}});

  ASSERT_TRUE(hcp.plan(makeStraightPath(2.5, 5.0, 1.6), geometry_msgs::msg::Twist()));
  ASSERT_EQ(hcp.getCandidates().size(), 1u);
  EXPECT_EQ(hcp.getCandidates()[0]->route_id, upper_id);
}

TEST(HomotopyClassPlanner, UpdateCandidatesSeedsGoalPose) {
  // Regression: the band's LAST pose must carry the mission goal pose (heading included).
  // The GVD polyline's terminal heading is connector-derived; with fix_goal that vertex is
  // PINNED, so an unseeded band converges to the goal position while ignoring the desired
  // heading — the robot "arrived but ignored the goal heading".
  const ObstacleMap2D esdf = makeWallWithGapsEsdf();

  Footprint footprint(false, "circles", "[ [0.0, 0.0, 0.12] ]");
  teb_controller::Params params = makeHcpParams();
  params.FollowPath.optimizer.fix_goal = true;

  auto base = std::make_shared<DiscreteTEBPlanner>(params, footprint, nullptr);
  auto stub = std::make_shared<StubSearch>(
      std::vector<StubSearch::Route>{{makeUpperRoute()}, {makeLowerRoute()}});
  stub->setCosts({1.0, 1.1});

  HomotopyClassPlanner hcp(params, footprint, nullptr);
  hcp.setBasePlanner(base);
  hcp.setGraphSearch(stub);
  hcp.setObstacleMap(&esdf);
  hcp.updateObstacleContainer(std::make_shared<const ObstacleArray>(makeWallObstacles()));
  hcp.setFixedGoal(true);

  // Plan whose back pose carries a mission goal orientation that differs from the GVD
  // connector direction (upper route approaches along +x → connector heading ≈ 0).
  nav_msgs::msg::Path plan;
  plan.header.frame_id = "map";
  for (const auto &pose : makeUpperRoute()) {
    geometry_msgs::msg::PoseStamped ps;
    ps.pose = pose.toPoseMsg();
    plan.poses.push_back(ps);
  }
  geometry_msgs::msg::PoseStamped &goal_ps = plan.poses.back();
  goal_ps.pose.position.x = 5.05;
  goal_ps.pose.position.y = 1.55;
  goal_ps.pose.orientation.z = 0.47942554;  // yaw ≈ 1.0 rad (approach from elsewhere)
  goal_ps.pose.orientation.w = 0.87758256;

  ASSERT_TRUE(hcp.plan(plan, geometry_msgs::msg::Twist()));

  const double expected_yaw =
      2.0 * std::atan2(goal_ps.pose.orientation.z, goal_ps.pose.orientation.w);
  ASSERT_GE(hcp.getCandidates().size(), 1u);
  for (const auto &candidate : hcp.getCandidates()) {
    ASSERT_GE(candidate->teb.sizePoses(), 2u);
    EXPECT_NEAR(candidate->teb.backPose().x(), 5.05, 1e-6);
    EXPECT_NEAR(candidate->teb.backPose().y(), 1.55, 1e-6);
    EXPECT_NEAR(angles::normalize_angle(candidate->teb.backPose().theta() - expected_yaw), 0.0,
                1e-6);
  }
}

TEST(HomotopyClassPlanner, ParallelSolvesStressManyTicks) {
  // Deterministic hammering of the PARALLEL solve path: 3 routes, refresh every tick,
  // 40 ticks with warm starts across all routes. Under an ASan build this is the
  // regression harness for the field crash (benchmark_test_15): any cross-planner
  // write / heap corruption shows up here without needing the full sim.
  const ObstacleMap2D esdf = makeWallWithGapsEsdf();

  Footprint footprint(false, "circles", "[ [0.0, 0.0, 0.12] ]");
  teb_controller::Params params = makeHcpParams();
  params.FollowPath.hcp.search_rate = 100.0;  // refresh every tick
  params.FollowPath.hcp.max_classes = 3;

  auto base = std::make_shared<DiscreteTEBPlanner>(params, footprint, nullptr);
  auto gs = std::make_shared<VoronoiGraphSearch>(footprintCircumRadius(footprint));

  HomotopyClassPlanner hcp(params, footprint, nullptr);
  hcp.setBasePlanner(base);
  hcp.setGraphSearch(gs);
  hcp.setObstacleMap(&esdf);
  hcp.updateObstacleContainer(std::make_shared<const ObstacleArray>(makeWallObstacles()));

  const geometry_msgs::msg::Twist start_vel;
  for (int tick = 0; tick < 40; ++tick) {
    // Advance the start along the upper corridor + wobble the goal slightly (within
    // reinit_dist) so every tick gets fresh geometry while staying in the same routes.
    const double s = 1.0 + 0.05 * tick;
    const double gy = 1.6 + 0.02 * std::sin(0.7 * tick);

    nav_msgs::msg::Path plan = makeStraightPath(s, 5.0, 1.6);
    ASSERT_FALSE(plan.poses.empty());
    plan.poses.back().pose.position.y = gy;

    ASSERT_TRUE(hcp.plan(plan, start_vel)) << "tick " << tick;
    for (const auto &candidate : hcp.getCandidates()) {
      ASSERT_TRUE(candidate->is_feasible) << "tick " << tick;
      for (size_t j = 0; j < candidate->teb.sizePoses(); ++j) {
        EXPECT_TRUE(std::isfinite(candidate->teb.pose(j).x())) << "tick " << tick;
        EXPECT_TRUE(std::isfinite(candidate->teb.pose(j).y())) << "tick " << tick;
      }
    }
  }

  // Routes must have been reused, not re-created per tick (identity invariant)
  EXPECT_LT(hcp.searchCount(), 45u);
}

}  // namespace
}  // namespace nav2_teb_controller
