#include "nav2_teb_controller/homotopy/voronoi_graph.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <queue>
#include <rclcpp/rclcpp.hpp>

namespace nav2_teb_controller {

namespace {
constexpr double kLargeDist = 1e6;  ///< Out-of-bounds distance: suppresses border ridges

struct Pixel {
  int x;
  int y;
};
}  // namespace

void VoronoiGraph::build(const ObstacleMap2D &esdf, const Eigen::Vector2d &start,
                         const Eigen::Vector2d &goal, double min_clearance) {
  nodes_.clear();
  adj_.clear();
  start_id_ = -1;
  goal_id_ = -1;
  esdf_ = &esdf;

  if (!esdf.isInitialized() || min_clearance <= 0.0)
    return;

  res_ = esdf.resolution();
  const int nx = static_cast<int>(esdf.sizeX());
  const int ny = static_cast<int>(esdf.sizeY());
  const auto &grid = esdf.rawGrid();
  const auto idx = [nx](int x, int y) { return static_cast<size_t>(y) * nx + x; };
  const auto inBounds = [&nx, &ny](int x, int y) { return x >= 0 && y >= 0 && x < nx && y < ny; };
  const auto cellDist = [&](int x, int y) {
    return inBounds(x, y) ? static_cast<double>(grid[idx(x, y)]) : 0.0;
  };
  const auto world = [&](int x, int y) {
    // Cell-center world coordinates — matches ObstacleMap2D::interpolate's convention
    // (corner origin, centers at origin + (i + 0.5) * res)
    return Eigen::Vector2d(esdf.originX() + (x + 0.5) * res_, esdf.originY() + (y + 0.5) * res_);
  };

  // Node ids MUST equal their index in nodes_ — adjacency lookups and the shared graph
  // algorithms (graph.nodes()[edge.to_id]) rely on it. Steiner nodes pushed by addEdge()
  // also size-index, so deriving id from nodes_.size() here keeps both in lockstep.
  auto addNode = [&](const Eigen::Vector2d &pos) {
    const int id = static_cast<int>(nodes_.size());
    nodes_.push_back({pos, id, false, false});
    adj_.emplace_back();
    return id;
  };

  // ── Step 1: ridge extraction (medial-axis approximation) ────────────────
  // A cell belongs to the GVD if it keeps min_clearance from all obstacles AND is a local
  // maximum across at least one axis-aligned neighbor pair.
  std::vector<uint8_t> mask(static_cast<size_t>(nx) * ny, 0);
  for (int y = 0; y < ny; ++y) {
    for (int x = 0; x < nx; ++x) {
      const double d = cellDist(x, y);
      if (d < min_clearance)
        continue;
      const bool h = d >= cellDist(x - 1, y) && d >= cellDist(x + 1, y);
      const bool v = d >= cellDist(x, y - 1) && d >= cellDist(x, y + 1);
      if (h || v)
        mask[idx(x, y)] = 1;
    }
  }

  auto degree8 = [&](int x, int y) {
    int deg = 0;
    for (int dy = -1; dy <= 1; ++dy)
      for (int dx = -1; dx <= 1; ++dx) {
        if ((dx == 0 && dy == 0) || !inBounds(x + dx, y + dy))
          continue;
        deg += mask[idx(x + dx, y + dy)];
      }
    return deg;
  };

  // ── Step 2: bounded spur pruning (removes skeleton noise tips) ──────────
  for (int pass = 0; pass < kSpurPruneIterations; ++pass) {
    auto next = mask;
    bool changed = false;
    for (int y = 0; y < ny; ++y)
      for (int x = 0; x < nx; ++x)
        if (mask[idx(x, y)] && degree8(x, y) <= 1) {
          next[idx(x, y)] = 0;
          changed = true;
        }
    mask.swap(next);
    if (!changed)
      break;
  }

  // ── Step 3: connected components (8-connectivity) ───────────────────────
  const int n_cells = nx * ny;
  std::vector<int> comp(static_cast<size_t>(n_cells), -1);
  std::vector<std::vector<Pixel>> comp_pixels;
  for (int y = 0; y < ny; ++y)
    for (int x = 0; x < nx; ++x) {
      if (!mask[idx(x, y)] || comp[idx(x, y)] != -1)
        continue;
      const int c = static_cast<int>(comp_pixels.size());
      comp_pixels.emplace_back();
      std::queue<Pixel> q;
      q.push({x, y});
      comp[idx(x, y)] = c;
      while (!q.empty()) {
        const auto [px, py] = q.front();
        q.pop();
        comp_pixels[c].push_back({px, py});
        for (int dy = -1; dy <= 1; ++dy)
          for (int dx = -1; dx <= 1; ++dx) {
            const int nxp = px + dx;
            const int nyp = py + dy;
            if (!inBounds(nxp, nyp) || !mask[idx(nxp, nyp)] || comp[idx(nxp, nyp)] != -1)
              continue;
            comp[idx(nxp, nyp)] = c;
            q.push({nxp, nyp});
          }
      }
    }

  // pixel → representative node id (-1 = plain chain pixel)
  std::vector<int> pixel_node(static_cast<size_t>(n_cells), -1);
  std::vector<uint8_t> visited(static_cast<size_t>(n_cells), 0);
  // Junction-cluster bookkeeping — persists across components (components partition the
  // mask, so each junction pixel is visited exactly once; no per-component reset needed).
  std::vector<uint8_t> junction_seen(static_cast<size_t>(n_cells), 0);
  const auto stepCost = [&](const Pixel &a, const Pixel &b) {
    return (a.x != b.x && a.y != b.y) ? res_ * std::numbers::sqrt2 : res_;
  };

  for (const auto &pixels : comp_pixels) {
    if (pixels.empty())
      continue;

    std::vector<Pixel> junctions;
    std::vector<Pixel> endpoints;
    for (const auto &p : pixels) {
      const int deg = degree8(p.x, p.y);
      if (deg >= 3)
        junctions.push_back(p);
      else if (deg == 1)
        endpoints.push_back(p);
    }

    if (junctions.empty() && endpoints.empty()) {
      // ── Closed loop without junctions (isolated blob): split into two arcs so both
      //    directions around the obstacle remain representable. Midpoint Steiner nodes keep
      //    the two arcs distinguishable for Yen's algorithm (no parallel edges).
      const Pixel &p0 = pixels.front();
      const int id_a = addNode(world(p0.x, p0.y));
      pixel_node[idx(p0.x, p0.y)] = id_a;

      // Trace the full ring once collecting ordered pixels.
      std::vector<Pixel> ring;
      {
        Pixel prev = p0;
        Pixel cur = p0;
        bool started = false;
        for (int dy = -1; dy <= 1 && !started; ++dy)
          for (int dx = -1; dx <= 1 && !started; ++dx) {
            if ((dx == 0 && dy == 0) || !inBounds(p0.x + dx, p0.y + dy))
              continue;
            const Pixel np{p0.x + dx, p0.y + dy};
            if (mask[idx(np.x, np.y)]) {
              cur = np;
              started = true;
            }
          }
        if (started) {
          ring.push_back(cur);
          visited[idx(cur.x, cur.y)] = 1;
          while (!(cur.x == p0.x && cur.y == p0.y)) {
            Pixel next{-1, -1};
            for (int dy = -1; dy <= 1; ++dy)
              for (int dx = -1; dx <= 1; ++dx) {
                if ((dx == 0 && dy == 0) || !inBounds(cur.x + dx, cur.y + dy))
                  continue;
                const Pixel np{cur.x + dx, cur.y + dy};
                if (!mask[idx(np.x, np.y)] || (np.x == prev.x && np.y == prev.y))
                  continue;
                next = np;
              }
            if (next.x < 0)
              break;
            prev = cur;
            cur = next;
            if (cur.x == p0.x && cur.y == p0.y)
              break;
            ring.push_back(cur);
            visited[idx(cur.x, cur.y)] = 1;
          }
        }
      }
      if (ring.size() < 2)
        continue;

      // Half-perimeter split point
      double total = 0.0;
      std::vector<double> cum;
      cum.reserve(ring.size());
      Pixel prev_p = p0;
      for (const auto &p : ring) {
        total += stepCost(prev_p, p);
        cum.push_back(total);
        prev_p = p;
      }
      total += stepCost(prev_p, p0);  // closing step back to p0

      const double half = 0.5 * total;
      const size_t split_it = std::lower_bound(cum.begin(), cum.end(), half) - cum.begin();
      const Pixel &pb = ring[std::min(split_it, ring.size() - 1)];
      const int id_b = addNode(world(pb.x, pb.y));
      pixel_node[idx(pb.x, pb.y)] = id_b;

      // Quarter-point Steiner nodes keep the two arcs distinct (no parallel A-B edge)
      const double quarter = 0.25 * total;
      const size_t q1_it = std::lower_bound(cum.begin(), cum.end(), quarter) - cum.begin();
      const Pixel &mq1 = ring[std::min(q1_it, ring.size() - 1)];
      const int id_m1 = addNode(world(mq1.x, mq1.y));
      pixel_node[idx(mq1.x, mq1.y)] = id_m1;
      const size_t q3_it = std::lower_bound(cum.begin(), cum.end(), 3.0 * quarter) - cum.begin();
      const Pixel &mq3 = ring[std::min(q3_it, ring.size() - 1)];
      const int id_m2 = addNode(world(mq3.x, mq3.y));
      pixel_node[idx(mq3.x, mq3.y)] = id_m2;

      addEdge(id_m1, id_b, std::max(0.5 * total - cum[std::min(q1_it, cum.size() - 1)], res_));
      addEdge(id_b, id_m2, std::max(3.0 * quarter - 0.5 * total, res_));
      addEdge(id_m2, id_a, std::max(total - 3.0 * quarter, res_));
      addEdge(id_a, id_m1, std::max(cum[std::min(q1_it, cum.size() - 1)], res_));
      continue;
    }

    // ── Junction clusters → one node per 8-connected cluster of junction pixels ──
    // Mask-based flood fill straight off the grid: O(junction pixels) total. The previous
    // implementation scanned the full junction list per visited pixel (O(J²)) and
    // allocated a full-map buffer per component — the dominant GVD-build cost on
    // rack-dense maps (benchmark_test_18: 275 ms avg / 563 ms max).
    for (const auto &seed : junctions) {
      const size_t seed_idx = idx(seed.x, seed.y);
      if (junction_seen[seed_idx])
        continue;

      Eigen::Vector2d acc(0.0, 0.0);
      int count = 0;
      std::queue<Pixel> q;
      q.push(seed);
      junction_seen[seed_idx] = 1;
      std::vector<Pixel> members;
      while (!q.empty()) {
        const Pixel cur = q.front();
        q.pop();
        members.push_back(cur);
        acc += world(cur.x, cur.y);
        ++count;
        for (int dy = -1; dy <= 1; ++dy)
          for (int dx = -1; dx <= 1; ++dx) {
            if ((dx == 0 && dy == 0) || !inBounds(cur.x + dx, cur.y + dy))
              continue;
            const Pixel np{cur.x + dx, cur.y + dy};
            const size_t nidx = idx(np.x, np.y);
            if (junction_seen[nidx] || !mask[nidx] || degree8(np.x, np.y) < 3)
              continue;
            junction_seen[nidx] = 1;
            q.push(np);
          }
      }

      const int id = addNode(acc / static_cast<double>(count));
      for (const auto &m : members)
        pixel_node[idx(m.x, m.y)] = id;
    }

    // ── Endpoints → one node each ──
    for (const auto &p : endpoints) {
      if (pixel_node[idx(p.x, p.y)] != -1)
        continue;
      pixel_node[idx(p.x, p.y)] = addNode(world(p.x, p.y));
    }

    // ── Chains: walk from every node-cluster boundary into degree-2 territory ──
    for (const auto &p : pixels) {
      const int u = pixel_node[idx(p.x, p.y)];
      if (u == -1)
        continue;
      for (int dy = -1; dy <= 1; ++dy)
        for (int dx = -1; dx <= 1; ++dx) {
          if ((dx == 0 && dy == 0) || !inBounds(p.x + dx, p.y + dy))
            continue;
          const Pixel np{p.x + dx, p.y + dy};
          if (!mask[idx(np.x, np.y)] || visited[idx(np.x, np.y)])
            continue;
          const int v_direct = pixel_node[idx(np.x, np.y)];
          if (v_direct != -1) {
            if (v_direct != u)
              addEdge(u, v_direct, stepCost(p, np));  // adjacent clusters/endpoints
            continue;
          }
          // Walk the chain until the next node pixel or dead end
          double len = stepCost(p, np);
          Pixel prev_px = p;
          Pixel cur = np;
          visited[idx(cur.x, cur.y)] = 1;
          int v = -1;
          while (true) {
            if (pixel_node[idx(cur.x, cur.y)] != -1) {
              v = pixel_node[idx(cur.x, cur.y)];
              break;
            }
            Pixel next{-1, -1};
            for (int wy = -1; wy <= 1; ++wy)
              for (int wx = -1; wx <= 1; ++wx) {
                if ((wx == 0 && wy == 0) || !inBounds(cur.x + wx, cur.y + wy))
                  continue;
                const Pixel nn{cur.x + wx, cur.y + wy};
                if (!mask[idx(nn.x, nn.y)])
                  continue;
                if (nn.x == prev_px.x && nn.y == prev_px.y)
                  continue;
                if (visited[idx(nn.x, nn.y)] && pixel_node[idx(nn.x, nn.y)] == -1)
                  continue;
                next = nn;
              }
            if (next.x < 0)
              break;  // dead end (pruned stub) — drop the partial chain
            len += stepCost(cur, next);
            prev_px = cur;
            cur = next;
            visited[idx(cur.x, cur.y)] = 1;
          }
          if (v != -1 && v != u)
            addEdge(u, v, len);
        }
    }
  }

  // ── Step 4: start / goal connectors ─────────────────────────────────────
  start_id_ = addNode(start);
  goal_id_ = addNode(goal);
  nodes_[start_id_].is_start = true;
  nodes_[goal_id_].is_goal = true;

  if (segmentClear(start, goal, min_clearance)) {
    addEdge(start_id_, goal_id_, (start - goal).norm());
  } else {
    const int s_anchor = nearestVisibleNode(start, 0.5 * min_clearance);
    if (s_anchor >= 0)
      addEdge(start_id_, s_anchor, (start - nodes_[s_anchor].pos).norm());
    const int g_anchor = nearestVisibleNode(goal, 0.5 * min_clearance);
    if (g_anchor >= 0)
      addEdge(goal_id_, g_anchor, (goal - nodes_[g_anchor].pos).norm());
  }

  size_t edge_count = 0;
  for (const auto &a : adj_)
    edge_count += a.size();
  RCLCPP_DEBUG(rclcpp::get_logger("VoronoiGraph"),
               "Built GVD graph: %zu nodes, %zu edges (undirected)", nodes_.size(),
               edge_count / 2);
}

bool VoronoiGraph::segmentClear(const Eigen::Vector2d &a, const Eigen::Vector2d &b,
                                double clearance) const {
  if (!esdf_)
    return false;
  const double len = (b - a).norm();
  const int n_samples = std::max(2, static_cast<int>(std::ceil(len / (0.5 * res_))));
  for (int i = 0; i <= n_samples; ++i) {
    const double t = static_cast<double>(i) / n_samples;
    const Eigen::Vector2d p = a + t * (b - a);
    if (esdf_->queryDistance(p.x(), p.y()) < clearance)
      return false;
  }
  return true;
}

int VoronoiGraph::nearestVisibleNode(const Eigen::Vector2d &p, double clearance) const {
  int best = -1;
  double best_dist = std::numeric_limits<double>::infinity();
  for (const auto &node : nodes_) {
    if (node.is_start || node.is_goal)
      continue;
    const double d = (node.pos - p).norm();
    if (d >= best_dist)
      continue;
    if (!segmentClear(p, node.pos, clearance))
      continue;
    best_dist = d;
    best = node.id;
  }
  return best;
}

void VoronoiGraph::addEdge(int u, int v, double cost) {
  if (cost <= 0.0 || u < 0 || v < 0 || u >= static_cast<int>(nodes_.size()) ||
      v >= static_cast<int>(nodes_.size()))
    return;

  if (u == v)
    return;  // self-loops never improve any path and pollute the visualization

  // Skip degenerate edges (coincident node positions, e.g. a connector onto a GVD node
  // the start/goal sits exactly on)
  if ((nodes_[u].pos - nodes_[v].pos).squaredNorm() < 1e-12)
    return;

  auto link = [this](int a, int b, double c) {
    adj_[a].push_back({a, b, c});
    adj_[b].push_back({b, a, c});
  };

  // Subdivide long chains with Steiner nodes so path-to-pose chords stay close to the GVD
  const int splits = static_cast<int>(std::ceil(cost / kMaxEdgeLen)) - 1;
  const double seg_cost = cost / (splits + 1);
  int prev = u;
  for (int s = 1; s <= splits; ++s) {
    const double t = static_cast<double>(s) / (splits + 1);
    const Eigen::Vector2d p = nodes_[u].pos * (1.0 - t) + nodes_[v].pos * t;
    nodes_.push_back({p, static_cast<int>(nodes_.size()), false, false});
    adj_.emplace_back();
    const int w = static_cast<int>(nodes_.size()) - 1;
    link(prev, w, seg_cost);
    prev = w;
  }
  link(prev, v, seg_cost);
}

}  // namespace nav2_teb_controller
