# Status, Roadmap & Gotchas

Live tracker for what is implemented, what is planned, and the traps that bite. Referenced from
[AGENTS.md](../AGENTS.md) and [README.md](../README.md) — this file is the single place where
project-state details live.

## Implemented

### Homotopy Class Planning (HCP)
- `HomotopyClassPlanner`: homotopy-distinct candidate paths → per-class TEB optimization →
  best-candidate selection with class-switch hysteresis and pose-normalized cost comparison.
  One persistent `DiscreteTEBPlanner` per homotopy class (matched via H-signature across ticks,
  LRU-bounded + TTL-pruned) — candidates never contaminate each other's warm start.
- `VoronoiGraphSearch` (default): reduced Generalized Voronoi Diagram extracted from the ESDF
  (ridge cells ≥ min_clearance, spur pruning, junction/endpoint reduction, Steiner-subdivided
  edges, closed-loop splitting, clearance-checked start/goal connectors). All paths keep at
  least the footprint circumradius from obstacles by construction.
- `VisibilityGraphSearch` (fallback): polygon keypoints offset outward, ESDF clearance-thresholded
  visibility.
- Shared graph algorithms (`graph_algorithms.hpp`): templated Dijkstra / Yen K-shortest paths /
  path-to-poses / H-signature dedup; searches loop with growing K until enough DISTINCT classes.
- `HSignature`: winding-number invariant (swept angle minus principal endpoint delta, snapped to
  integer) — same-class paths compare equal regardless of geometry.
- Grows-K enumeration lives in both searches; `hcp.max_classes` bounds the per-tick class budget.

### Controller & Integration
- TEBController plugin lifecycle (configure/activate/deactivate/cleanup) + `setPlan`,
  `computeVelocityCommands`, `setSpeedLimit`
- Costmap converter integration (obstacle polygons for visualization/obstacles)
- Parameter library via `generate_parameter_library` (schema =
  `config/teb_controller_parameters.yaml`; doc/parameters.md auto-generated via `make docs`)
- TEBVisualizer: 7 RViz topics (local plan, lookahead, poses, obstacles, curvature radii, footprint)
- `PathHandler` (step 2 of `computeVelocityCommands`: TF lookup + `pruneGlobalPlan` + `transformAndTrimPlan` +
  overwrite of the start pose with the robot pose). Holds a `Costmap2D` ref + global frame (unit-testable).
- **Sticky local-goal hysteresis** (`trajectory.local_goal_hysteresis`, default 1.0 m): once the lookahead
  window's furthest pose (the TEB endpoint) has advanced it is anchored by its *pose* and only advances, never
  recedes, unless the freshly trimmed goal falls more than `local_goal_hysteresis` behind it (genuine reversal).
  Pose-anchored (not index-anchored) so it survives the per-tick prune step; `PathHandler::reset()` is called
  from `TEBController::setPlan` to drop the sticky goal across missions. Kills the oscillation where a temporary
  homotopy detour + the speed-scaled lookahead / costmap-window cut pulled the local goal back tick-to-tick.
- `BandController` abstraction (step 6): abstract base `BandController` (NVI, applies common velocity/steering
  `applySaturation` after the implementation-specific `computeRawCommand`), selected inline in
  `TEBController::configure` via a small `if` on `path_tracker.type`. `FeedForwardController` (reads the planned
  band velocity) is the only implemented tracker; `StanleyController` / `LyapunovController` were reverted and
  will be re-added later. Consumed by: `TEBController::configure`.

### Trajectory Optimization (g2o)
- 27 custom g2o edge classes + 2 vertex types (21 wired in `buildGraph`; acceleration/jerk/steering-rate
  Start+Goal variants)
- `addEdgesGeneric<EdgeType, InfoDim>` template edge factory
- 3-phase stepwise optimization (obstacles+G3+kinematics → kinodynamics → efficiency), per-phase weight
  ramp, divergence detection, cost breakdown logging
- Solvers: eigen (default), cholmod, csparse, dense; algorithms: gauss_newton, levenberg_marquardt

### Obstacle Avoidance & Feasibility
- ESDF (Meijster O(n) EDT) with bilinear query (C¹) + analytical gradient
- Footprint: circle model (polygon → circles via `scripts/footprint_polygon_to_circles.py`),
  ESDF-based collision check, `checkFeasibility` hard stop
- `EdgeESDFObstacle` soft constraint per footprint circle (+ inflation term, robust kernel)
- Lean queries: `queryDistance()` (distance-only) and a shared `interpolate()` core — distance-only
  call sites (feasibility, obstacle-edge culling, footprint check) skip gradient + normalization

### TEB Utilities
- initFromPath, autoResize, updateAndPrune, checkFeasibility, computeCurvature
- `BandController` module: extractVelocity, getVelocityCommand, saturateVelocity, saturateSteeringAngle,
  convertAckermannToTwist, convertTwistToAckermann (moved out of teb_utils)
- `PathHandler`: pruneGlobalPlan, transformAndTrimPlan (moved out of teb_utils)
- Velocity saturation (Twist, proportional option), Ackermann↔Twist conversions,
  steering-angle rate saturation, computeCurvature

### Verification
- 23 gtest suites, one per edge class (numeric-Jacobian checks via `test_jacobian_utils.hpp`)
- `make build` / `make format` / `make lint` / `make test` / `make docs` / `make docs-check`
- Visualization throttled to ~30 Hz in the control loop (publishers are additionally subscriber-gated)
- `include_shim/tl_expected/expected.hpp` shadows the deprecated parameter_traits header at build time
  (removes the `#pragma message` deprecation noise; plain `-I` beats `-isystem` /opt/ros). **Permanent
  for Jazzy**: upstream keeps the deprecated include in `parameter_traits.hpp:37` for backward compat;
  removal only lands in Lyrical+ — do not wait for a jazzy package update.

### Graph Lifecycle (P4, Stage A)
- Vertex pooling: `VertexPose`/`VertexTimeDiff` objects are reused across ticks and optimization phases
  (pool grown on demand in `AddTEBVertices`; g2o never deletes them, see gotcha below) — replaces ~6N
  `new`s per control tick with value updates.
- Robust-kernel pooling: `EdgeESDFObstacle` Huber kernels come from `robust_kernel_pool_`, indexed by
  `robust_kernel_pool_idx_` (reset per phase). The g2o edge destructor does NOT free the kernel
  (verified against libg2o 2020.5.29 disassembly), so pooled kernels survive `optimizer_->clear()`.
- Benchmark diagnostics: `teb_profiler` reports per-window scalar samples — `teb_size` (band size) and
  `outer_iters_{obstacles,kinodynamics,full}` (early-exit vs full iteration budget).

### Plan Pruning / Trimming (performance)
- `pruneGlobalPlan` + `transformAndTrimPlan` now take a precomputed `tf2::Transform` (plan frame →
  global frame, latest known, `tf2::TimePointZero`) instead of doing their own blocking TF lookups.
  The old stamped-time `lookupTransform(..., timeout=0.5 s)` blocked up to the timeout whenever the
  exact plan-stamp transform was not cached — the source of single-tick ~10 ms `prune_trim` spikes
  (max_total 13–15.5 ms, loop dips to ~69 Hz). Robot pose → plan frame is now pure math; all distance
  loops and `doTransform`s are unchanged in semantics.

## Planned / To Do (Stubs)

- **HCP refinements (see [plan_hcp_voronoi.md](plan_hcp_voronoi.md))**: obstacle sets that
  enter/leave the sliding window legitimately reset class identity (signature size changes);
  `max_classes` sequential solves scale planning latency (~1 solve per class per tick); grid
  connected-component signatures (dropping costmap_converter from HCP) deferred.
- **Via points**: `EdgeViaPoints` / `AddEdgesViaPoints` (empty)
- **Preferred rotation direction**: `AddEdgesPreferRotDir` / `edge_prefer_rotdir.h` (not wired)
- **Legacy costmap obstacle association**: `AddEdgesObstacles` / `edge_costmap_obstacle.h` (not wired;
  `legacy_obstacle_association` param unused)
- **ESDF-aware autoResize**: nudge midpoint away from obstacles
- **Recovery behaviors module** (`recovery.*` params unused)
- **Integration tests** (`test/integration/` is empty; integration test suite planned)
- **Kinematic constraint rework (PLAN ONLY)**: normalized dimensionless residuals + single
  `weight_kinematics` + limit-based edge gating (`limit > 0` instead of `weight == 0`), staged toward a
  violation-feedback weight ramp and an augmented-Lagrangian outer loop. Research-backed design agreed
  Aug 2026 — see [plan_kinematic_constraints.md](plan_kinematic_constraints.md). Do not implement without
  revisiting the open decisions listed there.
- **TEB-K** (cubic Hermite spline segments) — future research feature, **not implemented**

## Gotchas / Traps

- `robot_model`: only `"diff_drive"` and `"ackermann"` (car-like) are valid. The old `"bicycle"` value was
  removed from the schema — it silently added NO kinematics edge.
- Holonomic mode is NOT a param: it is inferred from `v_max_y > 0` (schema default `v_max_y: 0.5` → holonomic!
  runtime config sets `0.0`). Holonomic switches velocity/acceleration edges to the 3D variants.
- `weight_path_smoothness` is declared but `EdgePathSmoothness` is wired with `weight_shortest_path`
  (both in phase Full) — likely a bug, the smoothness weight param is dead.
- `EdgeESDFObstacle` skips pose 0 and last pose (`for i in [1, n-2]`); start pose is also overwritten with the
  true robot pose before planning.
- **Tail exclusion**: obstacle edges are skipped for poses within `s_excl =
  max(2·v_max_x/max_vel_theta, circum_radius + min_obstacle_dist)` of the band end — the tail cannot bend
  around obstacles (goal/anchor fixed) and only produced kinks/false divergence. If the whole band is shorter
  than `s_excl`, no obstacle edges are added at all (only the `checkFeasibility` hard stop).
  `addEdgesESDFObstacles` in `src/planner/optimal_planner.cpp`.
- `checkFeasibility` returns the first colliding pose index; `stop_cmd` only gates the path tracker output,
  the planner still runs.
- Steering feedback (`/tricycle_state`, ackermann msg) is used as initial steering angle for
  `EdgeStartSteeringAngle` / `EdgeSteeringRateStart` and for `saturateSteeringAngle`.
- `no_inner_iterations` / `no_outer_iterations` are split equally across the 3 optimization phases.
- `clearGraph()` removes the pooled vertices from the optimizer via `HyperGraph::removeVertex`
  (no deletion) and lets `optimizer->clear()` delete only the per-phase edges. NEVER call
  `optimizer->clear()` with pooled vertices still in the graph — `HyperGraph::clear()` deletes every
  vertex it owns (verified in libg2o disassembly), leaving dangling pool pointers (use-after-free).
- g2o's edge destructor does NOT own/free the robust kernel in this build — pooled kernels survive
  `clearGraph()` without any detach. Do not `delete` pooled kernels or the optimizer's `clear()` path
  would double-free. (`setRobustKernel(nullptr)` is only needed if a future g2o takes ownership.)
- `initFromPath` creates one band pose per global-plan pose **without a `max_samples` cap** (only
  `autoResize`'s insert path respects it) — a long plan → oversized band on cold start/reinit.
  Deliberately untouched (tracked separately).