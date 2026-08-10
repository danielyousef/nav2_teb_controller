# Status, Roadmap & Gotchas

Live tracker for what is implemented, what is planned, and the traps that bite. Referenced from
[AGENTS.md](../AGENTS.md) and [README.md](../README.md) — this file is the single place where
project-state details live.

## Implemented

### Controller & Integration
- TEBController plugin lifecycle (configure/activate/deactivate/cleanup) + `setPlan`,
  `computeVelocityCommands`, `setSpeedLimit`
- Costmap converter integration (obstacle polygons for visualization/obstacles)
- Parameter library via `generate_parameter_library` (schema =
  `config/teb_controller_parameters.yaml`; doc/parameters.md auto-generated via `make docs`)
- TEBVisualizer: 7 RViz topics (local plan, lookahead, poses, obstacles, curvature radii, footprint)

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

### TEB Utilities
- initFromPath, autoResize, updateAndPrune, extractVelocity, getVelocityCommand,
  pruneGlobalPlan, transformAndTrimPlan
- Velocity saturation (Twist, proportional option), Ackermann↔Twist conversions,
  steering-angle rate saturation, computeCurvature

### Verification
- 23 gtest suites, one per edge class (numeric-Jacobian checks via `test_jacobian_utils.hpp`)
- `make build` / `make format` / `make lint` / `make test` / `make docs` / `make docs-check`

## Planned / To Do (Stubs)

- **Homotopy Class Planning (TBD)**: `HomotopyClassPlanner::plan()`, `VisibilityGraphSearch::search()`,
  `VisibilityGraph::build()`, `HSignature::compute()` are stubs. Keep `hcp.activate: false`;
  `configure()` logs an error if enabled. (interfaces in `include/nav2_teb_controller/homotopy/`)
- **Via points**: `EdgeViaPoints` / `AddEdgesViaPoints` (empty)
- **Preferred rotation direction**: `AddEdgesPreferRotDir` / `edge_prefer_rotdir.h` (not wired)
- **Legacy costmap obstacle association**: `AddEdgesObstacles` / `edge_costmap_obstacle.h` (not wired;
  `legacy_obstacle_association` param unused)
- **ESDF-aware autoResize**: nudge midpoint away from obstacles
- **Recovery behaviors module** (`recovery.*` params unused)
- **Integration tests** (`test/integration/` is empty; integration test suite planned)
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
- `checkFeasibility` returns the first colliding pose index; `stop_cmd` only gates the path tracker output,
  the planner still runs.
- Steering feedback (`/tricycle_state`, ackermann msg) is used as initial steering angle for
  `EdgeStartSteeringAngle` / `EdgeSteeringRateStart` and for `saturateSteeringAngle`.
- `no_inner_iterations` / `no_outer_iterations` are split equally across the 3 optimization phases.
- `clearGraph()` does NOT delete vertices themselves — only edges (via `optimizer->clear()`); vertices are
  rebuilt every `buildGraph` call.