# AGENTS.md — nav2_teb_controller

## Overview

Nav2 controller plugin implementing the **classic (discrete) Timed Elastic Band (TEB)** local planner
([teb_local_planner](https://github.com/rst-tu-dortmund/teb_local_planner) by RST – TU Dortmund, ported to ROS 2).
The trajectory is a sequence of `N` SE(2) poses + `N-1` time intervals `Δt` (`TimedElasticBand`); velocities and
higher derivatives are obtained by finite differences (difference quotients). The band is deformed by sparse
nonlinear least-squares optimization with **g2o** (Gauss-Newton / Levenberg-Marquardt, eigen/cholmod/csparse/dense solvers)
considering kinematics, dynamics, obstacle avoidance, and execution time.

> **Note:** This is the *discrete* TEB formulation (piecewise-linear poses + dt). The TEB-K variant with polynomial
> (cubic Hermite) trajectory segments is **planned but NOT implemented** — do not describe it as done.

Key dependencies: `libg2o`, `Eigen3`, `pluginlib`, `costmap_converter`, `ackermann_msgs`.

## Documentation (read first)

- `doc/architecture.md` — system architecture, module inventory, control-loop data flow (with diagrams)
- `doc/control_concept.md` — TEB theory, g2o graph formulation, per-edge math, control law
- `doc/parameters.md` — full parameter reference. **AUTO-GENERATED** from
  `config/teb_controller_parameters.yaml` via `make docs` (source of truth = the schema). Never edit
  `doc/parameters.md` by hand; add/change `description:` in the schema (it also shows up at runtime via
  `ros2 param describe`). `make docs-check` fails if the doc drifted.
- `README.md` — user-facing overview, quick start

## Build & Test

```bash
make build       # colcon build --packages-select nav2_teb_controller
make test        # colcon test + test-result
make format      # clang-format --dry-run --Werror (only src/*.cpp, *.hpp)
make format-fix  # clang-format -i
make lint        # run-clang-tidy
make lint-fix    # clang-tidy --fix
make docs        # regenerate doc/parameters.md from config/teb_controller_parameters.yaml
make docs-check  # fail if doc/parameters.md is out of date (CI)
```

Individual test:
```bash
colcon test --packages-select nav2_teb_controller --event-handlers console_direct+ --ctest-args -R test_edge_time_optimal
```

Note: `colcon test` also runs uncrustify/flake8/pep257/lint_cmake over the whole package; several of those
pre-existing style failures are unrelated to C++ code (launch files, CMake) — the gtest suites (one per edge
class, planner graph-lifecycle, homotopy) are the authoritative functional tests.

## Directory Layout

```
├── config/
│   ├── teb_controller_parameters.yaml   # generate_parameter_library schema (source of truth for params)
│   └── teb_controller_params.yaml       # Nav2 runtime config example (controller_server params)
├── include/nav2_teb_controller/
│   ├── core/
│   │   ├── timed_elastic_band.hpp       # TEB container: poses_ + timediffs_ (dt), invariant sizePoses == sizeTimeDiffs+1
│   │   ├── pose_se2.hpp                 # SE(2) pose (x, y, theta), g2o-compatible oplus
│   │   ├── footprint.hpp                # Footprint = N circles (polygon parsed to radius-0 circles)
│   │   └── teb_utils.hpp                # Free functions: init, autoResize, prune, velocity, feasibility
│   ├── g2o_types/                       # g2o edges + vertices (27 edge classes, 2 vertices)
│   │   ├── vertex_pose.h / vertex_timediff.h
│   │   ├── base_teb_edges.h             # BaseTeb{Unary,Binary,Multi}Edge templates (carry params_ ptr)
│   │   ├── penalties.h                  # Linear penalty functions (soft constraints) + derivatives
│   │   ├── edge_velocity.h / edge_velocity_holonomic.h
│   │   ├── edge_acceleration*.h         # 6 files: Start / plain / Goal × (2D, holonomic 3D)
│   │   ├── edge_jerk.h (stale!) / edge_jerk_new.h   # EdgeJerk, EdgeJerkStart, EdgeJerkGoal
│   │   ├── edge_snap.h                  # G4: 5 poses + 4 dt
│   │   ├── edge_kinematics_diff.h / edge_kinematic_car_like.h
│   │   ├── edge_steering_rate*.h        # 3 files (Start / plain / Goal)
│   │   ├── edge_start_steering_angle.h, edge_steering_angle_goal.h, edge_goal_angular_velocity_zero.h
│   │   ├── edge_g3_continuity.h         # Curvature-rate / steering-rate continuity (unwrap)
│   │   ├── edge_time_optimal.h, edge_shortest_path.h, edge_path_smoothness.h
│   │   ├── edge_esdf_obstacle.h         # Active obstacle edge (per-footprint-circle + inflation)
│   │   ├── edge_costmap_obstacle.h      # Legacy (NOT wired)
│   │   ├── edge_via_point.h             # NOT wired
│   │   └── edge_prefer_rotdir.h         # NOT wired
│   ├── homotopy/                        # Homotopy Class Planning (implemented, hcp.activate)
│   │   ├── homotopy_class_planner.hpp   # Wrapper: per-class planners, hysteresis selection
│   │   ├── graph_search_interface.hpp   # Search ABC (+ setObstacleMap)
│   │   ├── graph_algorithms.hpp         # Templated Dijkstra / Yen / dedup (shared)
│   │   ├── voronoi_graph*.hpp           # DEFAULT search: reduced GVD from the ESDF
│   │   ├── visibility_graph*.hpp        # Fallback search: offset keypoints + clearance
│   │   ├── h_signature.hpp, teb_candidate.hpp
│   ├── obstacles/esdf.hpp               # ObstacleMap2D: Meijster EDT + bilinear query + gradient
│   ├── planner/
│   │   ├── planner_interface.hpp        # PlannerBase + PlannerInterface<TEB> ABCs
│   │   └── optimal_planner.hpp          # DiscreteTEBPlanner (g2o graph builder)
│   ├── visualization/teb_visualizer.hpp # 7 RViz publishers
│   └── math_utils.hpp                   # average_angle, fast_sigmoid
├── src/
│   ├── teb_controller.cpp               # Main Nav2 controller plugin (lifecycle)
│   ├── core/teb_utils.cpp               # All utilities impl
│   ├── planner/optimal_planner.cpp      # Graph building + 3-phase optimization
│   ├── homotopy/homotopy_class_planner.cpp  # Stub
│   ├── homotopy/visibility_graph_search.cpp # Stub
│   └── obstacles/esdf.cpp               # Meijster EDT algorithm
├── test/unit/g2o_types/                # gtest per edge class (numeric-Jacobian checks)
├── test/unit/planner/                  # planner graph-lifecycle tests
├── test/unit/homotopy/                 # H-signature / graph algorithms / Voronoi / HCP tests
├── scripts/footprint_polygon_to_circles.py
├── scripts/gen_params_docs.py   # generates doc/parameters.md from the schema (make docs)
├── launch/nav2_teb_controller_launch.py
└── doc/                                # architecture.md, control_concept.md, parameters.md (auto-generated via make docs)
```

## Architecture

### Control Loop (per `computeVelocityCommands` tick)

```
Global Plan (nav_msgs/Path, from Nav2 planner)
  → pruneGlobalPlan (erase poses behind robot)
  → transformAndTrimPlan (lookahead window, length max_global_plan_lookahead_dist)
  → overwrite front pose with actual robot pose
  → updateObstacleContainer (costmap_converter obstacle polygons)   [optional]
  → ESDF.update(*costmap) at costmap_converter_rate                 [throttled]
  → DiscreteTEBPlanner::plan(transformed_plan, start_vel)
      → cold start: initFromPath | warm start: updateAndPrune (or reinit if goal moved > reinit_dist/angle)
      → autoResize (insert/delete poses by dt / segment length / angle)
      → 3-phase optimization (obstacles+G3+kinematics → kinodynamics → full)
      → writeBackOptimizedValues, divergence check, clearGraph
  → checkFeasibility (ESDF + footprint circles, within lookahead)   [stop on collision]
  → getVelocityCommand (lookahead extraction over poses/dt)
  → saturateVelocity + saturateSteeringAngle (Ackermann, rate-limited)
  → TwistStamped cmd_vel
```

### g2o Graph Structure

- **Vertices**: `VertexPose` (SE(2), 3 DOF) per pose; `VertexTimeDiff` (1 DOF) per segment. Start pose always
  fixed; goal pose fixed if `fix_goal` or the trimmed plan reaches the global plan end.
- **Edges** are grouped per optimization phase (see `buildGraph` in `src/planner/optimal_planner.cpp`):
  1. **Phase Obstacles** (always): `addEdgesESDFObstacles` + G3 continuity + kinematics (diff_drive or car-like).
  2. **Phase Kinodynamics**: velocity (holonomic variant if `v_max_y > 0`), snap, steering-angle-goal,
     goal-angular-velocity-zero, acceleration (Start/Goal variants), steering rate (Start/Goal),
     start-steering-angle, jerk (Start/Goal).
  3. **Phase Full**: time optimal (on dt vertices), shortest path, path smoothness.
- Edge factory: `addEdgesGeneric<EdgeType, InfoDim>(EdgeDescriptor, weights)` — descriptor
  `{num_poses, num_timediffs, stride, offset, info_dim}` defines vertex wiring and iteration range.
- Solvers: eigen (default), cholmod, csparse, dense. Algorithms: gauss_newton, levenberg_marquardt.
- Memory: edges are `new`ed per graph build and freed by `clearGraph()`; vertices are detached from edges before
  `optimizer->clear()` so the TEB data stays alive (raw ownership is by design, clang-tidy suppressed).

### ESDF (ObstacleMap2D)

- Meijster algorithm: O(n) EDT on binary occupancy grid (LETHAL cells), 2 phases (column scan, row parabola envelope).
- `query(x, y)` → bilinear interpolation (C¹) + normalized analytical gradient (Eikonal |∇d|=1).
- Used for both `checkFeasibility` (hard stop) and `EdgeESDFObstacle` (soft gradient push per footprint circle
  + center inflation term, Huber robust kernel).

## Status, Roadmap & Gotchas

The full project state — implemented features, planned work, and the gotchas/traps — lives in
[`doc/status.md`](doc/status.md) (single source of truth, also linked from the README). Keep that file in
sync when implementing or stubbing features.

## Conventions

- C++20, Google style (`.clang-format`), clang-tidy with relaxed rules
- Namespace: `nav2_teb_controller`
- Headers: `#pragma once` (new code), `#ifndef` guards (legacy g2o types)
- `class` → CamelCase, `function` → camelCase, `variable`/`member` → snake_case
- `_` suffix for class members (private)
- `EIGEN_MAKE_ALIGNED_OPERATOR_NEW` on classes with Eigen members
- ROS2 lifecycle node pattern for plugin
- Parameters via `generate_parameter_library` (code-generated from `config/teb_controller_parameters.yaml`)
- g2o edges own their vertex connections (raw `new`/`delete` by design, per clang-tidy suppression)
