# Nav2 TEB Controller — Parameters

> **Auto-generated** from the parameter schema `config/teb_controller_parameters.yaml` (via `scripts/gen_params_docs.py`, invoked by `make docs`). Never edit this file by hand — change the schema `description:` fields instead.

The same descriptions appear at runtime via `ros2 param describe`. The `Consumed by` column lists the code (edge / controller function) that reads each parameter; `—` marks parameters that are not read anywhere.

## Default Config

```yaml
teb_controller:
  ros__parameters:
    FollowPath:
      hcp:
        activate: false
        efficiency_anchor_floor: 0.2
        feasibility_gate: true
        max_classes: 5.0
        min_clearance: 0.3
        parallel_optimization: true
        progress_slack: 0.5
        route_grace_time: 1.0
        search_rate: 10.0
        selection_hysteresis: 0.9
        switch_block_time: 2.0
        window_containment: true
      log_level: debug
      obstacles:
        cost_exponent: 2.0
        costmap_converter_plugin: ''
        costmap_converter_rate: 1.0
        costmap_converter_spin_thread: true
        cutoff_dist: 1.0
        feasibility_check: 1.0
        include_dynamic_obstacles: false
        inflation_dist: 1.9
        legacy_obstacle_association: false
        min_obstacle_cost: 128.0
        min_obstacle_dist: 0.1
      optimizer:
        activate: true
        algorithm: gauss_newton
        divergence_detection_enable: false
        divergence_detection_max_chi_squared: 10.0
        divergence_detection_max_chi_violation_rate: 0.5
        divergence_detection_max_path_length_factor: 3.0
        early_exit_min_delta: 0.001
        exact_arc_length: true
        fast_mode: true
        fix_goal: true
        free_goal_vel: false
        no_inner_iterations: 100.0
        no_outer_iterations: 100.0
        optimizer_backend: g2o
        penalty_epsilon: 0.1
        solver: eigen
        stepwise_optimization: true
        verbose: false
      path_tracker:
        activate: true
        type: feedforward
      recovery:
        activate: false
        divergence_detection_enable: false
      robot:
        a_max_theta: 0.0
        a_max_x: 0.25
        a_max_y: 0.0
        footprint:
          model: ''
          points: ''
          use_local_costmap: true
        has_steering: true
        jerk_max_theta: 2.0
        jerk_max_x: 2.0
        jerk_max_y: 2.0
        min_turning_radius: 0.5
        robot_model: diff_drive
        snap_max_theta: 0.0
        snap_max_x: 0.0
        steering_rate_max: 0.5
        use_proportional_saturation: true
        v_max_theta: 0.0
        v_max_x: 0.5
        v_max_x_backwards: 0.0
        v_max_y: 0.5
        wheelbase: 1.055
      trajectory:
        allow_init_backward: true
        auto_resize: true
        control_look_ahead_poses: 1.0
        control_min_look_ahead_time: 0.0
        dt_hyst: 0.05
        dt_ref: 0.3
        global_plan_prune_distance: 0.5
        local_goal_hysteresis: 1.0
        max_angle_diff: 0.15
        max_global_plan_lookahead_dist: 5.0
        max_samples: 50.0
        max_seg_length: 0.3
        min_prune_distance: 0.0
        min_samples: 3.0
        min_seg_length: 0.1
        overwrite_plan_orientation: false
        reinit_angle: 1.055
        reinit_dist: 1.055
      visualization:
        activate: false
        publish_rate: 30.0
      weights:
        weight_a_max_theta: 1.0
        weight_a_max_x: 1.0
        weight_a_max_y: 1.0
        weight_adapt_factor: 1.0
        weight_g3_continuity: 1.0
        weight_goal_angular_vel_zero: 1.0
        weight_inflation: 50.0
        weight_jerk_max_theta: 1.0
        weight_jerk_max_x: 1.0
        weight_kinematics_forward_drive: 1.0
        weight_kinematics_nh: 1.0
        weight_kinematics_turning_radius: 1.0
        weight_max_steering_rate: 1.0
        weight_obstacle: 50.0
        weight_path_smoothness: 1.0
        weight_shortest_path: 1.0
        weight_snap_max_theta: 1.0
        weight_snap_max_x: 1.0
        weight_start_steering_angle: 1.0
        weight_time_optimal: 1.0
        weight_v_max_theta: 1.0
        weight_v_max_x: 1.0
        weight_v_max_y: 1.0
        weight_zero_steering_angle_goal: 1.0

```

## `log_level`

| Param | Type | Default | Constraints | Description | Consumed by |
|---|---|---|---|---|---|
| `FollowPath.log_level` | `string` | `"debug"` | (read-only)<br>one of the specified values: ['debug', 'info', 'warn', 'error'] | ROS logging level (debug\|info\|warn\|error). | TEBController::configure (logger setup). |

## `hcp`

| Param | Type | Default | Constraints | Description | Consumed by |
|---|---|---|---|---|---|
| `FollowPath.hcp.activate` | `bool` | `false` |  | Activate homotopy class planning: homotopy-distinct candidate paths are found on a Voronoi (GVD) graph extracted from the ESDF, each class is optimized by its own persistent DiscreteTEBPlanner instance, and the best feasible candidate is selected (with class-switch hysteresis). Falls back to the direct planner without obstacle data or when no classes are found. | TEBController::configure, HomotopyClassPlanner::plan. |
| `FollowPath.hcp.max_classes` | `int` | `5` | parameter must be within bounds [1, 10] | Maximum number of homotopy classes to explore per tick. Each class costs one full TEB optimization, so this directly scales planning latency. | HomotopyClassPlanner::plan (candidate budget). |
| `FollowPath.hcp.search_rate` | `double` | `10.0` | parameter must be within bounds [0.1, 100.0] | Rate [Hz] at which the graph search itself (GVD extraction + Yen enumeration) is re-executed, independent of the controller rate. Between refreshes the cached homotopy classes are reused and only the per-class TEB optimization runs. The search is additionally forced when a plan endpoint moved further than trajectory.reinit_dist since the last search. | HomotopyClassPlanner::plan. |
| `FollowPath.hcp.min_clearance` | `double` | `0.3` | parameter must be within bounds [0.05, 10.0] | Minimum obstacle clearance [m] for Voronoi (GVD) ridge cells and the graph-search connector segments. This shapes the topology graph only — physical footprint safety is enforced separately by the ESDF obstacle edges and the feasibility gate. Choose well below the footprint circumradius so aisles/corridors remain representable; typical value ≈ 0.2–0.5 × robot radius. | TEBController::configure, VoronoiGraphSearch / VisibilityGraphSearch construction. |
| `FollowPath.hcp.selection_hysteresis` | `double` | `0.9` | parameter must be within bounds [0.01, 1.0] | Hysteresis factor for best-route selection: a competing route must be CHEAPER than the current best route by this factor (efficiency_new <= efficiency_best * selection_hysteresis) to take over, where the comparison uses the EFFICIENCY cost category (time-optimal + shortest-path + smoothness edges), not the total chi2. 0.9 = the new route needs a 10% efficiency advantage. Prevents flip-flopping between near-equal routes. | HomotopyClassPlanner::selectBestCandidate. |
| `FollowPath.hcp.route_grace_time` | `double` | `1.0` | parameter must be within bounds [0.0, 10.0] | Grace period [s] for best-route continuity: when the previous best route's GVD class blinks out of a search round (transient divergence or re-extraction churn) but its route entry still exists, a candidate is re-synthesized from the route's last known polyline so it competes with a fresh cost — instead of the tick collapsing to cheapest-of-the-day and ping-ponging between homotopy classes. After the grace expires (route genuinely gone/blocked) selection proceeds normally. 0 disables. | HomotopyClassPlanner::synthesizePrevRouteCandidate. |
| `FollowPath.hcp.feasibility_gate` | `bool` | `true` |  | Reject candidates whose optimized band still intersects lethal ESDF cells within the footprint (same check as the controller's hard feasibility stop, applied per candidate after optimization). Through-obstacle bands then never win selection even when their penalty cost is low. | HomotopyClassPlanner::passesFeasibilityGate. |
| `FollowPath.hcp.window_containment` | `bool` | `true` |  | Reject candidates whose optimized band leaves the mapped local-costmap window (0.5 m inward margin). Outside the window the ESDF clamps to its boundary value, so unknown space reads as free and collision-free — such phantom routes carry near-zero costs, win selection, then vanish in later rounds (churn source, benchmark_test_20). Skipped when the band's final pose lies outside the window itself (long-haul legs). | HomotopyClassPlanner::staysInWindow. |
| `FollowPath.hcp.progress_slack` | `double` | `0.5` | parameter must be within bounds [0.0, 1.0] | Progress guard for best-route takeover: while the current best route is offered, a competing route must not be a detour — its band arc length may exceed the current best's by at most this fraction (0.1 = max 10% longer) IN ADDITION to the efficiency hysteresis. All bands span robot → mission goal, so arc length is a comparable progress estimate (ΣΔt alone barely rewards shorter geometry). Not applied when the current route is missing/infeasible — dead-end escapes may be arbitrarily longer. | HomotopyClassPlanner::selectBestCandidateIndex. |
| `FollowPath.hcp.efficiency_anchor_floor` | `double` | `0.2` | parameter must be within bounds [0.0, 100.0] | Distrust threshold for the efficiency hysteresis anchor: anchors below this floor come from bands that read free space beyond the mapped local-costmap window (benchmark_test_20 anchors of 0.007–0.07) and are meaningless. While such a route is current best, it loses its hysteresis/progress protection and its ranking privilege — realistic competitors take over by plain min-cost; the distrusted route remains available as a last-resort fallback. Set 0 to disable. | HomotopyClassPlanner::selectBestCandidateIndex. |
| `FollowPath.hcp.switch_block_time` | `double` | `2.0` | parameter must be within bounds [0.0, 30.0] | Time [s] after a best-route switch during which further switches are blocked outright, regardless of cost differences (unless the current best route becomes infeasible). Tune to the robot's dynamics — slow machines need longer blocks. | HomotopyClassPlanner::selectBestCandidate. |
| `FollowPath.hcp.parallel_optimization` | `bool` | `true` |  | Optimize the per-route TEB candidates in parallel (one thread per route). Route identity assignment and best-route selection remain sequential; each route owns an isolated optimizer instance, so solves are independent. Set false to force sequential optimization (debugging / single-core targets). | HomotopyClassPlanner::optimizeCandidates. |

## `path_tracker`

| Param | Type | Default | Constraints | Description | Consumed by |
|---|---|---|---|---|---|
| `FollowPath.path_tracker.activate` | `bool` | `true` |  | If true, extract the velocity command from the optimized band unless a collision stop was issued. If false, always output a zero velocity. | TEBController::computeVelocityCommands (gates the BandController output). |
| `FollowPath.path_tracker.type` | `string` | `"feedforward"` | one of the specified values: ['feedforward'] | Selects the BandController implementation that turns the optimized band into a velocity command. Currently only 'feedforward' (read the planned band velocity) is implemented; selection is done inline in TEBController::configure. | TEBController::configure. |

## `trajectory`

| Param | Type | Default | Constraints | Description | Consumed by |
|---|---|---|---|---|---|
| `FollowPath.trajectory.reinit_dist` | `double` | `1.055` |  | Distance [m] the goal must have moved for the band to be re-initialized from the global plan instead of being warm-started (updateAndPrune). | DiscreteTEBPlanner::plan. |
| `FollowPath.trajectory.reinit_angle` | `double` | `1.055` |  | Heading change [rad] threshold for re-initialization, like reinit_dist. | DiscreteTEBPlanner::plan. |
| `FollowPath.trajectory.auto_resize` | `bool` | `true` |  | Automatically insert/remove band poses during optimization based on time steps, segment lengths and heading differences. | DiscreteTEBPlanner::optimizeTEB (autoResize). |
| `FollowPath.trajectory.dt_ref` | `double` | `0.3` |  | Desired time difference [s] between consecutive band poses; used for auto-resizing and for the velocity-command lookahead. | autoResize, getVelocityCommand. |
| `FollowPath.trajectory.dt_hyst` | `double` | `0.05` |  | Hysteresis [s] around dt_ref to avoid oscillating pose insertions/removals. | autoResize. |
| `FollowPath.trajectory.min_samples` | `int` | `3` |  | Minimum number of band poses. | autoResize, initFromPath, updateAndPrune. |
| `FollowPath.trajectory.max_samples` | `int` | `50` |  | Maximum number of band poses. | autoResize. |
| `FollowPath.trajectory.min_seg_length` | `double` | `0.1` |  | Minimum segment length [m]; poses are inserted if a segment is longer than max_seg_length (or shorter than min_seg_length with dt > dt_ref). | autoResize. |
| `FollowPath.trajectory.max_seg_length` | `double` | `0.3` |  | Maximum segment length [m] before poses are inserted. | autoResize. |
| `FollowPath.trajectory.max_angle_diff` | `double` | `0.15` |  | Maximum heading difference [rad] between consecutive poses before a pose is inserted. | autoResize. |
| `FollowPath.trajectory.overwrite_plan_orientation` | `bool` | `false` |  | Set the initial pose orientations from the direction of the path segments, overwriting the orientations given in the global plan. | initFromPath. |
| `FollowPath.trajectory.allow_init_backward` | `bool` | `true` |  | Allow the band to be initialized with backward motion (reversed pose sequence at the start). | initFromPath. |
| `FollowPath.trajectory.control_look_ahead_poses` | `int` | `1` |  | Number of band segments used as lookahead for the velocity extraction. | getVelocityCommand. |
| `FollowPath.trajectory.max_global_plan_lookahead_dist` | `double` | `5.0` |  | Maximum lookahead distance [m] of the global plan that is transformed into the robot frame. | transformAndTrimPlan. |
| `FollowPath.trajectory.global_plan_prune_distance` | `double` | `0.5` |  | Poses of the global plan farther behind the robot than this distance [m] are pruned. | pruneGlobalPlan. |
| `FollowPath.trajectory.control_min_look_ahead_time` | `double` | `0.0` |  | Minimum lookahead time [s] for the velocity extraction. | getVelocityCommand. |
| `FollowPath.trajectory.min_prune_distance` | `double` | `0.0` |  | Minimum distance [m] the band start pose may lag behind the robot pose before the first pose is dropped. | updateAndPrune. |
| `FollowPath.trajectory.local_goal_hysteresis` | `double` | `1.0` | parameter must be within bounds [0.0, 50.0] | Hysteresis [m] on the local lookahead goal (the furthest global-plan pose kept as the TEB endpoint).           Once the local goal has advanced along the global plan it is sticky (it only ever advances, never recedes) unless the freshly trimmed goal falls more than this distance behind the sticky one (a genuine large reversal). Prevents the TEB endpoint from oscillating when the robot briefly drives away on an alternative homotopy and the time-based lookahead (max_global_plan_lookahead_dist, speed-scaled) or the costmap-window cut would otherwise pull the local goal back tick-to-tick. 0 disables (classic, non-sticky behavior). | PathHandler::transformAndTrimPlan. |

## `robot`

| Param | Type | Default | Constraints | Description | Consumed by |
|---|---|---|---|---|---|
| `FollowPath.robot.footprint.use_local_costmap` | `bool` | `true` |  | Use the costmap robot radius for the circular footprint instead of the polygon given in model/points. | TEBController::configure (Footprint). |
| `FollowPath.robot.footprint.model` | `string` | `""` |  | Footprint model: 'polygon' or 'circle'. | TEBController::configure (Footprint). |
| `FollowPath.robot.footprint.points` | `string` | `""` |  | Footprint polygon points as 'x1,y1;x2,y2;...' (used when model='polygon'), converted to a circle set via scripts/footprint_polygon_to_circles.py. | TEBController::configure (Footprint). |
| `FollowPath.robot.robot_model` | `string` | `"diff_drive"` | (read-only)<br>one of the specified values: ['diff_drive', 'ackermann'] | Robot kinematic model: 'diff_drive' or 'ackermann' (car-like/bicycle driving). Selects the kinematics edge and Ackermann-specific edges. | DiscreteTEBPlanner::buildGraph. |
| `FollowPath.robot.wheelbase` | `double` | `1.055` | parameter must be within bounds [0.1, 10.0] | Wheelbase L [m] for the ackermann (car-like) model: steering-rate/steering-angle edges and the steering-angle saturation. | EdgeSteeringRate*, EdgeStartSteeringAngle, saturateSteeringAngle. |
| `FollowPath.robot.v_max_x` | `double` | `0.5` | parameter must be within bounds [0.0, 10.0] | Maximum forward linear velocity [m/s]. | EdgeVelocity*/EdgeVelocityHolonomic and saturateVelocity (also speed-limit scaled). |
| `FollowPath.robot.v_max_x_backwards` | `double` | `0.0` | parameter must be within bounds [0.0, 10.0] | Maximum backward (reverse) linear velocity [m/s]; 0 disables reverse motion. | EdgeVelocity* (forward-drive) and saturateVelocity. |
| `FollowPath.robot.v_max_y` | `double` | `0.5` | parameter must be within bounds [0.0, 10.0] | Maximum lateral linear velocity [m/s]. Values > 0 switch the planner into holonomic mode (3-DOF velocity/acceleration edges). | buildGraph (holonomic switch), EdgeVelocityHolonomic, EdgeAccelerationHolonomic*, saturateVelocity. |
| `FollowPath.robot.v_max_theta` | `double` | `0.0` | parameter must be within bounds [0.0, 10.0] | Maximum angular velocity [rad/s]. | EdgeVelocity* and saturateVelocity. |
| `FollowPath.robot.a_max_x` | `double` | `0.25` | parameter must be within bounds [0.0, 10.0] | Maximum linear acceleration [m/s²]. | EdgeAcceleration{Start,Goal}. |
| `FollowPath.robot.a_max_y` | `double` | `0.0` | parameter must be within bounds [0.0, 10.0] | Maximum lateral acceleration [m/s²] (holonomic mode only). | EdgeAccelerationHolonomic{Start,Goal}. |
| `FollowPath.robot.a_max_theta` | `double` | `0.0` | parameter must be within bounds [0.0, 10.0] | Maximum angular acceleration [rad/s²]. | EdgeAcceleration{Start,Goal}. |
| `FollowPath.robot.jerk_max_x` | `double` | `2.0` | parameter must be within bounds [0.0, 100.0] | Maximum linear jerk [m/s³]. | EdgeJerk{Start,Goal}. |
| `FollowPath.robot.jerk_max_y` | `double` | `2.0` | parameter must be within bounds [0.0, 100.0] | Maximum lateral jerk [m/s³]. | none |
| `FollowPath.robot.jerk_max_theta` | `double` | `2.0` | parameter must be within bounds [0.0, 100.0] | Maximum angular jerk [rad/s³]. | EdgeJerk{Start,Goal}. |
| `FollowPath.robot.snap_max_x` | `double` | `0.0` | parameter must be within bounds [0.0, 1000.0] | Maximum linear snap [m/s⁴] (4th derivative, edge snap). | EdgeSnap. |
| `FollowPath.robot.snap_max_theta` | `double` | `0.0` | parameter must be within bounds [0.0, 1000.0] | Maximum angular snap [rad/s⁴] (G4). | EdgeSnap. |
| `FollowPath.robot.steering_rate_max` | `double` | `0.5` | parameter must be within bounds [0.0, 10.0] | Maximum steering rate [rad/s] (ackermann / car-like). | EdgeSteeringRate{Start,Goal}, EdgeStartSteeringAngle, saturateSteeringAngle. |
| `FollowPath.robot.min_turning_radius` | `double` | `0.5` | parameter must be within bounds [0.0, 100.0] | Minimum turning radius R_min [m] (ackermann / car-like). | EdgeKinematicsCarlike. |
| `FollowPath.robot.has_steering` | `bool` | `true` |  | Reserved for a future steering-aware mode. | none |
| `FollowPath.robot.use_proportional_saturation` | `bool` | `true` |  | Use proportional velocity saturation (all components scaled down when one exceeds its limit) instead of hard clamping. | saturateVelocity. |

## `optimizer`

| Param | Type | Default | Constraints | Description | Consumed by |
|---|---|---|---|---|---|
| `FollowPath.optimizer.optimizer_backend` | `string` | `"g2o"` | (read-only)<br>one of the specified values: ['g2o', 'ceres'] | Optimization backend selector. Only 'g2o' is implemented; this parameter is currently NOT read by the code (kept in the schema for compatibility). | none |
| `FollowPath.optimizer.solver` | `string` | `"eigen"` | (read-only)<br>one of the specified values: ['cholmod', 'eigen', 'csparse', 'dense'] | g2o linear solver backend: cholmod, eigen, csparse or dense. | DiscreteTEBPlanner (SparseOptimizer setup). |
| `FollowPath.optimizer.algorithm` | `string` | `"gauss_newton"` | (read-only)<br>one of the specified values: ['gauss_newton', 'levenberg_marquardt'] | g2o iteration algorithm: gauss_newton or levenberg_marquardt. | DiscreteTEBPlanner (SparseOptimizer setup). |
| `FollowPath.optimizer.activate` | `bool` | `true` |  | Enable trajectory optimization at all; if false, optimizeTEB() returns without optimizing the band. | DiscreteTEBPlanner::optimizeTEB. |
| `FollowPath.optimizer.fast_mode` | `bool` | `true` |  | Require a minimum time difference when inserting poses (autoResize fast mode); avoids inserting too many poses near the start. | autoResize. |
| `FollowPath.optimizer.exact_arc_length` | `bool` | `true` |  | Use exact arc-length velocities instead of the linearized difference-quotient approximation in the edges that support it. | EdgeSteeringRate*, EdgeKinematicsCarlike, EdgeG3Continuity. |
| `FollowPath.optimizer.no_inner_iterations` | `int` | `100` | parameter must be within bounds [1, 1000] | Maximum number of inner iterations per optimization call; together with no_outer_iterations split 25/25/50 across the 3 optimization phases (the first two phases get 1/4 each, the remainder goes to the final full phase). | DiscreteTEBPlanner::optimizeTEB. |
| `FollowPath.optimizer.no_outer_iterations` | `int` | `100` | parameter must be within bounds [1, 1000] | Maximum number of outer iterations per optimization call; together with no_inner_iterations split 25/25/50 across the 3 optimization phases (the first two phases get 1/4 each, the remainder goes to the final full phase). | DiscreteTEBPlanner::optimizeTEB. |
| `FollowPath.optimizer.early_exit_min_delta` | `double` | `0.001` | parameter must be within bounds [0.0, 1.0] | Relative chi² improvement threshold for the per-phase convergence early-exit. A phase stops iterating once the relative improvement between two outer iterations drops below this value (\|chi²_old - chi²_current\| / chi²_old). 0 disables the early-exit (always run the full iteration budget). | DiscreteTEBPlanner::runPhase. |
| `FollowPath.optimizer.penalty_epsilon` | `double` | `0.1` | parameter must be within bounds [0.0, 1.0] | Buffer [s] before a constraint boundary within which the linear penalty ramps up to the weight. | all penalty-based edges (penalties.h). |
| `FollowPath.optimizer.fix_goal` | `bool` | `true` |  | Fix the goal pose vertex (position and orientation cannot be optimized). | buildGraph (goal vertex). |
| `FollowPath.optimizer.free_goal_vel` | `bool` | `false` |  | If true, do not enforce v=0 and angular velocity=0 at the goal (velocity is free at the goal). | buildGraph (vel_goal_ flag) and the goal velocity edges. |
| `FollowPath.optimizer.verbose` | `bool` | `false` |  | Print per-phase optimization statistics and costs. | DiscreteTEBPlanner::optimizeTEB (logging). |
| `FollowPath.optimizer.stepwise_optimization` | `bool` | `true` |  | Enable the 3-phase optimization (1: obstacles+G3+kinematics, 2: kinodynamics, 3: efficiency edges) instead of a single full pass. | DiscreteTEBPlanner::optimizeTEB. |
| `FollowPath.optimizer.divergence_detection_enable` | `bool` | `false` |  | Enable divergence detection (chi², violation rate and path length) with band re-initialization on failure. | DiscreteTEBPlanner::optimizeTEB. |
| `FollowPath.optimizer.divergence_detection_max_chi_squared` | `double` | `10.0` |  | Maximum per-iteration chi² before the optimization is flagged as divergent. | divergence detection in optimizeTEB. |
| `FollowPath.optimizer.divergence_detection_max_chi_violation_rate` | `double` | `0.5` |  | Maximum allowed fraction of iterations with an increasing chi² before the optimization is flagged as divergent. | divergence detection in optimizeTEB. |
| `FollowPath.optimizer.divergence_detection_max_path_length_factor` | `double` | `3.0` |  | Maximum factor of path length growth relative to the initial band before the optimization is flagged as divergent. | divergence detection in optimizeTEB. |

## `weights`

| Param | Type | Default | Constraints | Description | Consumed by |
|---|---|---|---|---|---|
| `FollowPath.weights.weight_adapt_factor` | `double` | `1.0` |  | Factor that ramps up the obstacle weights over the outer iterations (weight multiplier). | DiscreteTEBPlanner::optimizeTEB. |
| `FollowPath.weights.weight_time_optimal` | `double` | `1.0` | parameter must be within bounds [0.0, 1000.0] | Weight of the time-optimality objective pushing the sum of time intervals ΣT_i towards 0, i.e. executing the band as fast as possible. | EdgeTimeOptimal. |
| `FollowPath.weights.weight_shortest_path` | `double` | `1.0` | parameter must be within bounds [0.0, 1000.0] | Weight of the path-length objective pulling the band towards the shortest connection of the poses. | EdgeShortestPath — and note EdgePathSmoothness is wired with this weight as well (weight_path_smoothness is currently unused). |
| `FollowPath.weights.weight_path_smoothness` | `double` | `1.0` | parameter must be within bounds [0.0, 1000.0] | Weight of the path-smoothing edge (EdgePathSmoothness). | none |
| `FollowPath.weights.weight_v_max_x` | `double` | `1.0` | parameter must be within bounds [0.0, 1000.0] | Weight of the maximum linear velocity constraint (x). | EdgeVelocity/EdgeVelocityHolonomic. |
| `FollowPath.weights.weight_v_max_y` | `double` | `1.0` | parameter must be within bounds [0.0, 1000.0] | Weight of the maximum lateral velocity constraint (y, holonomic mode). | EdgeVelocityHolonomic. |
| `FollowPath.weights.weight_v_max_theta` | `double` | `1.0` | parameter must be within bounds [0.0, 1000.0] | Weight of the maximum angular velocity constraint. | EdgeVelocity*. |
| `FollowPath.weights.weight_a_max_x` | `double` | `1.0` | parameter must be within bounds [0.0, 1000.0] | Weight of the maximum linear acceleration constraint. | EdgeAcceleration{Start,Goal}. |
| `FollowPath.weights.weight_jerk_max_x` | `double` | `1.0` | parameter must be within bounds [0.0, 1000.0] | Weight of the maximum linear jerk constraint. | EdgeJerk{Start,Goal}. |
| `FollowPath.weights.weight_jerk_max_theta` | `double` | `1.0` | parameter must be within bounds [0.0, 1000.0] | Weight of the maximum angular jerk constraint. | EdgeJerk{Start,Goal}. |
| `FollowPath.weights.weight_snap_max_x` | `double` | `1.0` | parameter must be within bounds [0.0, 1000.0] | Weight of the maximum linear snap constraint (G4). | EdgeSnap. |
| `FollowPath.weights.weight_snap_max_theta` | `double` | `1.0` | parameter must be within bounds [0.0, 1000.0] | Weight of the maximum angular snap constraint (G4). | EdgeSnap. |
| `FollowPath.weights.weight_a_max_y` | `double` | `1.0` | parameter must be within bounds [0.0, 1000.0] | Weight of the maximum lateral acceleration constraint (holonomic only). | EdgeAccelerationHolonomic{Start,Goal}. |
| `FollowPath.weights.weight_a_max_theta` | `double` | `1.0` | parameter must be within bounds [0.0, 1000.0] | Weight of the maximum angular acceleration constraint. | EdgeAcceleration{Start,Goal}. |
| `FollowPath.weights.weight_kinematics_nh` | `double` | `1.0` | parameter must be within bounds [0.0, 100000.0] | Weight of the non-holonomic (driving) kinematics constraint. | EdgeKinematics{2D,3D} (nonholonomic part). |
| `FollowPath.weights.weight_kinematics_forward_drive` | `double` | `1.0` | parameter must be within bounds [0.0, 1000.0] | Weight penalizing backward motion of a diff-drive robot (forward-drive objective). | EdgeKinematicsDiffDrive. |
| `FollowPath.weights.weight_kinematics_turning_radius` | `double` | `1.0` | parameter must be within bounds [0.0, 1000.0] | Weight of the minimum turning radius constraint (ackermann / car-like). | EdgeKinematicsCarlike. |
| `FollowPath.weights.weight_max_steering_rate` | `double` | `1.0` | parameter must be within bounds [0.0, 1000.0] | Weight of the maximum steering rate constraint. | EdgeSteeringRate{Start,Goal}. |
| `FollowPath.weights.weight_zero_steering_angle_goal` | `double` | `1.0` | parameter must be within bounds [0.0, 1000.0] | Weight driving the steering angle to zero at the goal. | EdgeSteeringAngleGoal. |
| `FollowPath.weights.weight_start_steering_angle` | `double` | `1.0` | parameter must be within bounds [0.0, 1000.0] | Weight keeping the start pose steering angle equal to the measured feedback angle. | EdgeStartSteeringAngle. |
| `FollowPath.weights.weight_goal_angular_vel_zero` | `double` | `1.0` | parameter must be within bounds [0.0, 1000.0] | Weight driving the angular velocity to zero at the goal. | EdgeGoalAngularVelocityZero. |
| `FollowPath.weights.weight_g3_continuity` | `double` | `1.0` | parameter must be within bounds [0.0, 1000.0] | Weight of the G3 continuity edge (bounded change of curvature/steering rate along the band). | EdgeG3Continuity. |
| `FollowPath.weights.weight_obstacle` | `double` | `50.0` | parameter must be within bounds [0.0, 1000.0] | Weight of the ESDF obstacle avoidance per footprint circle. | EdgeESDFObstacle. |
| `FollowPath.weights.weight_inflation` | `double` | `50.0` | parameter must be within bounds [0.0, 1000.0] | Weight of the ESDF inflation term (pushes the band center out of the inflated obstacle area). | EdgeESDFObstacle. |

## `obstacles`

| Param | Type | Default | Constraints | Description | Consumed by |
|---|---|---|---|---|---|
| `FollowPath.obstacles.include_dynamic_obstacles` | `bool` | `false` |  | Include dynamic obstacles in the obstacle container. Reserved — not implemented, currently unused. | none |
| `FollowPath.obstacles.legacy_obstacle_association` | `bool` | `false` |  | Use the legacy costmap cost association for obstacles. | none |
| `FollowPath.obstacles.feasibility_check` | `double` | `1.0` |  | Lookahead distance [m] along the band that is checked for collisions with the ESDF plus footprint. On collision a stop command is issued. | TEBController::computeVelocityCommands (checkFeasibility). |
| `FollowPath.obstacles.min_obstacle_cost` | `int` | `128` |  | Legacy obstacle cost threshold (costmap obstacle association). | none |
| `FollowPath.obstacles.inflation_dist` | `double` | `1.9` |  | Distance [m] around obstacles within which the obstacle potential applies (inflated area). | EdgeESDFObstacle (inflation term). |
| `FollowPath.obstacles.min_obstacle_dist` | `double` | `0.1` |  | Desired minimum distance [m] between the robot contour and the obstacles. | EdgeESDFObstacle (distance cost and gradient). |
| `FollowPath.obstacles.cutoff_dist` | `double` | `1.0` |  | Distance [m] above which the ESDF obstacle gradient is cut off (buffer for the cost function). | EdgeESDFObstacle. |
| `FollowPath.obstacles.cost_exponent` | `double` | `2.0` |  | Exponent of the obstacle cost function (1/d^cost_exponent). | EdgeESDFObstacle. |
| `FollowPath.obstacles.costmap_converter_plugin` | `string` | `""` |  | Plugin name for the costmap→obstacle-polygon conversion (e.g. costmap_converter::CostmapToPolygonsDBSMCCH). | TEBController::initCostmapConverter. |
| `FollowPath.obstacles.costmap_converter_rate` | `double` | `1.0` |  | Rate [Hz] at which the ESDF is re-computed from the costmap. | TEBController::computeVelocityCommands (ESDF update throttling). |
| `FollowPath.obstacles.costmap_converter_spin_thread` | `bool` | `true` |  | Run the costmap conversion in a separate thread. | TEBController::initCostmapConverter. |

## `recovery`

| Param | Type | Default | Constraints | Description | Consumed by |
|---|---|---|---|---|---|
| `FollowPath.recovery.activate` | `bool` | `false` |  | Activate recovery behaviors (planned feature). | none |
| `FollowPath.recovery.divergence_detection_enable` | `bool` | `false` |  | Enable recovery handling on divergence. | none |

## `visualization`

| Param | Type | Default | Constraints | Description | Consumed by |
|---|---|---|---|---|---|
| `FollowPath.visualization.activate` | `bool` | `false` |  | Publish visualization markers. | none |
| `FollowPath.visualization.publish_rate` | `double` | `30.0` | parameter must be within bounds [0.0, 1000.0] | Publish rate [Hz] of the visualization topics, throttled in the control loop (7 publishers; per-topic subscriber gating applies in addition). 0 = publish on every control tick (no throttling). | TEBController::computeVelocityCommands (visualization throttle). |
