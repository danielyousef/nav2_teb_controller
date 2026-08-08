# nav2_teb_controller

A modern C++20 reimplementation of the **classic (discrete) Timed Elastic Band (TEB)** local planner for
[Nav2](https://nav2.ros.org/) (ROS 2 Jazzy+). The trajectory is a band of SE(2) poses with time steps (Δt);
velocities and higher derivatives are computed by finite differences. The band is deformed by **sparse
nonlinear least-squares optimization** (`libg2o`, Gauss-Newton / Levenberg-Marquardt) considering kinematics,
dynamics, obstacle avoidance, and execution time.

Based on the original [`teb_local_planner`](https://github.com/rst-tu-dortmund/teb_local_planner) by
RST – TU Dortmund, redesigned for ROS 2 with ESDF-based obstacle avoidance.

> **Documentation (start here)**
> - [`doc/architecture.md`](doc/architecture.md) — system architecture, modules, control-loop data flow (diagrams)
> - [`doc/control_concept.md`](doc/control_concept.md) — TEB theory, g2o graph formulation, per-edge math, control law
> - [`doc/parameters.md`](doc/parameters.md) — full parameter reference (**auto-generated** from
>   `config/teb_controller_parameters.yaml` via `make docs`; edit the schema, not the doc)
> - [`doc/status.md`](doc/status.md) — status & roadmap (implemented / planned features), gotchas & traps
> - [`AGENTS.md`](AGENTS.md) — developer cheat-sheet (structure, references to the docs above)

## Features

- **Trajectory optimization via g2o** — Gauss-Newton or Levenberg-Marquardt, 4 solver backends (eigen, cholmod,
  csparse, dense)
- **27 custom g2o edge classes** (21 wired into the graph) — velocity, acceleration, jerk, snap, kinematics
  (diff-drive + car-like), steering rate/angle, G³ continuity, time-optimal, shortest path, path smoothness,
  ESDF obstacles
- **ESDF obstacle avoidance** — Meijster O(n) Euclidean Distance Transform, bilinear interpolation + analytical
  gradients, per-footprint-circle soft constraints with robust kernel
- **Footprint-aware collision checking** — circle model (polygon → circles), hard feasibility stop
- **3-phase stepwise optimization** — obstacles+kinematics → kinodynamics → efficiency objectives
- **Robot models** — `diff_drive` (forward-drive preference) and `ackermann` (turning radius, steering rate,
  steering feedback from `/tricycle_state`); holonomic mode by `v_max_y > 0`
- **Dynamic parameter reconfiguration** via `generate_parameter_library`
- **RViz visualization** — 7 topics: local plan, lookahead, poses, obstacles, curvature radii, footprint

## Videos

| New TEB RViz<br/>[<img src="https://img.youtube.com/vi/BqfFwOXZO0k/hqdefault.jpg" width="240" alt="New TEB RViz"/>](https://www.youtube.com/watch?v=BqfFwOXZO0k) | New TEB RViz obstacles<br/>[<img src="https://img.youtube.com/vi/a3creSJZxis/hqdefault.jpg" width="240" alt="New TEB RViz obstacles"/>](https://www.youtube.com/watch?v=a3creSJZxis) | New TEB RViz loose goal<br/>[<img src="https://img.youtube.com/vi/3201fSL2xvI/hqdefault.jpg" width="240" alt="New TEB RViz loose goal"/>](https://www.youtube.com/watch?v=3201fSL2xvI) |
|:---:|:---:|:---:|

## Status

The detailed status & roadmap (with gotchas) lives in [`doc/status.md`](doc/status.md). In short:

**Implemented**
- TEBController plugin lifecycle + Nav2 controller API (`setPlan`, `computeVelocityCommands`, `setSpeedLimit`)
- g2o trajectory optimization: 3-phase stepwise, 27 edge classes (21 wired), 2 vertex types, `addEdgesGeneric` factory
- ESDF obstacle avoidance (Meijster EDT, gradients, per-footprint-circle soft constraints)
- Footprint collision check (circle model), hard feasibility stop
- TEB utilities: init, autoResize, prune, velocity extraction, Ackermann↔Twist conversion, saturation
- Robot models: `diff_drive`, `ackermann` (steering feedback via `/tricycle_state`); holonomic by `v_max_y > 0`
- Parameters via `generate_parameter_library`, RViz visualization (7 topics), gtest unit tests

**Planned / not yet implemented**
- Homotopy Class Planning (interfaces only, `plan()` is a stub) — keep `hcp.activate: false`
- Via-points, preferred rotation direction, legacy obstacle association (edges not wired)
- ESDF-aware autoResize, recovery behaviors, integration tests (`test/integration/` empty)
- TEB-K (cubic Hermite spline segments) — research feature, not implemented

## Quick Start

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/danielyousef/nav2_teb_controller.git # This repo
cd ~/ros2_ws
sudo apt update
sudo apt install python3-vcstool
vcs import src < src/nav2_teb_controller/nav2_teb_controller.repos
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to nav2_teb_controller
source install/setup.bash
```

## Configuration

Add to Nav2 params:

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "nav2_teb_controller::TEBController"
      # ... see config/teb_controller_params.yaml for a complete tuned example
```

Key defaults to be aware of (see `doc/parameters.md`):
- `hcp.activate` must stay `false` (homotopy planning not implemented).
- `robot_model` ∈ {`diff_drive`, `ackermann`} (`"bicycle"` was removed from the schema).
- `v_max_y > 0` switches the planner to holonomic (default schema value is `0.5` — set `0.0` for non-holonomic).

## Tests

```bash
make test                    # all tests
colcon test --packages-select nav2_teb_controller \
  --event-handlers console_direct+ \
  --ctest-args -R test_edge_time_optimal  # single test
```

## Parameter Documentation

`doc/parameters.md` is **generated** from the parameter schema
[`config/teb_controller_parameters.yaml`](config/teb_controller_parameters.yaml) (the single source of truth —
also used by `generate_parameter_library` at build time and shown at runtime by `ros2 param describe`):

```bash
make docs          # regenerate doc/parameters.md
make docs-check    # fail if doc/parameters.md is out of date (CI)
```

Never edit `doc/parameters.md` by hand — change the schema descriptions instead.

## References

- C. Rösmann, W. Feiten, T. Wösch, F. Hoffmann, T. Bertram: *Trajectory modification considering dynamic constraints of autonomous robots*, ROBOTIK 2012.
- C. Rösmann, F. Hoffmann, T. Bertram: *Integrated online trajectory planning and optimization in distinctive topologies*, RAS 2017.

## License

BSD 3-Clause. See LICENSE.
