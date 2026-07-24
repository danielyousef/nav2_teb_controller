# nav2_teb_controller

A modern C++20 reimplementation of the **Timed Elastic Band (TEB)** local planner for [Nav2](https://nav2.ros.org/) (ROS 2 Jazzy+). Uses polynomial (cubic Hermite) trajectory segments optimized via `libg2o` sparse nonlinear least-squares, supporting differential drive, Ackermann/bicycle, and tricycle kinematic models.

Based on the original [`teb_local_planner`](https://github.com/rst-tu-dortmund/teb_local_planner) by RST – TU Dortmund, redesigned for ROS 2 with improved optimization, ESDF-based obstacle avoidance, and homotopy class planning.

## Features

- **Trajectory optimization via g2o** — Gauss-Newton or Levenberg-Marquardt with 4 solver backends (eigen, cholmod, csparse, dense)
- **31 custom g2o edge types** — velocity, acceleration, jerk, snap, kinematics (diff-drive + car-like), steering rate, time-optimal, shortest-path, path smoothness, G³ continuity, ESDF obstacles, etc.
- **ESDF obstacle avoidance** — Meijster O(n) Euclidean Distance Transform with bilinear interpolation and analytical gradients
- **Footprint-aware collision checking** — circle model, polygon-to-circles decomposition
- **Homotopy Class Planning** (stub) — visibility graph search with H-signature filtering for multi-topology planning
- **Robot models** — differential drive, Ackermann/bicycle, holonomic
- **Dynamic parameter reconfiguration** via `generate_parameter_library`
- **RViz visualization** — 7 topics: local plan, lookahead, poses, obstacles, curvature radii, footprint

## Status

| Component | Status |
|-----------|--------|
| TEBController plugin (lifecycle) | ✅ Complete |
| Trajectory optimization (g2o) | ✅ Complete |
| ESDF (Meijster EDT) | ✅ Complete |
| Footprint collision check | ✅ Complete |
| TEB utilities (init, resize, prune, velocity) | ✅ Complete |
| Velocity saturation (Twist + Ackermann) | ✅ Complete |
| All 31 g2o edge/vertex types | ✅ Complete |
| Generic edge template (`addEdgesGeneric`) | ✅ Complete |
| Parameter library | ✅ Complete |
| Visualization (7 RViz topics) | ✅ Complete |
| Unit tests (gtest) | ✅ Complete |
| HomotopyClassPlanner::plan() | 🚧 Stub |
| VisibilityGraphSearch::search() | 🚧 Stub |
| VisibilityGraph::build() | 🚧 Stub |
| HSignature::compute() | 🚧 Stub |
| EdgeViaPoints | 🚧 Not wired |
| Integration tests | 🚧 Empty |

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
```

See `config/teb_controller_params.yaml` for full parameter reference.

## Tests

```bash
make test                    # all tests
colcon test --packages-select nav2_teb_controller \
  --event-handlers console_direct+ \
  --ctest-args -R test_edge_time_optimal  # single test
```

## References

- C. Rösmann, W. Feiten, T. Wösch, F. Hoffmann, T. Bertram: *Trajectory modification considering dynamic constraints of autonomous robots*, ROBOTIK 2012.
- C. Rösmann, F. Hoffmann, T. Bertram: *Integrated online trajectory planning and optimization in distinctive topologies*, RAS 2017.

## License

BSD 3-Clause. See LICENSE.
