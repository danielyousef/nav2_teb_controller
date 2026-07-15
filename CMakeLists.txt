cmake_minimum_required(VERSION 3.20)
project(nav2_teb_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Build type default ────────────────────────────────────────────────────────
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# ── Options ───────────────────────────────────────────────────────────────────
option(TEB_CONTROLLER_OPTIMIZER_BACKEND "Optimizer backend: g2o or ceres" "g2o")
option(USE_ANALYTIC_JACOBI
  "Analytic (closed-form) g2o edge Jacobians; OFF = g2o numeric differentiation (slow)" ON)

# Passed to the library AND the test targets (the edge classes are header-only templates)
add_compile_definitions(USE_ANALYTIC_JACOBI=$<BOOL:${USE_ANALYTIC_JACOBI}>)

# ── Dependencies ──────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(costmap_converter REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(g2o REQUIRED)

# ── Parameter library (code generation) ──────────────────────────────────────
generate_parameter_library(
  teb_controller_parameters
  config/teb_controller_parameters.yaml
)

# ── Source files ──────────────────────────────────────────────────────────────
set(SOURCES
  src/teb_controller.cpp
  src/core/teb_utils.cpp
  src/path_handler.cpp
  src/band_controller/band_controller.cpp
  src/band_controller/feed_forward_controller.cpp
  #src/core/timed_elastic_band.cpp
  #src/planner/graph_search.cpp
  src/homotopy/homotopy_class_planner.cpp
  src/homotopy/visibility_graph_search.cpp
  src/homotopy/visibility_graph.cpp
  src/homotopy/h_signature.cpp
  src/planner/optimal_planner.cpp
  src/obstacles/esdf.cpp
  #src/recovery/recovery_behaviors.cpp
  #src/visualization/visualization.cpp
)

# ── Plugin library ────────────────────────────────────────────────────────────
add_library(nav2_teb_controller SHARED ${SOURCES})

# Header shim: shadows the deprecated <tl_expected/expected.hpp> from
# ros-jazzy-parameter-traits
target_include_directories(nav2_teb_controller PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include_shim
)

target_include_directories(nav2_teb_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

target_include_directories(nav2_teb_controller SYSTEM PUBLIC
  ${costmap_converter_INCLUDE_DIRS}
)

# ament deps FIRST — establishes plain signature for target_link_libraries
ament_target_dependencies(nav2_teb_controller
  rclcpp
  rclcpp_lifecycle
  nav2_core
  costmap_converter
  nav2_costmap_2d
  nav2_util
  nav_msgs
  geometry_msgs
  ackermann_msgs
  visualization_msgs
  tf2_geometry_msgs
  pluginlib
  angles
)

# All remaining libs — plain signature to match ament_target_dependencies
target_link_libraries(nav2_teb_controller
  teb_controller_parameters
  g2o::core
  g2o::stuff
  g2o::solver_eigen      # 
  g2o::solver_cholmod    # 
  g2o::solver_csparse    # 
  g2o::solver_dense      # 
  Eigen3::Eigen
)

# ── pluginlib export ──────────────────────────────────────────────────────────
pluginlib_export_plugin_description_file(nav2_core nav2_teb_controller_plugin.xml)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS nav2_teb_controller teb_controller_parameters
  EXPORT nav2_teb_controller_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/ launch/
  DESTINATION share/${PROJECT_NAME}/config
)

# ── Tests ─────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Alle ament_lint checks deaktivieren
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_cppcheck_FOUND TRUE)
  set(ament_cmake_lint_cmake_FOUND TRUE)
  set(ament_cmake_xmllint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_pep257_FOUND TRUE)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  add_subdirectory(test)
endif()

# ── Ament export ──────────────────────────────────────────────────────────────
ament_export_include_directories(include)
ament_export_libraries(nav2_teb_controller)
ament_export_targets(nav2_teb_controller_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  nav2_core
  costmap_converter
  nav2_costmap_2d
  nav2_util
  nav_msgs
  ackermann_msgs
  geometry_msgs
  visualization_msgs
  tf2_geometry_msgs
  pluginlib
  Eigen3
  g2o
  angles
)  

ament_package()
