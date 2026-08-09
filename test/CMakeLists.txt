# ── test_edge_time_optimal ────────────────────────────────────────────────────
ament_add_gtest(test_edge_time_optimal
  unit/g2o_types/test_edge_time_optimal.cpp
)
target_include_directories(test_edge_time_optimal PRIVATE ${CMAKE_SOURCE_DIR}/include)
ament_target_dependencies(test_edge_time_optimal rclcpp)
target_link_libraries(test_edge_time_optimal ${PROJECT_NAME})

# ── test_edge_path_smoothness ─────────────────────────────────────────────────
ament_add_gtest(test_edge_path_smoothness
  unit/g2o_types/test_edge_path_smoothness.cpp
)
target_include_directories(test_edge_path_smoothness PRIVATE ${CMAKE_SOURCE_DIR}/include)
ament_target_dependencies(test_edge_path_smoothness rclcpp)
target_link_libraries(test_edge_path_smoothness ${PROJECT_NAME})

# ── test_edge_shortest_path ───────────────────────────────────────────────────
ament_add_gtest(test_edge_shortest_path
  unit/g2o_types/test_edge_shortest_path.cpp
)
target_include_directories(test_edge_shortest_path PRIVATE ${CMAKE_SOURCE_DIR}/include)
ament_target_dependencies(test_edge_shortest_path rclcpp)
target_link_libraries(test_edge_shortest_path ${PROJECT_NAME})

# ── test_edge_velocity ────────────────────────────────────────────────────────
ament_add_gtest(test_edge_velocity
  unit/g2o_types/test_edge_velocity.cpp
)
target_include_directories(test_edge_velocity PRIVATE ${CMAKE_SOURCE_DIR}/include)
ament_target_dependencies(test_edge_velocity rclcpp)
target_link_libraries(test_edge_velocity ${PROJECT_NAME})

# ── test_edge_velocity_holonomic ──────────────────────────────────────────────
ament_add_gtest(test_edge_velocity_holonomic
  unit/g2o_types/test_edge_velocity_holonomic.cpp
)
target_include_directories(test_edge_velocity_holonomic PRIVATE ${CMAKE_SOURCE_DIR}/include)
ament_target_dependencies(test_edge_velocity_holonomic rclcpp)
target_link_libraries(test_edge_velocity_holonomic ${PROJECT_NAME})

# ── test_edge_snap ────────────────────────────────────────────────────────────
ament_add_gtest(test_edge_snap
  unit/g2o_types/test_edge_snap.cpp
)
target_include_directories(test_edge_snap PRIVATE ${CMAKE_SOURCE_DIR}/include)
ament_target_dependencies(test_edge_snap rclcpp)
target_link_libraries(test_edge_snap ${PROJECT_NAME})

# ── test_edge_acceleration ────────────────────────────────────────────────────
ament_add_gtest(test_edge_acceleration
  unit/g2o_types/test_edge_acceleration.cpp
)
target_include_directories(test_edge_acceleration PRIVATE ${CMAKE_SOURCE_DIR}/include)
ament_target_dependencies(test_edge_acceleration rclcpp)
target_link_libraries(test_edge_acceleration ${PROJECT_NAME})

# ── test_edge_acceleration_start ──────────────────────────────────────────────
ament_add_gtest(test_edge_acceleration_start
  unit/g2o_types/test_edge_acceleration_start.cpp
)
target_include_directories(test_edge_acceleration_start PRIVATE ${CMAKE_SOURCE_DIR}/include)
ament_target_dependencies(test_edge_acceleration_start rclcpp)
target_link_libraries(test_edge_acceleration_start ${PROJECT_NAME})

# ── test_edge_acceleration_goal ───────────────────────────────────────────────
ament_add_gtest(test_edge_acceleration_goal
  unit/g2o_types/test_edge_acceleration_goal.cpp
)
target_include_directories(test_edge_acceleration_goal PRIVATE ${CMAKE_SOURCE_DIR}/include)
ament_target_dependencies(test_edge_acceleration_goal rclcpp)
target_link_libraries(test_edge_acceleration_goal ${PROJECT_NAME})

# ── test_edge_acceleration_holonomic ──────────────────────────────────────────
ament_add_gtest(test_edge_acceleration_holonomic
  unit/g2o_types/test_edge_acceleration_holonomic.cpp
)
target_include_directories(test_edge_acceleration_holonomic PRIVATE ${CMAKE_SOURCE_DIR}/include)
ament_target_dependencies(test_edge_acceleration_holonomic rclcpp)
target_link_libraries(test_edge_acceleration_holonomic ${PROJECT_NAME})

# ── test_edge_acceleration_holonomic_start ────────────────────────────────────
ament_add_gtest(test_edge_acceleration_holonomic_start
  unit/g2o_types/test_edge_acceleration_holonomic_start.cpp
)
target_include_directories(test_edge_acceleration_holonomic_start PRIVATE ${CMAKE_SOURCE_DIR}/include)
ament_target_dependencies(test_edge_acceleration_holonomic_start rclcpp)
target_link_libraries(test_edge_acceleration_holonomic_start ${PROJECT_NAME})

# ── test_edge_acceleration_holonomic_goal ─────────────────────────────────────
ament_add_gtest(test_edge_acceleration_holonomic_goal
  unit/g2o_types/test_edge_acceleration_holonomic_goal.cpp
)
target_include_directories(test_edge_acceleration_holonomic_goal PRIVATE ${CMAKE_SOURCE_DIR}/include)
ament_target_dependencies(test_edge_acceleration_holonomic_goal rclcpp)
target_link_libraries(test_edge_acceleration_holonomic_goal ${PROJECT_NAME})

# ── test_edge_jerk ────────────────────────────────────────────────────────────
ament_add_gtest(test_edge_jerk
  unit/g2o_types/test_edge_jerk.cpp
)
target_include_directories(test_edge_jerk PRIVATE ${CMAKE_SOURCE_DIR}/include)
ament_target_dependencies(test_edge_jerk rclcpp)
target_link_libraries(test_edge_jerk ${PROJECT_NAME})

# ── test_edge_esdf_obstacle ───────────────────────────────────────────────────
ament_add_gtest(test_edge_esdf_obstacle
  unit/g2o_types/test_edge_esdf_obstacle.cpp
)
target_include_directories(test_edge_esdf_obstacle PRIVATE ${CMAKE_SOURCE_DIR}/include)
ament_target_dependencies(test_edge_esdf_obstacle rclcpp)
target_link_libraries(test_edge_esdf_obstacle ${PROJECT_NAME})

# ── test_edge_g3_continuity ───────────────────────────────────────────────────
ament_add_gtest(test_edge_g3_continuity
  unit/g2o_types/test_edge_g3_continuity.cpp
)
target_include_directories(test_edge_g3_continuity PRIVATE ${CMAKE_SOURCE_DIR}/include)
ament_target_dependencies(test_edge_g3_continuity rclcpp)
target_link_libraries(test_edge_g3_continuity ${PROJECT_NAME})

# ── test_edge_kinematics_diff ─────────────────────────────────────────────────
ament_add_gtest(test_edge_kinematics_diff
  unit/g2o_types/test_edge_kinematics_diff.cpp
)
target_include_directories(test_edge_kinematics_diff PRIVATE ${CMAKE_SOURCE_DIR}/include)
ament_target_dependencies(test_edge_kinematics_diff rclcpp)
target_link_libraries(test_edge_kinematics_diff ${PROJECT_NAME})

# ── test_edge_kinematic_car_like ──────────────────────────────────────────────
ament_add_gtest(test_edge_kinematic_car_like
  unit/g2o_types/test_edge_kinematic_car_like.cpp
)
target_include_directories(test_edge_kinematic_car_like PRIVATE ${CMAKE_SOURCE_DIR}/include)
ament_target_dependencies(test_edge_kinematic_car_like rclcpp)
target_link_libraries(test_edge_kinematic_car_like ${PROJECT_NAME})

# ── test_edge_steering_rate ───────────────────────────────────────────────────
ament_add_gtest(test_edge_steering_rate
  unit/g2o_types/test_edge_steering_rate.cpp
)
target_include_directories(test_edge_steering_rate PRIVATE ${CMAKE_SOURCE_DIR}/include)
ament_target_dependencies(test_edge_steering_rate rclcpp)
target_link_libraries(test_edge_steering_rate ${PROJECT_NAME})

# ── test_edge_steering_rate_start ─────────────────────────────────────────────
ament_add_gtest(test_edge_steering_rate_start
  unit/g2o_types/test_edge_steering_rate_start.cpp
)
target_include_directories(test_edge_steering_rate_start PRIVATE ${CMAKE_SOURCE_DIR}/include)
ament_target_dependencies(test_edge_steering_rate_start rclcpp)
target_link_libraries(test_edge_steering_rate_start ${PROJECT_NAME})

# ── test_edge_steering_rate_goal ──────────────────────────────────────────────
ament_add_gtest(test_edge_steering_rate_goal
  unit/g2o_types/test_edge_steering_rate_goal.cpp
)
target_include_directories(test_edge_steering_rate_goal PRIVATE ${CMAKE_SOURCE_DIR}/include)
ament_target_dependencies(test_edge_steering_rate_goal rclcpp)
target_link_libraries(test_edge_steering_rate_goal ${PROJECT_NAME})

# ── test_edge_start_steering_angle ────────────────────────────────────────────
ament_add_gtest(test_edge_start_steering_angle
  unit/g2o_types/test_edge_start_steering_angle.cpp
)
target_include_directories(test_edge_start_steering_angle PRIVATE ${CMAKE_SOURCE_DIR}/include)
ament_target_dependencies(test_edge_start_steering_angle rclcpp)
target_link_libraries(test_edge_start_steering_angle ${PROJECT_NAME})

# ── test_edge_steering_angle_goal ─────────────────────────────────────────────
ament_add_gtest(test_edge_steering_angle_goal
  unit/g2o_types/test_edge_steering_angle_goal.cpp
)
target_include_directories(test_edge_steering_angle_goal PRIVATE ${CMAKE_SOURCE_DIR}/include)
ament_target_dependencies(test_edge_steering_angle_goal rclcpp)
target_link_libraries(test_edge_steering_angle_goal ${PROJECT_NAME})

# ── test_edge_goal_angular_velocity_zero ──────────────────────────────────────
ament_add_gtest(test_edge_goal_angular_velocity_zero
  unit/g2o_types/test_edge_goal_angular_velocity_zero.cpp
)
target_include_directories(test_edge_goal_angular_velocity_zero PRIVATE ${CMAKE_SOURCE_DIR}/include)
ament_target_dependencies(test_edge_goal_angular_velocity_zero rclcpp)
target_link_libraries(test_edge_goal_angular_velocity_zero ${PROJECT_NAME})
