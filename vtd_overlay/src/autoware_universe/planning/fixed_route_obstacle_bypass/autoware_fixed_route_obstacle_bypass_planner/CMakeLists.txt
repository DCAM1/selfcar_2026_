cmake_minimum_required(VERSION 3.14)
project(autoware_fixed_route_obstacle_bypass_planner)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME}_core SHARED
  src/corridor.cpp
  src/geometry.cpp
  src/object_tracker.cpp
  src/parameters.cpp
  src/trajectory.cpp
)

ament_auto_add_library(${PROJECT_NAME}_nodes SHARED
  src/final_validator_node.cpp
  src/planner_node.cpp
)
target_link_libraries(${PROJECT_NAME}_nodes ${PROJECT_NAME}_core)

rclcpp_components_register_node(${PROJECT_NAME}_nodes
  PLUGIN "autoware::fixed_route_obstacle_bypass::FixedRouteObstacleBypassPlannerNode"
  EXECUTABLE fixed_route_obstacle_bypass_planner_node
  EXECUTOR MultiThreadedExecutor
)
rclcpp_components_register_node(${PROJECT_NAME}_nodes
  PLUGIN "autoware::fixed_route_obstacle_bypass::FixedRouteFinalTrajectoryValidatorNode"
  EXECUTABLE fixed_route_final_trajectory_validator_node
  EXECUTOR MultiThreadedExecutor
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_auto_add_gtest(test_fixed_route_core
    test/test_corridor.cpp
    test/test_geometry.cpp
    test/test_object_tracker.cpp
    test/test_trajectory.cpp
  )
  target_link_libraries(test_fixed_route_core ${PROJECT_NAME}_core)
endif()

autoware_ament_auto_package(INSTALL_TO_SHARE config launch)
