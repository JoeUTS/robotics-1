cmake_minimum_required(VERSION 3.8)
project(cost_map_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)

set(dependencies "ament_index_cpp" "rclcpp" "nav2_costmap_2d")

set(cost_map_test_SRCS
  src/HazardCostMap.cpp
)

set(cost_map_test_HDRS
  src/HazardCostMap.h
)

add_executable(cost_map_test ${cost_map_test_SRCS} ${cost_map_test_MOCS})
ament_target_dependencies(cost_map_test ${dependencies})

install(TARGETS
cost_map_test
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY 
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
