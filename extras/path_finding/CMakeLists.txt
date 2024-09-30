cmake_minimum_required(VERSION 3.8)
project(pathfinder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

set(dependencies "ament_index_cpp" "rclcpp" "std_msgs" "geometry_msgs" "nav_msgs" "tf2" "tf2_geometry_msgs" "visualization_msgs")

set(pathfinder_SRCS
  src/pathfinder.cpp
)

set(pathfinder_HDRS
  include/pathfinder.h
)

add_library(pathfinder ${pathfinder_SRCS} ${pathfinder_MOCS})
ament_target_dependencies(pathfinder ${dependencies})

set(placeholder_SRCS
  src/placeholder.cpp
  src/main.cpp
)

set(placeholder_HDRS
  include/placeholder.h
)

add_executable(placeholder ${placeholder_SRCS} ${placeholder_MOCS})
ament_target_dependencies(placeholder ${dependencies})
target_link_libraries(placeholder pathfinder)

install(TARGETS
  pathfinder
  placeholder
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY 
  launch
  map
  rviz
  config
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
