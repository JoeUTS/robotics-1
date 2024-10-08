cmake_minimum_required(VERSION 3.8)
project(circle_remove)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

set(dependencies "ament_index_cpp" "rclcpp" "rclcpp_action" "std_msgs" "geometry_msgs" "nav_msgs" "nav2_msgs" "sensor_msgs" "image_transport" "cv_bridge" "OpenCV")

set(circle_remove_SRCS
  src/circle_remove.cpp
  src/main.cpp
)

set(circle_remove_HDRS
  src/circle_remove.h
)

add_executable(circle_remove ${circle_remove_SRCS} ${circle_remove_MOCS})
ament_target_dependencies(circle_remove ${dependencies})

install(TARGETS
circle_remove
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
