cmake_minimum_required(VERSION 3.8)
project(tutorial1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

#rosidl_generate_interfaces(${PROJECT_NAME}
#)

set(dependencies "ament_index_cpp" "rclcpp" "std_msgs" "sensor_msgs" "image_transport" "cv_bridge" "OpenCV")

set(image_converter_SRCS
  src/imageconverter.cpp
  src/main.cpp
)

set(image_converter_HDRS
  include/imageconverter.h
)

#rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

add_executable(image_converter ${image_converter_SRCS} ${image_converter_MOCS})
ament_target_dependencies(image_converter ${dependencies})
target_link_libraries(image_converter "${cpp_typesupport_target}")

install(TARGETS image_converter
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
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
