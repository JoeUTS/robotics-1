cmake_minimum_required(VERSION 3.8)
project(lab2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)


#rosidl_generate_interfaces(${PROJECT_NAME}
# empty - place custom messages and services here
#)

set(dependencies "ament_index_cpp" "rclcpp" "std_msgs" "sensor_msgs")

set(EXE_NAME_SRCS
  # nil
)

set(EXE_NAME_HDRS
  # nil
)

#rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

add_executable(EXE_NAME ${EXE_NAME_SRCS} ${EXE_NAME_MOCS})
ament_target_dependencies(EXE_NAME ${dependencies})
target_link_libraries(EXE_NAME "${cpp_typesupport_target}")

install(TARGETS EXE_NAME
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
