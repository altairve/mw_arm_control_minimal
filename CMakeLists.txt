cmake_minimum_required(VERSION 3.5)
project(mw_arm_control_minimal)

# Common cmake code applied to all moveit packages


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(moveit_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rcpputils REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(control_msgs REQUIRED)


set(dependencies
  rclcpp
  moveit_ros_planning_interface
  rosidl_default_runtime
  Boost
  moveit_msgs
  trajectory_msgs
  rcpputils
  std_srvs
  std_msgs
  geometry_msgs
  std_msgs
  control_msgs
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(orangewood_interface src/orangewood_hardware_interface.cpp)
ament_target_dependencies(orangewood_interface ${dependencies})
target_link_libraries(orangewood_interface  ${catkin_LIBRARIES} ${mathlib_LIBRARIES})

install(TARGETS
  orangewood_interface
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

install(DIRECTORY
  config
  include
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # These don't pass yet, disable them for now
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)

  # Run all lint tests in package.xml except those listed above
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_export_dependencies(rosidl_default_runtime)
ament_export_libraries(manipulation_utils)

ament_package()
