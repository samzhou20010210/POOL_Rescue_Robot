cmake_minimum_required(VERSION 3.8)
project(rov_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rov_description REQUIRED)
find_package(rov_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# Thrust allocation library (does not use ROS): wrench -> 8 thruster forces.
add_library(thrust_allocator src/thrust_allocator.cpp)
target_compile_features(thrust_allocator PUBLIC cxx_std_17)
# Position-independent, so it can also be linked into shared libraries
# (the Gazebo plugin in rov_simulation).
set_target_properties(thrust_allocator PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(thrust_allocator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(thrust_allocator PUBLIC Eigen3::Eigen PRIVATE yaml-cpp::yaml-cpp)

# Planar hydrodynamic model (does not use ROS), shared with rov_simulation.
add_library(planar_model src/planar_model.cpp)
target_compile_features(planar_model PUBLIC cxx_std_17)
set_target_properties(planar_model PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(planar_model PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(planar_model PUBLIC Eigen3::Eigen PRIVATE yaml-cpp::yaml-cpp)

# Trajectory tracking and the path-to-trajectory converter (no ROS).
add_library(trajectory_tracking src/tracking_controller.cpp)
target_compile_features(trajectory_tracking PUBLIC cxx_std_17)
set_target_properties(trajectory_tracking PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(trajectory_tracking PUBLIC planar_model)

# FollowTrajectory server: PD + model feedforward -> wrench_command.
add_executable(trajectory_controller_node src/trajectory_controller_node.cpp)
target_compile_features(trajectory_controller_node PUBLIC cxx_std_17)
target_link_libraries(trajectory_controller_node
  trajectory_tracking
  ament_index_cpp::ament_index_cpp
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2_ros::tf2_ros
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${rov_interfaces_TARGETS})

# Temporary trajectory source: nav_msgs/Path -> FollowTrajectory goal.
# Thin ROS 2 node around the library.
add_executable(thrust_allocator_node src/thrust_allocator_node.cpp)
target_compile_features(thrust_allocator_node PUBLIC cxx_std_17)
target_link_libraries(thrust_allocator_node
  thrust_allocator
  ament_index_cpp::ament_index_cpp
  rclcpp::rclcpp
  ${diagnostic_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${rov_interfaces_TARGETS}
  ${std_msgs_TARGETS})

add_executable(controller_node src/controller_node.cpp)
target_compile_features(controller_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  controller_node
  "rclcpp"
  "rclcpp_action"
  "geometry_msgs"
  "nav_msgs"
  "nav2_msgs"
  "tf2"
  "tf2_geometry_msgs"
  "tf2_ros"
)

install(TARGETS controller_node thrust_allocator_node trajectory_controller_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})
install(TARGETS thrust_allocator planar_model trajectory_tracking
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_thrust_allocator test/test_thrust_allocator.cpp)
  target_link_libraries(test_thrust_allocator thrust_allocator)
  # The tests use the real V4 layout from the installed rov_description.
  target_compile_definitions(test_thrust_allocator PRIVATE
    THRUSTERS_V4_YAML="${rov_description_DIR}/../config/thrusters_v4.yaml")

  ament_add_gtest(test_tracking_controller test/test_tracking_controller.cpp)
  target_link_libraries(test_tracking_controller trajectory_tracking)
  target_compile_definitions(test_tracking_controller PRIVATE
    HYDRODYNAMICS_V4_YAML="${rov_description_DIR}/../config/hydrodynamics_v4.yaml")
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 yaml-cpp)

ament_package()
