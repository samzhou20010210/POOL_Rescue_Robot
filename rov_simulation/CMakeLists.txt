cmake_minimum_required(VERSION 3.8)
project(rov_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rov_control REQUIRED)
find_package(rov_description REQUIRED)
find_package(rov_interfaces REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
# Gazebo Harmonic libraries (from the ROS 2 Jazzy gz_*_vendor packages).
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-msgs REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED COMPONENTS register)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport REQUIRED)

# Planar dynamics library (does not use ROS). It reuses the V4 allocation
# matrix from rov_control's thrust_allocator library.
add_library(planar_dynamics src/planar_dynamics.cpp)
target_compile_features(planar_dynamics PUBLIC cxx_std_17)
set_target_properties(planar_dynamics PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(planar_dynamics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(planar_dynamics
  PUBLIC rov_control::planar_model rov_control::thrust_allocator Eigen3::Eigen
  PRIVATE yaml-cpp::yaml-cpp)

# Simulated swimmer motion (no ROS) and the fake shore tracker node.
add_library(swimmer_motion src/swimmer_motion.cpp)
target_compile_features(swimmer_motion PUBLIC cxx_std_17)
target_include_directories(swimmer_motion PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(swimmer_motion PUBLIC Eigen3::Eigen)

add_executable(fake_shore_node src/fake_shore_node.cpp)
target_compile_features(fake_shore_node PUBLIC cxx_std_17)
target_link_libraries(fake_shore_node swimmer_motion rclcpp::rclcpp ${rov_interfaces_TARGETS}
  ${std_msgs_TARGETS})

# Simulated battery and leak sensors (step 10).
add_executable(fake_vehicle_sensors_node src/fake_vehicle_sensors_node.cpp)
target_compile_features(fake_vehicle_sensors_node PUBLIC cxx_std_17)
target_link_libraries(fake_vehicle_sensors_node rclcpp::rclcpp ${rov_interfaces_TARGETS}
  ${sensor_msgs_TARGETS})

# Thrust-driven simulator node.
add_executable(dynamics_sim_node src/dynamics_sim_node.cpp)
target_compile_features(dynamics_sim_node PUBLIC cxx_std_17)
target_link_libraries(dynamics_sim_node
  planar_dynamics
  ament_index_cpp::ament_index_cpp
  rclcpp::rclcpp
  tf2_ros::tf2_ros
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${rov_interfaces_TARGETS}
  ${std_srvs_TARGETS})

add_executable(sim_node src/sim_node.cpp)
target_compile_features(sim_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  sim_node
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "tf2_ros"
)

# Gazebo system plugin: thrust for the planar Gazebo backend.
add_library(rov_planar_thruster_system SHARED src/planar_thruster_system.cpp)
target_compile_features(rov_planar_thruster_system PUBLIC cxx_std_17)
target_link_libraries(rov_planar_thruster_system
  planar_dynamics
  gz-msgs::core
  gz-plugin::register
  gz-sim::gz-sim
  gz-transport::core)

# ROS node that forwards thruster_command to the plugin.
add_executable(gazebo_thruster_relay src/gazebo_thruster_relay.cpp)
target_compile_features(gazebo_thruster_relay PUBLIC cxx_std_17)
target_link_libraries(gazebo_thruster_relay
  rclcpp::rclcpp
  ${nav_msgs_TARGETS}
  ${tf2_msgs_TARGETS}
  gz-msgs::core
  gz-transport::core
  ${rov_interfaces_TARGETS}
  ${std_srvs_TARGETS})

install(TARGETS rov_planar_thruster_system
  LIBRARY DESTINATION lib)
install(TARGETS sim_node dynamics_sim_node gazebo_thruster_relay fake_shore_node
  fake_vehicle_sensors_node
  DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/open_loop_thrust_test.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})
install(TARGETS planar_dynamics swimmer_motion
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
  ament_add_gtest(test_swimmer_motion test/test_swimmer_motion.cpp)
  target_link_libraries(test_swimmer_motion swimmer_motion)

  ament_add_gtest(test_planar_dynamics test/test_planar_dynamics.cpp)
  target_link_libraries(test_planar_dynamics planar_dynamics yaml-cpp::yaml-cpp)
  # The tests use the real V4 parameter files from the installed rov_description.
  target_compile_definitions(test_planar_dynamics PRIVATE
    HYDRODYNAMICS_V4_YAML="${rov_description_DIR}/../config/hydrodynamics_v4.yaml"
    THRUSTERS_V4_YAML="${rov_description_DIR}/../config/thrusters_v4.yaml")
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 rov_control yaml-cpp)

ament_package()
