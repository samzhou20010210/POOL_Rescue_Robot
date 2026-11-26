cmake_minimum_required(VERSION 3.8)
project(rov_planning)

# Numerical code (Eigen, FIRI): without optimization the corridor takes
# about 150x longer, too slow for 5 Hz replanning. Build optimized unless a
# build type is given.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rov_control REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)
# ROS packages for trajectory_generator_node only (the library has no ROS).
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rov_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(visualization_msgs REQUIRED)

# Planning core (does not use ROS). The vendored GCOPTER headers are a
# SYSTEM include so their warnings do not hide ours.
add_library(planning_core
  src/corridor_2d.cpp
  src/planning_worker.cpp
  src/rov_model.cpp
  src/surface_traj_opt.cpp
  src/swimmer_prediction.cpp
  src/reference_trajectory.cpp
  src/terminal_pose.cpp
  src/wrench_polytope.cpp)
target_compile_features(planning_core PUBLIC cxx_std_17)
set_target_properties(planning_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(planning_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_include_directories(planning_core SYSTEM PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/gcopter/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}/third_party>)
target_link_libraries(planning_core PUBLIC
  rov_control::planar_model
  rov_control::thrust_allocator
  Eigen3::Eigen)
target_link_libraries(planning_core PRIVATE yaml-cpp::yaml-cpp)
target_link_libraries(planning_core PUBLIC Threads::Threads)

# Swimmer tracks from ROS messages (TF at the measurement stamp), shared by
# the swimmer layer and the trajectory generator (step 9).
add_library(swimmer_tracks_ros STATIC src/swimmer_tracks_ros.cpp)
target_compile_features(swimmer_tracks_ros PUBLIC cxx_std_17)
set_target_properties(swimmer_tracks_ros PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(swimmer_tracks_ros PUBLIC
  planning_core
  rclcpp::rclcpp
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2_ros::tf2_ros
  ${rov_interfaces_TARGETS})

# Nav2 costmap layer for predicted swimmer positions (step 9).
add_library(swimmer_layer SHARED src/swimmer_layer.cpp)
target_compile_features(swimmer_layer PUBLIC cxx_std_17)
target_link_libraries(swimmer_layer
  swimmer_tracks_ros
  ament_index_cpp::ament_index_cpp
  nav2_costmap_2d::nav2_costmap_2d_core
  pluginlib::pluginlib
  rclcpp::rclcpp
  tf2::tf2
  ${rov_interfaces_TARGETS})
pluginlib_export_plugin_description_file(nav2_costmap_2d swimmer_layer_plugin.xml)
install(TARGETS swimmer_layer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Single-shot surface-approach planning node (step 8B).
add_executable(trajectory_generator_node src/trajectory_generator_node.cpp)
target_compile_features(trajectory_generator_node PUBLIC cxx_std_17)
target_link_libraries(trajectory_generator_node
  swimmer_tracks_ros
  planning_core
  ament_index_cpp::ament_index_cpp
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2_ros::tf2_ros
  ${geometry_msgs_TARGETS}
  ${nav2_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${rov_interfaces_TARGETS}
  ${std_msgs_TARGETS}
  ${visualization_msgs_TARGETS})
install(TARGETS trajectory_generator_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})
install(DIRECTORY third_party/gcopter/include/
  DESTINATION include/${PROJECT_NAME}/third_party)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/plot_surface_trajectory.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES third_party/gcopter/LICENSE third_party/gcopter/README.md
  DESTINATION share/${PROJECT_NAME}/third_party/gcopter)
install(TARGETS planning_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo
  set(ament_cmake_cpplint_FOUND TRUE)
  # Vendored third-party code keeps its upstream style and license headers.
  file(GLOB_RECURSE THIRD_PARTY_FILES "${CMAKE_CURRENT_SOURCE_DIR}/third_party/*")
  set(AMENT_LINT_AUTO_FILE_EXCLUDE ${THIRD_PARTY_FILES})
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(rov_description REQUIRED)
  ament_add_gtest(test_planning_core test/test_planning_core.cpp)
  target_link_libraries(test_planning_core planning_core)
  target_compile_definitions(test_planning_core PRIVATE
    HYDRODYNAMICS_V4_YAML="${rov_description_DIR}/../config/hydrodynamics_v4.yaml"
    THRUSTERS_V4_YAML="${rov_description_DIR}/../config/thrusters_v4.yaml")

  ament_add_gtest(test_reference_trajectory test/test_reference_trajectory.cpp)
  target_link_libraries(test_reference_trajectory planning_core)

  ament_add_gtest(test_swimmer_prediction test/test_swimmer_prediction.cpp)
  target_link_libraries(test_swimmer_prediction planning_core)

  ament_add_gtest(test_swimmer_tracks_ros test/test_swimmer_tracks_ros.cpp)
  target_link_libraries(test_swimmer_tracks_ros swimmer_tracks_ros)

  ament_add_gtest(test_terminal_pose test/test_terminal_pose.cpp)
  target_link_libraries(test_terminal_pose planning_core)
  target_compile_definitions(test_terminal_pose PRIVATE
    TERMINAL_POSE_YAML="${CMAKE_CURRENT_SOURCE_DIR}/config/terminal_pose.yaml")

  ament_add_gtest(test_planning_worker test/test_planning_worker.cpp)
  target_link_libraries(test_planning_worker planning_core)
  target_compile_definitions(test_planning_worker PRIVATE
    HYDRODYNAMICS_V4_YAML="${rov_description_DIR}/../config/hydrodynamics_v4.yaml"
    THRUSTERS_V4_YAML="${rov_description_DIR}/../config/thrusters_v4.yaml"
    OPTIMIZER_YAML="${CMAKE_CURRENT_SOURCE_DIR}/config/surface_trajectory_optimizer.yaml")

  ament_add_gtest(test_warm_start test/test_warm_start.cpp TIMEOUT 120)
  target_link_libraries(test_warm_start planning_core)
  target_compile_definitions(test_warm_start PRIVATE
    HYDRODYNAMICS_V4_YAML="${rov_description_DIR}/../config/hydrodynamics_v4.yaml"
    THRUSTERS_V4_YAML="${rov_description_DIR}/../config/thrusters_v4.yaml"
    OPTIMIZER_YAML="${CMAKE_CURRENT_SOURCE_DIR}/config/surface_trajectory_optimizer.yaml")

  # The feedforward check runs the trajectory on the lightweight model.
  find_package(rov_simulation REQUIRED)
  ament_add_gtest(test_surface_traj_opt test/test_surface_traj_opt.cpp TIMEOUT 120)
  target_link_libraries(test_surface_traj_opt planning_core rov_simulation::planar_dynamics)
  target_compile_definitions(test_surface_traj_opt PRIVATE
    HYDRODYNAMICS_V4_YAML="${rov_description_DIR}/../config/hydrodynamics_v4.yaml"
    THRUSTERS_V4_YAML="${rov_description_DIR}/../config/thrusters_v4.yaml"
    ALLOCATOR_YAML="${rov_control_DIR}/../config/thrust_allocator.yaml"
    OPTIMIZER_YAML="${CMAKE_CURRENT_SOURCE_DIR}/config/surface_trajectory_optimizer.yaml")
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
# Threads: planning_core links Threads::Threads publicly (PlanningWorker).
ament_export_dependencies(Eigen3 rov_control Threads yaml-cpp)

ament_package()
