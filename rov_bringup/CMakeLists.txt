cmake_minimum_required(VERSION 3.8)
project(rov_bringup)

find_package(ament_cmake REQUIRED)

# Install launch files, RViz configs, Nav2 parameters, maps, and Gazebo
# worlds and models into share/rov_bringup/ so "ros2 launch rov_bringup ..."
# can find them.
install(DIRECTORY launch rviz config maps worlds models
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
