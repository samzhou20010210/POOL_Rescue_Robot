cmake_minimum_required(VERSION 3.8)
project(rov_description)

find_package(ament_cmake REQUIRED)

# Install the URDF, mesh, and thruster layout into share/rov_description/ so
# that "package://rov_description/..." resolves from the installed package.
install(DIRECTORY urdf meshes config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
