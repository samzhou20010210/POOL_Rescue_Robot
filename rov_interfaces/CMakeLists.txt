cmake_minimum_required(VERSION 3.8)
project(rov_interfaces)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ThrusterCommand.msg"
  "msg/TrajectoryPoint.msg"
  "msg/Trajectory.msg"
  "msg/SwimmerTrack.msg"
  "msg/LeakAlarm.msg"
  "msg/MissionEvent.msg"
  "msg/MissionState.msg"
  "msg/PlannerStatus.msg"
  "msg/SafetyStatus.msg"
  "msg/VictimTarget.msg"
  "msg/SwimmerTrackArray.msg"
  "action/FollowTrajectory.action"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
