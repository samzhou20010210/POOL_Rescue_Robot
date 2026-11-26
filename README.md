# ROV Simulation Demo (ROS 2 Jazzy)

A small ROS 2 Jazzy demo of an underwater ROV (the V4 model) moving in a
planar pool. You click a goal in RViz with **2D Goal Pose**, Nav2 plans a
route on a 2D map, and the controller drives the ROV along it. The ROV is
omnidirectional: it moves forward and sideways while holding its heading,
then turns to the goal heading and stops.

The robot can be simulated by a simple built-in simulator (`sim_node`) or by
**Gazebo Harmonic** (a 50 m × 25 m pool world).

## Packages

| Package | What it does |
|---|---|
| `rov_mission` | `mission_node`: receives RViz goals, asks Nav2 for a route, sends it to the controller |
| `rov_interfaces` | Messages and actions: `ThrusterCommand`, `Trajectory`, `FollowTrajectory` |
| `rov_control` | `controller_node`: `follow_path` action, omnidirectional path tracking. `thrust_allocator_node`: body wrench → 8 thruster forces |
| `rov_simulation` | `sim_node`: kinematic planar simulator (`/cmd_vel` in). `dynamics_sim_node`: thrust-driven planar model (`thruster_command` in). Gazebo thrust plugin and relay. Both publish `/odom` + TF |
| `rov_description` | V4 mesh, visual URDF, thruster layout (`config/thrusters_v4.yaml`), hydrodynamic parameters (`config/hydrodynamics_v4.yaml`) |
| `rov_bringup` | Launch files, maps, RViz configs, Nav2 settings, Gazebo world and model |

## Dependencies

- Ubuntu 24.04 with **ROS 2 Jazzy** (`ros-jazzy-desktop`)
- Nav2 (planning only), robot_state_publisher, and Gazebo Harmonic with the ROS bridge:

```bash
sudo apt update
sudo apt install \
  ros-jazzy-nav2-msgs ros-jazzy-nav2-map-server ros-jazzy-nav2-planner \
  ros-jazzy-nav2-navfn-planner ros-jazzy-nav2-lifecycle-manager \
  ros-jazzy-robot-state-publisher ros-jazzy-tf2-geometry-msgs \
  ros-jazzy-ros-gz
```

Or let rosdep install everything listed in the `package.xml` files:

```bash
cd ~/main/pr_robot/rov_ws
rosdep install --from-paths src --ignore-src -y
```

## Build

Always build from the workspace folder (`rov_ws`), not from `src/`:

```bash
cd ~/main/pr_robot/rov_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to rov_bringup
source install/setup.bash
```

## Run

Run `source install/setup.bash` in every new terminal first.

| Demo | Command |
|---|---|
| Click-to-drive, no map | `ros2 launch rov_bringup sim_nav.launch.py` |
| Plan + drive in the pool (simple simulator) | `ros2 launch rov_bringup sim_nav.launch.py planned_drive:=true map_yaml:=pool_map.yaml` |
| **Plan + drive in the pool with Gazebo** (RViz + Gazebo) | `ros2 launch rov_bringup sim_nav.launch.py planned_drive:=true backend:=gazebo` |
| Planning preview only (robot stays still) | `ros2 launch rov_bringup sim_nav.launch.py planning_preview:=true map_yaml:=pool_map.yaml` |
| Wall test map (default map) | `ros2 launch rov_bringup sim_nav.launch.py planned_drive:=true` |
| Gazebo pool scene only (nothing moves) | `ros2 launch rov_bringup pool_gazebo.launch.py` |
| Thrust-driven simulator, lightweight model | `ros2 launch rov_bringup thrust_sim.launch.py` |
| Thrust-driven simulator, Gazebo (`gui:=false` for no window) | `ros2 launch rov_bringup thrust_sim.launch.py backend:=gazebo` |

Then in RViz, click **2D Goal Pose**, click on the map at the goal, and drag
to set the final heading. Stop with **Ctrl+C**.

Optional arguments for the map modes: `start_x:=`, `start_y:=`, `start_yaw:=`
(start pose on the map, metres/radians).

Sending a goal from a terminal instead of RViz (map modes):

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: -10.0, y: -4.0}, orientation: {w: 1.0}}}"
```

### Open-loop thrust tests

With `thrust_sim.launch.py` running, in a second terminal:

```bash
# lightweight model
ros2 run rov_simulation open_loop_thrust_test.py --backend-label dynamics --output-dir results/dynamics
# Gazebo (uses Gazebo's clock)
ros2 run rov_simulation open_loop_thrust_test.py --backend-label gazebo --output-dir results/gazebo \
  --ros-args -p use_sim_time:=true
```

It resets the ROV before every run, measures steady surge, sway and yaw speeds,
coast-down and command loss, and writes `open_loop_speed.csv` and
`open_loop_coast.csv`.

## Coordinates (pool)

- 2D map `pool_map.yaml`: centred on the pool, interior x −25..25 m, y −12.5..12.5 m.
  The ROV starts in the bottom-left corner at (−24, −11.5).
- Gazebo world: origin at the pool's bottom-left corner, interior x 0..50 m, y 0..25 m.
  `world (x, y) = map (x + 25, y + 12.5)`.
- `thrust_sim.launch.py` (both backends): odom = Gazebo world frame; the ROV starts
  at (5, 5) facing +x (`start_x:=`, `start_y:=`, `start_yaw:=`).
