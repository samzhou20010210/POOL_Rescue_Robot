# Implementation Steps — Surface High-Speed Approach

Design reference: `SURFACE_APPROACH_DESIGN.md` (section numbers below refer to it).
Execute one step at a time. Each step lists its goal, where the code goes, what to build, what is out of scope, and the acceptance check that must pass before the next step starts.

| Step | Module | Package | Depends on |
|---|---|---|---|
| 1 | Interfaces | `rov_interfaces` (new) | — |
| 2 | Thrust allocation | `rov_control` | 1 |
| 3 | Planar dynamics sim | `rov_simulation`, `rov_description` | 1, 2 |
| 4 | Gazebo thrust + hydrodynamics | `rov_simulation`, `rov_bringup` | 3 |
| 5 | Trajectory-tracking controller (PD + feedforward) | `rov_control` | 2, 3 |
| 6 | Planning core library (ROS-free) | `rov_planning` (new) | — |
| 7 | MINCO trajectory optimizer (ROS-free) | `rov_planning` | 6 |
| 8A | Terminal-pose generation (ROS-free) + offline tests | `rov_planning` | 6 |
| 8B | Trajectory generator node: online planning + controller integration | `rov_planning`, `rov_bringup` | 5, 7, 8A |
| 9 | Swimmer prediction layer | `rov_planning`, `rov_simulation` | 8B |
| 10 | Mission state machine + safety monitor | `rov_mission` | 8B, 9 |
| 11 | End-to-end scenarios and metrics | `rov_bringup` | 10 |

Steps 6–7 do not depend on ROS and can run in parallel with steps 2–5.

---

## Step 1 — Interfaces

**Goal.** Create the `rov_interfaces` package that holds the custom messages and actions the later steps need.

**Where.** New package `rov_interfaces` (ament_cmake, rosidl).

**Build.**
- The message and action definitions provided by the user. The three needed are:
  - per-thruster thrust command (8 forces in N),
  - time-parameterized trajectory (position, velocity, acceleration, yaw, yaw rate, timestamps),
  - follow-trajectory action (goal = trajectory; feedback = progress and tracking error; result = success / reason).
- Field definitions come from the user. Do not invent fields.

**Out of scope.** Using the interfaces in any other package.

**Done when.** `colcon build --packages-select rov_interfaces` succeeds and `ros2 interface show` prints every definition.

---

## Step 2 — Thrust allocation

**Goal.** Convert a desired planar body wrench (Fx, Fy, Mz) and an attitude-hold wrench into 8 thruster forces (design §4).

**Where.** `rov_control`: a ROS-free library (`thrust_allocator`) plus a thin node.

**Build.**
- Load thruster geometry (r_i, e_i) from `rov_description/config/thrusters_v4.yaml` and build the allocation matrix B.
- QP: `min ‖B·f − tau‖²_W + λ‖f‖²`, subject to `f_min ≤ f ≤ f_ctrl` (W, λ, f_ctrl from §6.2 / §6.6). Port the user's existing allocation QP implementation.
- Saturation flag: set when any thruster reaches its limit.
- Node: wrench in → thrust command out, at 50 Hz.

**Out of scope.** Thrust-to-PWM conversion, vertical-thruster attitude control logic.

**Done when.** Unit tests pass for:
- pure surge, pure sway and pure yaw wrenches: allocated forces reproduce the wrench within 1%;
- a wrench beyond limits: forces clamp to limits and the saturation flag is set;
- geometry read from YAML matches the hand-computed B for at least 2 thrusters.

---

## Step 3 — Planar dynamics simulation (lightweight)

**Goal.** Replace the kinematic `sim_node` with a thrust-driven planar 3-DOF model including added mass and linear + quadratic damping, so control and planning can be tested quickly without Gazebo.

**Where.** `rov_simulation` (sim_node), `rov_description` (new hydrodynamic parameter YAML with every §6.1 value).

**Build.**
- Input: thrust command (8 forces). Compute body wrench via B; integrate `M_eff·ν̇ + C(ν)ν + D(ν)ν = τ` at 200 Hz (RK4). Publish odometry and TF at 50 Hz.
- First-order thrust lag τ_m (§6.2).
- Optional constant water current (§6.3, v_c) as a parameter, default 0.
- Command timeout → zero thrust.
- An open-loop test script: apply 20/40/60/80/100% surge thrust and record steady speed; cut thrust at 1.5 m/s and record stop distance.

**Out of scope.** Heave, roll, pitch; waves; Gazebo.

**Done when.**
- Steady surge speed at f_plan on all 4 horizontal thrusters falls within ±10% of the value predicted by the damping parameters (≈1.5 m/s with initial values).
- The open-loop script produces a speed–thrust table and stop distance, saved to CSV.
- All parameters come from YAML; none are hard-coded.

---

## Step 4 — Gazebo thrust + hydrodynamics

**Goal.** Make the Gazebo backend thrust-driven, with hydrodynamics matching the step 3 model.

**Where.** `rov_simulation` / `rov_bringup` (SDF model, world, bridge config, launch).

**Build.**
- Remove `VelocityControl` from the V4 model.
- Add one gz `Thruster` plugin per thruster at the positions and axes from `thrusters_v4.yaml`.
- Add the gz `Hydrodynamics` plugin with added mass and damping from the §6.1 YAML.
- Buoyancy: set so the robot floats at draft T (§6.1). Restore gravity in the world.
- Bridge: thrust command → gz thruster topics; gz odometry → ROS odometry.
- Launch argument `backend:=sim|gazebo` keeps working.

**Out of scope.** Waves, wind, tether, underwater phase.

**Done when.** The step 3 open-loop script, run against Gazebo, gives steady speeds within ±15% of the lightweight sim and stop distances within ±20%.


**Status (recorded 2026-11-25): PASS.** Gravity is on (a temporary copy of `worlds/pool.sdf`); buoyancy, heave damping and roll/pitch restoring come from `PlanarThrusterSystem` with the provisional values in `rov_simulation/config/gazebo_hydrostatics.yaml` (waterplane area 0.2508 m², draft 0.20 m). The ROV floats at z = −0.0500 ± 0.0005 m, level, real-time factor 1.03. Step 3 open-loop script against Gazebo with gravity: steady speeds 0.00 % from the lightweight sim (limit ±15 %), stop distances −0.18 % / −1.43 % (limit ±20 %). Artifacts: `test_results/step4/gazebo_gravity/`, `test_results/step4/comparison_gravity.csv`.

---

## Step 5 — Trajectory-tracking controller v1

**Goal.** Replace the velocity-output path follower with a controller that tracks a time-parameterized trajectory and outputs a body wrench (design §4, v1).

**Where.** `rov_control`: ROS-free `tracking_controller` library plus the controller node.

**Build.**
- Control law: `tau = tau_ff + K_p·e_p + K_d·e_v`, with `tau_ff = M_eff·a_ref + D(v_ref)·v_ref`; gains from §6.6. Errors in body frame. Yaw error wrapped to [−π, π].
- Serves the follow-trajectory action; samples the trajectory at 50 Hz.
- Vertical thrusters: roll/pitch hold at 0, limited to f_att.
- Temporary trajectory source for testing: a converter that turns the existing `nav_msgs/Path` into a trajectory with a trapezoidal velocity profile (v_cruise, a_max). This is replaced in step 8.

**Out of scope.** NMPC; replanning.

**Done when.** In the lightweight sim and in Gazebo, following a 20 m straight line and an L-shaped path at 1.5 m/s:
- tracking-error RMS ≤ 0.15 m, max ≤ 0.3 m;
- terminal error ≤ 0.15 m and ≤ 0.1 rad;
- with feedforward disabled, RMS error is visibly larger (confirms feedforward works).


**Status (recorded 2026-11-25): PASS on both backends** (matrix run `scenario_runs/matrix_20261125_final`, scenarios `controller_*`; the runner sends the 20 m line and the L directly to the controller, the Path converter is retired from the build):

| Run | Backend | RMS (m) | Max (m) | Terminal (m / rad) |
|---|---|---|---|---|
| line, feedforward on | dynamics / gazebo | 0.033 / 0.038 | 0.088 / 0.103 | 0.017, 0.000 / 0.021, 0.000 |
| L, feedforward on | dynamics / gazebo | 0.042 / 0.043 | 0.096 / 0.101 | 0.019, 0.000 / 0.021, 0.001 |
| line / L, feedforward off | both | 0.665–0.673 | 0.990–0.995 | aborted at the 1.0 m tracking limit |

Feedforward off gives about 17× the RMS error (visibly larger, as required).

---

## Step 6 — Planning core library (ROS-free)

**Goal.** Build the reusable pieces the optimizer needs, with no ROS dependency.

**Where.** New package `rov_planning`: `third_party/gcopter/`, `include/rov_planning/`, `test/`.

**Build.**
- Vendor from GCOPTER (https://github.com/ZJU-FAST-Lab/GCOPTER) only: `minco.hpp`, `lbfgs.hpp`, `firi.hpp`, `sdlp.hpp`, `geo_utils.hpp`, plus its LICENSE. No ROS1 code, no OMPL, no `flatness.hpp`, no `sfc_gen.hpp`.
- `rov_model`: given (p, v, a, ψ, ψ̇, ψ̈) in world frame, return the required body wrench τ and its gradients w.r.t. v and a (§3, thrust-feasibility constraint).
- `wrench_polytope`: compute `H·tau ≤ h` offline from B and f_plan (horizontal thrusters only).
- `corridor_2d`: from a 2D path and an occupancy grid, generate convex polygons (seed spacing Δs, max N_poly), expressed as half-spaces in (x, y, ψ) with zero ψ coefficient.

**Out of scope.** The optimization itself; ROS nodes.

**Done when.** Unit tests pass for:
- `rov_model`: at constant surge velocity u, τ_x equals the damping force −X_u·u − X_u|u|·u|u|; gradients match finite differences to 1e-4 relative;
- `wrench_polytope`: every wrench produced by f within limits satisfies `H·tau ≤ h`; a wrench needing f > f_plan violates it;
- `corridor_2d`: on a test map, consecutive polygons overlap, contain the path, and contain no occupied cell.

---

## Step 7 — MINCO trajectory optimizer (ROS-free)

**Goal.** Optimize a (x, y, ψ) MINCO trajectory inside the corridor, subject to thrust feasibility, the braking curve and terminal state (§3 items 4–5).

**Where.** `rov_planning`: `surface_traj_opt.hpp/.cpp`, tests, and a plotting script.

**Build.**
- Structure the cost and gradient propagation after GCOPTER's `gcopter.hpp`. Before writing code, summarize how `gcopter.hpp` builds its cost, penalties and gradients, and how this step maps onto it.
- Cost: ∫‖jerk‖² + ρ_T·T + w_τ·Σ‖max(0, Hτ_k − h)‖³ + w_v·(velocity / braking-curve violations) + yaw-rate violations.
- Corridor as hard constraint via GCOPTER-style constraint elimination.
- Terminal state v = 0, a = 0; optional fixed terminal yaw.
- Input: start state, terminal pose, corridor, parameters. Output: trajectory + diagnostics (cost terms, iterations, max constraint violation).
- Plot script: x–y path with corridor, speed vs time with v_max(d), thrust-constraint margin vs time.

**Out of scope.** ROS; replanning.

**Done when.** On three test cases (open 50 m straight, L-turn around a wall, gap between two obstacles):
- converges in < 50 ms each on the development machine;
- max thrust-constraint violation ≤ 2% of f_plan;
- trajectory stays inside the corridor;
- speed never exceeds v_max(d);
- feeding the output into the step 3 sim open-loop (feedforward only) ends within 0.5 m of the goal on the straight case.

**Status (recorded 2026-11-25): FAIL on the < 50 ms target only; all other checks pass.** Remeasured 2026-11-25 (`build/rov_planning/test_surface_traj_opt --gtest_filter='*StraightFiftyMetres:*LTurnAroundAWall:*NarrowGap'`, Release build): cold solves 233 ms (486 evaluations), 809 ms (2291), 119 ms (605) for straight / L-turn / gap; warm-started replans (`build/rov_planning/test_warm_start`) 75–76 ms including checks. Online, the 5 Hz end-to-end requirement of step 8B is met separately (see there). Earlier record (2026-11-24):

| Case | Solve time (cold start, optimized build) | L-BFGS evaluations |
|---|---|---|
| 50 m straight | 184 ms | 271 |
| L-turn | 847 ms | 1707 (about 1100 iterations) |
| Narrow gap | 81 ms | 298 |

All other checks pass (corridor, thrust ≤ 2% of f_plan, speed ≤ v_max(d), terminal rest, feedforward run 0.03 m from the goal). The gradients are exact (finite-difference error 2.6e-9), so the shortfall is slow convergence, not a wrong gradient. A wider penalty smoothing broke the speed limit, and a faster initial time allocation did not help. The iteration cap was raised to 3000 (provisional) so the L-turn converges. To be revisited in step 8B with warm starts from the previous trajectory; the target is not relaxed. Update (step 8B): warm starts cut a replan to about 0.1 s (straight and L-turn), still above 50 ms; w_v was raised to 1e5 (provisional, design 6.5 note), which made cold solves slower (L-turn about 1.1 s). The < 50 ms target stands.

---

## Step 8A — Terminal-pose generation

**Goal.** Generate the terminal pose for both victim states (design §3 item 1), with a stable approach heading, and check terminal clearance, without ROS.

**Where.** `rov_planning`: `terminal_pose.hpp/.cpp`, `config/terminal_pose.yaml`, tests.

**Build.**
- One bow-facing function for both victim states: heading ψ_goal, `p_goal = p_v − d·[cos ψ_goal, sin ψ_goal]`, `d = x_bow + L_arm + r_body + g`, with g = g_dive (submerged) or g_surface (surface), each configurable.
- L_arm has no default: the configuration must give an explicit (simulation) value; a missing or null value is an error.
- Approach heading: recomputed outside d + d_hold of the victim, held inside, released beyond d + d_hold + d_hyst, reset per mission. Re-aimed only when the victim has moved more than Δp_reaim from its position at the last lock or re-aim and the bearing differs by more than ψ_reaim; robot motion alone never re-aims.
- Invalid inputs (non-finite positions, robot at the victim with no held heading, missing or inconsistent settings, negative g_min) are rejected with an error.
- Separate clearance check at a reached pose: stowed outline to victim body ≥ g_min, outline to blocked cells ≥ wall clearance, whole outline inside the map; a malformed grid fails the check. This is independent of the acceptance limits.
- Remove the obsolete gripper-centre parameter (p_grip) once confirmed unused.

**Out of scope.** ROS nodes, A*, replanning, controller changes. The step 5 controller tolerances (0.15 m, 0.1 rad) stay as they are.

**Done when.** Offline unit tests pass for:
- both victim states: pose on the approach line, bow facing the victim, standoffs d_surface and d_dive, nominal victim gap equal to g;
- heading stability: noisy robot and victim positions inside the hold zone keep the heading constant; the robot sweeping sideways in front of a stationary victim does not re-aim; a genuine victim move (beyond Δp_reaim and ψ_reaim) re-aims once; outside the zone the heading follows the bearing;
- invalid inputs and settings are rejected, including a negative g_min;
- the clearance check fails a pose that is inside the 0.25 m acceptance limit but too close to the victim, a pose too close to a wall, an outline that extends outside the map, and a malformed grid.

---

## Step 8B — Trajectory generator node

**Goal.** Run the full planning pipeline online: terminal pose → A* → corridor → MINCO → trajectory, with 5 Hz replanning (§3 items 1, 6, 7).

**Where.** `rov_planning`: `trajectory_generator_node`; Nav2 planner config in `rov_bringup` (`use_astar: true`).

**Build.**
- Use the step 8A terminal-pose generator (and its heading hold) for both victim states.
- Request the A* path from `planner_server`, build the corridor, optimize, publish / send the trajectory to the controller.
- 5 Hz timer replanning, warm start from the previous trajectory, starting from the current state on the current trajectory.
- Immediate replanning triggers: target shift > Δp_v, tracking error > e_replan.
- Failure handling: keep the last feasible trajectory; retry at 0.7·v_cruise; after 3 failures, brake to stop and hold.
- Remove the temporary Path→trajectory converter from step 5.
- Revisit the step 7 solve time with warm starts (target < 50 ms).

**Out of scope.** Swimmers; mission phases. The step 5 controller tolerances (0.15 m, 0.1 rad) are kept.

**Done when.** In both sim backends:
- 50 m transit in ≤ 40 s, terminal error ≤ e_goal (0.25 m) and ≤ e_ψ (0.15 rad) (simulation acceptance limits), bow facing the victim in both victim states, and the separate clearance check passes at the reached pose;
- a target moved 2 m mid-run is reached without stopping;
- a forced optimizer failure falls back as specified;
- RViz shows the path, corridor and trajectory.


**Status (recorded 2026-11-25): PASS on both backends** (matrix run `scenario_runs/matrix_20261125_final`; reproduce with `source install/setup.bash && GZ_PARTITION=rovtest ros2 run rov_bringup run_scenario.py --matrix matrix --domain-id 77` (from `rov_ws`)):
- 50 m transit (`transit_50m`, 45.35 m route): arrival 34.63 s / 34.57 s (≤ 40 s), terminal error 0.017 / 0.020 m and 0.004 / 0.005 rad; clearance check passes at the reached pose.
- Bow facing the victim: submerged (`baseline_submerged`) 0.005 / 0.008 rad, surface (`surface_victim`) 0.001 / 0.001 rad; surface final-zone max speed 0.284 m/s (v_final 0.3 m/s).
- Target moved 2 m mid-run (`moved_target`): reached without stopping, minimum cruise speed 1.24 / 1.26 m/s.
- Forced optimizer failures: `forced_failure_keep` keeps the active trajectory and recovers at 0.7 v_cruise; `forced_failure_brake` brakes along the path after 3 failures, holds, and resumes. `invalid_replacement`: the controller rejects an invalid trajectory and the current goal continues.
- 5 Hz replanning, measured end to end (dynamics / gazebo, nominal runs): 4.97–5.02 Hz, p95 latency 120–160 ms, max ≤ 210 ms.
- RViz (`rviz/surface_approach.rviz`, `rviz:=true`): `planned_path`, `corridor`, `trajectory`, `terminal_pose` (all recorded in the bags).
- Path→trajectory converter retired from the build and launch; its source files are still in `rov_control` (deletion pending).

---

## Step 9 — Swimmer prediction layer

**Goal.** Keep the planner and controller away from non-target swimmers (§3 item 2, §5).

**Where.** `rov_planning`: `swimmer_layer` Nav2 costmap plugin. `rov_simulation`: fake shore publishes moving swimmers.

**Build.**
- Costmap layer: for each swimmer track, mark predicted positions over t_pred as discs whose radius grows with v_swim·t, plus r_swim clearance.
- Trajectory generator: immediate replan when a predicted swimmer disc intersects the current trajectory.
- Fake shore: configurable swimmers with straight / back-and-forth motion and noise.

**Out of scope.** Slow-zone speed limits (step 10).

**Done when.** In scenarios with 1–3 swimmers crossing the route, the robot's minimum distance to every swimmer stays ≥ r_swim and the robot still reaches the goal.


**Status (recorded 2026-11-25): PASS on both backends** (matrix run `scenario_runs/matrix_20261125_final`). Malformed swimmer stamps are dropped; the conflict check is interval-based (no gaps between trajectory samples) and covers the outgoing, active and brake trajectories with aligned timestamps; conflicts trigger replanning, or BLOCKED and brake/hold. Minimum hull-to-swimmer distance (truth, dynamics / gazebo) and arrival: `swimmers_1` 5.52 / 5.98 m, 30.6 / 30.5 s; `swimmers_2` 6.60 / 6.62 m, 32.3 / 32.1 s; `swimmers_3` 4.26 / 4.26 m, 42.8 / 42.7 s (r_swim 1.0 m); all accepted.

---

## Step 10 — Mission state machine and safety monitor

**Goal.** Implement the 6 mission phases (§2) and the independent safety rules (§5).

**Where.** `rov_mission`: rewrite the state machine; new `safety_monitor` node.

**Build.**
- Phases: STANDBY, TRANSIT, FINAL_APPROACH, DIVE_AND_GRASP (stub: logs and holds), SURFACE_HOLD, RETURN, plus SAFETY_STOP. Transitions as in §2.
- Safety monitor: every rule in the §5 table, with thresholds from §6.7. It acts directly on the speed limit and can command stop / hold, independent of the state machine.
- Log every phase transition and every safety trigger with timestamp and reason.

**Out of scope.** Real dive and grasp.

**Done when.** A scripted test triggers each §5 rule once and each produces the specified action; a full run goes STANDBY → TRANSIT → FINAL_APPROACH → SURFACE_HOLD / DIVE_AND_GRASP → RETURN for both victim states.


**Status (recorded 2026-11-25): PASS on both backends** (matrix run `scenario_runs/matrix_20261125_final`). `rov_mission`: `mission_manager_node` (typed `rov_interfaces` events, stale events > 2 s ignored, resume only with fresh odometry and a valid trajectory) and `safety_monitor_node` (ROS-free `safety_rules`, every §5 rule, priority leak > hold > return). Both branches complete: `mission_submerged` STANDBY→TRANSIT→FINAL_APPROACH→DIVE_AND_GRASP→RETURN→STANDBY, `mission_surface` …→SURFACE_HOLD→RETURN→STANDBY. Each rule triggered once with its action: `safety_near_wall`, `safety_odometry_timeout`, `safety_heartbeat_lost`, `safety_heartbeat_abort`, `safety_thrust_saturation`, `safety_low_battery`, `safety_leak`, `safety_mission_timeout`, `safety_swimmer_nearby`, `safety_swimmer_predicted`; plus `safety_simultaneous` (heartbeat lost + low battery) and `stale_events`. Finding: the near-wall rule cannot be exercised in motion with the documented parameters (A* robot radius 1.3 m keeps base_link ≥ 1.3 m from walls); it is tested from a start 0.45 m from the wall (design §5 notes). Dive and grasp remain a stub.

---

## Step 11 — End-to-end scenarios and metrics

**Goal.** Make every test run reproducible and measured.

**Where.** `rov_bringup`: scenario launch files and configs; a metrics script.

**Build.**
- Scenario set: open-pool transit, victim near a wall, moving swimmers, water current 0.2 m/s, target moved mid-run, heartbeat loss.
- Each run records a rosbag with the topics listed in design §7.
- Metrics script reads a bag and outputs the §7 indicators (arrival time, terminal error, tracking RMS / max, saturation fraction, min swimmer / wall distance, peak power, replan counts) as CSV.

**Done when.** One command runs the whole scenario set in the lightweight sim and produces a metrics CSV; the same set runs in Gazebo.


**Status (recorded 2026-11-25): PASS — complete.** The full matrix (`config/scenarios/matrix.yaml`, 31 scenarios × dynamics and gazebo) passed 62/62: `source install/setup.bash && GZ_PARTITION=rovtest ros2 run rov_bringup run_scenario.py --matrix matrix --domain-id 77` (from `rov_ws`). Results: `scenario_runs/matrix_20261125_final/summary_20261125_053504.csv` and one directory per run (bag, `metrics.csv`, `checks.txt`, logs). Wall clearance uses the recorded `/map` (minimum 0.975 m in `transit_50m`, 1.53 / 1.58 m in `victim_near_wall`); swimmer clearance uses `/swimmer_truth`. Water current 0.2 m/s (`water_current`): terminal error 0.040 / 0.037 m, tracking max 0.153 / 0.159 m. `peak_power_w` and `energy_j` are left empty: there is no thruster current/voltage in simulation and no calibrated force-to-power model (design §7 experiment 3).

---

## Later (not scheduled)

- NMPC controller (acados) replacing step 5's v1 law, using the same model and thrust limits.
- Kinodynamic A* front end, only if step 11 data shows A* paths causing optimizer failures or slow trajectories.
