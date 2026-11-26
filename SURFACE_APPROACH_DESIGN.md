# Pool Rescue ROV — Surface High-Speed Approach Design (v0.1)

This file describes the design of the surface phase: the robot leaves the shore, travels at speed to the drowning person, and stops at the terminal point from which the dive and grasp begin. Read it as the source of truth for design intent, target values and parameters. Values marked "estimate" or "TBD" are placeholders until identification experiments replace them.

---

## 1. Design targets

The robot cruises at 1.5 m/s, sprints at 2.0 m/s, and in both victim states stops **bow-on to the victim**, at a standoff that places the victim just ahead of the two front-mounted arms. The arms grasp the victim under the armpits and lift them to the surface, so the terminal pose is defined relative to the bow and the arms, never with the hull over the victim.

| Item | Value | Note |
|---|---|---|
| Cruise speed | 1.5 m/s | Sustainable at ~70% thrust of the 4 horizontal thrusters |
| Sprint speed | 2.0 m/s | Max 30 s per sprint (T500 peak-thrust duration limit) |
| Planning accel / decel limit | 1.5 m/s² | Leaves thrust margin for feedback control |
| Yaw rate limit | 1.5 rad/s | Camera always faces the victim |
| Terminal point, victim submerged | Bow facing victim, base_link at d_dive behind victim along the approach direction, velocity 0 | Then dive to grasp depth and close in horizontally with vision |
| Terminal point, victim at surface | Bow facing victim, base_link at d_surface behind victim, velocity 0 | Arms extend to grasp, or the victim holds on |
| Terminal acceptance (simulation) | ≤ 0.25 m horizontal, ≤ 0.15 rad yaw | Simulation acceptance limits for the surface phase, not controller tolerances. Victim and wall clearance are checked separately (§3 item 1) |
| Wall clearance | 0.5 m | Hull edge to pool wall |
| Clearance to non-target swimmers | 1.0 m | Hull edge to predicted swimmer position |
| 50 m full-length transit time | ≤ 40 s | Including acceleration and braking |

Why these numbers:

- **Speed ceiling.** T500 peak thrust is 16.1 kgf (158 N) at 24 V and can be held for ~1 min. The 4 horizontal thrusters at ±45° give ~450 N peak / ~280 N sustained in surge. Estimated quadratic drag at the surface is ~100·v² N (A ≈ 0.2 m², C_d ≈ 1.0), and wave-making drag rises sharply above ~1.25 m/s (Froude number 0.4 for a 1 m hull). 3 m/s is not reachable with this hull and thruster set.
- **Braking is not the limit.** Drag plus reverse thrust stops the robot from 1.5 m/s in about 0.5 m. Top speed is limited by drag, not by deceleration.
- **Standoff from the bow, not the hull center.** In both victim states the bow faces the victim and base_link stops short of the victim by `d = x_bow + L_arm + r_body + g`: bow offset from base_link (x_bow = 0.50 m), L_arm = how far the **stowed** arms protrude beyond the bow, victim body radius (r_body = 0.25 m), and a gap g between the stowed arm tips and the victim's body. L_arm is not known yet (arm CAD); simulation runs must set an explicit value (0.10 m, provisional), there is no default.
- **Submerged: g_dive = 0.30 m (provisional simulation value).** The robot must descend without landing on the victim and then close the last distance horizontally at grasp depth, so it stops slightly farther out.
- **Surface: g_surface = 0.15 m (provisional simulation value).** No descent; the victim only needs to be close to the arms.
- **The terminal pose is a staging point.** It only sets up the dive / grasp phase, where onboard vision does the fine horizontal alignment. Whether the deployed arms actually reach the victim from this pose, and whether they deploy without touching hull, victim or wall, is not verified here: it needs the arm CAD (deployed reach, L_arm) and is to be validated before g_dive and g_surface are fixed.
- **Acceptance limits vs. clearance.** 0.25 m / 0.15 rad are simulation acceptance limits for the surface phase. They do not guarantee clearance: 0.25 m is larger than g_surface, so a stop inside the limits can still put the arm tips into the victim. Clearance to the victim (stowed outline vs. body) and to the walls is therefore checked separately at the reached pose. The trajectory controller keeps its own, tighter completion tolerances (0.15 m, 0.1 rad, §6.6).

---

## 2. Mission phases

```
STANDBY --(shore alarm)--> TRANSIT --(within d_hold = 2 m of the terminal point)--> FINAL_APPROACH --> [victim submerged?]
            (approach heading recomputed every replan)          (approach heading held)   |
                                                                       |-- yes --> DIVE_AND_GRASP: stop bow-on, base_link d_dive short of the victim --(victim lifted)--> RETURN
                                                                       |-- no  --> SURFACE_HOLD:   stop bow-on, base_link d_surface short of the victim --(victim secured)--> RETURN

SAFETY_STOP can preempt any motion phase: decelerate to 0 and hold position.
```

- **STANDBY**: wait for the shore alarm.
- **TRANSIT**: high-speed approach at cruise speed with rolling replanning. The approach heading is recomputed at every replan.
- **FINAL_APPROACH**: within d_hold (2 m) of the terminal point, follow the braking curve. The approach heading is held (see §3 item 1), so the bow does not swing with small position or measurement changes.
- **DIVE_AND_GRASP**: stopped bow-on at d_dive; hold heading, descend to grasp depth, close in horizontally with vision, grasp under the armpits with the two front arms, lift to the surface. (Detailed design of this phase is outside this document.)
- **SURFACE_HOLD**: victim at surface; stopped bow-on at d_surface; the front arms grasp the victim or the victim holds on.
- **RETURN**: bring the victim back to the shore.

---

## 3. Planning

Three-stage pipeline: **A\* front end → FIRI safe corridor → MINCO back end**, with a thrust-feasibility constraint that keeps the trajectory executable under water drag. Replan at 5 Hz.

1. **Terminal pose generation.** From the shore-reported victim position p_v and state (surface / submerged). One bow-facing formula for both states; only the standoff differs.
   - `p_goal = p_v − d · [cos ψ_goal, sin ψ_goal]`, heading ψ_goal, with `d = x_bow + L_arm + r_body + g`. The bow and the front arms face the victim.
   - Submerged: g = g_dive (d = d_dive). Surface: g = g_surface (d = d_surface).
   - **Approach heading ψ_goal.** The bearing from the robot to p_v.
     - Recomputed at every update while the robot is farther than d + d_hold from the victim (TRANSIT).
     - Held once the robot is within d + d_hold of the victim (FINAL_APPROACH). p_goal is still updated from each new p_v, along the held heading, so measurement noise and small victim motion move the goal without turning the bow.
     - Re-aimed (recomputed once and held again) only when the victim itself has moved: its position must have shifted by more than Δp_reaim (0.3 m, = Δp_v) from where it was at the last lock or re-aim, **and** the bearing to it must differ from the held heading by more than ψ_reaim (0.15 rad, = e_ψ). The victim position at that moment becomes the new reference. Robot motion alone (for example sideways tracking error) never re-aims, however much it changes the bearing, and measurement noise below Δp_reaim is ignored.
     - Released (recomputed every update again) if the robot moves back beyond d + d_hold + d_hyst (hysteresis), and reset at the start of a new mission.
     - Undefined, and rejected as invalid input, when the robot is at the victim position with no held heading.
   - **Clearance check (separate from the acceptance limits).** At the reached pose, the gap between the stowed outline (hull plus stowed arms, base_link x from −0.50 to x_bow + L_arm, half-width 0.52 m) and the victim's body must be ≥ g_min (g_min ≥ 0; a negative value is rejected), and the outline must keep the wall clearance (0.5 m) to blocked or unknown map cells. The check fails if the map is malformed (resolution, origin, size or data length) or if any part of the stowed outline lies outside the map.
2. **Front end: A\*.** NavFn with `use_astar=true`, costmap resolution 0.05 m, rectangular footprint 1.00 × 1.04 m, inflation 0.5 m. A custom swimmer layer writes the predicted positions of non-target swimmers into the costmap.
3. **Safe corridor: FIRI.** Seed points every 1.0 m along the A\* path; generate 2D convex polygons, at most 30.
4. **Back end: MINCO.** s = 3, optimized dimensions (x, y, ψ), initial time allocation from v_cruise, 16 samples per piece. Cost = ∫‖jerk‖² + ρ_T·T + constraint penalties.
   - Corridor: hard constraint via GCOPTER-style constraint elimination.
   - Thrust feasibility: the body wrench required at every sample must lie inside the attainable wrench polytope (below).
   - Velocity: ‖v‖ ≤ v_max(d); |ω| ≤ 1.5 rad/s.
   - Terminal state: v = 0, a = 0.
5. **Braking curve.** With d the distance to the terminal point: `v_max(d) = min(v_cruise, sqrt(2·a_b·d))`, a_b = 1.0 m/s². For a surface victim, additionally cap speed at 0.3 m/s within d ≤ 2.0 m. For a submerged victim, no extra cap: follow the braking curve straight to the d_dive stop point.
   - Implementation (step 8B): outside the final zone the cap is reached at a_b, `v_fz(d) = sqrt(v_final² + 2·a_b·(d − d_final))`, so `v_max(d) = min(v_cruise, sqrt(2·a_b·d), v_fz(d))` is continuous. The optimizer's speed penalty starts the zone 0.25 m further out (provisional) because the kink falls between penalty samples; validation uses d_final itself.
   - A speed cap that arrives while moving (safety limit, 0.7·v_cruise retry) is phased in along `sqrt((v0 + 0.1)² − 2·a_b·s)` from the start of the replan (s = distance from the start, 0.1 m/s provisional headroom), the fastest legal slow-down.
6. **Replanning.** Timer at 5 Hz, initialized from the current state, warm-started from the previous trajectory. Replan immediately if the target moves > 0.3 m, a predicted swimmer position intersects the trajectory, or tracking error > 0.5 m.
7. **Failure handling.** If optimization fails, keep executing the last feasible trajectory and retry with 0.7·v_cruise. After 3 consecutive failures, brake to a stop and hold position.

### Thrust-feasibility constraint

For each sample k, compute the required body wrench from the planar 3-DOF model, and require that the 4 horizontal thrusters can produce it within limits:

```
tau_k = M_eff * nu_dot_k + C(nu_k) * nu_k + D(nu_k) * nu_k
tau_k = B * f,    f_min <= f <= f_max
```

The attainable wrench set is a convex polytope `H * tau <= h`, computed offline from B and f_plan. In the online optimization, each sample adds a penalty:

```
J_tau = w_tau * sum_k || max(0, H * tau_k - h) ||_3^3
```

### Notes

- V4 is nearly square (1.00 × 1.04 m), so surge and sway drag are similar. ψ is planned independently and keeps the camera pointed at the victim.
- Reference implementation: GCOPTER (https://github.com/ZJU-FAST-Lab/GCOPTER). Take only the MINCO, FIRI and optimizer code; do not bring in its ROS1 layer.
- DDR-opt is not used: its core handles differential-drive nonholonomic constraints, which a fully actuated ROV does not have.

---

## 4. Control and thrust allocation

The controller tracks the time-parameterized MINCO trajectory at 50 Hz. Version 1 is PD plus model feedforward; version 2 replaces it with NMPC. Both use the same model parameters.

- **Trajectory sampling.** Each control cycle takes p_ref, v_ref, a_ref from the current trajectory and transforms them to the body frame.
- **v1 control law.** `tau = tau_ff + K_p·e_p + K_d·e_v`, with feedforward `tau_ff = M_eff·a_ref + D(v_ref)·v_ref`. At high speed most of the thrust comes from feedforward; feedback only corrects model error.
- **v2 control law.** acados NMPC, horizon 1.0 s, 20 steps, same model and thrust limits as the planning back end.
- **Thrust allocation QP.** 50 Hz: `min ‖B·f − tau‖²_W + λ‖f‖²`, subject to `f_min ≤ f ≤ f_ctrl`. Publishes a saturation flag for safety monitoring.
- **Vertical thrusters.** During the surface phase they hold roll and pitch at 0, each limited to 20% of peak thrust, to suppress bow-up at speed.
- Thrust commands are per-thruster forces in N; the driver layer converts force to PWM.

---

## 5. Safety rules

Safety rules run in an independent safety monitor with priority over the mission state machine. Triggered actions act directly on planning and control.

| Trigger | Threshold | Action |
|---|---|---|
| Non-target swimmer nearby | within 3.0 m | Speed limit drops to 0.5 m/s |
| Predicted swimmer position enters clearance | within 1.0 m | Brake to stop, hold, replan immediately |
| Near pool wall | hull edge within 0.5 m | Speed limit drops to 0.3 m/s |
| Odometry timeout | 0.2 s | All thruster forces set to 0 |
| Shore heartbeat lost (surface phase) | 1.0 s | Decelerate to 0 along current trajectory, hold position |
| Heartbeat still lost | 5.0 s | Abort mission, hold, wait for manual takeover |
| Sustained thrust saturation | 2.0 s | Replan with 0.7·v_cruise |
| Low battery | 20% | Abort mission and return |
| Leak sensor alarm | any | Abort mission, surface, return |
| Mission timeout without arrival | 120 s | Abort mission, hold position |

---

Implementation notes (step 10):
- Rules combine: the lowest speed limit applies; stop / zero thrust / replan flags add up; one abort by priority surface-and-return (leak) > hold (heartbeat 5 s, mission timeout) > return (battery). Battery and leak readings older than 3 s (provisional) or stamped in the future are ignored; a heartbeat stamped in the future is not a fresh heartbeat.
- "Surface phase" for the heartbeat rules = every mission phase except STANDBY and DIVE_AND_GRASP.
- After SAFETY_STOP the mission resumes only with fresh odometry (≤ t_odom) and a valid trajectory (a plan dispatched after the clear, or an active hold in a rescue phase).
- Finding: the near-wall rule (hull within 0.5 m, base_link within about 1.0 m of a wall) cannot be reached by planned motion, because A* (robot radius 1.3 m) keeps base_link at least 1.3 m from walls and cannot plan from closer. It acts only after a disturbance; a vehicle that ends up that close cannot plan away. To be revisited with the costmap / corridor clearances.

## 6. Parameters

All parameters live in YAML files, named after the symbols below. "Source" states where the current value comes from; "estimate" and "TBD" values are replaced by identified values after the experiments in Section 7. Hydrodynamic coefficients follow Fossen's sign convention (damping and added mass are negative).

### 6.1 Hull and hydrodynamics

| Parameter | Symbol | Initial | Unit | Source | Used by |
|---|---|---|---|---|---|
| Mass | m | 50 | kg | estimate → weighing | sim, planning, control |
| Yaw moment of inertia | I_z | 8.67 | kg·m² | CAD | sim, planning, control |
| Center of gravity | r_g | (0, 0, 0) | m | CAD → suspension test | sim, allocation |
| Center of buoyancy | r_b | TBD | m | CAD → still-water test | sim |
| Displaced volume | ∇ | TBD | m³ | CAD → still-water test | sim |
| Static draft | T | 0.20 | m | estimate → measured | drag estimate, sim |
| Frontal area (surge) | A_x | 0.21 | m² | 1.04 m × T | drag estimate |
| Frontal area (sway) | A_y | 0.20 | m² | 1.00 m × T | drag estimate |
| Added mass (surge) | X_udot | −15 | kg | estimate → identification | sim, planning, control |
| Added mass (sway) | Y_vdot | −15 | kg | estimate → identification | sim, planning, control |
| Added inertia (yaw) | N_rdot | −1.5 | kg·m² | estimate → identification | sim, planning, control |
| Linear damping (surge) | X_u | −10 | N·s/m | estimate → identification | sim, planning, control |
| Linear damping (sway) | Y_v | −10 | N·s/m | estimate → identification | sim, planning, control |
| Linear damping (yaw) | N_r | −5 | N·m·s | estimate → identification | sim, planning, control |
| Quadratic damping (surge) | X_u\|u\| | −100 | N·s²/m² | ½ρC_dA_x, C_d = 1.0 → identification | sim, planning, control |
| Quadratic damping (sway) | Y_v\|v\| | −100 | N·s²/m² | ½ρC_dA_y, C_d = 1.0 → identification | sim, planning, control |
| Quadratic damping (yaw) | N_r\|r\| | −30 | N·m·s² | estimate → identification | sim, planning, control |
| Wave-drag knee speed | v_wave | 1.25 | m/s | Fr = 0.4, L = 1.0 m → fitted from runs | drag model |
| Bow offset from base_link | x_bow | 0.50 | m | CAD (hull length 1.00 m / 2) | terminal pose generation |
| Forward protrusion of stowed arms beyond the bow | L_arm | TBD; simulation 0.10 (provisional, must be set explicitly) | m | arm CAD | terminal pose generation, clearance check |
| Arm grasp workspace (deployed, horizontal range ahead of bow) | — | TBD | m | arm CAD | validating g_dive / g_surface (staging point reach) |

### 6.2 Thrusters (T500 × 8)

Layout: thrusters 01–04 horizontal at ±45°, 05–08 vertical (+Z).

| Parameter | Symbol | Initial | Unit | Source | Used by |
|---|---|---|---|---|---|
| Mount position and thrust axis | r_i, e_i | `thrusters_v4.yaml` | m, unit vector | CAD | allocation, sim |
| Allocation matrix | B | computed from r_i, e_i | — | computed | allocation, planning |
| Peak forward thrust | f_peak | 158 (16.1 kgf @ 24 V) | N | manufacturer spec | driver, safety check |
| Peak reverse thrust | f_rev | TBD | N | manufacturer curve → bench test | allocation, planning |
| Planning thrust limit | f_plan | 110 | N | set, ~70% of peak | planning back end |
| Control thrust limit | f_ctrl | 140 | N | set | allocation |
| Allowed peak-thrust duration | t_peak | 60 | s | manufacturer guidance | safety monitor |
| Thrust–PWM curve | f(pwm) | manufacturer table | N, µs | spec → bench test | driver |
| PWM deadband | pwm_db | TBD | µs | bench test | driver |
| Thrust time constant | τ_m | 0.1 | s | estimate → bench test | sim, NMPC |
| Bus voltage | V_bus | 24 | V | set | driver, power |
| Peak power per thruster | P_peak | ~1000 | W | spec | battery sizing |
| Polarity / propeller handedness per thruster | s_i | TBD | ±1 | measured after assembly | driver |

### 6.3 Environment

| Parameter | Symbol | Initial | Unit | Source | Used by |
|---|---|---|---|---|---|
| Water density | ρ | 997 | kg/m³ | fresh water, 25 °C | drag, buoyancy |
| Gravity | g | 9.81 | m/s² | constant | sim |
| Water temperature | T_w | record every run | °C | measured on site | density correction, thruster heating |
| Pool size | L × W × H | 50 × 25 × 5 | m | sim world → measured on site | map, sim |
| Lane ropes, ladders, steps | — | TBD | m | measured on site | static map |
| Inlet / outlet positions and flow speed | v_c | sim 0–0.2 | m/s | measured on site | disturbance tests |
| Wind speed (outdoor pool) | v_wind | record every run | m/s | measured on site | disturbance tests |

Water temperature must be recorded every run: the M500 motor used in the T500 is rated for full power at 24 V in 2–27 °C water, and heated pools typically run at 26–28 °C, right at the limit.

### 6.4 Perception and localization

| Parameter | Symbol | Initial | Unit | Source | Used by |
|---|---|---|---|---|---|
| Shore target update rate | f_shore | 30 | Hz | set | mission, EKF |
| Shore end-to-end latency | t_shore | TBD | ms | LED sync-flash test | EKF delay compensation |
| Shore position noise | σ_shore | 0.05 | m | estimate → calibration | EKF, terminal pose |
| Victim state | s_v | {surface, submerged} | — | shore output | terminal pose, mission |
| Swimmer track update rate | f_swim | 10 | Hz | set | swimmer layer |
| Shore–onboard clock sync error | Δt_sync | ≤ 5 | ms | chrony, measured | EKF |
| Pool frame → shore camera extrinsics | T_pool_cam | TBD | — | calibration board | shore localization |
| EKF output rate | f_ekf | 50 | Hz | set | control |

At 1.5 m/s, every 100 ms of shore latency is 15 cm of position error, so the EKF compensates measurements by timestamp rather than using the latest measurement as-is.

### 6.5 Planning

| Parameter | Symbol | Initial | Unit | Used by |
|---|---|---|---|---|
| Map resolution | res | 0.05 | m | costmap |
| Robot footprint | — | 1.00 × 1.04 | m | costmap |
| Inflation radius | r_infl | 0.5 | m | costmap |
| Cruise speed | v_cruise | 1.5 | m/s | back end |
| Sprint speed | v_sprint | 2.0 | m/s | back end |
| Acceleration limit | a_max | 1.5 | m/s² | back end |
| Yaw rate limit | ω_max | 1.5 | rad/s | back end |
| Yaw acceleration limit | α_max | 2.0 | rad/s² | back end |
| Braking-curve deceleration | a_b | 1.0 | m/s² | back end |
| MINCO order | s | 3 | — | back end |
| Time weight | ρ_T | 1024 | — | back end (tune) |
| Samples per piece | K | 16 | — | back end |
| Thrust-constraint penalty weight | w_τ | 1e4 | — | back end (tune) |
| Velocity-constraint penalty weight | w_v | 1e5 (provisional; tuned up from 1e4, see note below) | — | back end (tune) |
| FIRI seed spacing | Δs | 1.0 | m | corridor |
| Max corridor polygons | N_poly | 30 | — | corridor |
| Replanning rate | f_replan | 5 | Hz | trajectory generator |
| Target-shift replan threshold | Δp_v | 0.3 | m | trajectory generator |
| Tracking-error replan threshold | e_replan | 0.5 | m | trajectory generator |
| Body radius (victim) | r_body | 0.25 | m | terminal pose |
| Standoff gap, submerged victim | g_dive | 0.30 (provisional simulation value) | m | terminal pose |
| Standoff gap, surface victim | g_surface | 0.15 (provisional simulation value) | m | terminal pose |
| Heading hold distance (FINAL_APPROACH zone) | d_hold | 2.0 | m | terminal pose |
| Heading re-aim threshold (bearing) | ψ_reaim | 0.15 | rad | terminal pose |
| Heading re-aim threshold (victim shift since last lock / re-aim) | Δp_reaim | 0.3 | m | terminal pose |
| Heading hold release hysteresis | d_hyst | 0.5 (provisional) | m | terminal pose |
| Least gap, stowed outline to victim body | g_min | 0.0 (provisional, must be ≥ 0) | m | clearance check |
| Surface-victim final zone / speed cap | d_final, v_final | 2.0, 0.3 | m, m/s | back end |
| Terminal horizontal acceptance (simulation) | e_goal | 0.25 | m | mission, step 8 acceptance (not a controller tolerance) |
| Terminal yaw acceptance (simulation) | e_ψ | 0.15 | rad | mission, step 8 acceptance (not a controller tolerance) |
| Swimmer prediction horizon | t_pred | 2.0 | s | swimmer layer |
| Assumed max swimmer speed | v_swim | 2.0 | m/s | swimmer layer |

Provisional implementation parameters added in steps 8B–10 (all in YAML, marked provisional): final-zone penalty buffer 0.25 m and cap phase-in headroom 0.1 m/s (optimizer); retry period 1.0 s and target max age 1.0 s (trajectory generator); swimmer max track age 1.0 s and future-stamp tolerance 0.1 s (prediction); sensor max age 3.0 s and future tolerance 0.5 s (safety monitor); mission event max age 2.0 s and the RETURN shore pose (mission manager). The Gazebo backend's hydrostatics (waterplane area 0.2508 m² so the hull floats at T = 0.20 m with m = 50 kg, heave damping −350 N·s/m, roll/pitch restoring 24.5 N·m/rad and damping −10 N·m·s/rad) are provisional simulation values; ∇ and r_b stay TBD.

Note on w_v: with 1e4, warm-started replans of the L-turn test case ended about 0.02 m/s above the braking curve in the last 2 cm before the goal in 20 of 20 runs: the violation is so short that its integrated penalty cost less than the time it saved. With 1e5 the same runs passed 20 of 20. The higher weight makes cold solves slower (about 1.4–1.7x on the step 7 cases). The value stays provisional until the optimizer is tuned for the < 50 ms target.

### 6.6 Control

| Parameter | Symbol | Initial | Unit | Note |
|---|---|---|---|---|
| Control rate | f_ctrl | 50 | Hz | |
| Position gain (x, y) | K_p,xy | 200 | N/m | ω_n ≈ 1.75 rad/s |
| Velocity gain (x, y) | K_d,xy | 150 | N·s/m | ζ ≈ 0.66 |
| Yaw gain | K_p,ψ | 50 | N·m/rad | |
| Yaw-rate gain | K_d,ψ | 30 | N·m·s/rad | |
| Feedforward enabled | use_ff | true | — | |
| Allocation weight | W | diag(1, 1, 2) | — | |
| Allocation regularization | λ | 1e-3 | — | |
| NMPC horizon / steps | T_h, N | 1.0, 20 | s, — | v2 controller |
| Vertical-thruster attitude-hold limit | f_att | 32 | N | 20% of peak |
| Trajectory completion tolerance (position) | — | 0.15 | m | controller reports success inside this (step 5); tighter than the 0.25 m simulation acceptance |
| Trajectory completion tolerance (yaw) | — | 0.1 | rad | as above; tighter than the 0.15 rad simulation acceptance |

### 6.7 Mission and safety

| Parameter | Symbol | Initial | Unit |
|---|---|---|---|
| Odometry timeout | t_odom | 0.2 | s |
| Heartbeat timeout (stop) | t_hb_stop | 1.0 | s |
| Heartbeat timeout (abort) | t_hb_abort | 5.0 | s |
| Thrust saturation duration | t_sat | 2.0 | s |
| Swimmer slow-zone radius / speed | r_slow, v_slow | 3.0, 0.5 | m, m/s |
| Swimmer clearance | r_swim | 1.0 | m |
| Near-wall speed limit | v_wall | 0.3 | m/s |
| Low-battery threshold | SOC_min | 20 | % |
| Mission timeout | t_mission | 120 | s |

---

## 7. Parameter identification

The "estimate" and "TBD" values above come from these 8 experiments, run in order.

| Experiment | Parameters obtained | Method |
|---|---|---|
| 1. Weighing and suspension | m, r_g | Scale; two-point suspension for CG |
| 2. Still-water float | T, ∇, r_b | Measure draft at four corners; shift a known weight and measure heel |
| 3. Bench thrust test | f(pwm), f_rev, pwm_db, τ_m, current | Single thruster on a load cell; PWM sweep plus steps |
| 4. Constant-thrust surge runs | X_u, X_u\|u\|, v_wave | One run each at 20/40/60/80/100% thrust; fit steady speed to F = −X_u·u − X_u\|u\|·u\|u\| |
| 5. Constant-thrust sway and yaw runs | Y_v, Y_v\|v\|, N_r, N_r\|r\| | Same as 4, pure sway and pure rotation |
| 6. Step-thrust start | X_udot, Y_vdot, N_rdot | Initial acceleration from rest: a_0 = F / (m − X_udot) |
| 7. Coast-down | Damping cross-check | Cut thrust at 1.5 m/s and record the velocity decay |
| 8. Shore latency test | t_shore | Onboard LED flashes on timestamps; compare with shore detection time |

Each test result is judged by: arrival time, terminal horizontal error, tracking-error RMS and max, thrust-saturation time fraction, minimum distance to swimmers and walls, peak power and energy, replan and replan-failure counts, plus water temperature, wind, circulation-pump state and starting battery level.
