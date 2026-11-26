# Scenario metrics (implementation step 11)

`scripts/bag_to_metrics.py BAG_DIR [--expect SCENARIO.yaml]` turns one
recorded run into one CSV row and, with `--expect`, checks the scenario's
expectations (exit 0 only if all pass). `scripts/run_scenario.py SCENARIO...`
(or `--matrix matrix`) launches, records, drives, checks and summarizes runs;
exit 0 all passed, 1 an expectation failed, 2 infrastructure failure
(startup or end-condition timeout, no bag, required data missing, processes
left after SIGKILL).

Only recorded data is used. Time base: the bag's receive time of each
message, so reprocessing a bag gives the same row. Times are relative to
the first target (`/victim_target`, `/shore_alarm`, `/victim`, `/goal_pose`)
or, for a runner-driven controller test, the first controller feedback
("t = 0"). Reports: `/planner_report` and `/scenario_report` (runner-driven
tests); `/mission_report` and `/safety_report` lines are also matched by the
report expectations.

## Columns

| Column | Unit | Definition | Source topic(s) |
|---|---|---|---|
| `scenario`, `bag` | – | Scenario name and bag path | runner arguments |
| `victim_state` | – | `submerged` / `surface`, from the first PLANNED report | `/planner_report` |
| `target_x_m`, `target_y_m` | m | First target sent, map frame | `/victim_target` |
| `arrival_status` | – | From the first final report after t = 0: `accepted` (ACCEPTED), `not_accepted` (NOT ACCEPTED), `rejected` (REJECTED, BLOCKED, DISCARDED, NOT SENT), `timeout` (none in the bag), `no_target` | `/planner_report` |
| `arrival_time_s` | s | t of the ACCEPTED report (controller finished and the reached pose passed the 0.25 m / 0.15 rad acceptance and clearance checks); empty unless `accepted` | `/planner_report` |
| `run_end_s` | s | t of the first final report, whatever its status | `/planner_report` |
| `terminal_error_m` | m | Horizontal distance from base_link (TF map → odom → base_link, last sample at or before the final report) to the terminal pose of the last PLANNED / REPLANNED report | `/tf`, `/tf_static`, `/planner_report` |
| `terminal_yaw_error_rad` | rad | \|yaw error\| for the same pair, wrapped to [0, π] | same |
| `reported_terminal_error_m` | m | The error the trajectory generator itself reported in ACCEPTED / NOT ACCEPTED (cross-check) | `/planner_report` |
| `final_speed_mps` | m/s | Planar speed of the last odometry sample at or before the final report | `/odom` |
| `tracking_rms_m`, `tracking_max_m` | m | RMS and maximum of the controller's horizontal tracking error (robot to the reference at that instant) over all feedback samples from t = 0 to the final report; feedback is published at 10 Hz | `/follow_trajectory/_action/feedback` |
| `tracking_samples` | – | Number of feedback samples used | same |
| `saturation_time_s` | s | Time with `/thrust_saturated` true in the tracking window (first feedback sample to the final report), each flag held until the next message | `/thrust_saturated` |
| `saturation_window_s` | s | Time covered by saturation flags in that window | same |
| `saturation_fraction` | – | `saturation_time_s / saturation_window_s` (0..1) | same |
| `plans` | – | Number of PLANNED reports | `/planner_report` |
| `periodic_replans` | – | REPLANNED reports from the 5 Hz timer | `/planner_report` |
| `replan_latency_mean_ms`, `_p95_ms`, `_max_ms` | ms | "replan total" of every REPLANNED report: trigger to dispatch (A*, corridor, MINCO, checks) | `/planner_report` |
| `replan_rate_hz` | Hz | (PLANNED + REPLANNED − 1) / time between the first and last of them | `/planner_report` |
| `brakes`, `blocked`, `swimmer_conflicts` | – | BRAKING, BLOCKED and SWIMMER CONFLICT reports | `/planner_report` |
| `final_report` | – | Text of the first final report (status, reason checks) | `/planner_report`, `/scenario_report` |
| `max_speed_mps` | m/s | Highest odometry planar speed from t = 0 to the final report | `/odom` |
| `final_zone_max_speed_mps` | m/s | Highest odometry speed while base_link is within 2.0 m of the final terminal pose (surface cap, design 3 item 5) | `/odom`, `/tf` |
| `cruise_min_speed_mps` | m/s | Lowest speed from first reaching 80 % of the run's peak speed until base_link comes within 2.5 m of the terminal pose ("without stopping") | `/odom`, `/tf` |
| `bow_error_rad` | rad | \|bearing from base_link to the latest victim − yaw\| at the end (bow faces the victim) | `/tf`, victim topics |
| `min_wall_distance_m` | m | Least distance from the stowed hull outline (terminal_pose.yaml) to a blocked or unknown map cell over the whole recording (10 Hz poses, Euclidean distance transform of the recorded `/map`, minus half a cell) | `/map`, `/tf` |
| `min_swimmer_distance_m` | m | Least distance from the stowed hull outline to any swimmer's noise-free position at the same stamp (interpolated) | `/swimmer_truth`, `/tf` |
| `mission_phases` | – | Sequence of mission phases | `/mission_state` |
| `mission_complete_s` | s | t of STANDBY after RETURN | `/mission_state` |
| `safety_rules` | – | Every safety rule that was active at least once | `/safety_status` |
| `zero_thrust_max_force_n` | N | Largest thruster force while zero_thrust had been in force for at least 0.1 s | `/safety_status`, `/thruster_command` |
| `replans` | – | Number of REPLANNED reports (replacements sent) | `/planner_report` |
| `replan_failures` | – | Number of REPLAN FAILED reports (incl. optimizer timeouts, late or conflicting replacements) | `/planner_report` |
| `replans_discarded` | – | Number of REPLAN DISCARDED reports | `/planner_report` |
| `notes` | – | Missing data and the reasons for empty columns | – |

## Empty columns (not supported yet)

| Column | Unit | Why empty / missing input |
|---|---|---|
| `peak_power_w`, `energy_j` | W, J | No electrical input: the simulation has no thruster current or battery voltage, and there is no calibrated force → electrical power model yet (bench thrust test, design §7 experiment 3). `/thruster_command` (forces) is recorded so power can be computed later. |

`min_wall_distance_m` is empty for bags without `/map` (for example the
step 11 first baseline of 2026-11-24); `min_swimmer_distance_m` is empty
when no swimmer existed.

Design §7 also lists water temperature, wind, circulation-pump state and
starting battery level. These are pool-test conditions, not simulation
outputs, so they are not columns yet.

## Expectations (scenario `expect:`)

`status` / `reason` (regex on the final report), `limits` (`{column: {min,
max}}`; a missing value fails), `reports_present` / `reports_absent`
(regexes), `phases` (mission phases in this order, others may be between),
`safety_rules` / `safety_rules_absent`, `required_topics` (minimum message
counts; the runner adds /odom, /tf, /tf_static, /planner_report, /map).
