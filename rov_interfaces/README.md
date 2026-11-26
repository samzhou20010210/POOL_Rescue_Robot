# rov_interfaces

Messages and actions for the surface high-speed approach
(see `../SURFACE_APPROACH_DESIGN.md`, sections 3 and 4).

| Interface | Purpose |
|---|---|
| `msg/ThrusterCommand` | Force for each of the 8 thrusters (N) |
| `msg/TrajectoryPoint` | One sample of a planar trajectory |
| `msg/Trajectory` | Time-parameterized planar trajectory (list of samples) |
| `action/FollowTrajectory` | Ask the controller to track a trajectory |
| `msg/SwimmerTrack` | One non-target swimmer track: ID, stamp, frame, position (m), velocity (m/s) |
| `msg/SwimmerTrackArray` | All current swimmer tracks from the shore tracker |
| `msg/VictimTarget` | Victim position (m) and state (surface / submerged): shore alarm and target updates |
| `msg/PlannerStatus` | Trajectory generator events (planned, replanned, failed, blocked, braking, holding, arrived) |
| `msg/SafetyStatus` | Safety monitor output: speed limit (m/s), stop / hold, zero thrust, replan, abort |
| `msg/MissionState` | Mission phase and the last transition |
| `msg/MissionEvent` | Simulated or operator mission events (rescue complete, manual takeover) |
| `msg/LeakAlarm` | Leak sensor state |

## Units

SI units everywhere: m, m/s, m/s², rad, rad/s, N, s.

## Frames

- `ThrusterCommand`: `header.frame_id = base_link` (x forward, y left, z up).
- `Trajectory`: `header.frame_id = map` (pool-centred, x along the pool
  length, y across, z up). Positions, velocities and accelerations in
  `TrajectoryPoint` are all in this frame, **not** in the body frame.
- `yaw`: angle from the map x axis to the base_link x axis,
  counter-clockwise positive. It is continuous (unwrapped) along a
  trajectory. Errors (`yaw_error`, `final_yaw_error`) are wrapped to [−π, π].

## Timestamps

- `ThrusterCommand.header.stamp`: time the command was computed.
- `Trajectory.header.stamp`: absolute start time t0. Point k is the
  reference at `t0 + points[k].time_from_start`. The first point has
  `time_from_start = 0` and times strictly increase. After the last point,
  the reference holds the last point (v = 0, a = 0).
- With `use_sim_time:=true` all stamps are simulation time.

## Thruster order

`ThrusterCommand.force[i]` is `thruster_0(i+1)` in
`rov_description/config/thrusters_v4.yaml`:

| Index | Thruster | Type |
|---|---|---|
| 0–3 | `thruster_01`–`thruster_04` | horizontal, ±45° in the x-y plane |
| 4–7 | `thruster_05`–`thruster_08` | vertical, +z |

Positive force pushes the vehicle along that thruster's `direction` vector;
negative is reverse thrust. Limits (for example f_ctrl = 140 N, provisional)
are parameters, not part of the message.
