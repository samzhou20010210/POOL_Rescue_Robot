#!/usr/bin/env python3
# Copyright 2026 sam
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Compute surface-approach run metrics from one rosbag (offline).

Usage:
  bag_to_metrics.py BAG_DIR [--scenario NAME] [--backend NAME] [--output FILE.csv]
                    [--expect SCENARIO.yaml]

Writes a CSV header and one row (to stdout without --output). Only the
recorded data is used, with the bag receive times as the time base, so the
same bag always gives the same row. With --expect, the scenario's
expectations are checked against the row and the recorded events; the exit
code is then 0 only if every check passes. Metric definitions and units:
rov_bringup/doc/scenario_metrics.md.
"""

import argparse
import csv
import math
import os
import re
import sys

from ament_index_python.packages import get_package_share_directory
import numpy as np
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from scipy import ndimage
import yaml

COLUMNS = [
    'scenario', 'backend', 'bag', 'victim_state', 'target_x_m', 'target_y_m',
    'arrival_status', 'arrival_time_s', 'run_end_s', 'final_report',
    'terminal_error_m', 'terminal_yaw_error_rad', 'reported_terminal_error_m',
    'final_speed_mps',
    'tracking_rms_m', 'tracking_max_m', 'tracking_samples',
    'saturation_fraction', 'saturation_time_s', 'saturation_window_s',
    'plans', 'replans', 'periodic_replans', 'replan_failures', 'replans_discarded',
    'replan_latency_mean_ms', 'replan_latency_p95_ms', 'replan_latency_max_ms',
    'replan_rate_hz', 'brakes', 'blocked', 'swimmer_conflicts',
    'min_swimmer_distance_m', 'min_wall_distance_m', 'max_speed_mps',
    'final_zone_max_speed_mps', 'cruise_min_speed_mps', 'bow_error_rad',
    'mission_phases', 'mission_complete_s', 'safety_rules', 'zero_thrust_max_force_n',
    'peak_power_w', 'energy_j',
    'notes',
]

# Final planner reports and the arrival status they give.
FINAL_REPORTS = [
    ('NOT ACCEPTED', 'not_accepted'),
    ('ACCEPTED', 'accepted'),
    ('REJECTED', 'rejected'),
    ('DISCARDED', 'rejected'),
    ('NOT SENT', 'rejected'),
]

NUMBER = r'(-?\d+(?:\.\d+)?)'
TERMINAL_POSE = re.compile(rf'terminal pose \({NUMBER}, {NUMBER}, {NUMBER} rad\)')
REPORTED_ERROR = re.compile(rf'error {NUMBER} m / {NUMBER} rad')
VICTIM_STATE = re.compile(r'^PLANNED (\w+) target')
REPLAN_TOTAL = re.compile(rf'replan total {NUMBER} ms')
PHASES = ['STANDBY', 'TRANSIT', 'FINAL_APPROACH', 'DIVE_AND_GRASP', 'SURFACE_HOLD', 'RETURN',
          'SAFETY_STOP']

UNSUPPORTED = (
    'peak_power_w, energy_j: no electrical input recorded (the simulation has no '
    'thruster current or voltage, and no calibrated force-to-power model exists yet, '
    'design 7 experiment 3)')


def read_bag(path):
    """Return {topic: [(receive_time_s, message), ...]} in bag order."""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    classes = {}
    messages = {topic: [] for topic in types}
    while reader.has_next():
        topic, data, stamp_ns = reader.read_next()
        if topic not in classes:
            classes[topic] = get_message(types[topic])
        messages[topic].append((stamp_ns * 1e-9, deserialize_message(data, classes[topic])))
    return messages


def yaw_of(q):
    """Yaw (rad) of a geometry_msgs quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(angle):
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def planar(transform):
    """(x, y, yaw) of a geometry_msgs Transform."""
    return (transform.translation.x, transform.translation.y, yaw_of(transform.rotation))


def compose(a, b):
    """Planar a * b for poses (x, y, yaw)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2]))


def last_before(series, t):
    """Last (time, value) of a time-ordered series with time <= t, or None."""
    found = None
    for item in series:
        if item[0] > t:
            break
        found = item
    return found


def robot_poses(messages):
    """Time-ordered [(t_receive, (x, y, yaw), t_stamp)]: base_link in the map frame (TF)."""
    map_odom = None
    for _, message in messages.get('/tf_static', []):
        for tf in message.transforms:
            if tf.header.frame_id == 'map' and tf.child_frame_id == 'odom':
                map_odom = planar(tf.transform)
    if map_odom is None:
        return []
    poses = []
    for stamp, message in messages.get('/tf', []):
        for tf in message.transforms:
            if tf.header.frame_id == 'odom' and tf.child_frame_id == 'base_link':
                t_stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
                poses.append((stamp, compose(map_odom, planar(tf.transform)), t_stamp))
    return poses


def outline():
    """Return the stowed hull shape: front, rear and half width in m (terminal_pose.yaml)."""
    path = os.path.join(get_package_share_directory('rov_planning'), 'config',
                        'terminal_pose.yaml')
    with open(path) as stream:
        s = yaml.safe_load(stream)
    return (s['bow_offset'] + s['stowed_arm_protrusion'], s['stern_offset'],
            s['hull_half_width'])


def outline_points(pose, shape, step=0.02):
    """Points along the outline rectangle of the hull at pose (map frame)."""
    front, rear, half = shape
    corners = [(front, half), (-rear, half), (-rear, -half), (front, -half)]
    c, s = math.cos(pose[2]), math.sin(pose[2])
    points = []
    for (x0, y0), (x1, y1) in zip(corners, corners[1:] + corners[:1]):
        n = max(1, int(math.hypot(x1 - x0, y1 - y0) / step))
        for k in range(n):
            x = x0 + (x1 - x0) * k / n
            y = y0 + (y1 - y0) * k / n
            points.append((pose[0] + c * x - s * y, pose[1] + s * x + c * y))
    return np.array(points)


def footprint_distance(pose, shape, point):
    """Distance (m) from the hull outline at pose to a point; 0 inside."""
    front, rear, half = shape
    dx, dy = point[0] - pose[0], point[1] - pose[1]
    c, s = math.cos(pose[2]), math.sin(pose[2])
    bx, by = c * dx + s * dy, -s * dx + c * dy
    ex = max(-rear - bx, 0.0, bx - front)
    ey = max(abs(by) - half, 0.0)
    return math.hypot(ex, ey)


def min_wall_distance(messages, poses, shape):
    """Least hull-outline distance (m) to a blocked or unknown map cell, or None."""
    maps = messages.get('/map', [])
    if not maps or not poses:
        return None
    grid = maps[-1][1]
    info = grid.info
    data = np.array(grid.data, dtype=np.int16).reshape(info.height, info.width)
    blocked = (data >= 65) | (data < 0)
    # Distance (cells) from every cell centre to the nearest blocked cell centre.
    distance = ndimage.distance_transform_edt(~blocked) * info.resolution
    ox, oy, res = info.origin.position.x, info.origin.position.y, info.resolution
    best = math.inf
    for _, pose, _ in poses[::5]:  # every 5th TF sample (10 Hz)
        pts = outline_points(pose, shape)
        i = np.clip(((pts[:, 0] - ox) / res).astype(int), 0, info.width - 1)
        j = np.clip(((pts[:, 1] - oy) / res).astype(int), 0, info.height - 1)
        # Cell centre to cell centre, minus half a cell to the blocked cell's edge.
        best = min(best, float(np.min(distance[j, i])) - 0.5 * res)
    return max(best, 0.0)


def swimmer_truth(messages):
    """{id: [(t, x, y)]} from the noise-free swimmer topic (stamps in seconds)."""
    tracks = {}
    for _, message in messages.get('/swimmer_truth', []):
        for track in message.tracks:
            t = track.header.stamp.sec + track.header.stamp.nanosec * 1e-9
            tracks.setdefault(track.id, []).append((t, track.position.x, track.position.y))
    return tracks


def min_swimmer_distance(messages, poses, shape):
    """(least hull-to-swimmer distance in m, swimmer id) or (None, None)."""
    tracks = swimmer_truth(messages)
    if not tracks or not poses:
        return None, None
    best, best_id = math.inf, None
    for _, pose, t_stamp in poses:
        for swimmer_id, samples in tracks.items():
            if t_stamp < samples[0][0] - 0.05 or t_stamp > samples[-1][0] + 0.05:
                continue  # the swimmer does not exist at this time
            times = [q[0] for q in samples]
            x = float(np.interp(t_stamp, times, [q[1] for q in samples]))
            y = float(np.interp(t_stamp, times, [q[2] for q in samples]))
            d = footprint_distance(pose, shape, (x, y))
            if d < best:
                best, best_id = d, swimmer_id
    return (best if math.isfinite(best) else None), best_id


def fmt(value, digits=6):
    """CSV text for a number (empty if None)."""
    if value is None:
        return ''
    if isinstance(value, int):
        return str(value)
    return f'{value:.{digits}f}'


def compute(messages, scenario, bag, backend=''):
    """(row: dict of CSV text, data: events for the expectation checks)."""
    row = {column: '' for column in COLUMNS}
    row['scenario'] = scenario
    row['backend'] = backend
    row['bag'] = bag
    notes = []
    data = {'reports': [], 'phases': [], 'safety_rules': set(), 'topic_counts': {}}
    data['topic_counts'] = {topic: len(items) for topic, items in messages.items()}

    targets = messages.get('/victim_target', []) + messages.get('/shore_alarm', []) + \
        messages.get('/victim', []) + messages.get('/goal_pose', [])
    targets.sort(key=lambda item: item[0])
    # /scenario_report: outcomes of runner-driven tests (step 5 controller runs).
    all_reports = sorted([(t, m.data) for topic in ('/planner_report', '/scenario_report')
                          for t, m in messages.get(topic, [])])
    feedback_times = [t for t, _ in messages.get('/follow_trajectory/_action/feedback', [])]
    # No target (runner-driven controller test): start at the first feedback.
    t_start = targets[0][0] if targets else feedback_times[0] - 0.2 if feedback_times else \
        (all_reports[0][0] if all_reports else None)
    if t_start is None:
        row['arrival_status'] = 'no_target'
        row['notes'] = 'no target and no planner report recorded'
        return row, data
    if targets:
        first = targets[0][1]
        point = first.point if hasattr(first, 'point') else \
            first.position if hasattr(first, 'position') else first.pose.position
        row['target_x_m'] = fmt(point.x, 3)
        row['target_y_m'] = fmt(point.y, 3)
    reports = [(t, r) for t, r in all_reports if t >= t_start - 1e-6]
    # Mission and safety lines join the reports for the expectation checks.
    other = [(t, m.data) for topic in ('/mission_report', '/safety_report')
             for t, m in messages.get(topic, []) if t >= t_start - 1e-6]
    data['reports'] = sorted([(t - t_start, r) for t, r in reports + other])

    def count(prefix):
        return sum(1 for _, r in reports if r.startswith(prefix))

    row['plans'] = fmt(count('PLANNED'))
    row['replans'] = fmt(count('REPLANNED'))
    row['periodic_replans'] = fmt(count('REPLANNED (periodic)'))
    row['replan_failures'] = fmt(count('REPLAN FAILED') + count('PLAN FAILED'))
    row['replans_discarded'] = fmt(count('REPLAN DISCARDED'))
    row['brakes'] = fmt(count('BRAKING'))
    row['blocked'] = fmt(count('BLOCKED'))
    row['swimmer_conflicts'] = fmt(count('SWIMMER CONFLICT'))
    latencies = sorted(float(m.group(1)) for _, r in reports
                       for m in [REPLAN_TOTAL.search(r)] if r.startswith('REPLANNED') and m)
    if latencies:
        row['replan_latency_mean_ms'] = fmt(sum(latencies) / len(latencies), 1)
        row['replan_latency_p95_ms'] = fmt(
            latencies[min(len(latencies) - 1, int(math.ceil(0.95 * len(latencies))) - 1)], 1)
        row['replan_latency_max_ms'] = fmt(latencies[-1], 1)
    for _, report in reports:
        match = VICTIM_STATE.match(report)
        if match:
            row['victim_state'] = match.group(1)
            break

    # The first final report ends the approach (the mission may continue).
    final = None
    for t, report in reports:
        status = next((s for prefix, s in FINAL_REPORTS if report.startswith(prefix)), None)
        if status:
            final = (t, report, status)
            break
    if final is None:
        row['arrival_status'] = 'timeout'
        notes.append('no final planner report in the bag')
        t_end = max(t for topic in messages.values() for t, _ in topic)
    else:
        t_end, report, status = final
        row['arrival_status'] = status
        row['final_report'] = report.replace('\n', ' ')
        row['run_end_s'] = fmt(t_end - t_start, 3)
        if status == 'accepted':
            row['arrival_time_s'] = fmt(t_end - t_start, 3)
        match = REPORTED_ERROR.search(report)
        if match:
            row['reported_terminal_error_m'] = fmt(float(match.group(1)), 3)
    executing = [t for t, r in reports if r.startswith(('PLANNED', 'REPLANNED'))]
    if len(executing) > 1 and count('REPLANNED (periodic)') > 0:
        span = executing[-1] - executing[0]
        if span > 0:
            row['replan_rate_hz'] = fmt((len(executing) - 1) / span, 2)

    poses = robot_poses(messages)
    shape = outline()
    # Terminal error: robot (TF) at the end vs the terminal pose of the final
    # report (verified against the latest target), else the last plan's.
    terminal = None
    for t, report in reports:
        if t <= t_end and report.startswith(('PLANNED', 'REPLANNED', 'ACCEPTED', 'NOT ACC')):
            match = TERMINAL_POSE.search(report)
            if match:
                terminal = tuple(float(v) for v in match.groups())
    if row['arrival_status'] in ('accepted', 'not_accepted') and terminal:
        latest = last_before(poses, t_end)
        if latest:
            robot = latest[1]
            row['terminal_error_m'] = fmt(math.hypot(robot[0] - terminal[0],
                                                     robot[1] - terminal[1]), 4)
            row['terminal_yaw_error_rad'] = fmt(abs(wrap(robot[2] - terminal[2])), 4)
        else:
            notes.append('no TF map -> base_link for the terminal error')
        odom = last_before(messages.get('/odom', []), t_end)
        if odom:
            twist = odom[1].twist.twist.linear
            row['final_speed_mps'] = fmt(math.hypot(twist.x, twist.y), 4)
    speeds = [(t, math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y))
              for t, m in messages.get('/odom', []) if t_start <= t <= t_end]
    if speeds:
        row['max_speed_mps'] = fmt(max(v for _, v in speeds), 3)
    if speeds and poses and terminal:
        # Pair each speed with the TF pose received closest before it.
        paired = []
        k = 0
        for t, v in speeds:
            while k + 1 < len(poses) and poses[k + 1][0] <= t:
                k += 1
            pose = poses[k][1]
            paired.append((t, v, math.hypot(pose[0] - terminal[0], pose[1] - terminal[1])))
        zone = [v for _, v, d in paired if d <= 2.0]
        if zone:
            row['final_zone_max_speed_mps'] = fmt(max(zone), 3)
        # Cruise: from first reaching 80 % of the run's peak speed to first
        # coming within 2.5 m of the terminal pose (the final braking is
        # excluded). A stop or a dip in between shows as a low minimum.
        peak = max(v for _, v, _ in paired)
        started = next((i for i, (_, v, _) in enumerate(paired) if v >= 0.8 * peak), None)
        if started is not None:
            cruise = []
            for _, v, d in paired[started:]:
                if d <= 2.5:
                    break
                cruise.append(v)
            if cruise:
                row['cruise_min_speed_mps'] = fmt(min(cruise), 3)
    victims = [(t, m.point if hasattr(m, 'point') else m.position)
               for topic in ('/victim_target', '/victim', '/shore_alarm', '/shore_victim')
               for t, m in messages.get(topic, []) if t <= t_end]
    if victims and poses and row['arrival_status'] in ('accepted', 'not_accepted') and \
            row['victim_state']:
        victim = max(victims, key=lambda item: item[0])[1]
        latest = last_before(poses, t_end)
        if latest:
            robot = latest[1]
            bearing = math.atan2(victim.y - robot[1], victim.x - robot[0])
            row['bow_error_rad'] = fmt(abs(wrap(bearing - robot[2])), 4)

    # Tracking error: controller feedback while tracking.
    feedback = [(t, m.feedback.position_error)
                for t, m in messages.get('/follow_trajectory/_action/feedback', [])
                if t_start <= t <= t_end]
    if feedback:
        errors = [e for _, e in feedback]
        row['tracking_rms_m'] = fmt(math.sqrt(sum(e * e for e in errors) / len(errors)), 4)
        row['tracking_max_m'] = fmt(max(errors), 4)
        row['tracking_samples'] = fmt(len(errors))
    else:
        notes.append('no controller feedback recorded')

    # Saturation: zero-order hold of /thrust_saturated over the tracking window.
    if feedback:
        t_begin = feedback[0][0]
        flags = [(t, m.data) for t, m in messages.get('/thrust_saturated', [])
                 if t_begin <= t <= t_end]
        if flags:
            saturated = covered = 0.0
            for (t, flag), (t_next, _) in zip(flags, flags[1:] + [(t_end, None)]):
                covered += t_next - t
                saturated += (t_next - t) if flag else 0.0
            row['saturation_time_s'] = fmt(saturated, 3)
            row['saturation_window_s'] = fmt(covered, 3)
            row['saturation_fraction'] = fmt(saturated / covered if covered > 0 else None, 4)
        else:
            notes.append('no /thrust_saturated messages while tracking')

    # Clearances over the whole recording.
    wall = min_wall_distance(messages, poses, shape)
    if wall is None:
        notes.append('min_wall_distance_m: no /map recorded' if poses else
                     'min_wall_distance_m: no robot pose recorded')
    row['min_wall_distance_m'] = fmt(wall, 3)
    swimmer, swimmer_id = min_swimmer_distance(messages, poses, shape)
    if swimmer is None:
        notes.append('min_swimmer_distance_m: no swimmers in this run (/swimmer_truth empty)')
    else:
        row['min_swimmer_distance_m'] = fmt(swimmer, 3)
        notes.append(f'closest swimmer: track {swimmer_id}')

    # Mission phases and safety rules.
    phases = []
    for t, m in messages.get('/mission_state', []):
        name = PHASES[m.phase] if m.phase < len(PHASES) else str(m.phase)
        if not phases or phases[-1][1] != name:
            phases.append((t - t_start, name))
    data['phases'] = phases
    if phases:
        row['mission_phases'] = '>'.join(name for _, name in phases)
        names = [name for _, name in phases]
        if 'RETURN' in names:
            after = names.index('RETURN')
            for t, name in phases[after:]:
                if name == 'STANDBY':
                    row['mission_complete_s'] = fmt(t, 3)
                    break
    for t, m in messages.get('/safety_status', []):
        for rule in m.active_rules:
            data['safety_rules'].add(rule)
    row['safety_rules'] = ' '.join(sorted(data['safety_rules']))
    # Zero thrust: largest thruster force while zero_thrust has been in force
    # for at least 0.1 s (allocator latency).
    zero_since = None
    windows = []
    for t, m in messages.get('/safety_status', []):
        if m.zero_thrust and zero_since is None:
            zero_since = t
        elif not m.zero_thrust and zero_since is not None:
            windows.append((zero_since + 0.1, t))
            zero_since = None
    if zero_since is not None:
        windows.append((zero_since + 0.1, math.inf))
    if windows and messages.get('/thruster_command'):
        forces = [max(abs(f) for f in m.force) for t, m in messages['/thruster_command']
                  if any(a <= t <= b for a, b in windows)]
        if forces:
            row['zero_thrust_max_force_n'] = fmt(max(forces), 3)

    notes.append(UNSUPPORTED)
    row['notes'] = '; '.join(notes)
    return row, data


def number(row, column):
    """Float value of a CSV column, or None if empty."""
    text = row.get(column, '')
    return float(text) if text not in ('', None) else None


def check_expectations(expect, row, data):
    """List of (check, passed, detail) for a scenario's `expect` block."""
    results = []
    if not expect:
        return results
    status = expect.get('status')
    if status:
        results.append(('status', row['arrival_status'] == status,
                        f'{row["arrival_status"]} (expected {status})'))
    reason = expect.get('reason')
    if reason:
        ok = re.search(reason, row.get('final_report', '')) is not None
        results.append(('reason', ok, f'final report "{row.get("final_report", "")[:120]}" '
                                      f'vs /{reason}/'))
    for column, bounds in (expect.get('limits') or {}).items():
        value = number(row, column)
        if value is None:
            results.append((f'limit {column}', False, 'no value (missing data)'))
            continue
        if 'max' in bounds:
            results.append((f'{column} <= {bounds["max"]}', value <= bounds['max'],
                            f'{value:g}'))
        if 'min' in bounds:
            results.append((f'{column} >= {bounds["min"]}', value >= bounds['min'],
                            f'{value:g}'))
    reports = [r for _, r in data['reports']]
    for pattern in expect.get('reports_present') or []:
        hits = sum(1 for r in reports if re.search(pattern, r))
        results.append((f'report /{pattern}/ present', hits > 0, f'{hits} matching'))
    for pattern in expect.get('reports_absent') or []:
        hits = [r for r in reports if re.search(pattern, r)]
        results.append((f'report /{pattern}/ absent', not hits,
                        hits[0][:120] if hits else 'none'))
    phases = expect.get('phases')
    if phases:
        seen = [name for _, name in data['phases']]
        k = 0
        for name in seen:
            if k < len(phases) and name == phases[k]:
                k += 1
        results.append(('mission phases', k == len(phases),
                        f'seen {">".join(seen) or "none"}; expected in order '
                        f'{">".join(phases)}'))
    for rule in expect.get('safety_rules') or []:
        results.append((f'safety rule {rule}', rule in data['safety_rules'],
                        ' '.join(sorted(data['safety_rules'])) or 'none'))
    for rule in expect.get('safety_rules_absent') or []:
        results.append((f'safety rule {rule} absent', rule not in data['safety_rules'],
                        ' '.join(sorted(data['safety_rules'])) or 'none'))
    for topic, minimum in (expect.get('required_topics') or {}).items():
        got = data['topic_counts'].get(topic, 0)
        results.append((f'topic {topic} >= {minimum} messages', got >= minimum, f'{got}'))
    return results


REPLACED_KEYS = ('end', 'required_topics')  # keys an override replaces as a whole


def merge(base, override):
    """Deep merge of scenario dicts: non-empty dicts merge, anything else replaces."""
    result = dict(base)
    for key, value in override.items():
        if isinstance(value, dict) and value and isinstance(result.get(key), dict) and \
                key not in REPLACED_KEYS:
            result[key] = merge(result[key], value)
        else:
            result[key] = value
    return result


def load_scenario(path):
    """Scenario dict from a YAML file, with its `base` (same directory) merged underneath."""
    with open(path) as stream:
        scenario = yaml.safe_load(stream)
    base = scenario.pop('base', None)
    if base:
        scenario = merge(load_scenario(os.path.join(os.path.dirname(path), base + '.yaml')),
                         scenario)
    return scenario


def write_csv(path, rows):
    """Write rows (dicts) with the standard header; '-' writes to stdout."""
    out = sys.stdout if path == '-' else open(path, 'w', newline='')
    try:
        writer = csv.DictWriter(out, fieldnames=COLUMNS, extrasaction='ignore')
        writer.writeheader()
        for row in rows:
            writer.writerow(row)
    finally:
        if path != '-':
            out.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument('bag', help='rosbag directory')
    parser.add_argument('--scenario', default='', help='scenario name for the row')
    parser.add_argument('--backend', default='', help='backend name for the row')
    parser.add_argument('--output', default='-', help='CSV file (default: stdout)')
    parser.add_argument('--expect', default='', help='scenario YAML: check its expectations')
    args = parser.parse_args()
    row, data = compute(read_bag(args.bag), args.scenario, args.bag, args.backend)
    write_csv(args.output, [row])
    if not args.expect:
        return 0
    expect = load_scenario(args.expect).get('expect') or {}
    results = check_expectations(expect, row, data)
    for check, passed, detail in results:
        print(f'{"PASS" if passed else "FAIL"}  {check}: {detail}', file=sys.stderr)
    return 0 if results and all(passed for _, passed, _ in results) else 1


if __name__ == '__main__':
    sys.exit(main())
