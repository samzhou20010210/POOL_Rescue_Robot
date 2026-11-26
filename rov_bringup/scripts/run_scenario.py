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
Run surface-approach regression scenarios, record them, and check the results.

Usage:
  ros2 run rov_bringup run_scenario.py SCENARIO [SCENARIO ...]
      [--matrix FILE_OR_NAME] [--backend dynamics|gazebo|all]
      [--output-dir DIR] [--domain-id N]

SCENARIO is a name in rov_bringup/config/scenarios/ or a YAML path; --matrix
reads a list of scenarios and backends (e.g. --matrix matrix). For every
(scenario, backend) run, each step with a bounded timeout from the scenario:
  1. start the launch file in its own process group (log: launch.log);
  2. wait until planner, controller and trajectory generator are up;
  3. start "ros2 bag record" in its own process group, wait until it has
     subscribed;
  4. run the scripted events (targets, alarms, parameter changes, ...), each
     at a time, after a planner report, or on a mission phase;
  5. wait for the end condition (a report, a mission phase, or a time);
  6. stop the recorder, then the launch (SIGINT, then SIGTERM, then SIGKILL
     per process group) and check that no started process is left;
  7. compute the metrics row (bag_to_metrics.py) and check the scenario's
     expectations against it: status and reason, metric limits, required /
     forbidden reports, mission phases, safety rules, required topics.
Results: DIR/<name>_<backend>_<time>/{bag, metrics.csv, checks.txt, *.log}
and, for several runs, DIR/summary_<time>.csv.
Exit code: 0 if every run passed, 1 if an expectation failed, 2 for an
infrastructure failure (startup or end-condition timeout, no bag, required
data missing, or processes that did not stop).
"""

import argparse
import csv
import datetime
import importlib.util
import math
import os
import re
import signal
import subprocess
import sys
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_msgs.action import ComputePathToPose
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
import rclpy
from rclpy.action import ActionClient
import rclpy.duration
import rclpy.parameter
import rclpy.time
from rov_interfaces.action import FollowTrajectory
from rov_interfaces.msg import MissionEvent, MissionState, TrajectoryPoint, VictimTarget
from std_msgs.msg import String
import tf2_ros
import yaml

PHASES = ['STANDBY', 'TRANSIT', 'FINAL_APPROACH', 'DIVE_AND_GRASP', 'SURFACE_HOLD', 'RETURN',
          'SAFETY_STOP']
# Topics every run must contain (minimum message counts); scenarios add more.
REQUIRED_TOPICS = {'/odom': 50, '/tf': 50, '/tf_static': 1, '/map': 1}


def log(text):
    """Print one progress line with the wall time."""
    print(f'[run_scenario {datetime.datetime.now():%H:%M:%S}] {text}', flush=True)


def reference_trajectory(shape, speed, start, dt=0.02, accel=1.5):
    """
    Return the points of a step 5 test reference (map frame) as tuples.

    shape: 'line' (20 m straight) or 'l' (8 m, quarter circle of radius 2 m
    to the left, 8 m). Heading along the path; speed profile: smooth ramps
    (3x^2 - 2x^3) to `speed` (m/s) with a peak acceleration of `accel`.
    Returns [(t, x, y, vx, vy, ax, ay, yaw, yaw_rate)].
    """
    x0, y0, yaw0 = start
    radius = 2.0
    if shape == 'line':
        segments = [('line', 20.0)]
    elif shape == 'l':
        segments = [('line', 8.0), ('arc', math.pi / 2 * radius), ('line', 8.0)]
    else:
        raise ValueError(f'unknown reference shape {shape}')
    length = sum(s for _, s in segments)

    def geometry(s):
        x, y, heading = x0, y0, yaw0
        for kind, seg in segments:
            d = min(max(s, 0.0), seg)
            if kind == 'line':
                x += d * math.cos(heading)
                y += d * math.sin(heading)
            else:
                a = d / radius
                cx, cy = x - radius * math.sin(heading), y + radius * math.cos(heading)
                x = cx + radius * math.sin(heading + a)
                y = cy - radius * math.cos(heading + a)
                heading += a
            s -= seg
            if s <= 0.0:
                break
        return x, y, heading

    ramp = 1.5 * speed / accel  # smoothstep peak slope is 1.5 / ramp

    def smooth(v):
        v = min(max(v, 0.0), 1.0)
        return v * v * (3.0 - 2.0 * v)

    total = 2.0 * ramp + (length - speed * ramp) / speed
    raw = []
    s = 0.0
    t = 0.0
    while t <= total + 1e-9:
        x, y, heading = geometry(min(s, length))
        raw.append((t, x, y, heading))
        s += speed * min(smooth((t + 0.5 * dt) / ramp), smooth((total - t - 0.5 * dt) / ramp)) * dt
        t += dt
    points = []
    for k, (t, x, y, heading) in enumerate(raw):
        a, b = max(k - 1, 0), min(k + 1, len(raw) - 1)
        span = raw[b][0] - raw[a][0]
        vx, vy = (raw[b][1] - raw[a][1]) / span, (raw[b][2] - raw[a][2]) / span
        points.append([t, x, y, vx, vy, 0.0, 0.0, heading, (raw[b][3] - raw[a][3]) / span])
    for k in range(len(points)):
        a, b = max(k - 1, 0), min(k + 1, len(points) - 1)
        span = points[b][0] - points[a][0]
        points[k][5] = (points[b][3] - points[a][3]) / span
        points[k][6] = (points[b][4] - points[a][4]) / span
    points[-1][3:7] = [0.0, 0.0, 0.0, 0.0]
    points[-1][8] = 0.0
    return points


def scenario_path(name):
    """Path of a scenario name in config/scenarios/, or the name itself if a file."""
    if os.path.isfile(name):
        return name
    return os.path.join(get_package_share_directory('rov_bringup'), 'config', 'scenarios',
                        f'{name}.yaml')


def group_members(pgid):
    """Return the PIDs of the live processes in a process group."""
    members = []
    for entry in os.listdir('/proc'):
        if not entry.isdigit():
            continue
        try:
            with open(f'/proc/{entry}/stat') as stream:
                fields = stream.read().rsplit(')', 1)[1].split()
        except OSError:
            continue
        if fields[0] != 'Z' and int(fields[2]) == pgid:
            members.append(int(entry))
    return members


def stop_group(process, name, grace):
    """Stop a process group: SIGINT, then SIGTERM, then SIGKILL; True if none is left."""
    pgid = process.pid
    steps = []
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        if not group_members(pgid):
            break
        steps.append(sig.name)
        try:
            os.killpg(pgid, sig)
        except ProcessLookupError:
            break
        deadline = time.monotonic() + grace
        while group_members(pgid) and time.monotonic() < deadline:
            time.sleep(0.1)
    try:
        process.wait(timeout=1.0)
    except subprocess.TimeoutExpired:
        pass
    left = group_members(pgid)
    log(f'stopped {name} (group {pgid}) with {" -> ".join(steps) or "nothing needed"}; '
        f'processes left: {len(left)}')
    return not left


def load_metrics_module():
    """Import bag_to_metrics.py from this script's directory."""
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'bag_to_metrics.py')
    spec = importlib.util.spec_from_file_location('bag_to_metrics', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Driver:
    """ROS side of one run: readiness, events, reports and mission phases."""

    def __init__(self, backend='dynamics'):
        self.backend = backend
        # Gazebo nodes run on simulation time: stamp the scripted events with
        # the same clock, or they look stale (or from the future) to them.
        self.node = rclpy.create_node('scenario_runner', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=(backend == 'gazebo'))])
        self.reports = []
        self.phases = []
        self.planner = ActionClient(self.node, ComputePathToPose, '/compute_path_to_pose')
        self.follow = ActionClient(self.node, FollowTrajectory, '/follow_trajectory')
        self.publishers = {
            'victim_target': self.node.create_publisher(PointStamped, '/victim_target', 10),
            'shore_alarm': self.node.create_publisher(VictimTarget, '/shore_alarm', 10),
            'victim': self.node.create_publisher(VictimTarget, '/shore_victim', 10),
            'goal_pose': self.node.create_publisher(PoseStamped, '/goal_pose', 10),
            'mission_event': self.node.create_publisher(MissionEvent, '/mission_event', 10),
        }
        for topic in ('/planner_report', '/safety_report', '/mission_report'):
            self.node.create_subscription(
                String, topic, lambda m: self.reports.append((time.monotonic(), m.data)), 50)
        self.node.create_subscription(MissionState, '/mission_state', self.on_phase, 50)
        self.parameter_clients = {}
        self.scenario_report = self.node.create_publisher(String, '/scenario_report', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)
        self.pending_test = None  # (result future, description, final reference pose)

    def on_phase(self, message):
        name = PHASES[message.phase] if message.phase < len(PHASES) else str(message.phase)
        if not self.phases or self.phases[-1][1] != name:
            self.phases.append((time.monotonic(), name))
            log(f'mission phase {name}: {message.last_transition}')

    def spin(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def wait_until(self, condition, timeout, what):
        deadline = time.monotonic() + timeout
        while not condition():
            if time.monotonic() > deadline:
                raise TimeoutError(f'{what} not ready within {timeout:.0f} s')
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def ready(self, need_mission):
        target = self.publishers['victim_target'].get_subscription_count() >= 1
        mission = self.publishers['shore_alarm'].get_subscription_count() >= 1
        # The vehicle must exist too (Gazebo spawns it after the servers start).
        pose = self.tf_buffer.can_transform('map', 'base_link', rclpy.time.Time())
        # On simulation time the clock must have started before events are stamped.
        clock = self.node.get_clock().now().nanoseconds > 0
        return (clock and self.planner.server_is_ready() and self.follow.server_is_ready() and
                target and pose and (mission or not need_mission))

    def recorder_subscribed(self, topics):
        """Return True once the recorder subscribes to every published topic it records."""
        # A short run (e.g. an immediate rejection) could otherwise end before
        # the recorder has discovered the map or the planner report.
        for topic in topics:
            if not self.node.get_publishers_info_by_topic(topic):
                continue  # not published (yet): nothing to wait for
            if not any(info.node_name.startswith('rosbag2_recorder')
                       for info in self.node.get_subscriptions_info_by_topic(topic)):
                return False
        return True

    def set_parameter(self, node_name, name, value):
        """Set one parameter on a node through its set_parameters service."""
        service = f'/{node_name}/set_parameters'
        client = self.parameter_clients.get(service)
        if client is None:
            client = self.node.create_client(SetParameters, service)
            self.parameter_clients[service] = client
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f'{service} not available')
        parameter = Parameter(name=name)
        if isinstance(value, bool):
            parameter.value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=value)
        elif isinstance(value, int):
            parameter.value = ParameterValue(type=ParameterType.PARAMETER_INTEGER,
                                             integer_value=value)
        elif isinstance(value, float):
            parameter.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                             double_value=value)
        else:
            parameter.value = ParameterValue(type=ParameterType.PARAMETER_STRING,
                                             string_value=str(value))
        future = client.call_async(SetParameters.Request(parameters=[parameter]))
        deadline = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
        if not future.done() or not future.result().results[0].successful:
            raise RuntimeError(f'could not set {node_name}.{name} = {value}')

    def act(self, action):
        """Perform one scripted action (dict with exactly one key)."""
        kind, args = next(iter(action.items()))
        args = args or {}
        # age_s: stamp the message that many seconds in the past (stale events).
        # On simulation time the clock may not have run that long yet: wait.
        age = float(args.get('age_s', 0.0))
        deadline = time.monotonic() + age + 30.0
        while self.node.get_clock().now().nanoseconds < (age + 0.5) * 1e9:
            if time.monotonic() > deadline:
                raise RuntimeError(f'clock did not reach {age + 0.5:.1f} s for an age_s stamp')
            rclpy.spin_once(self.node, timeout_sec=0.02)
        now = self.node.get_clock().now()
        stamp = (now - rclpy.duration.Duration(seconds=float(args.get('age_s', 0.0)))).to_msg()
        if kind == 'victim_target':
            m = PointStamped()
            m.header.frame_id = args.get('frame_id', 'map')
            m.header.stamp = stamp
            m.point.x, m.point.y = float(args['x']), float(args['y'])
            self.publishers['victim_target'].publish(m)
        elif kind in ('shore_alarm', 'victim'):
            m = VictimTarget()
            m.header.frame_id = args.get('frame_id', 'map')
            m.header.stamp = stamp
            m.position.x, m.position.y = float(args['x']), float(args['y'])
            m.victim_state = VictimTarget.SURFACE if args.get('state') == 'surface' else \
                VictimTarget.SUBMERGED
            self.publishers[kind].publish(m)
        elif kind == 'goal_pose':
            m = PoseStamped()
            m.header.frame_id = 'map'
            m.header.stamp = stamp
            m.pose.position.x, m.pose.position.y = float(args['x']), float(args['y'])
            yaw = float(args.get('yaw', 0.0))
            m.pose.orientation.z, m.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            self.publishers['goal_pose'].publish(m)
        elif kind == 'mission_event':
            m = MissionEvent()
            m.header.stamp = stamp
            m.type = {'rescue_complete': MissionEvent.RESCUE_COMPLETE,
                      'manual_takeover': MissionEvent.MANUAL_TAKEOVER}[args['type']]
            m.detail = args.get('detail', 'simulated by the scenario runner')
            self.publishers['mission_event'].publish(m)
        elif kind == 'set_param':
            # node: a name, or {backend: name} when it differs per backend.
            node = args['node']
            if isinstance(node, dict):
                node = node[self.backend]
            self.set_parameter(node, args['name'], args['value'])
        elif kind == 'controller_trajectory':
            # Step 5 test: a reference sent straight to the controller.
            start = (float(args['x']), float(args['y']), float(args.get('yaw', 0.0)))
            points = reference_trajectory(args['shape'], float(args.get('speed', 1.5)), start)
            goal = FollowTrajectory.Goal()
            goal.trajectory.header.frame_id = 'map'
            goal.trajectory.header.stamp = stamp
            for t, x, y, vx, vy, ax, ay, yaw, rate in points:
                p = TrajectoryPoint()
                p.time_from_start = rclpy.duration.Duration(seconds=t).to_msg()
                p.x, p.y, p.vx, p.vy, p.ax, p.ay, p.yaw, p.yaw_rate = \
                    x, y, vx, vy, ax, ay, yaw, rate
                goal.trajectory.points.append(p)
            future = self.follow.send_goal_async(goal)
            deadline = time.monotonic() + 5.0
            while not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(self.node, timeout_sec=0.02)
            if not future.done() or not future.result().accepted:
                raise RuntimeError('the controller did not accept the test trajectory')
            last = points[-1]
            self.pending_test = (future.result().get_result_async(),
                                 f'{args["shape"]} at {args.get("speed", 1.5)} m/s',
                                 (last[1], last[2], last[7]))
            log(f'test trajectory sent: {len(points)} points, {points[-1][0]:.1f} s')
        elif kind == 'invalid_trajectory':
            # Times not increasing: the controller must REJECT it and keep the
            # running goal; an accepted invalid goal fails the run.
            goal = FollowTrajectory.Goal()
            goal.trajectory.header.frame_id = 'map'
            goal.trajectory.header.stamp = stamp
            goal.trajectory.points = [TrajectoryPoint(), TrajectoryPoint()]
            future = self.follow.send_goal_async(goal)
            deadline = time.monotonic() + 5.0
            while not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(self.node, timeout_sec=0.02)
            if not future.done():
                raise RuntimeError('no response to the invalid trajectory')
            if future.result().accepted:
                raise RuntimeError('the controller ACCEPTED an invalid trajectory')
            log('invalid trajectory rejected by the controller (as required)')
        else:
            raise ValueError(f'unknown action {kind}')
        log(f'action {kind} {args}')

    def poll_test(self):
        """Publish the outcome of a finished controller test trajectory."""
        if not self.pending_test or not self.pending_test[0].done():
            return
        future, what, pose = self.pending_test
        self.pending_test = None
        result = future.result().result
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            q = tf.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            reached = f'({tf.transform.translation.x:.3f}, {tf.transform.translation.y:.3f}, ' \
                f'{yaw:.3f} rad)'
        except tf2_ros.TransformException:
            reached = '(unknown)'
        text = (f'{"ACCEPTED" if result.success else "NOT ACCEPTED"}: controller test {what}: '
                f'{result.message}; reached {reached}, error {result.final_position_error:.3f} m '
                f'/ {abs(result.final_yaw_error):.3f} rad; terminal pose ({pose[0]:.3f}, '
                f'{pose[1]:.3f}, {pose[2]:.3f} rad)')
        message = String()
        message.data = text
        self.scenario_report.publish(message)
        self.reports.append((time.monotonic(), text))
        log(text)

    def report_seen(self, pattern, since):
        return any(t >= since and re.search(pattern, r) for t, r in self.reports)

    def phase_seen(self, name, after=None, since=0.0):
        names = [(t, n) for t, n in self.phases if t >= since]
        if after:
            idx = next((i for i, (_, n) in enumerate(names) if n == after), None)
            if idx is None:
                return False
            names = names[idx:]
        return any(n == name for _, n in names)


def run_events(driver, events, t0, end, run_s):
    """Run scripted events until the end condition; True if it was reached."""
    pending = [dict(e) for e in events or []]
    for event in pending:
        event['_armed_at'] = None
    deadline = t0 + run_s
    while time.monotonic() < deadline:
        now = time.monotonic()
        for event in pending:
            if event.get('_done'):
                continue
            if event['_armed_at'] is None:
                if 'at_s' in event and now - t0 >= float(event['at_s']):
                    event['_armed_at'] = now
                elif 'after_report' in event and driver.report_seen(event['after_report'], t0):
                    event['_armed_at'] = now
                elif 'on_phase' in event and driver.phase_seen(event['on_phase'], since=t0):
                    event['_armed_at'] = now
            if event['_armed_at'] is not None and \
                    now - event['_armed_at'] >= float(event.get('delay_s', 0.0)):
                event['_done'] = True
                for action in event.get('actions', []):
                    driver.act(action)
        driver.poll_test()
        if end_reached(driver, end, t0):
            return True
        rclpy.spin_once(driver.node, timeout_sec=0.02)
    return end_reached(driver, end, t0)


def end_reached(driver, end, t0):
    """Return True once the scenario's end condition has happened."""
    if 'time_s' in end:
        return time.monotonic() - t0 >= float(end['time_s'])
    if 'report' in end:
        return driver.report_seen(end['report'], t0)
    if 'phase' in end:
        return driver.phase_seen(end['phase'], after=end.get('after_phase'), since=t0)
    return False


def run_one(scenario, backend, out_dir, metrics):
    """One (scenario, backend) run. Returns (row, checks, infrastructure_errors)."""
    name = scenario['name']
    timeouts = scenario['timeouts']
    run_dir = os.path.join(out_dir, f'{name}_{backend}_{datetime.datetime.now():%Y%m%d_%H%M%S}')
    os.makedirs(run_dir)
    bag_dir = os.path.join(run_dir, 'bag')
    log(f'=== {name} on {backend} -> {run_dir}')
    errors = []
    notes = []
    processes = []
    arguments = dict(scenario['launch'].get('arguments', {}))
    arguments['backend'] = backend
    for key, value in (scenario['launch'].get('backend_arguments', {}).get(backend) or {}).items():
        arguments[key] = value
    command = ['ros2', 'launch', scenario['launch']['package'], scenario['launch']['file']] + [
        f'{k}:={v}' for k, v in arguments.items()]
    driver = Driver(backend)
    reached = False
    try:
        log('launch: ' + ' '.join(command))
        with open(os.path.join(run_dir, 'launch.log'), 'w') as launch_log:
            processes.append(('launch', subprocess.Popen(
                command, stdout=launch_log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                start_new_session=True)))
        need_mission = bool(scenario.get('mission', False))
        try:
            driver.wait_until(lambda: driver.ready(need_mission), timeouts['startup_s'],
                              'planner, controller and trajectory generator')
        except TimeoutError as error:
            # One clean retry: stop everything and launch again (a lifecycle
            # bring-up can hang on a lost DDS service call). Logged as a note.
            log(f'startup failed ({error}); stopping and launching again (retry 1 of 1)')
            notes.append(f'startup retried once after: {error}')
            for proc_name, process in reversed(processes):
                if not stop_group(process, proc_name, timeouts['shutdown_s']):
                    errors.append(f'{proc_name}: processes left after SIGKILL (cleanup failure)')
            processes.clear()
            with open(os.path.join(run_dir, 'launch_retry.log'), 'w') as launch_log:
                processes.append(('launch', subprocess.Popen(
                    command, stdout=launch_log, stderr=subprocess.STDOUT,
                    stdin=subprocess.DEVNULL, start_new_session=True)))
            driver.wait_until(lambda: driver.ready(need_mission), timeouts['startup_s'],
                              'planner, controller and trajectory generator (retry)')
        log('simulation ready')
        topics = list(scenario['record_topics'])
        with open(os.path.join(run_dir, 'record.log'), 'w') as record_log:
            processes.append(('recorder', subprocess.Popen(
                ['ros2', 'bag', 'record', '--include-hidden-topics', '-o', bag_dir,
                 '--topics'] + topics,
                stdout=record_log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                start_new_session=True)))
        # The recorder is ready once it also subscribes to a recorded trigger
        # topic (the planner or the mission manager is the other subscriber).
        probe = 'shore_alarm' if '/shore_alarm' in topics else 'victim_target'
        driver.wait_until(
            lambda: driver.publishers[probe].get_subscription_count() >= 2 and
            driver.recorder_subscribed(topics),
            timeouts['recorder_s'], 'rosbag recorder')
        driver.spin(timeouts['settle_s'])
        t0 = time.monotonic()
        reached = run_events(driver, scenario.get('events'), t0, scenario['end'],
                             timeouts['run_s'])
        if not reached:
            errors.append(f'end condition {scenario["end"]} not reached within '
                          f'{timeouts["run_s"]} s (unexpected timeout)')
            log('TIMEOUT: ' + errors[-1])
        driver.spin(timeouts['tail_s'])
    except (TimeoutError, RuntimeError, ValueError) as error:
        errors.append(str(error))
        log(f'ERROR: {error}')
    finally:
        for proc_name, process in reversed(processes):
            if not stop_group(process, proc_name, timeouts['shutdown_s']):
                errors.append(f'{proc_name}: processes left after SIGKILL (cleanup failure)')
        driver.node.destroy_node()
    with open(os.path.join(run_dir, 'reports.txt'), 'w') as stream:
        for _, report in driver.reports:
            stream.write(report + '\n')

    row = {'scenario': name, 'backend': backend, 'bag': bag_dir}
    checks = []
    if not os.path.isfile(os.path.join(bag_dir, 'metadata.yaml')):
        errors.append('no finalized bag')
    else:
        row, data = metrics.compute(metrics.read_bag(bag_dir), name, bag_dir, backend)
        metrics.write_csv(os.path.join(run_dir, 'metrics.csv'), [row])
        expect = dict(scenario.get('expect') or {})
        required = dict(REQUIRED_TOPICS)
        required.update(expect.get('required_topics') or {})
        expect['required_topics'] = required
        checks = metrics.check_expectations(expect, row, data)
        missing = [c for c, ok, _ in checks if not ok and c.startswith('topic ')]
        if missing:
            errors.append('required data missing: ' + ', '.join(missing))
    with open(os.path.join(run_dir, 'checks.txt'), 'w') as stream:
        for check, passed, detail in checks:
            stream.write(f'{"PASS" if passed else "FAIL"}  {check}: {detail}\n')
        for error in errors:
            stream.write(f'ERROR  {error}\n')
        for note in notes:
            stream.write(f'NOTE  {note}\n')
    for check, passed, detail in checks:
        log(f'  {"PASS" if passed else "FAIL"}  {check}: {detail}')
    for error in errors:
        log(f'  ERROR {error}')
    return row, checks, errors


def main():
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument('scenarios', nargs='*', help='scenario names or YAML paths')
    parser.add_argument('--matrix', default='', help='matrix file or name (config/scenarios/)')
    parser.add_argument('--backend', default='', help='dynamics, gazebo or all (overrides)')
    parser.add_argument('--output-dir', default='scenario_runs', help='parent directory')
    parser.add_argument('--domain-id', type=int, default=None, help='ROS_DOMAIN_ID for the run')
    args = parser.parse_args()
    if args.domain_id is not None:
        os.environ['ROS_DOMAIN_ID'] = str(args.domain_id)  # inherited by the children

    runs = []  # (scenario dict, backend)
    names = list(args.scenarios)
    matrix_backends = None
    if args.matrix:
        with open(scenario_path(args.matrix)) as stream:
            matrix = yaml.safe_load(stream)
        names += matrix['scenarios']
        matrix_backends = matrix.get('backends')
    if not names:
        parser.error('no scenario given')
    metrics = load_metrics_module()
    for name in names:
        scenario = metrics.load_scenario(scenario_path(name))
        backends = scenario.get('backends', ['dynamics'])
        if matrix_backends:
            backends = [b for b in matrix_backends if b in scenario.get('backends', backends)]
        if args.backend == 'all':
            backends = scenario.get('backends', ['dynamics', 'gazebo'])
        elif args.backend:
            backends = [args.backend]
        for backend in backends:
            runs.append((scenario, backend))

    out_dir = os.path.abspath(args.output_dir)
    os.makedirs(out_dir, exist_ok=True)
    rclpy.init()
    results = []
    try:
        for scenario, backend in runs:
            row, checks, errors = run_one(scenario, backend, out_dir, metrics)
            passed = not errors and checks and all(ok for _, ok, _ in checks)
            results.append((scenario['name'], backend, row, checks, errors, passed))
    finally:
        rclpy.shutdown()

    summary = os.path.join(out_dir, f'summary_{datetime.datetime.now():%Y%m%d_%H%M%S}.csv')
    with open(summary, 'w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['scenario', 'backend', 'result', 'failed_checks', 'errors'] +
                        metrics.COLUMNS)
        for name, backend, row, checks, errors, passed in results:
            failed = '; '.join(f'{c} ({d})' for c, ok, d in checks if not ok)
            writer.writerow([name, backend, 'PASS' if passed else 'FAIL', failed,
                             '; '.join(errors)] + [row.get(c, '') for c in metrics.COLUMNS])
    log(f'summary: {summary}')
    for name, backend, row, checks, errors, passed in results:
        log(f'{"PASS" if passed else "FAIL"}  {name} [{backend}]' +
            (f' - {"; ".join(errors)}' if errors else ''))
    if any(errors for _, _, _, _, errors, _ in results):
        return 2
    return 0 if all(passed for *_, passed in results) else 1


if __name__ == '__main__':
    sys.exit(main())
