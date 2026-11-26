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
Open-loop thrust tests for the thrust-driven simulators (implementation steps 3 and 4).

The script uses only ROS interfaces, so it runs unchanged against any backend
that consumes thruster_command (rov_interfaces/ThrusterCommand), publishes odom,
and offers the reset_vehicle service (std_srvs/Trigger):
  ros2 launch rov_bringup thrust_sim.launch.py backend:=dynamics   (or gazebo)

Every run first calls reset_vehicle and waits until the vehicle is at rest at
the start pose, so all runs start from the same initial conditions.

Runs:
  surge / sway / yaw  Hold a pure planar wrench (thrusters 01-04 only) at a
                      percentage of f_peak on the most loaded thruster, then
                      average the steady speed and compare it with the value
                      predicted from the YAML damping parameters.
  coast_down          Hold the thrust whose steady surge speed is 1.5 m/s,
                      send a zero command, and measure the distance and time
                      until the speed is below the stop threshold.
  command_loss        Hold f_plan, then stop publishing. The backend must cut
                      the thrust after its command timeout, so the vehicle
                      must slow down and stop.

A run fails if odometry goes stale, the reset does not bring the vehicle to
rest at the start pose, or the vehicle never drops below the stop threshold.

Results: <output-dir>/open_loop_speed.csv and <output-dir>/open_loop_coast.csv.
Exit code: 0 all runs passed and the f_plan surge speed is within the
tolerance, 1 only the f_plan speed check failed, 2 a run failed.
"""

import argparse
import csv
import math
import os
import sys
import time

from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from rov_interfaces.msg import ThrusterCommand
from std_srvs.srv import Trigger
import yaml

NUM_THRUSTERS = 8
HORIZONTAL = 4  # thrusters 01-04 are the horizontal ones (index 0-3)


class RunFailed(Exception):
    """A run could not be completed; the message is the CSV status."""


def steady_value(force, linear, quadratic):
    """Return s > 0 where force = a*s + b*s^2, with a = -linear, b = -quadratic."""
    a = -linear
    b = -quadratic
    sign = 1.0 if force >= 0.0 else -1.0
    force = abs(force)
    return sign * (-a + math.sqrt(a * a + 4.0 * b * force)) / (2.0 * b)


def coast_distance(effective_mass, linear, quadratic, start_speed, stop_speed):
    """Return the distance to slow from start_speed to stop_speed by drag alone."""
    a = -linear
    b = -quadratic
    return effective_mass / b * math.log((a + b * start_speed) / (a + b * stop_speed))


def yaw_of(odom):
    q = odom.pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def angle_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


class Vehicle:
    """Thruster geometry and hydrodynamic parameters loaded from the YAML files."""

    def __init__(self, thrusters_yaml, hydrodynamics_yaml):
        with open(thrusters_yaml, 'r') as stream:
            thrusters = yaml.safe_load(stream)
        with open(hydrodynamics_yaml, 'r') as stream:
            self.hydro = yaml.safe_load(stream)
        self.f_peak = float(thrusters['dynamics']['peak_forward_force'])
        self.f_plan = float(thrusters['dynamics']['planning_force_limit'])
        # Planar rows (Fx, Fy, Mz) of the allocation matrix for thrusters 01-04.
        columns = []
        for index in range(HORIZONTAL):
            entry = thrusters['thrusters']['thruster_{:02d}'.format(index + 1)]
            position = np.array(entry['position'], dtype=float)
            direction = np.array(entry['direction'], dtype=float)
            if abs(direction[2]) > 1e-9:
                raise ValueError('thruster_{:02d} is not horizontal'.format(index + 1))
            moment = np.cross(position, direction)
            columns.append([direction[0], direction[1], moment[2]])
        self.planar = np.array(columns).T  # 3 x 4

    def forces_for(self, wrench):
        """Return the minimum-norm forces (N) on thrusters 01-04 for [Fx, Fy, Mz]."""
        p = self.planar
        return p.T @ np.linalg.solve(p @ p.T, np.array(wrench, dtype=float))

    def wrench_of(self, forces):
        return self.planar @ np.array(forces[:HORIZONTAL], dtype=float)

    def scaled_forces(self, unit_wrench, percent):
        """Return forces for a pure wrench, the most loaded thruster at percent of f_peak."""
        unit = self.forces_for(unit_wrench)
        return unit * (percent / 100.0 * self.f_peak / np.max(np.abs(unit)))

    def damping(self, axis):
        names = {'surge': ('x_u', 'x_uu'), 'sway': ('y_v', 'y_vv'), 'yaw': ('n_r', 'n_rr')}
        linear, quadratic = names[axis]
        return (float(self.hydro['linear_damping'][linear]),
                float(self.hydro['quadratic_damping'][quadratic]))

    def effective_surge_mass(self):
        return float(self.hydro['mass']) - float(self.hydro['added_mass']['x_udot'])


class OpenLoopTest(Node):
    """Publishes thruster commands at a fixed rate and watches odometry."""

    def __init__(self, args):
        super().__init__('open_loop_thrust_test')
        self.args = args
        self.force = [0.0] * NUM_THRUSTERS
        self.publishing = True
        self.odom = None
        self.odom_count = 0
        self.last_odom_wall = None
        self.last_publish_time = None
        self.publisher = self.create_publisher(ThrusterCommand, 'thruster_command', 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.reset_client = self.create_client(Trigger, 'reset_vehicle')
        self.create_timer(1.0 / args.command_rate, self.on_timer)

    def on_odom(self, message):
        self.odom = message
        self.odom_count += 1
        self.last_odom_wall = time.monotonic()

    def on_timer(self):
        if self.publishing:
            self.publish_command()

    def publish_command(self):
        command = ThrusterCommand()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = 'base_link'
        command.force = [float(f) for f in self.force]
        self.publisher.publish(command)
        self.last_publish_time = self.now()

    def set_forces(self, horizontal_forces):
        self.force = [float(f) for f in horizontal_forces] + [0.0] * (NUM_THRUSTERS - HORIZONTAL)

    # Time on the node's clock (simulation time if use_sim_time is true).
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def spin_once(self):
        rclpy.spin_once(self, timeout_sec=0.01)
        if self.last_odom_wall is not None and \
                time.monotonic() - self.last_odom_wall > self.args.stale_timeout:
            raise RunFailed('stale_odometry')

    def spin_for(self, seconds):
        end = self.now() + seconds
        while self.now() < end:
            self.spin_once()

    def spin_until(self, condition, timeout, failure):
        end = self.now() + timeout
        while self.now() < end:
            self.spin_once()
            if condition():
                return
        raise RunFailed(failure)

    # Odometry readings (twist is in base_link).
    def body_velocity(self):
        t = self.odom.twist.twist
        return t.linear.x, t.linear.y, t.angular.z

    def speed(self):
        u, v, _ = self.body_velocity()
        return math.hypot(u, v)

    def pose(self):
        p = self.odom.pose.pose.position
        return p.x, p.y, yaw_of(self.odom)

    def at_rest_at_start(self):
        x, y, yaw = self.pose()
        sx, sy, syaw = self.args.start_pose
        return (self.speed() < self.args.stop_speed and
                abs(self.body_velocity()[2]) < self.args.stop_yaw_rate and
                math.hypot(x - sx, y - sy) < 0.02 and abs(angle_diff(yaw, syaw)) < 0.01)

    def wait_for_fresh_odometry(self, timeout):
        """Wait (wall time) until odometry is arriving, e.g. after an earlier outage."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.last_odom_wall is not None and \
                    time.monotonic() - self.last_odom_wall < self.args.stale_timeout:
                return
        raise RunFailed('stale_odometry')

    def reset(self):
        """Reset the vehicle and wait until it is at rest at the start pose."""
        self.wait_for_fresh_odometry(5.0)
        self.publishing = True
        self.set_forces([0.0] * HORIZONTAL)
        if not self.reset_client.wait_for_service(timeout_sec=5.0):
            raise RunFailed('no_reset_service')
        future = self.reset_client.call_async(Trigger.Request())
        self.spin_until(future.done, 5.0, 'reset_call_timeout')
        if not future.result().success:
            raise RunFailed('reset_refused')
        self.spin_for(0.3)  # let stale pre-reset odometry flush out
        try:
            self.spin_until(self.at_rest_at_start, 10.0, 'not_at_rest_after_reset')
        except RunFailed:
            x, y, yaw = self.pose()
            u, v, r = self.body_velocity()
            self.get_logger().error(
                'Not at rest at the start pose after reset: pose ({:.4f}, {:.4f}, {:.4f}), '
                'velocity ({:.4f}, {:.4f}, {:.4f})'.format(x, y, yaw, u, v, r))
            raise

    def average_body_velocity(self, seconds):
        """Average u, v, r over new odometry messages; also return the max heading change."""
        samples = []
        last_count = self.odom_count
        start_yaw = self.pose()[2]
        max_heading_change = 0.0
        end = self.now() + seconds
        while self.now() < end:
            self.spin_once()
            if self.odom_count != last_count:
                last_count = self.odom_count
                samples.append(self.body_velocity())
                max_heading_change = max(
                    max_heading_change, abs(angle_diff(self.pose()[2], start_yaw)))
        if not samples:
            raise RunFailed('stale_odometry')
        return np.mean(np.array(samples), axis=0), max_heading_change


def steady_run(node, vehicle, axis, label, forces, args):
    """Reset, hold the forces, and measure the steady velocity on one axis."""
    wrench = vehicle.wrench_of(forces)
    index = {'surge': 0, 'sway': 1, 'yaw': 2}[axis]
    linear, quadratic = vehicle.damping(axis)
    predicted = steady_value(wrench[index], linear, quadratic)
    row = {'backend': args.backend_label, 'axis': axis, 'run': label,
           'max_force_n': round(float(np.max(np.abs(forces))), 2),
           'fx_n': round(float(wrench[0]), 2), 'fy_n': round(float(wrench[1]), 2),
           'mz_nm': round(float(wrench[2]), 2),
           'unit': 'rad/s' if axis == 'yaw' else 'm/s', 'measured': '', 'predicted':
           round(predicted, 4), 'error_percent': '', 'off_axis_max': '',
           'heading_change_rad': '', 'status': 'ok'}
    try:
        node.reset()
        node.set_forces(forces)
        start_x, start_y, start_yaw = node.pose()
        node.spin_for(args.settle_time)
        mean, _ = node.average_body_velocity(args.average_time)
        x, y, yaw = node.pose()
        measured = float(mean[index])
        off_axis = max(abs(float(mean[i])) for i in range(3) if i != index)
        row.update({'measured': round(measured, 4),
                    'error_percent': round(100.0 * (measured - predicted) / predicted, 2),
                    'off_axis_max': round(off_axis, 5)})
        if axis == 'yaw':
            # A pure yaw run should turn in place.
            row['moved_m'] = round(math.hypot(x - start_x, y - start_y), 4)
        else:
            # A pure surge or sway run should keep its heading.
            row['heading_change_rad'] = round(abs(angle_diff(yaw, start_yaw)), 5)
    except RunFailed as failure:
        row['status'] = str(failure)
    return row


def coast_run(node, vehicle, args):
    """Hold the thrust for a steady 1.5 m/s, cut it, and measure the stop."""
    linear, quadratic = vehicle.damping('surge')
    surge_force = -linear * args.coast_start_speed - quadratic * args.coast_start_speed ** 2
    forces = vehicle.forces_for([surge_force, 0.0, 0.0])
    row = {'backend': args.backend_label, 'run': 'coast_down', 'hold_force_n':
           round(float(forces[0]), 2), 'speed_at_cut_mps': '', 'stop_speed_mps':
           args.stop_speed, 'stop_distance_m': '', 'stop_time_s': '',
           'time_to_5pct_drop_s': '', 'predicted_distance_drag_only_m': '', 'status': 'ok'}
    try:
        node.reset()
        node.set_forces(forces)
        node.spin_for(args.settle_time)
        node.set_forces([0.0] * HORIZONTAL)
        node.publish_command()  # cut now, not at the next timer tick
        measure_stop(node, row, args)
        row['predicted_distance_drag_only_m'] = round(coast_distance(
            vehicle.effective_surge_mass(), linear, quadratic,
            row['speed_at_cut_mps'], args.stop_speed), 4)
    except RunFailed as failure:
        row['status'] = str(failure)
    return row


def command_loss_run(node, vehicle, args):
    """Hold f_plan, stop publishing, and check that the vehicle stops."""
    forces = [vehicle.f_plan] * HORIZONTAL
    row = {'backend': args.backend_label, 'run': 'command_loss', 'hold_force_n':
           vehicle.f_plan, 'speed_at_cut_mps': '', 'stop_speed_mps': args.stop_speed,
           'stop_distance_m': '', 'stop_time_s': '', 'time_to_5pct_drop_s': '',
           'predicted_distance_drag_only_m': '', 'status': 'ok'}
    try:
        node.reset()
        node.set_forces(forces)
        node.spin_for(args.settle_time)
        node.publishing = False  # silence: no zero command is sent
        # The backend's timeout counts from the last command it received.
        measure_stop(node, row, args, start_time=node.last_publish_time)
    except RunFailed as failure:
        row['status'] = str(failure)
    node.publishing = True
    return row


def measure_stop(node, row, args, start_time=None):
    """Measure the time and distance from start_time (default: now) until the vehicle stops."""
    start_speed = node.speed()
    if start_time is None:
        start_time = node.now()
    # The newest odometry can be up to one period (20 ms at 50 Hz) old, which
    # is 3 cm at 1.5 m/s. Move its position forward to start_time.
    start_x, start_y, yaw = node.pose()
    stamp = node.odom.header.stamp
    age = start_time - (stamp.sec + stamp.nanosec * 1e-9)
    if -0.1 < age < 0.1:
        u, v, _ = node.body_velocity()
        start_x += (u * math.cos(yaw) - v * math.sin(yaw)) * age
        start_y += (u * math.sin(yaw) + v * math.cos(yaw)) * age
    row['speed_at_cut_mps'] = round(start_speed, 4)

    def stopped():
        if row['time_to_5pct_drop_s'] == '' and node.speed() < 0.95 * start_speed:
            row['time_to_5pct_drop_s'] = round(node.now() - start_time, 3)
        return node.speed() < args.stop_speed

    node.spin_until(stopped, args.coast_timeout, 'never_stopped')
    x, y, _ = node.pose()
    row['stop_distance_m'] = round(math.hypot(x - start_x, y - start_y), 4)
    row['stop_time_s'] = round(node.now() - start_time, 3)


def write_csv(path, rows, columns):
    with open(path, 'w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=columns, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(rows)


def parse_arguments(argv):
    description_dir = os.path.join(get_package_share_directory('rov_description'), 'config')
    parser = argparse.ArgumentParser(description=__doc__.strip().split('\n')[0])
    parser.add_argument('--thrusters-yaml',
                        default=os.path.join(description_dir, 'thrusters_v4.yaml'))
    parser.add_argument('--hydrodynamics-yaml',
                        default=os.path.join(description_dir, 'hydrodynamics_v4.yaml'))
    parser.add_argument('--backend-label', default='unknown', help='written to the CSV')
    parser.add_argument('--start-pose', type=float, nargs=3, default=[5.0, 5.0, 0.0],
                        metavar=('X', 'Y', 'YAW'),
                        help='pose reset_vehicle must return to (m, m, rad)')
    parser.add_argument('--surge-percentages', type=float, nargs='*',
                        default=[20.0, 40.0, 60.0, 80.0, 100.0])
    parser.add_argument('--sway-percentages', type=float, nargs='*',
                        default=[20.0, 60.0, 100.0])
    parser.add_argument('--yaw-percentages', type=float, nargs='*',
                        default=[20.0, 60.0, 100.0])
    parser.add_argument('--settle-time', type=float, default=5.0,
                        help='s held before measuring')
    parser.add_argument('--average-time', type=float, default=2.0,
                        help='s of odometry averaged')
    parser.add_argument('--coast-start-speed', type=float, default=1.5, help='m/s')
    parser.add_argument('--stop-speed', type=float, default=0.05,
                        help='m/s; slower than this counts as stopped')
    parser.add_argument('--stop-yaw-rate', type=float, default=0.02,
                        help='rad/s; at rest also needs a yaw rate below this')
    parser.add_argument('--coast-timeout', type=float, default=60.0, help='s')
    parser.add_argument('--stale-timeout', type=float, default=0.5,
                        help='s of wall time without odometry that fails a run')
    parser.add_argument('--tolerance', type=float, default=10.0,
                        help='allowed f_plan surge speed error in %%')
    parser.add_argument('--command-rate', type=float, default=50.0, help='Hz')
    parser.add_argument('--output-dir', default='.')
    return parser.parse_args(argv)


def main():
    rclpy.init(args=sys.argv)
    args = parse_arguments(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    vehicle = Vehicle(args.thrusters_yaml, args.hydrodynamics_yaml)
    node = OpenLoopTest(args)
    log = node.get_logger()

    end = time.monotonic() + 15.0
    while node.odom is None and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.odom is None:
        log.error('No odometry on "odom" within 15 s')
        return 2

    speed_rows = []
    runs = [('surge', '{:.0f}% f_peak'.format(p), vehicle.scaled_forces([1, 0, 0], p))
            for p in args.surge_percentages]
    runs.append(('surge', 'f_plan', np.array([vehicle.f_plan] * HORIZONTAL)))
    runs += [('sway', '{:.0f}% f_peak'.format(p), vehicle.scaled_forces([0, 1, 0], p))
             for p in args.sway_percentages]
    runs += [('yaw', '{:.0f}% f_peak'.format(p), vehicle.scaled_forces([0, 0, 1], p))
             for p in args.yaw_percentages]
    for axis, label, forces in runs:
        row = steady_run(node, vehicle, axis, label, forces, args)
        speed_rows.append(row)
        log.info('{:5s} {:12s} measured {:>8} predicted {:>8} {:6} error {:>6}% '
                 'off-axis {:>8} heading {:>8} [{}]'.format(
                     axis, label, row['measured'], row['predicted'], row['unit'],
                     row['error_percent'], row['off_axis_max'],
                     row['heading_change_rad'], row['status']))

    coast_rows = [coast_run(node, vehicle, args), command_loss_run(node, vehicle, args)]
    for row in coast_rows:
        log.info('{:12s} from {} m/s: stop distance {} m, stop time {} s, 5% drop after {} s '
                 '(drag-only prediction {} m) [{}]'.format(
                     row['run'], row['speed_at_cut_mps'], row['stop_distance_m'],
                     row['stop_time_s'], row['time_to_5pct_drop_s'],
                     row['predicted_distance_drag_only_m'], row['status']))
    node.set_forces([0.0] * HORIZONTAL)
    node.publish_command()

    os.makedirs(args.output_dir, exist_ok=True)
    write_csv(os.path.join(args.output_dir, 'open_loop_speed.csv'), speed_rows,
              ['backend', 'axis', 'run', 'max_force_n', 'fx_n', 'fy_n', 'mz_nm', 'unit',
               'measured', 'predicted', 'error_percent', 'off_axis_max',
               'heading_change_rad', 'moved_m', 'status'])
    write_csv(os.path.join(args.output_dir, 'open_loop_coast.csv'), coast_rows,
              ['backend', 'run', 'hold_force_n', 'speed_at_cut_mps', 'stop_speed_mps',
               'stop_distance_m', 'stop_time_s', 'time_to_5pct_drop_s',
               'predicted_distance_drag_only_m', 'status'])
    log.info('Wrote CSV files to {}'.format(os.path.abspath(args.output_dir)))

    failed = [r['run'] + ' ' + r.get('axis', '') for r in speed_rows + coast_rows
              if r['status'] != 'ok']
    plan = [r for r in speed_rows if r['axis'] == 'surge' and r['run'] == 'f_plan'][0]
    plan_ok = plan['status'] == 'ok' and abs(plan['error_percent']) <= args.tolerance
    log.info('f_plan surge speed error {}% (tolerance {:.0f}%): {}'.format(
        plan['error_percent'], args.tolerance, 'PASS' if plan_ok else 'FAIL'))
    if failed:
        log.error('Failed runs: {}'.format(', '.join(failed)))
    node.destroy_node()
    rclpy.shutdown()
    if failed:
        return 2
    return 0 if plan_ok else 1


if __name__ == '__main__':
    sys.exit(main())
