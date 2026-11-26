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
Plot optimized surface trajectories (implementation step 7).

Reads <name>_trajectory.csv and <name>_corridor.csv written by the optimizer
tests (run them with ROV_PLANNING_PLOT_DIR=<dir>) and saves <name>.png with:
  - the x-y path inside the corridor polygons,
  - speed against the braking-curve limit v_max(d) over time,
  - the thrust-constraint margin max(H tau - h) over time (<= 0 is feasible).

Usage: plot_surface_trajectory.py <dir>
"""

import csv
import glob
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402


def read_rows(path):
    with open(path, newline='') as stream:
        return [{k: float(v) for k, v in row.items()} for row in csv.DictReader(stream)]


def plot_case(trajectory_csv):
    name = os.path.basename(trajectory_csv)[:-len('_trajectory.csv')]
    samples = read_rows(trajectory_csv)
    corridor = read_rows(trajectory_csv.replace('_trajectory.csv', '_corridor.csv'))
    figure, (xy, speed, thrust) = plt.subplots(1, 3, figsize=(16, 4.5))

    polygons = {}
    for row in corridor:
        polygons.setdefault(int(row['polygon']), []).append((row['x'], row['y']))
    for corners in polygons.values():
        corners.append(corners[0])
        xy.plot([c[0] for c in corners], [c[1] for c in corners], color='0.7', linewidth=0.8)
    xy.plot([s['x'] for s in samples], [s['y'] for s in samples], color='tab:blue')
    xy.set_aspect('equal')
    xy.set_title(f'{name}: path and corridor')
    xy.set_xlabel('x (m)')
    xy.set_ylabel('y (m)')

    t = [s['t'] for s in samples]
    speed.plot(t, [s['speed'] for s in samples], label='speed')
    speed.plot(t, [s['v_max'] for s in samples], '--', label='v_max(d)')
    speed.set_title('speed vs braking curve')
    speed.set_xlabel('t (s)')
    speed.set_ylabel('m/s')
    speed.legend()

    thrust.plot(t, [s['thrust_margin'] for s in samples])
    thrust.axhline(0.0, color='k', linewidth=0.8)
    thrust.set_title('thrust margin max(H tau - h)')
    thrust.set_xlabel('t (s)')
    thrust.set_ylabel('N (<= 0 feasible)')

    figure.tight_layout()
    output = os.path.join(os.path.dirname(trajectory_csv), name + '.png')
    figure.savefig(output, dpi=110)
    print('wrote', output)


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        return 1
    files = sorted(glob.glob(os.path.join(sys.argv[1], '*_trajectory.csv')))
    for path in files:
        plot_case(path)
    return 0 if files else 1


if __name__ == '__main__':
    sys.exit(main())
