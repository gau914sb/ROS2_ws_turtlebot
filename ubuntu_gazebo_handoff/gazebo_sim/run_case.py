#!/usr/bin/env python3
"""
run_case.py
============
Single-command, unattended runner for one Gazebo replication case
(UBUNTU_GAZEBO_TASK.md section 4(c)). Launches Gazebo + the 4 robots at that
case's real initial conditions + two documentation cameras + the formation
controller, waits for the controller to stop (convergence or timeout),
shuts everything down, regenerates all plots (own-run + real-vs-Gazebo
overlays), writes run_meta.txt, and commits+pushes the run's output folder.

Usage:
    python3 run_case.py <Case-1b|Case-2|Case-3|Case-4>

Must be run with the ROS2/Gazebo environment already sourced (see
run_case.sh, which does this).
"""
import json
import os
import signal
import subprocess
import sys
import time
from datetime import datetime

import yaml

GAZEBO_SIM_DIR = os.path.dirname(os.path.abspath(__file__))
HANDOFF_DIR = os.path.dirname(GAZEBO_SIM_DIR)
REPO_ROOT = os.path.dirname(HANDOFF_DIR)
FIGURES_DIR = os.path.join(HANDOFF_DIR, 'APS Report', 'Figures')
PLOT_DIR = os.path.join(HANDOFF_DIR, 'Claude testing', 'algo testing')

VALID_CASES = ['Case-1b', 'Case-2', 'Case-3', 'Case-4']

# Extra wall-clock margin (beyond the case's max_duration) allowed for
# Gazebo/node startup and clean shutdown before run_case.py gives up waiting
# for control_status.json and force-terminates the launch.
STARTUP_SHUTDOWN_MARGIN_S = 90.0


def run(cmd, **kwargs):
    print(f'+ {" ".join(cmd)}')
    return subprocess.run(cmd, **kwargs)


def terminate_process_group(proc, grace_s=20.0):
    if proc.poll() is not None:
        return
    pgid = os.getpgid(proc.pid)
    print(f'Sending SIGINT to process group {pgid} ...')
    try:
        os.killpg(pgid, signal.SIGINT)
    except ProcessLookupError:
        return
    deadline = time.time() + grace_s
    while time.time() < deadline:
        if proc.poll() is not None:
            return
        time.sleep(1.0)
    print('Still running after SIGINT grace period, sending SIGTERM ...')
    try:
        os.killpg(pgid, signal.SIGTERM)
    except ProcessLookupError:
        return
    deadline = time.time() + 10.0
    while time.time() < deadline:
        if proc.poll() is not None:
            return
        time.sleep(1.0)
    print('Still running after SIGTERM, sending SIGKILL ...')
    try:
        os.killpg(pgid, signal.SIGKILL)
    except ProcessLookupError:
        pass


def main():
    if len(sys.argv) != 2 or sys.argv[1] not in VALID_CASES:
        print(f'Usage: python3 run_case.py <{"|".join(VALID_CASES)}>')
        sys.exit(1)

    case = sys.argv[1]
    case_yaml = os.path.join(GAZEBO_SIM_DIR, 'cases', f'{case}.yaml')
    with open(case_yaml, 'r') as f:
        case_cfg = yaml.safe_load(f)
    collision_avoidance = bool(case_cfg['collision_avoidance'])
    max_duration = float(case_cfg.get('max_duration', 240.0))

    timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    out_dir = os.path.join(FIGURES_DIR, case, 'gazebo', timestamp)
    os.makedirs(out_dir, exist_ok=True)
    print(f'Output directory: {out_dir}')

    status_path = os.path.join(out_dir, 'control_status.json')
    cam_top = os.path.join(out_dir, 'camera_top.mp4')
    cam_iso = os.path.join(out_dir, 'camera_iso.mp4')

    launch_file = os.path.join(GAZEBO_SIM_DIR, 'launch', 'spawn_case.launch.py')
    cmd = [
        'ros2', 'launch', launch_file,
        f'case_yaml:={case_yaml}',
        f'log_dir:={out_dir}',
        f'camera_top_output:={cam_top}',
        f'camera_iso_output:={cam_iso}',
    ]

    start_wall = datetime.now()
    log_file_path = os.path.join(out_dir, 'launch_output.log')
    log_file = open(log_file_path, 'w')
    proc = subprocess.Popen(cmd, stdout=log_file, stderr=subprocess.STDOUT, start_new_session=True)
    print(f'Launched (pid {proc.pid}), log: {log_file_path}')

    hard_cutoff = max_duration + STARTUP_SHUTDOWN_MARGIN_S
    t0 = time.time()
    crashed = False
    while True:
        if os.path.exists(status_path):
            print('control_status.json appeared -- controller has stopped.')
            break
        if proc.poll() is not None:
            print(f'Launch process exited early (code {proc.returncode}) before status file appeared.')
            crashed = True
            break
        if time.time() - t0 > hard_cutoff:
            print(f'Hard cutoff ({hard_cutoff:.0f}s) reached without a status file -- forcing shutdown.')
            break
        time.sleep(1.0)

    # Grace period for the camera recorder to flush remaining frames before
    # we tear everything down.
    time.sleep(3.0)
    terminate_process_group(proc)
    end_wall = datetime.now()
    log_file.close()

    # ---- sanity checks ----
    problems = []
    if not os.path.exists(os.path.join(out_dir, 'log_data.csv')):
        problems.append('log_data.csv missing')
    if not os.path.exists(status_path):
        problems.append('control_status.json missing')
    for cam_path, label in ((cam_top, 'camera_top.mp4'), (cam_iso, 'camera_iso.mp4')):
        if not os.path.exists(cam_path):
            problems.append(f'{label} missing')
        elif os.path.getsize(cam_path) < 1024:
            problems.append(f'{label} is suspiciously small ({os.path.getsize(cam_path)} bytes)')
    if crashed:
        problems.append('launch process exited before the controller finished')
    if problems:
        print('WARNING -- problems detected with this run:')
        for p in problems:
            print(f'  - {p}')

    status = {}
    if os.path.exists(status_path):
        with open(status_path) as f:
            status = json.load(f)

    # ---- plots ----
    real_csv = os.path.join(FIGURES_DIR, case, 'log_data.csv')
    gazebo_csv = os.path.join(out_dir, 'log_data.csv')

    if os.path.exists(gazebo_csv):
        run(['python3', os.path.join(PLOT_DIR, 'plot_case_from_csv.py'), out_dir])

        run([
            'python3', os.path.join(PLOT_DIR, 'plot_trajectory_overlay_generic.py'),
            real_csv, gazebo_csv, 'gazebo sim',
            os.path.join(out_dir, 'trajectory_real_vs_gazebo'),
            f'Real vs. Gazebo Trajectories ({case})',
        ])
        run([
            'python3', os.path.join(PLOT_DIR, 'plot_distance_overlay_generic.py'),
            real_csv, gazebo_csv, 'gazebo sim',
            os.path.join(out_dir, 'distances_real_vs_gazebo'),
            f'Real vs. Gazebo Inter-Robot Distances ({case})',
        ])
    else:
        print('Skipping plots -- no log_data.csv to plot.')

    # ---- run_meta.txt ----
    commit_hash = subprocess.run(
        ['git', 'rev-parse', 'HEAD'], cwd=REPO_ROOT, capture_output=True, text=True
    ).stdout.strip()

    meta_lines = [
        f'case: {case}',
        f'collision_avoidance: {collision_avoidance}',
        f'start_wall_clock: {start_wall.isoformat()}',
        f'end_wall_clock: {end_wall.isoformat()}',
        f'wall_duration_s: {(end_wall - start_wall).total_seconds():.1f}',
        f'converged: {status.get("converged")}',
        f'stop_reason: {status.get("stop_reason")}',
        f'final_time_s: {status.get("final_time_s")}',
        f'rows_logged: {status.get("rows_logged")}',
        f'conv_tol: {status.get("conv_tol")}',
        f'max_duration: {status.get("max_duration")}',
        f'git_commit: {commit_hash}',
    ]
    if problems:
        meta_lines.append('problems:')
        meta_lines += [f'  - {p}' for p in problems]

    with open(os.path.join(out_dir, 'run_meta.txt'), 'w') as f:
        f.write('\n'.join(meta_lines) + '\n')
    print('\n'.join(meta_lines))

    # ---- commit + push ----
    run(['git', 'add', out_dir], cwd=REPO_ROOT)
    commit_msg = f'Gazebo run: {case}, {timestamp}'
    commit_result = run(['git', 'commit', '-m', commit_msg], cwd=REPO_ROOT)
    if commit_result.returncode == 0:
        push_result = run(['git', 'push'], cwd=REPO_ROOT)
        if push_result.returncode != 0:
            print('WARNING: git push failed -- run was committed locally only.')
    else:
        print('Nothing to commit (or commit failed) for this run.')

    print(f'\nDone. Output: {out_dir}')


if __name__ == '__main__':
    main()
