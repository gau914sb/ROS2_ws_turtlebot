#!/usr/bin/env python3
"""
Plot (and optionally animate) robot trajectories and inter-robot distances
from a CSV log produced by MultiUnicycleController.save_csv() in
multi_unicycle_controller_double_angle.py.

Reproduces the trajectory and distance summary plots without re-running the
simulation:
  - Trajectory plot: full path per robot as a dashed/dotted line (transient
    behaviour), with connectivity edges drawn at the initial positions
    (dotted gray) and final positions (solid black).
  - Distance plot: d1..d5 vs time, with the desired distance per edge and
    the lower bound (Rs+r) / upper bound (Rc-r) drawn as reference lines.
  - Optional animation (--animate / --save-anim) showing both panels
    evolving together over time.

Usage:
    python3 plot_from_csv.py log_data.csv
    python3 plot_from_csv.py log_data.csv --outdir ./plots
    python3 plot_from_csv.py log_data.csv --animate
    python3 plot_from_csv.py log_data.csv --save-anim trajectory.gif --no-show
"""

import argparse
import os
import re
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Default formation topology for the double-angle controller (4 robots, 5 edges):
# edge k connects robots DEFAULT_EDGES[k-1] = (i, j) -- matches H_Nset /
# get_connected_pairs() in multi_unicycle_controller_double_angle.py
DEFAULT_EDGES = [(0, 1), (1, 2), (2, 3), (3, 0), (1, 3)]

ROBOT_COLORS = ['blue', 'red', 'green', 'orange', 'purple', 'brown', 'cyan', 'magenta']
EDGE_COLORS = ['blue', 'red', 'green', 'orange', 'purple']


def detect_robots(df):
    """Detect robot name prefixes from columns named '<name>_x'/'_y'/'_theta'."""
    names = []
    for col in df.columns:
        if col.endswith('_x'):
            prefix = col[:-2]
            if f'{prefix}_y' in df.columns and f'{prefix}_theta' in df.columns:
                names.append(prefix)
    return names


def detect_edge_ids(df):
    """Detect edge indices from columns named 'd<k>'."""
    pattern = re.compile(r'^d(\d+)$')
    ids = []
    for col in df.columns:
        m = pattern.match(col)
        if m:
            ids.append(int(m.group(1)))
    return sorted(ids)


def plot_connections(ax, positions, pairs, linestyle, linewidth, alpha, color, label=None):
    for idx, (i, j) in enumerate(pairs):
        x_vals = [positions[i, 0], positions[j, 0]]
        y_vals = [positions[i, 1], positions[j, 1]]
        if idx == 0 and label:
            ax.plot(x_vals, y_vals, linestyle=linestyle, linewidth=linewidth,
                    alpha=alpha, color=color, label=label)
        else:
            ax.plot(x_vals, y_vals, linestyle=linestyle, linewidth=linewidth,
                    alpha=alpha, color=color)


def plot_trajectories(df, robots, pairs, save_path=None, show=True):
    fig, ax = plt.subplots(figsize=(10, 10))

    initial_positions = []
    final_positions = []

    for idx, name in enumerate(robots):
        color = ROBOT_COLORS[idx % len(ROBOT_COLORS)]
        x = df[f'{name}_x'].to_numpy()
        y = df[f'{name}_y'].to_numpy()

        # Full transient path shown dashed
        ax.plot(x, y, linestyle='--', linewidth=1.2, alpha=0.7, color=color,
                label=f'{name} (transient)')

        # Start (circle) / end (square) markers
        ax.plot(x[0], y[0], 'o', color=color, markersize=10,
                markeredgecolor='black', markeredgewidth=1.5)
        ax.plot(x[-1], y[-1], 's', color=color, markersize=10,
                markeredgecolor='black', markeredgewidth=1.5)

        initial_positions.append([x[0], y[0]])
        final_positions.append([x[-1], y[-1]])

    initial_positions = np.array(initial_positions)
    final_positions = np.array(final_positions)

    plot_connections(ax, initial_positions, pairs, linestyle=':', linewidth=2.0,
                      alpha=0.6, color='gray', label='Initial connections')
    plot_connections(ax, final_positions, pairs, linestyle='-', linewidth=2.5,
                      alpha=0.85, color='black', label='Final connections')

    ax.set_xlabel('X (m)', fontsize=12)
    ax.set_ylabel('Y (m)', fontsize=12)
    ax.set_title('Multi-Robot Trajectories (from CSV log)', fontsize=14)
    ax.legend(loc='best', fontsize=9)
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')
    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f'Trajectory plot saved to {save_path}')
    if show:
        plt.show()
    else:
        plt.close(fig)


def compute_pairwise_distance(df, name_a, name_b):
    """Distance between two named robots at every logged tick, computed directly from x,y."""
    xa = df[f'{name_a}_x'].to_numpy()
    ya = df[f'{name_a}_y'].to_numpy()
    xb = df[f'{name_b}_x'].to_numpy()
    yb = df[f'{name_b}_y'].to_numpy()
    return np.sqrt((xa - xb) ** 2 + (ya - yb) ** 2)


EXTRA_PAIR_COLORS = ['magenta', 'cyan', 'brown', 'olive']


def plot_all_distances_combined(df, edge_ids, pairs, robots, extra_pairs,
                                 lower_bound, upper_bound, save_path, show=False):
    """
    Single combined plot: every formation-edge distance plus any extra
    (non-edge) pair distances computed directly from x,y, all together with
    the lower/upper bound reference lines. Saved to a fixed filename, so
    re-running overwrites the same file instead of creating a new one.
    """
    fig, ax = plt.subplots(figsize=(12, 8))
    t = df['time'].to_numpy()

    for k in edge_ids:
        color = EDGE_COLORS[(k - 1) % len(EDGE_COLORS)]
        i, j = pairs[k - 1]
        d = df[f'd{k}'].to_numpy()
        e = df[f'e{k}'].to_numpy()
        d_star = float(np.mean(d - e))

        label = f'Edge {k}: {robots[i]}-{robots[j]}'
        ax.plot(t, d, '-', color=color, linewidth=2, label=f'{label} (actual)')
        ax.axhline(y=d_star, color=color, linestyle='--', linewidth=1.2, alpha=0.6,
                   label=f'{label} (desired: {d_star:.3f}m)')

    for idx, (name_a, name_b) in enumerate(extra_pairs):
        if name_a not in robots or name_b not in robots:
            print(f'Skipping extra pair ({name_a}, {name_b}): robot not found in CSV columns')
            continue
        color = EXTRA_PAIR_COLORS[idx % len(EXTRA_PAIR_COLORS)]
        d_extra = compute_pairwise_distance(df, name_a, name_b)
        ax.plot(t, d_extra, '-', color=color, linewidth=2.5,
                label=f'{name_a}-{name_b} (from x,y)')

    ax.axhline(y=lower_bound, color='red', linestyle=':', linewidth=1.8,
               label=f'Lower bound Rs+r = {lower_bound:.3f}m')
    ax.axhline(y=upper_bound, color='black', linestyle=':', linewidth=1.8,
               label=f'Upper bound Rc-r = {upper_bound:.3f}m')

    ax.set_xlabel('Time (s)', fontsize=12)
    ax.set_ylabel('Distance (m)', fontsize=12)
    ax.set_title('All Inter-Robot Distances with Bounds', fontsize=14)
    ax.legend(loc='best', fontsize=8, ncol=2)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()

    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    print(f'Saved {save_path}')
    if show:
        plt.show()
    else:
        plt.close(fig)


def plot_distances(df, edge_ids, pairs, robots, save_path=None, show=True):
    fig, ax = plt.subplots(figsize=(12, 8))

    t = df['time'].to_numpy()
    lower_bound = df['lower_bound'].iloc[0]
    upper_bound = df['upper_bound'].iloc[0]

    for k in edge_ids:
        color = EDGE_COLORS[(k - 1) % len(EDGE_COLORS)]
        i, j = pairs[k - 1]
        d = df[f'd{k}'].to_numpy()
        e = df[f'e{k}'].to_numpy()
        d_star = float(np.mean(d - e))

        label = f'Edge {k}: {robots[i]}-{robots[j]}'
        ax.plot(t, d, '-', color=color, linewidth=2, label=f'{label} (actual)')
        ax.axhline(y=d_star, color=color, linestyle='--', linewidth=1.2, alpha=0.6,
                   label=f'{label} (desired: {d_star:.3f}m)')

    ax.axhline(y=lower_bound, color='red', linestyle=':', linewidth=1.8,
               label=f'Lower bound Rs+r = {lower_bound:.3f}m')
    ax.axhline(y=upper_bound, color='black', linestyle=':', linewidth=1.8,
               label=f'Upper bound Rc-r = {upper_bound:.3f}m')

    ax.set_xlabel('Time (s)', fontsize=12)
    ax.set_ylabel('Distance (m)', fontsize=12)
    ax.set_title('Inter-Robot Distances with Bounds (from CSV log)', fontsize=14)
    ax.legend(loc='best', fontsize=8, ncol=2)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f'Distances plot saved to {save_path}')
    if show:
        plt.show()
    else:
        plt.close(fig)


def animate(df, robots, pairs, edge_ids, interval=50, save_path=None, show=True):
    fig, (ax_traj, ax_dist) = plt.subplots(1, 2, figsize=(18, 8))

    xs = {name: df[f'{name}_x'].to_numpy() for name in robots}
    ys = {name: df[f'{name}_y'].to_numpy() for name in robots}
    thetas = {name: df[f'{name}_theta'].to_numpy() for name in robots}
    t = df['time'].to_numpy()
    num_frames = len(t)

    all_x = np.concatenate(list(xs.values()))
    all_y = np.concatenate(list(ys.values()))
    margin = 0.5
    ax_traj.set_xlim(all_x.min() - margin, all_x.max() + margin)
    ax_traj.set_ylim(all_y.min() - margin, all_y.max() + margin)
    ax_traj.set_aspect('equal')
    ax_traj.grid(True, alpha=0.3)
    ax_traj.set_title('Trajectories')
    ax_traj.set_xlabel('X (m)')
    ax_traj.set_ylabel('Y (m)')

    traj_lines, markers, arrows = {}, {}, {}
    for idx, name in enumerate(robots):
        color = ROBOT_COLORS[idx % len(ROBOT_COLORS)]
        traj_lines[name], = ax_traj.plot([], [], '--', color=color, linewidth=1.2,
                                          alpha=0.7, label=name)
        markers[name], = ax_traj.plot([], [], 'o', color=color, markersize=10,
                                       markeredgecolor='black', markeredgewidth=1.5)
        arrows[name] = ax_traj.arrow(0, 0, 0, 0, head_width=0.06, head_length=0.04,
                                      fc=color, ec=color)

    initial_positions = np.array([[xs[name][0], ys[name][0]] for name in robots])
    plot_connections(ax_traj, initial_positions, pairs, linestyle=':', linewidth=2.0,
                      alpha=0.5, color='gray', label='Initial connections')

    conn_lines = []
    for _ in pairs:
        line, = ax_traj.plot([], [], '-', linewidth=2.5, alpha=0.8, color='black')
        conn_lines.append(line)

    time_text = ax_traj.text(0.02, 0.98, '', transform=ax_traj.transAxes, fontsize=11,
                              va='top', family='monospace',
                              bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    ax_traj.legend(loc='upper right', fontsize=8)

    # Distance subplot
    lower_bound = df['lower_bound'].iloc[0]
    upper_bound = df['upper_bound'].iloc[0]
    all_d = np.concatenate([df[f'd{k}'].to_numpy() for k in edge_ids])
    ax_dist.set_xlim(0, t[-1] * 1.05 if t[-1] > 0 else 1.0)
    ax_dist.set_ylim(0, max(all_d.max(), upper_bound) * 1.1)
    ax_dist.axhline(y=lower_bound, color='red', linestyle=':', linewidth=1.5, alpha=0.7,
                     label=f'Lower bound = {lower_bound:.3f}m')
    ax_dist.axhline(y=upper_bound, color='black', linestyle=':', linewidth=1.5, alpha=0.7,
                     label=f'Upper bound = {upper_bound:.3f}m')
    ax_dist.set_xlabel('Time (s)')
    ax_dist.set_ylabel('Distance (m)')
    ax_dist.set_title('Distances')
    ax_dist.grid(True, alpha=0.3)

    dist_lines = []
    for k in edge_ids:
        color = EDGE_COLORS[(k - 1) % len(EDGE_COLORS)]
        i, j = pairs[k - 1]
        line, = ax_dist.plot([], [], '-', color=color, linewidth=2,
                              label=f'{robots[i]}-{robots[j]}')
        dist_lines.append(line)
    time_indicator, = ax_dist.plot([], [], 'k-', linewidth=1.5, alpha=0.7)
    ax_dist.legend(loc='upper right', fontsize=8)

    plt.tight_layout()

    def init():
        for name in robots:
            traj_lines[name].set_data([], [])
            markers[name].set_data([], [])
        for line in conn_lines:
            line.set_data([], [])
        for line in dist_lines:
            line.set_data([], [])
        time_indicator.set_data([], [])
        time_text.set_text('')
        return (list(traj_lines.values()) + list(markers.values()) + conn_lines +
                dist_lines + [time_indicator, time_text])

    def update(frame):
        current_positions = []
        for idx, name in enumerate(robots):
            color = ROBOT_COLORS[idx % len(ROBOT_COLORS)]
            traj_lines[name].set_data(xs[name][:frame + 1], ys[name][:frame + 1])
            markers[name].set_data([xs[name][frame]], [ys[name][frame]])
            arrows[name].remove()
            dx = 0.15 * np.cos(thetas[name][frame])
            dy = 0.15 * np.sin(thetas[name][frame])
            arrows[name] = ax_traj.arrow(xs[name][frame], ys[name][frame], dx, dy,
                                          head_width=0.06, head_length=0.04,
                                          fc=color, ec=color)
            current_positions.append([xs[name][frame], ys[name][frame]])
        current_positions = np.array(current_positions)

        for idx, (i, j) in enumerate(pairs):
            conn_lines[idx].set_data([current_positions[i, 0], current_positions[j, 0]],
                                      [current_positions[i, 1], current_positions[j, 1]])

        time_text.set_text(f't = {t[frame]:.1f}s')

        for idx, k in enumerate(edge_ids):
            dist_lines[idx].set_data(t[:frame + 1], df[f'd{k}'].to_numpy()[:frame + 1])
        time_indicator.set_data([t[frame], t[frame]], [0, ax_dist.get_ylim()[1]])

        return (list(traj_lines.values()) + list(markers.values()) + conn_lines +
                dist_lines + [time_indicator, time_text])

    anim = FuncAnimation(fig, update, frames=num_frames, init_func=init,
                          blit=False, interval=interval, repeat=True)

    if save_path:
        if save_path.endswith('.gif'):
            anim.save(save_path, writer='pillow', fps=max(1, 1000 // interval))
        else:
            anim.save(save_path, writer='ffmpeg', fps=max(1, 1000 // interval))
        print(f'Animation saved to {save_path}')

    if show:
        plt.show()
    else:
        plt.close(fig)


def main():
    parser = argparse.ArgumentParser(
        description='Plot trajectories/distances (and optionally animate) from a logged CSV.')
    parser.add_argument('csv_path', help='Path to log_data.csv')
    parser.add_argument('--outdir', default=None,
                         help='Directory to save PNG plots (default: alongside the CSV)')
    parser.add_argument('--animate', action='store_true',
                         help='Show an animation of trajectory + distances evolving over time')
    parser.add_argument('--save-anim', default=None,
                         help='Path to save the animation (e.g. anim.gif or anim.mp4); implies --animate')
    parser.add_argument('--no-show', action='store_true',
                         help='Do not display plots/animation interactively (only save files)')
    parser.add_argument('--extra-pair', action='append', nargs=2, metavar=('ROBOT_A', 'ROBOT_B'),
                         help='Extra robot pair(s) to include in the combined distance plot, '
                              'computed directly from x,y data (can repeat this flag). '
                              'Default: bot_0 bot_2')
    args = parser.parse_args()

    df = pd.read_csv(args.csv_path)

    robots = detect_robots(df)
    if not robots:
        raise ValueError('No robot position columns (e.g. bot_0_x/_y/_theta) found in CSV')

    edge_ids = detect_edge_ids(df)
    if not edge_ids:
        raise ValueError('No distance columns (e.g. d1, d2, ...) found in CSV')

    if len(edge_ids) > len(DEFAULT_EDGES):
        raise ValueError(
            f'Found {len(edge_ids)} distance columns but only {len(DEFAULT_EDGES)} default '
            'edge pairs are known; update DEFAULT_EDGES in this script to match your topology.')
    pairs = DEFAULT_EDGES[:len(edge_ids)]

    outdir = args.outdir or os.path.dirname(os.path.abspath(args.csv_path))
    os.makedirs(outdir, exist_ok=True)

    do_animate = args.animate or bool(args.save_anim)
    show_static = not args.no_show and not do_animate

    plot_trajectories(df, robots, pairs,
                       save_path=os.path.join(outdir, 'trajectory_from_csv.png'),
                       show=show_static)
    plot_distances(df, edge_ids, pairs, robots,
                    save_path=os.path.join(outdir, 'distances_from_csv.png'),
                    show=show_static)

    # ------------------------------------------------------------------
    # Single combined distance plot (all formation edges + extra pairs),
    # saved into a subfolder next to the CSV. Filename is fixed, so
    # re-running this script overwrites the same file instead of creating
    # a new one.
    # ------------------------------------------------------------------
    csv_dir = os.path.dirname(os.path.abspath(args.csv_path))
    dist_dir = os.path.join(csv_dir, 'distance_plots')
    os.makedirs(dist_dir, exist_ok=True)

    lower_bound = float(df['lower_bound'].iloc[0])
    upper_bound = float(df['upper_bound'].iloc[0])

    # Extra pair(s) computed directly from x,y (default: bot_0/bot_2, the
    # pair that ends up colliding), independent of the formation edge list.
    extra_pairs = args.extra_pair or [('bot_0', 'bot_2')]

    plot_all_distances_combined(
        df, edge_ids, pairs, robots, extra_pairs, lower_bound, upper_bound,
        save_path=os.path.join(dist_dir, 'all_distances.png'), show=False)

    if do_animate:
        animate(df, robots, pairs, edge_ids, save_path=args.save_anim,
                show=not args.no_show)


if __name__ == '__main__':
    main()
