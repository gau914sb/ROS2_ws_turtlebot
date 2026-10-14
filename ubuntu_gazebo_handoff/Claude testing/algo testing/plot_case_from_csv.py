"""
Regenerates the trajectory plot, the distance plot (as vector PDF/SVG,
not just PNG), and an animated GIF for one experiment case, directly
from its log_data.csv -- so the figures used in the report don't depend
on whatever ad hoc plotting script produced the originals.

The distance plot shows ALL SIX pairwise robot distances (the five
H_Nset graph edges logged directly in the CSV as d1..d5, PLUS the one
non-edge pair bot_0-bot_2, computed here from the raw x,y columns),
each with its own desired-distance reference line, plus the shared
upper/lower connectivity-collision bounds from the CSV.

The five graph-edge desired distances come directly from the CSV
(desired_i = d_i - e_i, since e_i is logged as actual-desired). The
sixth (bot_0-bot_2, not a controlled edge) is NOT directly available,
but the 5-edge graph (K4 minus the 0-2 edge) is generically rigid in
2D, so its length is fully determined by the other five: triangulate
robots 0 and 2 on opposite sides of the shared 1-3 diagonal using the
law of cosines, from the OTHER desired edge lengths, and read off the
resulting |desired_0 - desired_2|.

Usage:
    python plot_case_from_csv.py <case_dir>

<case_dir> must contain log_data.csv with columns:
    time, bot_0_x, bot_0_y, bot_0_theta, ..., bot_3_theta,
    d1..d5 (actual edge distances), e1..e5 (actual-desired per edge),
    lower_bound, upper_bound
"""
import csv
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

# H_Nset edges, in the same order as d1..d5 / e1..e5 in the CSV.
EDGES = [(0, 1), (1, 2), (2, 3), (3, 0), (1, 3)]
EDGE_COLORS = ["blue", "red", "green", "orange", "purple"]
DIAG_COLOR = "magenta"   # the non-edge bot_0-bot_2 pair


def load_csv(path):
    rows = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append({k: float(v) for k, v in row.items()})
    return rows


def desired_lengths(rows):
    """Average desired_i = d_i - e_i over the run (should be ~constant;
    averaging smooths out any per-step numerical noise)."""
    des = []
    for i in range(1, 6):
        vals = [r[f"d{i}"] - r[f"e{i}"] for r in rows]
        des.append(float(np.mean(vals)))
    return des   # [L01, L12, L23, L30, L13], matching EDGES order


def desired_diagonal_length(des):
    """Triangulate desired positions of robots 0 and 2 relative to the
    shared 1-3 edge (law of cosines), on opposite sides of it (since
    0-1-2-3 is a simple quadrilateral with 1-3 as the drawn diagonal),
    and return the resulting desired |0-2| distance."""
    L01, L12, L23, L30, L13 = des

    def third_vertex_offset(La, Lc, Lb):
        # Triangle with vertices (this point)-(pt1)-(pt3), known side
        # lengths La=dist(this,1), Lb=dist(this,3), Lc=L13=dist(1,3).
        # Returns (x, y_abs) of "this" point with 1 at origin, 3 at (Lc,0).
        cos_ang = (La**2 + Lc**2 - Lb**2) / (2 * La * Lc)
        cos_ang = np.clip(cos_ang, -1.0, 1.0)
        ang = np.arccos(cos_ang)
        return La * np.cos(ang), La * np.sin(ang)

    x0, y0 = third_vertex_offset(L01, L13, L30)   # point 0, above the line
    x2, y2 = third_vertex_offset(L12, L13, L23)   # point 2, below the line (mirrored)
    y2 = -y2

    return float(np.hypot(x0 - x2, y0 - y2))


def compute_all_distances(rows):
    """Returns (times, dist_matrix[6, T], desired[6], bound_lo, bound_hi).
    dist_matrix rows 0-4 = the 5 graph edges (EDGES order), row 5 = 0-2."""
    times = np.array([r["time"] for r in rows])
    T = len(rows)
    D = np.zeros((6, T))
    for i in range(5):
        D[i] = [r[f"d{i+1}"] for r in rows]
    x0 = np.array([r["bot_0_x"] for r in rows]); y0 = np.array([r["bot_0_y"] for r in rows])
    x2 = np.array([r["bot_2_x"] for r in rows]); y2 = np.array([r["bot_2_y"] for r in rows])
    D[5] = np.hypot(x0 - x2, y0 - y2)

    des5 = desired_lengths(rows)
    des6 = des5 + [desired_diagonal_length(des5)]

    lo = rows[0]["lower_bound"]
    hi = rows[0]["upper_bound"]
    return times, D, des6, lo, hi


def positions(rows):
    T = len(rows)
    P = np.zeros((T, 4, 2))
    TH = np.zeros((T, 4))
    for k, r in enumerate(rows):
        for b in range(4):
            P[k, b, 0] = r[f"bot_{b}_x"]
            P[k, b, 1] = r[f"bot_{b}_y"]
            TH[k, b] = r[f"bot_{b}_theta"]
    return P, TH


def plot_distances(times, D, des, lo, hi, out_path_noext):
    fig, ax = plt.subplots(figsize=(11, 7))
    labels = [f"bot_{i}-bot_{j}" for (i, j) in EDGES] + ["bot_0-bot_2"]
    colors = EDGE_COLORS + [DIAG_COLOR]

    for i in range(6):
        ax.plot(times, D[i], "-", color=colors[i], linewidth=2, label=f"{labels[i]} (actual)")
        ax.axhline(des[i], color=colors[i], linestyle="--", linewidth=1.3, alpha=0.7,
                    label=f"{labels[i]} (desired: {des[i]:.3f} m)")

    ax.axhline(lo, color="red", linestyle=":", linewidth=2, label=f"Lower bound = {lo:.3f} m")
    ax.axhline(hi, color="black", linestyle=":", linewidth=2, label=f"Upper bound = {hi:.3f} m")

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Distance (m)")
    ax.set_title("Inter-Robot Distances (all 6 pairs) with Bounds -- from CSV log")
    ax.legend(loc="center left", bbox_to_anchor=(1.01, 0.5), fontsize=8, ncol=1)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()

    fig.savefig(out_path_noext + ".pdf", bbox_inches="tight")
    fig.savefig(out_path_noext + ".svg", bbox_inches="tight")
    fig.savefig(out_path_noext + ".png", dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  wrote {out_path_noext}.pdf / .svg / .png")


def plot_trajectory(P, out_path_noext):
    fig, ax = plt.subplots(figsize=(8, 8))
    colors = ["blue", "red", "green", "orange"]
    for b in range(4):
        ax.plot(P[:, b, 0], P[:, b, 1], "--", color=colors[b], linewidth=1, alpha=0.7, label=f"bot_{b} (transient)")

    for (i, j) in EDGES:
        ax.plot([P[0, i, 0], P[0, j, 0]], [P[0, i, 1], P[0, j, 1]], ":", color="gray", linewidth=1.5,
                label="Initial connections" if (i, j) == EDGES[0] else None)
        ax.plot([P[-1, i, 0], P[-1, j, 0]], [P[-1, i, 1], P[-1, j, 1]], "-", color="black", linewidth=2,
                label="Final connections" if (i, j) == EDGES[0] else None)

    for b in range(4):
        ax.plot(*P[0, b], "o", color=colors[b], markersize=8, markeredgecolor="black")
        ax.plot(*P[-1, b], "s", color=colors[b], markersize=8, markeredgecolor="black")

    ax.set_aspect("equal")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("Multi-Robot Trajectories -- from CSV log")
    ax.legend(loc="best", fontsize=8)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()

    fig.savefig(out_path_noext + ".pdf", bbox_inches="tight")
    fig.savefig(out_path_noext + ".svg", bbox_inches="tight")
    fig.savefig(out_path_noext + ".png", dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  wrote {out_path_noext}.pdf / .svg / .png")


def make_gif(times, D, des, lo, hi, P, out_path, max_frames=300):
    T = len(times)
    step = max(1, T // max_frames)
    frame_idxs = list(range(0, T, step))
    if frame_idxs[-1] != T - 1:
        frame_idxs.append(T - 1)

    fig, (ax_traj, ax_dist) = plt.subplots(1, 2, figsize=(14, 7))
    colors = ["blue", "red", "green", "orange"]
    edge_colors = EDGE_COLORS + [DIAG_COLOR]
    labels = [f"bot_{i}-bot_{j}" for (i, j) in EDGES] + ["bot_0-bot_2"]

    all_x, all_y = P[:, :, 0], P[:, :, 1]
    pad = 0.3
    ax_traj.set_xlim(all_x.min() - pad, all_x.max() + pad)
    ax_traj.set_ylim(all_y.min() - pad, all_y.max() + pad)
    ax_traj.set_aspect("equal")
    ax_traj.set_title("Trajectories")
    ax_traj.grid(True, alpha=0.3)

    traj_lines = [ax_traj.plot([], [], "-", color=colors[b], linewidth=1.2)[0] for b in range(4)]
    markers = [ax_traj.plot([], [], "o", color=colors[b], markersize=9, markeredgecolor="black")[0] for b in range(4)]
    conn_lines = [ax_traj.plot([], [], "-", color="black", linewidth=2)[0] for _ in EDGES]
    time_text = ax_traj.text(0.02, 0.98, "", transform=ax_traj.transAxes, va="top", fontfamily="monospace")

    ax_dist.set_xlim(times[0], times[-1])
    ax_dist.set_ylim(0, max(D.max(), hi) * 1.1)
    ax_dist.set_xlabel("Time (s)")
    ax_dist.set_ylabel("Distance (m)")
    ax_dist.set_title("Distances")
    ax_dist.grid(True, alpha=0.3)
    ax_dist.axhline(lo, color="red", linestyle=":", linewidth=1.5)
    ax_dist.axhline(hi, color="black", linestyle=":", linewidth=1.5)
    for i in range(6):
        ax_dist.axhline(des[i], color=edge_colors[i], linestyle="--", linewidth=1, alpha=0.6)
    dist_lines = [ax_dist.plot([], [], "-", color=edge_colors[i], linewidth=1.5, label=labels[i])[0] for i in range(6)]
    ax_dist.legend(loc="upper right", fontsize=7)

    def update(frame_i):
        k = frame_idxs[frame_i]
        for b in range(4):
            traj_lines[b].set_data(P[:k+1, b, 0], P[:k+1, b, 1])
            markers[b].set_data([P[k, b, 0]], [P[k, b, 1]])
        for idx, (i, j) in enumerate(EDGES):
            conn_lines[idx].set_data([P[k, i, 0], P[k, j, 0]], [P[k, i, 1], P[k, j, 1]])
        time_text.set_text(f"t = {times[k]:.1f}s")
        for i in range(6):
            dist_lines[i].set_data(times[:k+1], D[i, :k+1])
        return traj_lines + markers + conn_lines + [time_text] + dist_lines

    anim = FuncAnimation(fig, update, frames=len(frame_idxs), blit=False, interval=50)
    anim.save(out_path, writer=PillowWriter(fps=20))
    plt.close(fig)
    print(f"  wrote {out_path}")


def process_case(case_dir):
    csv_path = os.path.join(case_dir, "log_data.csv")
    print(f"Processing {csv_path} ...")
    rows = load_csv(csv_path)
    times, D, des, lo, hi = compute_all_distances(rows)
    P, TH = positions(rows)

    plot_distances(times, D, des, lo, hi, os.path.join(case_dir, "distances_all6_from_csv"))
    plot_trajectory(P, os.path.join(case_dir, "trajectory_all6_from_csv"))
    make_gif(times, D, des, lo, hi, P, os.path.join(case_dir, "animation_from_csv.gif"))


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python plot_case_from_csv.py <case_dir>")
        sys.exit(1)
    process_case(sys.argv[1])
