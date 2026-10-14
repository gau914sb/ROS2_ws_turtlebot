"""
Generic real-vs-other distance overlay: all 6 pairwise robot distances,
real (solid) and a second source (dashed), same colors per pair, plus the
shared desired-distance reference lines and connectivity bounds -- the
distance-space counterpart to plot_trajectory_overlay_generic.py.

Same distance/desired-length computation as plot_case_from_csv.py
(desired_i = mean(d_i - e_i); the 6th, non-edge bot_0-bot_2 pair's desired
length is triangulated from the other 5 via the law of cosines).

Usage:
    python plot_distance_overlay_generic.py <real_csv> <other_csv> <other_label> <out_path_noext> [<title>]

Example (Gazebo vs real, Case-2):
    python plot_distance_overlay_generic.py \\
        "APS Report/Figures/Case-2/log_data.csv" \\
        "APS Report/Figures/Case-2/gazebo/2026-10-20_11-00-00/log_data.csv" \\
        "gazebo sim" \\
        "APS Report/Figures/Case-2/gazebo/2026-10-20_11-00-00/distances_real_vs_gazebo" \\
        "Real vs. Gazebo Inter-Robot Distances (Case-2)"
"""
import csv
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

EDGES = [(0, 1), (1, 2), (2, 3), (3, 0), (1, 3)]
EDGE_COLORS = ["blue", "red", "green", "orange", "purple"]
DIAG_COLOR = "magenta"


def load_csv(path):
    rows = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append({k: float(v) for k, v in row.items()})
    return rows


def desired_lengths(rows):
    des = []
    for i in range(1, 6):
        vals = [r[f"d{i}"] - r[f"e{i}"] for r in rows]
        des.append(float(np.mean(vals)))
    return des  # [L01, L12, L23, L30, L13], matching EDGES order


def desired_diagonal_length(des):
    L01, L12, L23, L30, L13 = des

    def third_vertex_offset(La, Lc, Lb):
        cos_ang = (La**2 + Lc**2 - Lb**2) / (2 * La * Lc)
        cos_ang = np.clip(cos_ang, -1.0, 1.0)
        ang = np.arccos(cos_ang)
        return La * np.cos(ang), La * np.sin(ang)

    x0, y0 = third_vertex_offset(L01, L13, L30)
    x2, y2 = third_vertex_offset(L12, L13, L23)
    y2 = -y2
    return float(np.hypot(x0 - x2, y0 - y2))


def compute_all_distances(rows):
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


def plot_overlay(t_real, D_real, t_other, D_other, des, lo, hi, other_label, title, out_path_noext):
    fig, ax = plt.subplots(figsize=(12, 7))
    labels = [f"bot_{i}-bot_{j}" for (i, j) in EDGES] + ["bot_0-bot_2"]
    colors = EDGE_COLORS + [DIAG_COLOR]

    for i in range(6):
        ax.plot(t_real, D_real[i], "-", color=colors[i], linewidth=2, label=f"{labels[i]} (real)")
        ax.plot(t_other, D_other[i], "--", color=colors[i], linewidth=1.6, alpha=0.85,
                label=f"{labels[i]} ({other_label})")
        ax.axhline(des[i], color=colors[i], linestyle="-.", linewidth=1.0, alpha=0.5,
                    label=f"{labels[i]} (desired: {des[i]:.3f} m)")

    ax.axhline(lo, color="red", linestyle=":", linewidth=2, label=f"Lower bound = {lo:.3f} m")
    ax.axhline(hi, color="black", linestyle=":", linewidth=2, label=f"Upper bound = {hi:.3f} m")

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Distance (m)")
    ax.set_title(title)
    ax.legend(loc="center left", bbox_to_anchor=(1.01, 0.5), fontsize=7.5, ncol=1)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()

    fig.savefig(out_path_noext + ".pdf", bbox_inches="tight")
    fig.savefig(out_path_noext + ".svg", bbox_inches="tight")
    fig.savefig(out_path_noext + ".png", dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {out_path_noext}.pdf / .svg / .png")


if __name__ == "__main__":
    if len(sys.argv) < 5:
        print(__doc__)
        sys.exit(1)
    real_csv, other_csv, other_label, out_noext = sys.argv[1:5]
    title = sys.argv[5] if len(sys.argv) > 5 else f"Real vs. {other_label} Inter-Robot Distances"
    t_real, D_real, des, lo, hi = compute_all_distances(load_csv(real_csv))
    t_other, D_other, _, _, _ = compute_all_distances(load_csv(other_csv))
    os.makedirs(os.path.dirname(out_noext) or ".", exist_ok=True)
    plot_overlay(t_real, D_real, t_other, D_other, des, lo, hi, other_label, title, out_noext)
