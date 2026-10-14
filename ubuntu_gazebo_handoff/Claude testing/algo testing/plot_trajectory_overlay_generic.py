"""
Generic real-vs-other trajectory overlay: same visual convention as
plot_trajectory_overlay.py (which is hardcoded to the MATLAB single-integrator
sim), but takes both CSV paths explicitly so it works for ANY second source
with the same CSV schema -- in particular, a Gazebo-run log_data.csv.

Real: solid line, circle = start, square = end.
Other: dashed line (same per-bot color), triangle = end (start is shared,
       same initial condition, so only one start marker is drawn).
Connections: initial (dotted gray, shared IC) once; final real (solid black)
and final other (dashed black) both drawn, to compare the achieved formation
shape, not just the paths getting there.

Usage:
    python plot_trajectory_overlay_generic.py <real_csv> <other_csv> <other_label> <out_path_noext> [<title>]

Example (Gazebo vs real, Case-2):
    python plot_trajectory_overlay_generic.py \\
        "APS Report/Figures/Case-2/log_data.csv" \\
        "APS Report/Figures/Case-2/gazebo/2026-10-20_11-00-00/log_data.csv" \\
        "gazebo sim" \\
        "APS Report/Figures/Case-2/gazebo/2026-10-20_11-00-00/trajectory_real_vs_gazebo" \\
        "Real vs. Gazebo Trajectories (Case-2)"
"""
import csv
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

EDGES = [(0, 1), (1, 2), (2, 3), (3, 0), (1, 3)]
BOT_COLORS = ["blue", "red", "green", "orange"]


def load_csv(path):
    rows = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append({k: float(v) for k, v in row.items()})
    return rows


def positions(rows):
    T = len(rows)
    P = np.zeros((T, 4, 2))
    for k, r in enumerate(rows):
        for b in range(4):
            P[k, b, 0] = r[f"bot_{b}_x"]
            P[k, b, 1] = r[f"bot_{b}_y"]
    return P


def plot_overlay(Preal, Pother, other_label, title, out_path_noext):
    fig, ax = plt.subplots(figsize=(8.5, 8.5))

    for b in range(4):
        ax.plot(Preal[:, b, 0], Preal[:, b, 1], "-", color=BOT_COLORS[b], linewidth=1.3, alpha=0.85,
                label=f"bot_{b} (real)")
        ax.plot(Pother[:, b, 0], Pother[:, b, 1], "--", color=BOT_COLORS[b], linewidth=1.3, alpha=0.85,
                label=f"bot_{b} ({other_label})")

    for (i, j) in EDGES:
        ax.plot([Preal[0, i, 0], Preal[0, j, 0]], [Preal[0, i, 1], Preal[0, j, 1]], ":", color="gray", linewidth=1.3,
                label="Initial connections" if (i, j) == EDGES[0] else None)
        ax.plot([Preal[-1, i, 0], Preal[-1, j, 0]], [Preal[-1, i, 1], Preal[-1, j, 1]], "-", color="black", linewidth=2,
                label="Final connections (real)" if (i, j) == EDGES[0] else None)
        ax.plot([Pother[-1, i, 0], Pother[-1, j, 0]], [Pother[-1, i, 1], Pother[-1, j, 1]], "--", color="black", linewidth=1.6,
                label=f"Final connections ({other_label})" if (i, j) == EDGES[0] else None)

    for b in range(4):
        ax.plot(*Preal[0, b], "o", color=BOT_COLORS[b], markersize=9, markeredgecolor="black", zorder=5,
                label="start (shared IC)" if b == 0 else None)
        ax.plot(*Preal[-1, b], "s", color=BOT_COLORS[b], markersize=9, markeredgecolor="black", zorder=5,
                label="end (real)" if b == 0 else None)
        ax.plot(*Pother[-1, b], "^", color=BOT_COLORS[b], markersize=10, markeredgecolor="black", zorder=5,
                label=f"end ({other_label})" if b == 0 else None)

    ax.set_aspect("equal")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title(title)
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.08), fontsize=8, ncol=3)
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
    title = sys.argv[5] if len(sys.argv) > 5 else f"Real vs. {other_label} Trajectories"
    Preal = positions(load_csv(real_csv))
    Pother = positions(load_csv(other_csv))
    os.makedirs(os.path.dirname(out_noext) or ".", exist_ok=True)
    plot_overlay(Preal, Pother, other_label, title, out_noext)
