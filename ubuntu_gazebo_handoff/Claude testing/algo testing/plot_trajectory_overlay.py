"""
Standalone trajectory plot overlaying REAL TurtleBot trajectories and the
MATLAB single-integrator simulation on the SAME axes, for direct visual
comparison -- the trajectory-space counterpart to the distance-comparison
plots that already exist (Case-X_distance_errors.png).

Real: solid line, circle = start, square = end (matches trajectory_all6_from_csv.png).
Sim:  dashed line (same per-bot color), triangle = end (start coincides with
      real start, same initial condition, so only one start marker is drawn).
Connections: initial (dotted gray, shared IC) once; final real (solid black)
and final sim (dashed black) both drawn, to compare the achieved formation
shape, not just the paths getting there.

Reads:
  <repo>/APS Report/Figures/<Case>/log_data.csv
  <repo>/Colab with Sushmita/sim_timesynced/<Case>_sim_single_integrator.csv
Writes:
  <repo>/APS Report/Figures/<Case>/trajectory_real_vs_sim.{png,pdf,svg}

Usage:
    python plot_trajectory_overlay.py             # all 4 cases
    python plot_trajectory_overlay.py Case-1b      # just one
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

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.join(HERE, "..", "..")
SIM_DIR = os.path.join(REPO, "Colab with Sushmita", "sim_timesynced")
FIG_DIR = os.path.join(REPO, "APS Report", "Figures")

CASES = ["Case-1b", "Case-2", "Case-3", "Case-4"]


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


def plot_overlay(Preal, Psim, case, out_path_noext):
    fig, ax = plt.subplots(figsize=(8.5, 8.5))

    for b in range(4):
        ax.plot(Preal[:, b, 0], Preal[:, b, 1], "-", color=BOT_COLORS[b], linewidth=1.3, alpha=0.85,
                label=f"bot_{b} (real)")
        ax.plot(Psim[:, b, 0], Psim[:, b, 1], "--", color=BOT_COLORS[b], linewidth=1.3, alpha=0.85,
                label=f"bot_{b} (single-int sim)")

    for (i, j) in EDGES:
        ax.plot([Preal[0, i, 0], Preal[0, j, 0]], [Preal[0, i, 1], Preal[0, j, 1]], ":", color="gray", linewidth=1.3,
                label="Initial connections" if (i, j) == EDGES[0] else None)
        ax.plot([Preal[-1, i, 0], Preal[-1, j, 0]], [Preal[-1, i, 1], Preal[-1, j, 1]], "-", color="black", linewidth=2,
                label="Final connections (real)" if (i, j) == EDGES[0] else None)
        ax.plot([Psim[-1, i, 0], Psim[-1, j, 0]], [Psim[-1, i, 1], Psim[-1, j, 1]], "--", color="black", linewidth=1.6,
                label="Final connections (sim)" if (i, j) == EDGES[0] else None)

    for b in range(4):
        ax.plot(*Preal[0, b], "o", color=BOT_COLORS[b], markersize=9, markeredgecolor="black", zorder=5,
                label="start (shared IC)" if b == 0 else None)
        ax.plot(*Preal[-1, b], "s", color=BOT_COLORS[b], markersize=9, markeredgecolor="black", zorder=5,
                label="end (real)" if b == 0 else None)
        ax.plot(*Psim[-1, b], "^", color=BOT_COLORS[b], markersize=10, markeredgecolor="black", zorder=5,
                label="end (sim)" if b == 0 else None)

    ax.set_aspect("equal")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title(f"Real vs.\\ MATLAB Single-Integrator Sim Trajectories ({case})")
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.08), fontsize=8, ncol=3)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()

    fig.savefig(out_path_noext + ".pdf", bbox_inches="tight")
    fig.savefig(out_path_noext + ".svg", bbox_inches="tight")
    fig.savefig(out_path_noext + ".png", dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  wrote {out_path_noext}.pdf / .svg / .png")


def process_case(case):
    real_csv = os.path.join(FIG_DIR, case, "log_data.csv")
    sim_csv = os.path.join(SIM_DIR, f"{case}_sim_single_integrator.csv")
    if not os.path.exists(real_csv):
        print(f"  SKIP {case}: {real_csv} not found")
        return
    if not os.path.exists(sim_csv):
        print(f"  SKIP {case}: {sim_csv} not found (run sim_timesynced_cases.m first)")
        return
    Preal = positions(load_csv(real_csv))
    Psim = positions(load_csv(sim_csv))
    plot_overlay(Preal, Psim, case, os.path.join(FIG_DIR, case, "trajectory_real_vs_sim"))


if __name__ == "__main__":
    cases = sys.argv[1:] if len(sys.argv) > 1 else CASES
    for case in cases:
        print(f"Processing {case} ...")
        process_case(case)
