"""
Standalone trajectory plot for the MATLAB single-integrator simulation of
each real experiment case, in the exact same visual style as
plot_case_from_csv.py's plot_trajectory() (used for the real-robot-only
trajectory plot, trajectory_all6_from_csv.png) -- so the two are a direct
apples-to-apples comparison, just not overlaid on the same axes.

Reads <repo>/Colab with Sushmita/sim_timesynced/<Case>_sim_single_integrator.csv
(written by sim_timesynced_cases.m, same column schema as the real log_data.csv),
writes <repo>/APS Report/Figures/<Case>/trajectory_matlab_sim.{png,pdf,svg}.

Usage:
    python plot_sim_trajectory_separate.py             # does all 4 cases
    python plot_sim_trajectory_separate.py Case-1b      # just one
"""
import csv
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

EDGES = [(0, 1), (1, 2), (2, 3), (3, 0), (1, 3)]

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


def plot_trajectory(P, title, out_path_noext):
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
    ax.set_title(title)
    ax.legend(loc="best", fontsize=8)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()

    fig.savefig(out_path_noext + ".pdf", bbox_inches="tight")
    fig.savefig(out_path_noext + ".svg", bbox_inches="tight")
    fig.savefig(out_path_noext + ".png", dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  wrote {out_path_noext}.pdf / .svg / .png")


def process_case(case):
    sim_csv = os.path.join(SIM_DIR, f"{case}_sim_single_integrator.csv")
    if not os.path.exists(sim_csv):
        print(f"  SKIP {case}: {sim_csv} not found (run sim_timesynced_cases.m first)")
        return
    rows = load_csv(sim_csv)
    P = positions(rows)
    out_dir = os.path.join(FIG_DIR, case)
    os.makedirs(out_dir, exist_ok=True)
    plot_trajectory(
        P,
        f"Multi-Robot Trajectories -- MATLAB Single-Integrator Sim ({case})",
        os.path.join(out_dir, "trajectory_matlab_sim"),
    )


if __name__ == "__main__":
    cases = sys.argv[1:] if len(sys.argv) > 1 else CASES
    for case in cases:
        print(f"Processing {case} ...")
        process_case(case)
