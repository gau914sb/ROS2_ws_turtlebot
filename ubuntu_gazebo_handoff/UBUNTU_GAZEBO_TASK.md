# TurtleBot3 Formation Control — Gazebo Replication Task

**Written for:** the Claude Code agent running on the Ubuntu machine that already has the ROS2/Gazebo workspace (launch file for 4 TurtleBots in an empty world, and the formation controller code).

**Context, so you're not working blind:** this comes from a research project (perimeter defence + multi-robot formation control, APS progress seminar). Four real TurtleBot3 robots ran a bearing-based single-integrator formation controller (connectivity preservation, with collision avoidance switchable on/off), tracked by an overhead camera, across four experiment cases (Case-1b, Case-2, Case-3, Case-4). Real hardware logs, MATLAB offline simulations (single-integrator and unicycle-converted), trajectory plots, distance plots, and comparison videos already exist for these four cases, produced on a separate machine (the one this document comes from). The task now is to reproduce the same four cases live in Gazebo, starting from the *exact* real-world initial conditions, and build a repeatable, screen-recording-free way to run and document any case on demand.

Do not start implementing until you've read this whole document once. Several steps depend on decisions made earlier in it.

---

## 0. What you should already have, and what's being sent to you

**Already on your machine (per the person you're working with):**
- A ROS2 launch file that spawns 4 TurtleBot3 robots in an empty Gazebo world.
- Controller code that runs the formation law on those 4 robots.
- A Claude Code agent (you) able to read/write files, run shell commands, and use git in this workspace.

**Being sent to you from the other machine** (the person will hand you an archive or point you at files — ask them if anything below is missing rather than guessing):
- `Claude testing/algo testing/plot_case_from_csv.py` — generates the trajectory plot, the 6-pairwise-distance plot (PDF/SVG/PNG), and an animated GIF from a CSV log. **Reuse this file unmodified** if at all possible (see §3).
- `Claude testing/algo testing/plot_trajectory_overlay_generic.py` and `plot_distance_overlay_generic.py` — **use these, unmodified, for the real-vs-Gazebo comparison plots.** Both take two CSV paths (real, other) plus a label and output path as plain command-line arguments, so they work on your Gazebo log without editing anything. See §3 for exact usage and what the output should look like.
- `Claude testing/algo testing/plot_sim_trajectory_separate.py` and `plot_trajectory_overlay.py` — reference only: these are the MATLAB-sim-vs-real equivalents from the other machine (hardcoded to MATLAB's output paths). Don't use them directly; `plot_trajectory_overlay_generic.py` is the same logic made reusable for your Gazebo CSVs instead.
- `Colab with Sushmita/Exp_Collision_single_inte.m`, `Experiment_code_collsion_main.m`, `sim_timesynced_cases.m` — the MATLAB reference implementation of the controller and its exact parameters (reproduced in §2 below so you don't need MATLAB to read them).
- `APS Report/Figures/Case-1b/log_data.csv`, `Case-2/log_data.csv`, `Case-3/log_data.csv`, `Case-4/log_data.csv` — the real hardware logs. You mainly need these for the initial conditions (§1) and the CSV schema (§3), but having the full files lets you sanity-check your Gazebo run's timing/scale against the real one if something looks off.

**Task 3b below** is about unpacking this bundle into your own repo in the same relative layout it has on the sending side. Do that first, before starting the Gazebo work, so paths in this document resolve the same way on both machines.

---

## 1. Initial conditions per case (x, y in metres, yaw in radians, world frame)

These are the first logged row of each real experiment's `log_data.csv` — i.e. the robots' actual starting poses on the real hardware. Use these to set each TurtleBot's spawn pose in Gazebo for that case (both position and yaw — the real robots did not start at yaw = 0).

### Case-1b (collision avoidance: **OFF**)
| Robot | x (m) | y (m) | yaw (rad) | yaw (deg) |
|---|---|---|---|---|
| bot_0 | 0.6705 | 0.1398 | 1.4427 | 82.7 |
| bot_1 | -0.6370 | 0.1786 | 1.6645 | 95.4 |
| bot_2 | -0.6128 | -0.1347 | -1.2970 | -74.3 |
| bot_3 | 0.6413 | -0.1755 | -1.4689 | -84.2 |

### Case-2 (collision avoidance: **ON**)
| Robot | x (m) | y (m) | yaw (rad) | yaw (deg) |
|---|---|---|---|---|
| bot_0 | 0.6530 | 0.1652 | 1.4560 | 83.4 |
| bot_1 | -0.6480 | 0.1707 | 1.6719 | 95.8 |
| bot_2 | -0.6442 | -0.1747 | -1.7312 | -99.2 |
| bot_3 | 0.6733 | -0.1736 | -1.7361 | -99.5 |

### Case-3 (collision avoidance: **OFF** — this is the real-collision case; it ends early)
| Robot | x (m) | y (m) | yaw (rad) | yaw (deg) |
|---|---|---|---|---|
| bot_0 | -0.2759 | -0.0533 | 1.3832 | 79.3 |
| bot_1 | -0.6187 | 0.4192 | -0.6873 | -39.4 |
| bot_2 | 0.1074 | -0.2319 | 1.9618 | 112.4 |
| bot_3 | -0.3406 | -0.3905 | -1.8267 | -104.7 |

### Case-4 (collision avoidance: **ON** — same IC family as Case-3, fixes the collision)
| Robot | x (m) | y (m) | yaw (rad) | yaw (deg) |
|---|---|---|---|---|
| bot_0 | -0.3034 | -0.0093 | 0.6830 | 39.1 |
| bot_1 | -0.6136 | 0.4315 | -0.9821 | -56.3 |
| bot_2 | 0.1132 | -0.2382 | 2.1886 | 125.4 |
| bot_3 | -0.3801 | -0.3463 | -2.1991 | -126.0 |

All four cases share the same connectivity bounds: `lower_bound = 0.295 m`, `upper_bound = 1.465 m` (these are `Rs + r` and `Rc - r` from the controller parameters below — they're logged per-row in the real CSV and should be logged the same way in your Gazebo runs).

If your launch file sets spawn poses via a YAML/params file rather than hardcoded arguments, generate one YAML per case rather than hand-editing the launch file four times — you'll reuse it every time this case is re-run.

---

## 2. Controller parameters — match these exactly

The real robots' controller (`multi_unicycle_controller_double_angle_experiment_ekf.py` on the original hardware) used these physical parameters. **Check what your existing controller code uses — if the numbers don't match these, your Gazebo run will not reproduce the real behaviour**, even from the correct initial conditions. Treat this as a required calibration check before trusting any case replication.

```
pstar (desired formation, per-robot [x, y] relative to the formation frame):
  bot_0: ( 0.4,  0.3)
  bot_1: (-0.4,  0.3)
  bot_2: (-0.7, -0.4)
  bot_3: ( 0.7, -0.4)

Formation graph edges (5 controlled pairs): (0,1), (1,2), (2,3), (3,0), (1,3)
Collision-avoidance pair (not a graph edge, only checked when CA is on): (0,2)

r      = 0.135   # robot radius / bearing-law margin
Rs     = 0.16    # inner (collision) radius parameter
Rc     = 1.6     # outer (connectivity) radius parameter
Rcrit  = 0.4     # collision-avoidance activation distance
v_max      = 0.055   # m/s, linear speed cap
omega_max  = 0.13    # rad/s, angular speed cap
length     = 0.25    # m, look-ahead distance for the unicycle conversion
dt         = 1/50    # 50 Hz control loop
conv_tol   = 0.080   # m, per-edge convergence tolerance (all |edge_err| <= this -> stop)
```

`lower_bound = Rs + r = 0.295`, `upper_bound = Rc - r = 1.465` (already given per-case above; these are the two reference lines the distance plot draws).

### Velocity saturation is mandatory, not just a parameter to note

`v_max = 0.055 m/s` and `omega_max = 0.13 rad/s` are **hard per-tick clamps on the commanded (v, ω) sent to each robot**, applied every control cycle, after the look-ahead unicycle conversion — exactly as in the real controller:

```
v = clamp(v, -v_max, +v_max)        # -0.055 .. 0.055 m/s
omega = clamp(omega, -omega_max, +omega_max)   # -0.13 .. 0.13 rad/s
```

This is **not** the TurtleBot3's own default speed limit — stock burger/waffle limits in Gazebo are much higher (on the order of 0.22 m/s linear, ~2.8 rad/s angular). If your existing controller relies on Gazebo/TurtleBot3's default limits instead of explicitly clamping to the values above, the robots will move far faster than the real ones did, and nothing about the replication is valid even if the initial conditions and formation law are otherwise correct. **Explicitly verify this clamp exists in your controller code before running any case** — don't assume it's there because the parameter is defined somewhere.

A fast sanity check once you have a run: real Case-1b/2/4 took roughly 130–180s to converge (§4a). If your Gazebo run converges in a few seconds, the velocity clamp is almost certainly missing or not being applied — stop and fix that before treating any plot or video from that run as meaningful.

The single-integrator law itself (bearing-based, using `pstar`/the graph edges above, with the `(0,2)` pair added when collision avoidance is on) is implemented in `Colab with Sushmita/Exp_Collision_single_inte.m` and `sim_timesynced_cases.m`'s `formation_velocity()` function if you need the exact math — don't re-derive it from scratch if your existing controller already implements this law; just confirm the parameter values above match.

Per-case collision-avoidance flag (already noted above, repeated for clarity):

| Case | Collision avoidance |
|---|---|
| Case-1b | off |
| Case-2 | on |
| Case-3 | off |
| Case-4 | on |

---

## 3. Logging format — match the existing CSV schema exactly

So that the existing plotting script (`plot_case_from_csv.py`) can be reused **unmodified** on your Gazebo data, your logger must write a CSV with exactly this header:

```
time,bot_0_x,bot_0_y,bot_0_theta,bot_1_x,bot_1_y,bot_1_theta,bot_2_x,bot_2_y,bot_2_theta,bot_3_x,bot_3_y,bot_3_theta,d1,d2,d3,d4,d5,e1,e2,e3,e4,e5,lower_bound,upper_bound
```

Where:
- `time` — seconds since the run started (does not need to match wall-clock; just needs to be monotonically increasing from ~0).
- `bot_b_x`, `bot_b_y`, `bot_b_theta` — robot `b`'s pose in the world/arena frame (theta in radians). Pull this from Gazebo ground-truth odometry (or your localization topic, whichever your controller actually closes the loop on — log the same source the controller uses, so the plots reflect what the controller "saw").
- `d1..d5` — the 5 graph-edge distances, same order as the edges in §2: `d1=|p1-p0|, d2=|p2-p1|, d3=|p3-p2|, d4=|p0-p3|, d5=|p3-p1|`.
- `e1..e5` — `d_i - desired_i`, where `desired_i` is the distance between the corresponding two robots' `pstar` entries (constant per run).
- `lower_bound`, `upper_bound` — constant per run, `0.295` and `1.465` (repeat the same value every row; the existing script expects a column, not a scalar).

Run `python3 plot_case_from_csv.py <your_case_dir>` once you have a `log_data.csv` in that directory — it writes `trajectory_all6_from_csv.{png,pdf,svg}`, `distances_all6_from_csv.{png,pdf,svg}`, and `animation_from_csv.gif` into the same folder. That's "the proper distance plots as we did in this whole thing" — don't write a new plotting script for this; reuse the one you were sent.

### Exactly what these plots should look like (so you can self-check against the spec, not just against "the script ran without errors")

**Trajectory plot** (`trajectory_all6_from_csv.png`, one CSV / one source):
- Square-aspect axes, X (m) vs Y (m).
- Each robot's full path as a thin dashed line in a fixed color: bot_0 = blue, bot_1 = red, bot_2 = green, bot_3 = orange.
- A circle marker at each robot's start position, a square marker at its end position, same per-bot color, black edge.
- The 5 formation-graph edges drawn twice: once at the first timestep as a gray dotted "Initial connections" pentagon/quadrilateral, once at the last timestep as a black solid "Final connections" shape.
- A legend identifying all of the above, and a grid.
- **What it should actually show**: for Case-1b/2/4, the final (black solid) connection shape should look noticeably more regular/compact than the initial (gray dotted) one — that's the formation converging. For Case-3, the final shape should look barely different from a mid-collapse state, since that run ends early.

**Distance plot** (`distances_all6_from_csv.png`, one CSV / one source):
- X axis = time (s), Y axis = distance (m).
- 6 solid lines, one per robot pair (5 graph edges + the bot_0-bot_2 diagonal), fixed colors: bot_0-bot_1 = blue, bot_1-bot_2 = red, bot_2-bot_3 = green, bot_3-bot_0 = orange, bot_1-bot_3 = purple, bot_0-bot_2 = magenta.
- Each pair also gets a dashed horizontal line at its own desired distance, same color.
- A red dotted horizontal line at the lower bound (0.295 m) and a black dotted one at the upper bound (1.465 m).
- **What it should actually show**: for Case-1b/2/4, all 6 solid lines should trend toward their matching dashed desired-distance line and stay between the two bound lines once converged (briefly dipping close to the lower bound is fine and matches the real data; a line crossing a bound and staying there is not). For Case-3, expect one pair (the bot_0-bot_2 diagonal) to visibly dive toward the lower bound and the run to end there — that's the real collision this case is named for, and your Gazebo run should show the same qualitative failure, not converge cleanly where the real one didn't.

**Real-vs-Gazebo trajectory overlay** (new, you generate this — see below for the command):
- Same axes/marker/color conventions as above, but now **two sources overlaid**: real = solid line + circle (start) + square (end); Gazebo = dashed line (same per-bot color) + triangle (end only — start is shared, same IC, so there's only one start marker per bot).
- Final connections drawn for **both**: real as solid black, Gazebo as dashed black — so you can directly compare the two final formation shapes, not just the two sets of paths.
- Run it per case, once you have that case's Gazebo `log_data.csv`:
  ```
  python3 plot_trajectory_overlay_generic.py \
    "APS Report/Figures/<Case>/log_data.csv" \
    "APS Report/Figures/<Case>/gazebo/<timestamp>/log_data.csv" \
    "gazebo sim" \
    "APS Report/Figures/<Case>/gazebo/<timestamp>/trajectory_real_vs_gazebo" \
    "Real vs. Gazebo Trajectories (<Case>)"
  ```
- **What it should actually show**: the dashed (Gazebo) path should be visibly smoother/more direct than the solid (real) path — Gazebo doesn't have the real camera/EKF noise — but should end up forming a similar-looking final shape (the dashed black final-connections shape and the solid black one should be comparable in size and rough orientation, not wildly different).

**Real-vs-Gazebo distance overlay** (new, you generate this):
- Same 6-pair/color convention as the single-source distance plot, but each pair now has **two** lines: real = solid, Gazebo = dashed, both in that pair's color. Desired-distance lines become dash-dot (so they're visually distinct from both the real-solid and gazebo-dashed data lines) instead of plain dashed. Bounds unchanged (red/black dotted).
- Run it the same way as the trajectory overlay:
  ```
  python3 plot_distance_overlay_generic.py \
    "APS Report/Figures/<Case>/log_data.csv" \
    "APS Report/Figures/<Case>/gazebo/<timestamp>/log_data.csv" \
    "gazebo sim" \
    "APS Report/Figures/<Case>/gazebo/<timestamp>/distances_real_vs_gazebo" \
    "Real vs. Gazebo Inter-Robot Distances (<Case>)"
  ```
- **What it should actually show**: the dashed (Gazebo) lines should settle onto their dash-dot desired lines faster and with far less noise than the solid real lines — if Gazebo's dashed lines are just as noisy as the real ones, or don't converge at all, something is off with the controller-parameter match in §2, not with the plotting.

If what you get doesn't match these descriptions in the ways called out above, don't just ship it — that's the signal to go back and check the IC/parameter match (§1, §2) or the logging (§3) before assuming the plot script is wrong.

---

## 4. Step-by-step

### (a) Run the four cases in Gazebo with the real initial conditions

1. Unpack the sent files first (see §0 and task (b) below) so this document's paths match your repo.
2. For each case in `{Case-1b, Case-2, Case-3, Case-4}`:
   - Set the 4 robots' spawn poses to that case's table in §1.
   - Confirm/set the controller's collision-avoidance flag per the table in §2.
   - Confirm the controller's physical parameters match §2 (one-time check, not per-case).
   - Confirm the v_max/omega_max clamp described in §2 is actually applied in your control loop, not left at Gazebo/TurtleBot3's default speed limits (one-time check, not per-case — but don't skip it).
   - Run the simulation until all `|e_i| <= conv_tol` (converged) or a sensible timeout (Case-1b/2/4 ran roughly 130–180s real-time on hardware; Case-3 ended early, around 10s, due to the real collision — don't force it to run longer looking for convergence that was never going to happen).
   - Log every control-loop tick (or a fixed-rate subsample, e.g. the same 50 Hz as the control loop) to a `log_data.csv` with the exact schema in §3.
   - Run `plot_case_from_csv.py` on that case's output directory.
3. Sanity check: for Case-1b/2/4, the final `d_i` values should land close to the real hardware's final distances (visible in the sent `log_data.csv` files, last row) — not identical, since Gazebo dynamics and the real unicycle/EKF loop differ, but in the same ballpark and converging to the same formation shape. For Case-3, expect the two robots on the uncontrolled `(0,2)` pair to approach a near-collision before the run ends, same as the real log.

### (b) Unpack and arrange the sent files, then commit

1. Ask for (or locate) the archive/bundle from the other machine if you don't already have it.
2. Arrange it inside your own repo using the **same relative layout** it has on the sending side:
   - `Claude testing/algo testing/` — the Python scripts (`plot_case_from_csv.py` and friends).
   - `Colab with Sushmita/` — the MATLAB reference files.
   - `APS Report/Figures/Case-1b/`, `Case-2/`, `Case-3/`, `Case-4/` — the real `log_data.csv` files (and anything else you were sent for them).
   - This document (`UBUNTU_GAZEBO_TASK.md`) at the repo root.
3. Don't rename or restructure beyond this — the hardcoded relative paths in the sent scripts (e.g. `plot_sim_trajectory_separate.py`'s `../../APS Report/Figures/`) assume this exact layout relative to `Claude testing/algo testing/`.
4. Add your own Gazebo-run outputs in a clearly separate location so they're never confused with the real-hardware data, e.g. `APS Report/Figures/Case-1b/gazebo/<timestamp>/` (see (c) below for the exact structure).
5. `git add` the new files, commit with a clear message (e.g. `"Add Gazebo replication of real TurtleBot3 cases 1b-4"`), and push if a remote is configured. Check `git status` before committing — don't `git add -A` blindly if there's unrelated in-progress work in this workspace.

### (c) Write new, reusable "run this case autonomously" tooling

This is new code, separate from whatever ad hoc controller/launch code already exists — the point is a **single command that fully runs and documents one case with no manual steps and no screen recording**.

Build a script (or ROS2 launch file + small Python orchestrator, whichever fits your existing stack better) that, given a case name, does all of the following unattended:

1. **Launch** Gazebo with the 4 TurtleBots at that case's initial conditions (§1) and the controller with that case's collision-avoidance flag (§2).
2. **Spawn two camera viewpoints** of the arena for recording — do not screen-record:
   - A **top-down (orthographic-ish) view**, looking straight down at the arena, framing all 4 robots' expected working area.
   - An **isometric view**, angled so depth and robot orientation are visible.
   - How to implement this depends on which Gazebo you have:
     - **Gazebo Classic (gazebo11 + gazebo_ros)**: add two camera sensor models to the world (or two `gazebo_ros_camera` plugins), each publishing an image topic; record each topic's image stream to video (e.g. a small `image_transport` subscriber node writing frames via OpenCV `VideoWriter`, or `ros2 bag record` the two topics and convert afterward).
     - **New Gazebo (gz-sim / Ignition, e.g. Harmonic/Fortress with `ros_gz_bridge`)**: either use two camera sensors + `ros_gz_bridge` the same way, or use gz-sim's own GUI recording plugin / `gz sim -r --record` if you specifically need the full visual render rather than a sensor feed — check which your launch file already uses before picking an approach.
   - Verify with a short test run that both video files actually contain the arena and robots, not a blank/default camera pose, before relying on this for a real case run.
3. **Log** `log_data.csv` per §3 for the run.
4. **Stop** the simulation on convergence or timeout (same criteria as (a)).
5. **Generate plots**: run, in order, as the automatic last step (not a separate manual command):
   - `plot_case_from_csv.py` on the resulting log (Gazebo-only trajectory + distance + GIF, §3).
   - `plot_trajectory_overlay_generic.py` against that case's real `log_data.csv` and this run's Gazebo `log_data.csv` (real-vs-Gazebo trajectory overlay, §3).
   - `plot_distance_overlay_generic.py`, same two CSVs (real-vs-Gazebo distance overlay, §3).
6. **Save everything under one time-date-stamped folder per run**, so repeated runs of the same case never overwrite each other:
   ```
   APS Report/Figures/<Case>/gazebo/<YYYY-MM-DD_HH-MM-SS>/
     log_data.csv
     trajectory_all6_from_csv.png / .pdf / .svg
     distances_all6_from_csv.png / .pdf / .svg
     animation_from_csv.gif
     trajectory_real_vs_gazebo.png / .pdf / .svg
     distances_real_vs_gazebo.png / .pdf / .svg
     camera_top.mp4
     camera_iso.mp4
     run_meta.txt        # case name, CA flag, start/end wall-clock time, convergence/timeout reason, git commit hash of the code that produced this run
   ```
7. **Autonomous trigger**: make running this as simple as `./run_case.sh Case-2` (or equivalent) — so that when asked to "run Case-2" (or all four), it's one command with zero manual steps or screen-recording setup in between.
8. **Commit and push every run's output**, not just once for the unpacked input files. The person on the other end will `git pull`/clone this repo to see the videos, CSVs, and plots — if a run's folder isn't committed, they can't see it. After each case run (or after a batch of them), `git add` that run's whole timestamped folder, commit (e.g. `"Gazebo run: Case-2, 2026-10-20_11-00-00"`), and push. Heads-up, not a reason to skip this: the camera videos are tens of MB each, so the repo will grow substantially over repeated runs — that's expected and accepted here, not a sign to compress/omit them or switch to Git LFS unless you're specifically asked to.

Once this is built, do a full dry run of all four cases end to end before calling it done, and check the output folder structure actually matches §4(c)'s layout for all four — including that the commit for each actually landed and (if a remote exists) pushed.

---

## 5. When you're done

Report back (to the person, not by editing this file):
- Which Gazebo variant you used and why.
- Any parameter mismatches you found between the existing controller and §2's values, and whether you fixed them or left them (and why).
- Confirmation that the v_max/omega_max clamp was (or wasn't, and why) already present in the existing controller, and the actual convergence time you measured per case vs. the real ~130–180s.
- The four output folders' paths.
- Anything in this document that turned out to be wrong or ambiguous once you had real Gazebo output in front of you — that feedback is more useful than silently working around it.
