# Real-Hardware Formation Experiment — Step-by-Step Guide

This is the complete procedure for running a real 4-TurtleBot3 formation
experiment with the overhead-camera + ArUco + EKF pipeline in this
`experiment/` folder. It was assembled by reading the docstrings/code of
the individual scripts (`calibrate_ground_origin.py`, `move_to_initial_ekf.py`,
`multi_unicycle_controller_double_angle_experiment_ekf.py`) — no such
consolidated guide existed before this; see **Appendix C** for the gaps
that still need filling in from memory/shell history.

## Overview of the pipeline

```
RealSense camera  --(image topic)-->  aruco_tracker (aruco_opencv)
                                              |
                                      /aruco_detections + TF
                                              |
                          calibrate_ground_origin.py (run once, keep alive)
                                              |
                              TF: camera_color_optical_frame -> ground_origin
                                              |
                    +-------------------------+-------------------------+
                    |                                                   |
          move_to_initial_ekf.py                 multi_unicycle_controller_double_angle_experiment_ekf.py
          (drive robots to case IC)              (the actual formation controller)
```

Every script resolves robot pose via TF lookups `ground_origin -> marker_N`
(published by `aruco_tracker`), through a per-robot EKF that smooths the
raw ArUco measurement. Nothing below will produce real poses until the
camera, `aruco_tracker`, and `calibrate_ground_origin.py` are all running.

---

## 0. One-time per-session setup

### 0.1 Get all 4 robots on the WiFi network

Each robot is a Raspberry Pi 4 running Ubuntu Server, configured via
netplan (not NetworkManager). Known-good config, confirmed working
(see **Appendix B** for the full story of what breaks this and why):

```yaml
network:
  version: 2
  renderer: networkd
  ethernets:
    eth0:
      dhcp4: true
      optional: true
  wifis:
    wlan0:
      dhcp4: true
      optional: true
      access-points:
        "TP-Link_8782":
          password: "46660213"
```

Apply with `sudo netplan apply`, verify with `ip addr show wlan0`.

SSH credentials follow the pattern `botN`/`botN` (e.g. `ssh bot3@<ip>`,
password `bot3`).

### 0.2 Source the workspace on each robot / control machine

```bash
source /opt/ros/jazzy/setup.bash
source ~/<workspace>/install/setup.bash   # if applicable
```

---

## 1. Launch the RealSense camera

**Gap**: the exact command used historically isn't recorded anywhere in
this repo (see Appendix C). The controller scripts expect the color image
on:
```
/camera/camera/color/image_raw/compressed
```
which matches the standard `realsense2_camera` package's default topic
layout, e.g.:
```bash
ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true enable_depth:=false
```
Confirm the actual topic name matches what's above once it's running:
```bash
ros2 topic list | grep image_raw
```

## 2. Launch ArUco marker detection

This is the stock ROS2 `aruco_opencv` package (not custom code in this
repo). Its default config (`/opt/ros/jazzy/share/aruco_opencv/config/
aruco_tracker.yaml`) expects `cam_base_topic: camera/image_raw`, so it
needs remapping/parameter overrides to match the RealSense's actual
(compressed) topic and this project's marker size:

```bash
ros2 launch aruco_opencv aruco_tracker.launch.xml \
  cam_base_topic:=camera/camera/color/image_raw \
  image_sub_compressed:=true \
  marker_size:=0.0742 \
  marker_dict:=4X4_50
```
(`marker_size` and `marker_dict` above are the current values already
baked into `aruco_tracker.yaml`; override only if your markers differ.)

Verify detections are flowing:
```bash
ros2 topic echo /aruco_detections --once
```

## 3. Calibrate (or reload) the ground origin

Needs `aruco_tracker` already running (step 2).

**First time, or if the camera has physically moved** — place one robot's
marker flat on the floor at the exact point/heading you want as world
origin:
```bash
python3.12 experiment/calibrate_ground_origin.py --marker-id 3
```
(`--marker-id 3` uses `bot_3`'s marker as the reference; use whichever
robot is convenient. More samples for a noisier camera: `--samples 100`.)

**If the camera hasn't moved since the last calibration** — skip sampling:
```bash
python3.12 experiment/calibrate_ground_origin.py --load
```

**Leave this terminal running for the entire experiment** — it publishes a
static TF that only reaches nodes started *after* it, so if it dies,
every pose lookup downstream breaks silently.

## 4. Move all 4 robots to the case's initial positions

Edit the `TARGETS` dict near the top of `move_to_initial_ekf.py` to the
case you're about to run (see **Appendix A** for the per-case values),
then:
```bash
python3.12 experiment/move_to_initial_ekf.py
```
It drives each robot through rotate → drive → rotate-to-heading phases
and exits automatically once all 4 are within tolerance
(`POS_TOL = 0.03 m`, `HEADING_TOL = 0.10 rad`).

## 5. Set the correct control law variant for the case

`multi_unicycle_controller_double_angle_experiment_ekf.py` hardcodes
**one** active control law at a time, in the `# 3. Choose your Control Law`
section of `control_loop()`. Before running, edit it so the correct line
is the only one **uncommented**, per **Appendix A**'s table:

```python
# VARIANT 1: Connectivity Preservation Only
d_dt = - (self.Hbar_Nset.T @ R_psi_N @ xi)

# VARIANT 2: Connectivity Preservation + Collision Avoidance
# d_dt = - (self.Hbar_Nset.T @ R_psi_N @ xi + self.Hbar_Mset.T @ R_psi_M @ xi_beta)

# Variant 3
# d_dt = -10000000 * (self.Hbar_Nset.T @ R_psi_N @ E)
```

This is the single most common mistake to make when switching cases —
double-check it every time.

## 6. Run the formation controller

```bash
python3.12 experiment/multi_unicycle_controller_double_angle_experiment_ekf.py
```

What it does automatically:
- Waits for all 4 robots' markers to be seen at least once.
- Starts MP4 recording (RealSense color feed) on the first nonzero
  velocity command — saved to
  `~/Downloads/ros2_ws/experiment/recordings/run_<timestamp>.mp4`.
- Logs trajectory/distance data at 10 Hz.
- **Stops automatically** once all 5 edge errors are within
  `convergence_tolerance = 0.080 m` (resumes control if it later drifts
  back out of tolerance).
- `Ctrl+C` stops it early at any time — it still saves whatever was
  recorded.

On shutdown it writes to `~/turtlebot3_simulations/plots/<timestamp>/`:
- `trajectory.png`, `distances.png`
- `log_data.csv` (same schema as the real `log_data.csv` files under
  `APS Report/Figures/<Case>/` — `time, bot_N_x/y/theta, d1..d5, e1..e5,
  lower_bound, upper_bound`)

## 7. After the run

Move/copy the new `log_data.csv` into `APS Report/Figures/<Case>/` if
this is meant to become the new reference run for that case, and
regenerate the standard plots with the reusable script:
```bash
python3 "Claude testing/algo testing/plot_case_from_csv.py" "APS Report/Figures/<Case>"
```

---

## Appendix A: Per-case reference (law variant + initial conditions)

All four cases share the same `pstar`, graph edges, and physical
parameters (`r=0.135, Rs=0.16, Rc=1.6, Rcrit=0.4, v_max=0.055,
omega_max=0.13, length=0.25, conv_tol=0.080`, control loop at 50 Hz) —
only the **control law variant** and **initial conditions** change.

| Case | Control law variant | Collision avoidance |
|---|---|---|
| Case-1b | **Variant 3** (raw-E-driven, gain 10,000,000) | off |
| Case-2  | **Variant 1** (connectivity only, no gain) | off |
| Case-3  | **Variant 1** (connectivity only, no gain) | off |
| Case-4  | **Variant 2** (connectivity + CA, no gain) | on |

Initial conditions (x, y in metres, yaw in radians, world/`ground_origin`
frame) — from each case's real `log_data.csv` first row:

**Case-1b**
```
bot_0: x= 0.6705, y= 0.1398, yaw= 1.4427
bot_1: x=-0.6370, y= 0.1786, yaw= 1.6645
bot_2: x=-0.6128, y=-0.1347, yaw=-1.2970
bot_3: x= 0.6413, y=-0.1755, yaw=-1.4689
```

**Case-2**
```
bot_0: x= 0.6530, y= 0.1652, yaw= 1.4560
bot_1: x=-0.6480, y= 0.1707, yaw= 1.6719
bot_2: x=-0.6442, y=-0.1747, yaw=-1.7312
bot_3: x= 0.6733, y=-0.1736, yaw=-1.7361
```

**Case-3 / Case-4** (same IC family; `move_to_initial_ekf.py`'s active
`TARGETS` block is a rounded version of this pair, closer to Case-4):
```
Case-3:
  bot_0: x=-0.2759, y=-0.0533, yaw= 1.3832
  bot_1: x=-0.6187, y= 0.4192, yaw=-0.6873
  bot_2: x= 0.1074, y=-0.2319, yaw= 1.9618
  bot_3: x=-0.3406, y=-0.3905, yaw=-1.8267

Case-4:
  bot_0: x=-0.3034, y=-0.0093, yaw= 0.6830
  bot_1: x=-0.6136, y= 0.4315, yaw=-0.9821
  bot_2: x= 0.1132, y=-0.2382, yaw= 2.1886
  bot_3: x=-0.3801, y=-0.3463, yaw=-2.1991
```

Case-3 is the real-collision case — expect it to end early (~12s) with
bot_0/bot_2 nearly colliding; don't wait for a convergence that won't
happen. Case-1b/2/4 took roughly 155s/181s/203s to converge historically.

---

## Appendix B: WiFi / netplan troubleshooting (robot network drops)

If a robot stops connecting after a router SSID/password change, the
issues found and fixed (on bot3, then confirmed as a recurring pattern)
were:

1. **Wrong band**: netplan pointed at a `_5G`-suffixed SSID
   (`"TP-Link_8782_5G"`) that wasn't reliably associable — the Raspberry
   Pi 4's onboard WiFi chip is **self-managed** (`iw reg get` shows
   `phy#0` reporting its own regulatory domain, e.g. `99`, separate from
   and overriding whatever global domain `iw reg set` configures), so
   standard `iw reg set <country>` fixes do **not** reliably unlock 5GHz
   DFS channels on this hardware. **Fix: use the 2.4GHz SSID**
   (`"TP-Link_8782"`, no suffix) instead — confirmed strong signal
   (-13 to -14 dBm).
2. **Static IP conflicting with the wired interface**: `wlan0` was
   hardcoded to the exact same static IP as `eth0`'s DHCP lease (e.g.
   both `.153`), which breaks ARP/routing for whichever interface is
   also up. **Fix: use `dhcp4: true` on `wlan0`** (no `addresses:`/
   `routes:` block) rather than a static IP, unless you have a specific
   reason to need one — this removes the whole class of conflict.
3. Diagnosing without `iw`/`arp-scan`/`nmap` installed: use
   `wpa_cli -i wlan0 status` (shows `wpa_state`: `SCANNING` = target
   SSID not found in range; `ASSOCIATING` = found it, trying to join;
   `COMPLETED` = connected) and `wpa_cli -i wlan0 scan` +
   `wpa_cli -i wlan0 scan_results` to list visible networks without
   installing anything.
4. If a robot's Ethernet port shows only the amber/power LED and never
   the green link LED **even with a cable confirmed working on another
   robot**, that points to a hardware fault on that specific board's
   port (or, if it follows a specific SD card across multiple boards
   instead, a boot-config issue on that card) — not a netplan problem.

## Appendix C: Known documentation gaps

These were not found anywhere in the repo and are reconstructed/guessed
in this guide — please correct them once confirmed:
- The exact `ros2 launch realsense2_camera ...` command and arguments
  actually used (resolution, FPS, whether depth is needed).
- The exact `aruco_tracker` launch overrides actually used historically
  (topic remap, compressed-image flag) — the ones in section 2 are a
  best-effort reconstruction from `aruco_tracker.yaml`'s defaults vs.
  what the controller scripts expect.
