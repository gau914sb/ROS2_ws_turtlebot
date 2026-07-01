# TurtleBot3 Swarm Simulation — Migration Status
**Date:** 2026-06-07  
**Migration:** ROS 2 Humble + Gazebo Classic → ROS 2 Jazzy + Gazebo Harmonic

---

## System Configuration

| Item | Value |
|---|---|
| OS | Ubuntu 24.04.4 |
| ROS | ROS 2 Jazzy |
| Simulator | Gazebo Harmonic (gz-sim 8) |
| Workspace | `~/Downloads/ros2_ws/turtlebot/` |
| Robot Model | TurtleBot3 Burger × 4 |

---

## Quick Start (Tomorrow)

### Terminal 1 — Launch Gazebo + 4 Robots
```bash
source ~/Downloads/ros2_ws/turtlebot/install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo spawn_4_bots.launch.py
```
Wait ~10 seconds for all 4 `Entity creation successful` messages and bridges to start.

### Terminal 2 — Run Formation Controller
```bash
source /opt/ros/jazzy/setup.bash
source ~/Downloads/ros2_ws/turtlebot/install/setup.bash
python3 ~/Downloads/ros2_ws/turtlebot/src/turtlebot3_simulations/turtlebot3_gazebo/scripts/multi_unicycle_controller_color.py
```

---

## What Is Working ✅

- **Gazebo Harmonic** launches with 4 TurtleBot3 Burger robots at correct spawn positions
- **Per-robot SDF generation** — launch file generates unique SDFs per robot with absolute gz topic names (`/bot_0/odom`, `/bot_0/cmd_vel`, etc.) to avoid Gazebo topic-scoping ambiguity
- **ROS-Gz bridge** — `ros_gz_bridge` correctly forwards:
  - `/bot_X/odom` (GZ → ROS, `nav_msgs/Odometry`)
  - `/bot_X/cmd_vel` (ROS → GZ, `geometry_msgs/Twist`)
  - `/bot_X/tf`, `/bot_X/joint_states`
- **Odometry data flows** — confirmed via `gz topic -e -t /bot_0/odom`
- **Controller initializes** — detects all 4 robots and receives odometry within ~1 second
- **Robots move** — cmd_vel commands are confirmed reaching Gazebo via bridge log

---

## Key Architecture Decisions

### Multi-Robot Launch Strategy
The launch file (`spawn_4_bots.launch.py`) generates per-robot SDF files on the fly:
```python
# make_robot_sdf() replaces relative topic names with absolute ones:
# <topic>cmd_vel</topic>  →  <topic>/bot_0/cmd_vel</topic>
# <odom_topic>odom</odom_topic>  →  <odom_topic>/bot_0/odom</odom_topic>
```
Temp SDFs are written to `/tmp/tb3_*_bot_X.sdf` at launch time.

### Robot Spawn Positions
```
bot_0: (-1.0, -1.0)  yaw=0.00 rad
bot_1: ( 1.0, -1.0)  yaw=1.57 rad (90°)
bot_2: ( 1.0,  1.0)  yaw=3.14 rad (180°)
bot_3: (-1.0,  1.0)  yaw=-1.57 rad (-90°)
```

### Bridge Configuration
Each robot has its own YAML bridge config at:
`~/Downloads/ros2_ws/turtlebot/src/turtlebot3_simulations/turtlebot3_gazebo/params/bot_X_bridge.yaml`

QoS: odom subscribed with `BEST_EFFORT` (matches `ros_gz_bridge` publisher policy).

---

## What Is NOT Working / Needs Investigation ⚠️

### Formation Control Not Converging

**Symptom:** Distance errors (e1–e5) do not go to zero; robots either stabilize at wrong formation or errors grow.

**Root Cause Hypothesis:** Coordinate frame mismatch for robot yaw.

Gazebo Harmonic's DiffDrive odom reports theta, but it is **unclear** whether:
- **Option A:** theta is absolute world yaw (e.g., bot_1 spawned at 90° shows `odom_theta = 1.57`)
- **Option B:** theta is relative to spawn yaw (bot_1 always shows `odom_theta = 0.0` at rest)

**Diagnostic Test (do this first tomorrow):**
Check the controller init log after launching:
```
bot_0: Pos(-1.00, -1.00,   0.0°)  ← always correct
bot_1: Pos( 1.00, -1.00,  ???°)   ← key question
```

| bot_1 shows | Meaning | Action |
|---|---|---|
| `90.0°` | Odom reports RELATIVE theta (offset needed) | Current code is correct |
| `0.0°` | Odom reports ABSOLUTE world theta (no offset needed) | Remove spawn yaw offset |

### Controller File Location
```
~/Downloads/ros2_ws/turtlebot/src/turtlebot3_simulations/turtlebot3_gazebo/scripts/multi_unicycle_controller_color.py
```

### What Changed vs Original Controller

| Aspect | Original (Humble) | Current (Jazzy) |
|---|---|---|
| Position feedback | Overhead color camera (`PoseStamped`) | Odometry (`nav_msgs/Odometry`) |
| Topic | `/{name}/pose` | `/{name}/odom` |
| World frame | Camera gave absolute X,Y directly | Odom starts at (0,0) at spawn → spawn offset added |
| Yaw | Estimated from velocity direction | From odom quaternion ± spawn yaw |
| QoS | Default RELIABLE | BEST_EFFORT (matches bridge) |
| **Algorithm** | **Unchanged** | **Unchanged** |

> [!NOTE]
> The formation control algorithm (H matrix, pstar, Kc, Ks, barrier function) is **100% identical** to the original. Only the sensor interface was changed.

---

## File Map

```
~/Downloads/ros2_ws/turtlebot/
├── src/turtlebot3_simulations/turtlebot3_gazebo/
│   ├── launch/
│   │   └── spawn_4_bots.launch.py        ← main launch (per-robot SDF generation)
│   ├── params/
│   │   ├── bot_0_bridge.yaml             ← ROS-Gz bridge config per robot
│   │   ├── bot_1_bridge.yaml
│   │   ├── bot_2_bridge.yaml
│   │   └── bot_3_bridge.yaml
│   ├── models/turtlebot3_burger/
│   │   └── model.sdf                     ← base SDF (modified at runtime per robot)
│   └── scripts/
│       └── multi_unicycle_controller_color.py  ← formation controller
└── plots/                                ← trajectory plots saved here
```

---

## Original Backup
```bash
/tmp/tb3_backup/scripts/multi_unicycle_controller_color.py
```
> [!WARNING]
> This is in `/tmp` and will be lost on reboot. Copy to a safe location if needed:
> ```bash
> cp /tmp/tb3_backup/scripts/multi_unicycle_controller_color.py \
>    ~/Downloads/ros2_ws/turtlebot/original_controller_backup.py
> ```

---

## Useful Debug Commands

```bash
# Check what Gazebo topics exist
gz topic -l

# Verify odom is publishing (should show messages)
gz topic -e -t /bot_0/odom --duration 2

# Check ROS odom rate
ros2 topic hz /bot_0/odom

# Manually send cmd_vel to test bot movement
gz topic -t /bot_0/cmd_vel -m gz.msgs.Twist -p "linear: {x: 0.1}"

# Kill everything cleanly
pkill -f "gz sim" || true
pkill -f "ruby.*gz" || true
pkill -f parameter_bridge || true
```
