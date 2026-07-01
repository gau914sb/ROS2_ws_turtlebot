#!/bin/bash
# =============================================================================
# TurtleBot3 Swarm Migration Script: Humble/Gazebo-Classic → Jazzy/Harmonic
# Run from: ~/Downloads/ros2_ws/turtlebot/
# =============================================================================
set -e

WS="$HOME/Downloads/ros2_ws/turtlebot"
SRC="$WS/src"
OLD_WS="$HOME/Downloads/ros2_ws/src"

echo ""
echo "╔══════════════════════════════════════════════════════════════════╗"
echo "║  TurtleBot3 Migration: Jazzy + Gazebo Harmonic                  ║"
echo "╚══════════════════════════════════════════════════════════════════╝"
echo ""

# ── Step 0: Prerequisites ─────────────────────────────────────────────────────
echo "▶ [0/6] Installing prerequisites..."
sudo apt install -y git python3-colcon-common-extensions 2>&1 | grep -E "^(Setting|Unpacking|Get:|Err:)" || true

# ── Step 1: Clean old build artifacts ─────────────────────────────────────────
echo ""
echo "▶ [1/6] Cleaning old build/install/log..."
rm -rf "$WS/build" "$WS/install" "$WS/log"
echo "  ✅ Cleaned."

# ── Step 2: Clone fresh Jazzy-compatible repos ────────────────────────────────
echo ""
echo "▶ [2/6] Cloning fresh repos (Jazzy-compatible main branches)..."
rm -rf "$SRC"
mkdir -p "$SRC"
cd "$SRC"

git clone -b main --depth 1 https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b main --depth 1 https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b main --depth 1 https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
git clone -b main --depth 1 https://github.com/ROBOTIS-GIT/DynamixelSDK.git

echo "  ✅ Repos cloned:"
ls "$SRC"

# ── Step 3: Restore custom controller script ─────────────────────────────────
echo ""
echo "▶ [3/6] Restoring custom algorithm files..."
GAZEBO_DIR="$SRC/turtlebot3_simulations/turtlebot3_gazebo"
SCRIPTS_DIR="$GAZEBO_DIR/scripts"

# Restore multi_unicycle_controller_color.py from old workspace
if [ -f "$OLD_WS/turtlebot3_simulations/turtlebot3_gazebo/scripts/multi_unicycle_controller_color.py" ]; then
    cp "$OLD_WS/turtlebot3_simulations/turtlebot3_gazebo/scripts/multi_unicycle_controller_color.py" \
       "$SCRIPTS_DIR/"
    echo "  ✅ Restored: multi_unicycle_controller_color.py"
else
    echo "  ⚠️  multi_unicycle_controller_color.py not found in old workspace - will be written by migration script"
fi

# Fix hardcoded plot path in controller (~/ros2_ws → ~/Downloads/ros2_ws/turtlebot)
if [ -f "$SCRIPTS_DIR/multi_unicycle_controller_color.py" ]; then
    sed -i "s|~/ros2_ws/plots|~/Downloads/ros2_ws/turtlebot/plots|g" \
        "$SCRIPTS_DIR/multi_unicycle_controller_color.py"
    echo "  ✅ Fixed plot save path in controller."
fi

echo "  ✅ Custom files restored."

# ── Step 4: Apply Jazzy/Harmonic patches ─────────────────────────────────────
echo ""
echo "▶ [4/6] Applying Jazzy/Harmonic patches (CMakeLists, package.xml, SDF, launch)..."
echo "  (These files are written by the Python patcher - running now...)"

python3 "$WS/apply_jazzy_patches.py"

echo "  ✅ Patches applied."

# ── Step 5: Install ROS dependencies ─────────────────────────────────────────
echo ""
echo "▶ [5/6] Installing ROS2 Jazzy dependencies..."
source /opt/ros/jazzy/setup.bash

# Install missing deps
sudo apt install -y \
    ros-jazzy-turtlebot3-msgs \
    ros-jazzy-dynamixel-sdk \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-image \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-xacro \
    ros-jazzy-rviz2 \
    python3-opencv \
    python3-scipy \
    python3-matplotlib \
    2>&1 | grep -E "^(Setting|Unpacking|Get:|Err:|ros-jazzy)" || true

echo "  ✅ Dependencies installed."

# ── Step 6: Build ─────────────────────────────────────────────────────────────
echo ""
echo "▶ [6/6] Building workspace..."
cd "$WS"
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select turtlebot3_gazebo 2>&1 | tee /tmp/colcon_build.log

echo ""
echo "╔══════════════════════════════════════════════════════════════════╗"
echo "║  ✅ Migration complete!                                          ║"
echo "║                                                                  ║"
echo "║  To launch (Terminal 1):                                         ║"
echo "║    source ~/Downloads/ros2_ws/turtlebot/install/setup.bash      ║"
echo "║    export TURTLEBOT3_MODEL=burger                                ║"
echo "║    ros2 launch turtlebot3_gazebo swarm_color_detection.launch.py ║"
echo "║                                                                  ║"
echo "║  To run controller (Terminal 2, after Gazebo is up):             ║"
echo "║    source ~/Downloads/ros2_ws/turtlebot/install/setup.bash      ║"
echo "║    python3 ~/Downloads/ros2_ws/turtlebot/src/turtlebot3_simulations/turtlebot3_gazebo/scripts/multi_unicycle_controller_color.py ║"
echo "╚══════════════════════════════════════════════════════════════════╝"
