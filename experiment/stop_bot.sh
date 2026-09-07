#!/bin/bash
# stop_bot.sh  —  Emergency stop for ONE TurtleBot3 robot
# =========================================================
# 1. Kills any process specifically targeting this bot's cmd_vel topic
#    (e.g. a per-robot teleop_twist_keyboard remapped to it).
# 2. Floods that bot's cmd_vel topic with zero-velocity for 1 second.
#
# IMPORTANT: this does NOT kill a shared multi-robot controller (e.g.
# multi_unicycle_controller_double_angle_experiment_ekf.py or
# goto_pose_aruco_4bots.py) — those drive all 4 bots from one process, so
# killing them here would stop every robot, not just this one. If one of
# those is running, it will overwrite this stop on its next control tick
# regardless of the zero-velocity flood below; the script warns you if it
# detects one. Use stop_all_bots.sh if you actually want everything stopped.
#
# Usage:
#   bash experiment/stop_bot.sh <bot_name_or_index>
#   e.g. bash experiment/stop_bot.sh bot_2
#        bash experiment/stop_bot.sh 2        # same as bot_2
#   # or after chmod +x:
#   ./experiment/stop_bot.sh bot_2

source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=30

if [ -z "$1" ]; then
    echo "Usage: $0 <bot_name_or_index>"
    echo "  e.g. $0 bot_2   or   $0 2"
    exit 1
fi

# Accept either "bot_2" or a bare index "2"
ARG="$1"
if [[ "$ARG" =~ ^[0-9]+$ ]]; then
    BOT="bot_${ARG}"
else
    BOT="$ARG"
fi

TOPIC="/${BOT}/cmd_vel"
ZERO_TWIST="{header: {frame_id: 'base_link'}, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"

# Shared multi-robot controllers that drive ALL bots from one process —
# killing these would stop everything, so we only warn, never kill them here.
SHARED_PATTERNS=(
    "multi_unicycle_controller"
    "goto_pose_aruco"
    "move_to_initial"
)

echo ""
echo "╔══════════════════════════════════════════════════╗"
echo "║        EMERGENCY STOP — ${BOT}"
echo "╚══════════════════════════════════════════════════╝"
echo "  ROS_DOMAIN_ID : ${ROS_DOMAIN_ID}"
echo "  Topic         : ${TOPIC}"
echo ""

# ── Step 1: Kill any process specifically targeting this bot's topic ───────
echo "  [1/2] Killing any process targeting ${TOPIC} specifically..."
SELF_PID=$$
PIDS=$(pgrep -f "${TOPIC}" | grep -v "^${SELF_PID}$" || true)
if [ -n "$PIDS" ]; then
    echo "        Killing: PID(s) $PIDS"
    kill -SIGINT $PIDS 2>/dev/null || true
    sleep 1
    PIDS=$(pgrep -f "${TOPIC}" | grep -v "^${SELF_PID}$" || true)
    if [ -n "$PIDS" ]; then
        echo "        Force-killing: PID(s) $PIDS"
        kill -SIGKILL $PIDS 2>/dev/null || true
    fi
else
    echo "        No process found targeting ${TOPIC} specifically."
fi

# Warn (don't kill) if a shared multi-robot controller looks like it's running
for pattern in "${SHARED_PATTERNS[@]}"; do
    if pgrep -f "$pattern" > /dev/null 2>&1; then
        echo ""
        echo "  ⚠  A shared multi-robot process ('$pattern') appears to be running."
        echo "     It commands all 4 bots from one process and will likely override"
        echo "     this stop within ~20ms. Use stop_all_bots.sh to stop everything,"
        echo "     or Ctrl+C that process if you need ${BOT} to actually stay stopped."
        echo ""
    fi
done

# ── Step 2: Flood this bot's cmd_vel with zero-velocity ─────────────────────
echo "  [2/2] Sending zero-velocity to ${TOPIC} (1 second flood)..."
ros2 topic pub --rate 20 --times 20 "$TOPIC" \
    geometry_msgs/msg/TwistStamped "$ZERO_TWIST" > /dev/null 2>&1

echo ""
echo "  ✓ ${BOT} commanded to STOP."
echo ""
