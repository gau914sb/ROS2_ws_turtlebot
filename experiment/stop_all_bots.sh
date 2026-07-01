#!/bin/bash
# stop_all_bots.sh  —  Emergency stop for all TurtleBot3 robots
# ==============================================================
# 1. Kills any running controller scripts so they stop overriding
#    velocity commands.
# 2. Floods all cmd_vel topics with zero-velocity for 1 second
#    to ensure every bot receives the stop command.
#
# Usage:
#   bash experiment/stop_all_bots.sh
#   # or after chmod +x:
#   ./experiment/stop_all_bots.sh

source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=30

ZERO_TWIST="{header: {frame_id: 'base_link'}, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"
TOPICS=("/bot_0/cmd_vel" "/bot_1/cmd_vel" "/bot_2/cmd_vel" "/bot_3/cmd_vel")

# Controller script names to kill
CONTROLLER_PATTERNS=(
    "multi_unicycle_controller"
    "goto_pose_aruco"
    "stop_all_bots"   # exclude self below
)

echo ""
echo "╔══════════════════════════════════════════════════╗"
echo "║        EMERGENCY STOP — ALL BOTS                ║"
echo "╚══════════════════════════════════════════════════╝"
echo "  ROS_DOMAIN_ID : ${ROS_DOMAIN_ID}"
echo ""

# ── Step 1: Kill running controller processes ──────────────────────────────
echo "  [1/2] Killing controller processes..."
SELF_PID=$$
KILLED_ANY=false
for pattern in "${CONTROLLER_PATTERNS[@]}"; do
    # Find PIDs matching the pattern, excluding this script's own PID
    PIDS=$(pgrep -f "$pattern" | grep -v "^${SELF_PID}$" || true)
    if [ -n "$PIDS" ]; then
        echo "        Killing: $pattern (PID: $PIDS)"
        kill -SIGINT $PIDS 2>/dev/null || true
        KILLED_ANY=true
    fi
done

if [ "$KILLED_ANY" = true ]; then
    echo "        Waiting 1s for graceful shutdown..."
    sleep 1
    # Force-kill anything still running
    for pattern in "${CONTROLLER_PATTERNS[@]}"; do
        PIDS=$(pgrep -f "$pattern" | grep -v "^${SELF_PID}$" || true)
        if [ -n "$PIDS" ]; then
            echo "        Force-killing: $pattern (PID: $PIDS)"
            kill -SIGKILL $PIDS 2>/dev/null || true
        fi
    done
else
    echo "        No controller processes found."
fi

# ── Step 2: Flood all cmd_vel topics with zero-velocity ───────────────────
echo ""
echo "  [2/2] Sending zero-velocity to all bots (1 second flood)..."

# Launch background topic-pub processes that publish at 20 Hz for 1 second
for topic in "${TOPICS[@]}"; do
    ros2 topic pub --rate 20 --times 20 "$topic" \
        geometry_msgs/msg/TwistStamped "$ZERO_TWIST" > /dev/null 2>&1 &
done

# Wait for all pubs to complete (~1 second)
wait

echo ""
echo "  ✓ All bots commanded to STOP."
echo ""
