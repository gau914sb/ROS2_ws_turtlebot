#!/usr/bin/env python3
"""
stop_all_bots.py  —  Emergency stop for all TurtleBot3 robots
==============================================================
Step 1: Kill any running controller processes so they stop
        overriding velocity commands (controller runs at 50 Hz).
Step 2: Publish zero-velocity TwistStamped to every cmd_vel topic
        for 2 seconds at 20 Hz — bots will receive and hold the stop.

Usage
-----
  python3 experiment/stop_all_bots.py
"""

import os
import signal
import subprocess
import sys
import time

# ── Force ROS_DOMAIN_ID=30 regardless of shell environment ───────────────────
os.environ['ROS_DOMAIN_ID'] = '30'

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

# ── Topics to stop ────────────────────────────────────────────────────────────
CMD_VEL_TOPICS = [
    '/bot_0/cmd_vel',
    '/bot_1/cmd_vel',
    '/bot_2/cmd_vel',
    '/bot_3/cmd_vel',
]

# Controller script substrings to kill (matched against process cmdline)
CONTROLLER_PATTERNS = [
    'multi_unicycle_controller',
    'goto_pose_aruco',
    'closed_loop_aruco',
    'circumnavigation_controller',
]

# Zero-vel flood parameters
FLOOD_DURATION_S  = 2.0   # seconds to keep publishing after kill
FLOOD_RATE_HZ     = 20    # messages per second per topic


def kill_controllers() -> bool:
    """
    Send SIGINT to every running controller process.
    Returns True if any process was killed.
    """
    self_pid = os.getpid()
    killed_any = False

    for pattern in CONTROLLER_PATTERNS:
        try:
            result = subprocess.run(
                ['pgrep', '-f', pattern],
                capture_output=True, text=True
            )
            pids = [int(p) for p in result.stdout.split() if p.strip()]
            pids = [p for p in pids if p != self_pid]

            if not pids:
                continue

            for pid in pids:
                try:
                    print(f'    Killing PID {pid}  ({pattern})')
                    os.kill(pid, signal.SIGINT)
                    killed_any = True
                except ProcessLookupError:
                    pass

        except FileNotFoundError:
            # pgrep not available
            pass

    if killed_any:
        print('    Waiting 1.5s for graceful shutdown…')
        time.sleep(1.5)

        # Force-kill any survivors
        for pattern in CONTROLLER_PATTERNS:
            try:
                result = subprocess.run(
                    ['pgrep', '-f', pattern],
                    capture_output=True, text=True
                )
                pids = [int(p) for p in result.stdout.split() if p.strip()]
                pids = [p for p in pids if p != self_pid]
                for pid in pids:
                    try:
                        print(f'    Force-killing PID {pid}  ({pattern})')
                        os.kill(pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
            except FileNotFoundError:
                pass

    return killed_any


def main() -> None:
    print('\n╔══════════════════════════════════════════════════╗')
    print('║        EMERGENCY STOP — ALL BOTS                ║')
    print('╚══════════════════════════════════════════════════╝')
    print(f'  ROS_DOMAIN_ID : {os.environ["ROS_DOMAIN_ID"]}')
    print(f'  Topics        : {", ".join(CMD_VEL_TOPICS)}\n')

    # ── Step 1: Kill controller processes ────────────────────────────────────
    print('  [1/2] Killing controller processes…')
    killed = kill_controllers()
    if not killed:
        print('        No controller processes found (or already stopped).')

    # ── Step 2: Flood zero-velocity for FLOOD_DURATION_S seconds ─────────────
    print(f'\n  [2/2] Flooding zero-velocity at {FLOOD_RATE_HZ} Hz '
          f'for {FLOOD_DURATION_S:.0f}s…')

    rclpy.init()
    node = Node('stop_all_bots')

    publishers = {
        topic: node.create_publisher(TwistStamped, topic, 10)
        for topic in CMD_VEL_TOPICS
    }

    # Short settle — give publishers time to connect to any remaining subscribers
    time.sleep(0.2)

    zero_msg = TwistStamped()
    zero_msg.header.frame_id = 'base_link'
    # All velocity fields default to 0.0 — nothing else needed

    interval = 1.0 / FLOOD_RATE_HZ
    total_bursts = int(FLOOD_DURATION_S * FLOOD_RATE_HZ)

    for i in range(total_bursts):
        zero_msg.header.stamp = node.get_clock().now().to_msg()
        for pub in publishers.values():
            pub.publish(zero_msg)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(interval)

    print('\n  ✓ All bots commanded to STOP.')
    print('  ✓ Done.\n')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
