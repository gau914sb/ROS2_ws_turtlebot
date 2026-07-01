#!/usr/bin/env python3.12
"""
Move Robots to Initial Formation Positions  —  ArUco Feedback
==============================================================
Run this script BEFORE the formation controller to automatically
place all robots at the desired starting positions defined in TARGETS.

FEEDBACK : overhead camera → aruco_opencv → TF2  (ground_origin → marker_N)
COMMANDS : TwistStamped on /<robot>/cmd_vel

How it works
------------
  Phase 1 — ROTATE : Turn in-place to face the target position.
  Phase 2 — DRIVE  : Drive forward while correcting heading.
  Phase 3 — ROTATE : Turn in-place to reach the desired final heading.
  A robot is "done" when it is within POS_TOL (m) of its target and
  within HEADING_TOL (rad) of its desired heading.

Usage
-----
  python3.12 experiment/move_to_initial.py

  All targets are defined in the TARGETS dict below — edit them to match
  your desired pstar and initial orientations.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from geometry_msgs.msg import TwistStamped
from aruco_opencv_msgs.msg import ArucoDetection
from typing import Dict

# =============================================================================
# CONFIGURATION — edit these to match your setup
# =============================================================================

# Robot-to-marker mapping  (same as formation controller)
ROBOT_MARKER_MAP: Dict[str, int] = {
    'bot_0': 0,
    'bot_1': 1,
    'bot_2': 2,
    'bot_3': 3,
}

# Target positions AND headings for each robot.
# These come from pstar in the formation controller:
#   pstar = [0.2, 0.2,  1.6, 0.2,  2.4, 2.4,  0.2, 1.6]
#   bot_0: (0.2, 0.2)   bot_1: (1.6, 0.2)
#   bot_2: (2.4, 2.4)   bot_3: (0.2, 1.6)
#
# heading = desired final orientation in radians (0 = facing +x).
# Set to None if you don't care about final heading.
TARGETS: Dict[str, dict] = {
    'bot_0': {'x': 0.2,  'y': 0.2,  'heading': 0.0},
    'bot_1': {'x': 1.6,  'y': 0.2,  'heading': 0.0},
    'bot_2': {'x': 2.4,  'y': 2.4,  'heading': 0.0},
    'bot_3': {'x': 0.2,  'y': 1.6,  'heading': 0.0},
}

# Control gains and tolerances
K_LINEAR    = 0.5    # Proportional gain for linear velocity
K_ANGULAR   = 2.0    # Proportional gain for angular velocity
V_MAX       = 0.15   # Max linear velocity (m/s)  — keep slow for safety
OMEGA_MAX   = 1.5    # Max angular velocity (rad/s)
POS_TOL     = 0.03   # Position tolerance (m)  — "close enough"
HEADING_TOL = 0.10   # Heading tolerance (rad) — ~6 degrees

# TF / Topic config
WORLD_FRAME  = 'ground_origin'
ARUCO_TOPIC  = '/aruco_detections'
TF_TIMEOUT   = 0.05


# =============================================================================
# Helper
# =============================================================================

def normalize_angle(a: float) -> float:
    """Wrap angle to [-π, π]."""
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def quat_to_yaw(qx, qy, qz, qw) -> float:
    """Quaternion → yaw (rad)."""
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny, cosy)


# =============================================================================
# Node
# =============================================================================

class MoveToInitial(Node):
    """
    Drives each robot to its target (x, y, heading) using proportional control.

    States per robot:
      'rotate_to_target' — turn in place to face the target position
      'drive'            — drive forward while correcting heading
      'rotate_to_heading'— turn in place to the final desired heading
      'done'             — reached goal, velocity = 0
    """

    def __init__(self):
        super().__init__('move_to_initial')

        # ── Per-robot state ───────────────────────────────────────────────
        self.x:     Dict[str, float] = {}   # current x from TF
        self.y:     Dict[str, float] = {}   # current y from TF
        self.theta: Dict[str, float] = {}   # current yaw from TF
        self.seen:  Dict[str, bool]  = {}   # True after first TF lookup
        self.phase: Dict[str, str]   = {}   # finite state machine phase

        for name in ROBOT_MARKER_MAP:
            self.x[name]     = 0.0
            self.y[name]     = 0.0
            self.theta[name] = 0.0
            self.seen[name]  = False
            self.phase[name] = 'rotate_to_target'

        # ── TF2 ──────────────────────────────────────────────────────────
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── Publishers ────────────────────────────────────────────────────
        cmd_qos = QoSProfile(depth=10)
        self.pubs: Dict[str, any] = {
            name: self.create_publisher(TwistStamped, f'/{name}/cmd_vel', cmd_qos)
            for name in ROBOT_MARKER_MAP
        }

        # ── ArUco subscriber ─────────────────────────────────────────────
        aruco_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(ArucoDetection, ARUCO_TOPIC,
                                 self._aruco_cb, aruco_qos)

        # ── Control timer (20 Hz) ────────────────────────────────────────
        self.create_timer(0.05, self._control_loop)

        self.get_logger().info(
            '\n[MoveToInitial] Started.\n'
            '  Targets:\n' +
            '\n'.join(
                f'    {n}: ({t["x"]:.2f}, {t["y"]:.2f})  heading={math.degrees(t["heading"]):.0f}°'
                for n, t in TARGETS.items()
            ) +
            f'\n  Tolerances: pos={POS_TOL} m, heading={math.degrees(HEADING_TOL):.1f}°\n'
            f'  Waiting for all markers…'
        )

    # ── ArUco callback → TF lookups ───────────────────────────────────────

    def _aruco_cb(self, msg: ArucoDetection):
        """On every detection frame, look up TF for each visible marker."""
        visible = {m.marker_id for m in msg.markers}

        for name, mid in ROBOT_MARKER_MAP.items():
            if mid not in visible:
                continue
            try:
                tf = self.tf_buffer.lookup_transform(
                    WORLD_FRAME, f'marker_{mid}',
                    rclpy.time.Time(), Duration(seconds=TF_TIMEOUT),
                )
            except (LookupException, ConnectivityException, ExtrapolationException):
                continue

            self.x[name]     = tf.transform.translation.x
            self.y[name]     = tf.transform.translation.y
            q = tf.transform.rotation
            self.theta[name] = quat_to_yaw(q.x, q.y, q.z, q.w)
            self.seen[name]  = True

    # ── Control loop ──────────────────────────────────────────────────────

    def _control_loop(self):
        """20 Hz proportional controller for each robot."""

        all_done = True

        for name in ROBOT_MARKER_MAP:
            if not self.seen[name]:
                all_done = False
                continue

            if self.phase[name] == 'done':
                continue

            all_done = False

            tgt = TARGETS[name]
            dx  = tgt['x'] - self.x[name]
            dy  = tgt['y'] - self.y[name]
            dist = math.sqrt(dx * dx + dy * dy)
            angle_to_target = math.atan2(dy, dx)
            heading_err_to_target = normalize_angle(angle_to_target - self.theta[name])

            # ── Phase: ROTATE_TO_TARGET ───────────────────────────────
            if self.phase[name] == 'rotate_to_target':
                if dist < POS_TOL:
                    # Already at target — skip to heading phase
                    self.phase[name] = 'rotate_to_heading'
                    self._cmd(name, 0.0, 0.0)
                    continue

                if abs(heading_err_to_target) < 0.15:  # ~8.6 degrees
                    self.phase[name] = 'drive'
                    self.get_logger().info(f'{name}: aligned → driving to target')
                else:
                    omega = max(min(K_ANGULAR * heading_err_to_target, OMEGA_MAX), -OMEGA_MAX)
                    self._cmd(name, 0.0, omega)

            # ── Phase: DRIVE ──────────────────────────────────────────
            elif self.phase[name] == 'drive':
                if dist < POS_TOL:
                    self.phase[name] = 'rotate_to_heading'
                    self._cmd(name, 0.0, 0.0)
                    self.get_logger().info(
                        f'{name}: position reached ({self.x[name]:.3f}, {self.y[name]:.3f})'
                    )
                    continue

                # If heading drifts too far, go back to rotate phase
                if abs(heading_err_to_target) > 0.8:  # ~45 degrees
                    self.phase[name] = 'rotate_to_target'
                    self._cmd(name, 0.0, 0.0)
                    continue

                v     = min(K_LINEAR * dist, V_MAX)
                omega = max(min(K_ANGULAR * heading_err_to_target, OMEGA_MAX), -OMEGA_MAX)
                self._cmd(name, v, omega)

            # ── Phase: ROTATE_TO_HEADING ──────────────────────────────
            elif self.phase[name] == 'rotate_to_heading':
                desired_heading = tgt.get('heading')
                if desired_heading is None:
                    self.phase[name] = 'done'
                    self._cmd(name, 0.0, 0.0)
                    self.get_logger().info(f'{name}: ✓ DONE (no heading target)')
                    continue

                heading_err = normalize_angle(desired_heading - self.theta[name])
                if abs(heading_err) < HEADING_TOL:
                    self.phase[name] = 'done'
                    self._cmd(name, 0.0, 0.0)
                    self.get_logger().info(
                        f'{name}: ✓ DONE at ({self.x[name]:.3f}, {self.y[name]:.3f}) '
                        f'heading={math.degrees(self.theta[name]):.1f}°'
                    )
                else:
                    omega = max(min(K_ANGULAR * heading_err, OMEGA_MAX), -OMEGA_MAX)
                    self._cmd(name, 0.0, omega)

        # ── All robots done? ──────────────────────────────────────────
        if all_done and all(self.seen.values()):
            self.get_logger().info(
                '\n══════════════════════════════════════════════\n'
                '  ALL ROBOTS AT INITIAL POSITIONS — READY!\n'
                '  You can now start the formation controller.\n'
                '══════════════════════════════════════════════'
            )
            # Stop all and exit
            for name in ROBOT_MARKER_MAP:
                self._cmd(name, 0.0, 0.0)
            raise SystemExit(0)

    # ── Publish helper ────────────────────────────────────────────────────

    def _cmd(self, name: str, v: float, omega: float):
        """Publish a TwistStamped velocity command."""
        msg = TwistStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x  = float(v)
        msg.twist.angular.z = float(omega)
        self.pubs[name].publish(msg)


# =============================================================================
# Main
# =============================================================================

def main():
    rclpy.init()
    node = MoveToInitial()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        # Stop all robots on exit
        for name in ROBOT_MARKER_MAP:
            node._cmd(name, 0.0, 0.0)
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
