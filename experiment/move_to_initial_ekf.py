#!/usr/bin/env python3.12
"""
Move Robots to Initial Formation Positions  —  ArUco Feedback + EKF
=====================================================================
Run this script BEFORE the formation controller to automatically
place all robots at the desired starting positions defined in TARGETS.

FEEDBACK : overhead camera → aruco_opencv → TF2  (ground_origin → marker_N)
           → per-robot EKF (same filter as multi_unicycle_controller_double_angle_experiment_ekf.py)
COMMANDS : TwistStamped on /<robot>/cmd_vel

How it works
------------
  Phase 1 — ROTATE : Turn in-place to face the target position.
  Phase 2 — DRIVE  : Drive forward while correcting heading.
  Phase 3 — ROTATE : Turn in-place to reach the desired final heading.
  A robot is "done" when it is within POS_TOL (m) of its target and
  within HEADING_TOL (rad) of its desired heading.

EKF
---
  Each robot's (x, y, θ) estimate is filtered by a 3-state EKF instead of
  being taken directly from the raw ArUco/TF measurement:
    - predict(dt) runs every control tick (keeps the estimate alive between
      camera frames) AND once per ArUco detection (using the actual elapsed
      time since the last EKF touch), propagated using the last commanded
      (v, ω).
    - update(z_x, z_y, z_theta) corrects with the new TF measurement, but
      gates out outlier spikes via a Mahalanobis-distance test.

Usage
-----
  python3.12 experiment/move_to_initial_ekf.py

  All targets are defined in the TARGETS dict below — edit them to match
  your desired pstar and initial orientations.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

import numpy as np
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from geometry_msgs.msg import TwistStamped
from aruco_opencv_msgs.msg import ArucoDetection
from typing import Dict, Optional

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
# TARGETS: Dict[str, dict] = {
#     'bot_0': {'x': 0.6,  'y': 0.6,  'heading': 0.0},
#     'bot_1': {'x': -0.6,  'y': 0.6,  'heading': 0.0},
#     'bot_2': {'x': -0.6,  'y': -0.6,  'heading': 0.0},
#     'bot_3': {'x': 0.6,  'y': -0.6,  'heading': 0.0},
# }


# This initial condition is for no constraints case
TARGETS: Dict[str, dict] = {
    'bot_0': {'x': -0.3,  'y': 0.0,  'heading': np.pi/4},
    'bot_1': {'x': -0.6,  'y': 0.4,  'heading': -np.pi/4},
    'bot_2': {'x': 0.1,  'y': -0.2,  'heading': 2*np.pi/3},
    'bot_3': {'x': -0.4,  'y': -0.4,  'heading': -2*np.pi/3},
}

# TARGETS: Dict[str, dict] = {
#     'bot_0': {'x': 0.65,  'y': 0.2,  'heading': np.pi/2},
#     'bot_1': {'x': -0.65,  'y': 0.2,  'heading': np.pi/2},
#     'bot_2': {'x': -0.65,  'y': -0.2,  'heading': 3*np.pi/2},
#     'bot_3': {'x': 0.65,  'y': -0.2,  'heading': 3*np.pi/2},
# }

# TARGETS: Dict[str, dict] = {
#     'bot_0': {'x': 0.35,  'y': 0.1,  'heading': 0.28},
#     'bot_1': {'x': -0.1,  'y': 0.35,  'heading': 1.85},
#     'bot_2': {'x': -0.35,  'y': -0.1,  'heading': -2.86},
#     'bot_3': {'x': 0.1,  'y': -0.35,  'heading': -1.30},
# }


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

# Control loop period — matches the 20 Hz timer below. Used as the dt for
# the "keep the EKF alive between camera frames" predict step.
CONTROL_DT   = 0.05

# =============================================================================
# EKF NOISE PARAMETERS  (same values as multi_unicycle_controller_double_angle_experiment_ekf.py)
# =============================================================================
# Q: process noise — how much we trust the kinematic model (smaller = trust model more)
# R: measurement noise — how much we trust the camera (larger = trust camera less = more smoothing)
# To actually filter camera jitter: Q << R
Q_XY    = 0.001   # kinematic model is fairly smooth between frames
Q_THETA = 0.002
R_XY    = 0.05    # camera has positional jitter (marker wobble etc.)
R_THETA = 0.1     # heading from ArUco is noisier than position
GATE_THRESHOLD = 11.34  # chi-squared at 99% for 3 DOF — rejects true outlier spikes only


class RobotEKF:
    def __init__(self, name: str, x0: float = 0.0, y0: float = 0.0, theta0: float = 0.0):
        self.name = name
        self.x_est = np.array([x0, y0, theta0], dtype=float)
        self.P = np.diag([1.0, 1.0, 0.5])
        self.Q = np.diag([Q_XY, Q_XY, Q_THETA])
        self.R = np.diag([R_XY, R_XY, R_THETA])
        self.last_v = 0.0
        self.last_omega = 0.0
        self.initialized = False

    def initialize(self, x: float, y: float, theta: float):
        self.x_est = np.array([x, y, theta], dtype=float)
        self.P = np.diag([0.01, 0.01, 0.01])
        self.initialized = True

    def predict(self, dt: float):
        if not self.initialized: return
        x, y, theta = self.x_est
        v, omega = self.last_v, self.last_omega
        self.x_est[0] = x + v * math.cos(theta) * dt
        self.x_est[1] = y + v * math.sin(theta) * dt
        self.x_est[2] = self._wrap(theta + omega * dt)
        F = np.array([
            [1.0, 0.0, -v * math.sin(theta) * dt],
            [0.0, 1.0,  v * math.cos(theta) * dt],
            [0.0, 0.0,  1.0               ],
        ])
        self.P = F @ self.P @ F.T + self.Q

    def update(self, z_x: float, z_y: float, z_theta: float) -> bool:
        if not self.initialized:
            self.initialize(z_x, z_y, z_theta)
            return True
        z = np.array([z_x, z_y, z_theta])
        H = np.eye(3)
        y_inn = z - H @ self.x_est
        y_inn[2] = self._wrap(y_inn[2])
        S = H @ self.P @ H.T + self.R
        maha_sq = float(y_inn @ np.linalg.inv(S) @ y_inn)
        if maha_sq > GATE_THRESHOLD:
            return False
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x_est = self.x_est + K @ y_inn
        self.x_est[2] = self._wrap(self.x_est[2])
        I_KH = np.eye(3) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ self.R @ K.T
        return True

    @property
    def position(self):
        return float(self.x_est[0]), float(self.x_est[1]), float(self.x_est[2])

    @staticmethod
    def _wrap(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))


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

    Position/heading feedback comes from a per-robot EKF (RobotEKF above),
    filtered from the raw ArUco/TF measurement rather than used directly.
    """

    def __init__(self):
        super().__init__('move_to_initial')

        # ── Per-robot state ───────────────────────────────────────────────
        self.x:     Dict[str, float] = {}   # current x — EKF posterior estimate
        self.y:     Dict[str, float] = {}   # current y — EKF posterior estimate
        self.theta: Dict[str, float] = {}   # current yaw — EKF posterior estimate
        self.seen:  Dict[str, bool]  = {}   # True after first TF lookup (EKF initialized)
        self.phase: Dict[str, str]   = {}   # finite state machine phase

        # ── Per-robot EKF ─────────────────────────────────────────────────
        self.ekfs: Dict[str, RobotEKF] = {}
        # Timestamp (ns) of the last EKF predict/update cycle for this robot,
        # used to compute actual_dt for the ArUco-triggered predict step.
        self.last_ekf_time: Dict[str, Optional[int]] = {}

        for name in ROBOT_MARKER_MAP:
            self.x[name]     = 0.0
            self.y[name]     = 0.0
            self.theta[name] = 0.0
            self.seen[name]  = False
            self.phase[name] = 'rotate_to_target'
            self.ekfs[name]  = RobotEKF(name=name)
            self.last_ekf_time[name] = None

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
        self.create_timer(CONTROL_DT, self._control_loop)

        self.get_logger().info(
            '\n[MoveToInitial] Started (EKF-filtered feedback).\n'
            '  Targets:\n' +
            '\n'.join(
                f'    {n}: ({t["x"]:.2f}, {t["y"]:.2f})  heading={math.degrees(t["heading"]):.0f}°'
                for n, t in TARGETS.items()
            ) +
            f'\n  Tolerances: pos={POS_TOL} m, heading={math.degrees(HEADING_TOL):.1f}°\n'
            f'  Waiting for all markers…'
        )

    # ── ArUco callback → TF lookups → EKF predict+update ──────────────────

    def _aruco_cb(self, msg: ArucoDetection):
        """
        On every detection frame, look up TF for each visible marker and run
        it through that robot's EKF (predict with actual elapsed time, then
        gated measurement update), mirroring
        multi_unicycle_controller_double_angle_experiment_ekf.py's
        _aruco_callback.
        """
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

            # Raw measurement from TF
            z_x = tf.transform.translation.x
            z_y = tf.transform.translation.y
            q = tf.transform.rotation
            z_theta = quat_to_yaw(q.x, q.y, q.z, q.w)

            # ── Compute actual elapsed time for this robot's EKF cycle ──
            now_ns = self.get_clock().now().nanoseconds
            if self.last_ekf_time[name] is not None:
                actual_dt = (now_ns - self.last_ekf_time[name]) / 1e9
                if actual_dt > 0.2 or actual_dt <= 0.0:
                    actual_dt = 0.05  # clamp to nominal if stale or invalid
            else:
                actual_dt = 0.05   # first frame: use nominal dt
            self.last_ekf_time[name] = now_ns

            ekf = self.ekfs[name]

            # ── EKF predict with real elapsed time (before measurement) ──
            ekf.predict(actual_dt)

            # ── EKF measurement update (includes Mahalanobis outlier gate) ──
            if not self.seen[name]:
                # First detection: initialise the EKF and mark robot as ready
                ekf.initialize(z_x, z_y, z_theta)
                self.seen[name] = True
            else:
                accepted = ekf.update(z_x, z_y, z_theta)
                if not accepted:
                    self.get_logger().warn(
                        f'[EKF] marker_{mid} measurement rejected '
                        f'(outlier / spike) — using prediction only',
                        throttle_duration_sec=1.0,
                    )

            # Expose EKF posterior as this robot's state
            self.x[name], self.y[name], self.theta[name] = ekf.position

    # ── Control loop ──────────────────────────────────────────────────────

    def _control_loop(self):
        """
        20 Hz proportional controller for each robot.

        EKF predict step runs first, every cycle, for all robots that have
        been initialised — this propagates each robot's state estimate
        forward using the last commanded (v, ω), keeping the estimate alive
        even when the ArUco marker is temporarily invisible (same pattern as
        multi_unicycle_controller_double_angle_experiment_ekf.py's
        control_loop).
        """
        # ── EKF PREDICT — runs at the control rate regardless of camera visibility ──
        for name in ROBOT_MARKER_MAP:
            if self.seen[name]:
                self.ekfs[name].predict(CONTROL_DT)
                self.x[name], self.y[name], self.theta[name] = self.ekfs[name].position

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
        """
        Publish a TwistStamped velocity command.

        Side-effect: feeds the commanded (v, omega) into that robot's EKF so
        the next predict() step propagates using the actual command, not a
        stale one (mirrors set_velocity() in the formation controller).
        """
        msg = TwistStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x  = float(v)
        msg.twist.angular.z = float(omega)
        self.pubs[name].publish(msg)

        self.ekfs[name].last_v     = float(v)
        self.ekfs[name].last_omega = float(omega)


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
