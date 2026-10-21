#!/usr/bin/env python3.12
"""
Check Ground-Origin Axes  —  ArUco Feedback + EKF
=====================================================================
Drives ONE robot a short, safe distance purely along ground_origin's +X
axis, stops and announces it, then purely along +Y, stops and announces
it — so you can physically see, in the room, which direction each axis
of the calibrated `ground_origin` frame actually points.

Needs the same things already running as move_to_initial_ekf.py:
  - the camera + aruco_tracker
  - calibrate_ground_origin.py (either freshly calibrated, or reloaded
    with --load) publishing ground_origin

FEEDBACK : overhead camera -> aruco_opencv -> TF2  (ground_origin -> marker_N)
           -> per-robot EKF (same filter as move_to_initial_ekf.py /
           multi_unicycle_controller_double_angle_experiment_ekf.py)
COMMANDS : TwistStamped on /<robot>/cmd_vel

Usage
-----
  python3.12 experiment/check_axes.py

  Edit ROBOT_NAME / AXIS_CHECK_DISTANCE below if you want a different
  robot or a longer/shorter move.
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
# CONFIGURATION
# =============================================================================

ROBOT_MARKER_MAP: Dict[str, int] = {
    'bot_0': 0,
    'bot_1': 1,
    'bot_2': 2,
    'bot_3': 3,
}

ROBOT_NAME = 'bot_0'              # which robot to move for this check
AXIS_CHECK_DISTANCE = 0.3         # metres -- short and safe
PAUSE_SEC = 3.0                   # how long to hold still after each axis, for you to look

# Control gains and tolerances (same values as move_to_initial_ekf.py)
K_LINEAR    = 0.5
K_ANGULAR   = 2.0
V_MAX       = 0.15
OMEGA_MAX   = 1.5
POS_TOL     = 0.03
HEADING_TOL = 0.10

WORLD_FRAME  = 'ground_origin'
ARUCO_TOPIC  = '/aruco_detections'
TF_TIMEOUT   = 0.05
CONTROL_DT   = 0.05

# EKF noise parameters (same values as move_to_initial_ekf.py)
Q_XY    = 0.001
Q_THETA = 0.002
R_XY    = 0.05
R_THETA = 0.1
GATE_THRESHOLD = 11.34

# The two axis checks: relative (dx, dy) from wherever the robot currently
# is when that leg starts, plus the heading to face while moving (so the
# robot visibly points the way it's travelling), and a label for the log.
AXIS_LEGS = [
    {'dx': AXIS_CHECK_DISTANCE, 'dy': 0.0, 'heading': 0.0, 'label': '+X axis'},
    {'dx': 0.0, 'dy': AXIS_CHECK_DISTANCE, 'heading': math.pi / 2, 'label': '+Y axis'},
]


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


def normalize_angle(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def quat_to_yaw(qx, qy, qz, qw) -> float:
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny, cosy)


class CheckAxes(Node):
    """
    Drives ONE robot through AXIS_LEGS in order: rotate to face the leg's
    heading, drive AXIS_CHECK_DISTANCE in that direction, pause so you can
    see which way it went, then move to the next leg.
    """

    def __init__(self):
        super().__init__('check_axes')

        self.marker_id = ROBOT_MARKER_MAP[ROBOT_NAME]
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.seen = False
        self.ekf = RobotEKF(name=ROBOT_NAME)
        self.last_ekf_time: Optional[int] = None

        self.leg_index = 0
        self.phase = 'rotate_to_target'   # rotate_to_target -> drive -> rotate_to_heading -> pause
        self.target_x = 0.0
        self.target_y = 0.0
        self.leg_start_x = 0.0
        self.leg_start_y = 0.0
        self.pause_until: Optional[float] = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        cmd_qos = QoSProfile(depth=10)
        self.pub = self.create_publisher(TwistStamped, f'/{ROBOT_NAME}/cmd_vel', cmd_qos)

        aruco_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(ArucoDetection, ARUCO_TOPIC, self._aruco_cb, aruco_qos)
        self.create_timer(CONTROL_DT, self._control_loop)

        self.get_logger().info(
            f'\n[CheckAxes] Started. Robot: {ROBOT_NAME} (marker_{self.marker_id}).\n'
            f'  Will move {AXIS_CHECK_DISTANCE} m along +X, pause {PAUSE_SEC}s, '
            f'then {AXIS_CHECK_DISTANCE} m along +Y, pause {PAUSE_SEC}s.\n'
            f'  Waiting for marker...'
        )

    def _aruco_cb(self, msg: ArucoDetection):
        visible = {m.marker_id for m in msg.markers}
        if self.marker_id not in visible:
            return
        try:
            tf = self.tf_buffer.lookup_transform(
                WORLD_FRAME, f'marker_{self.marker_id}',
                rclpy.time.Time(), Duration(seconds=TF_TIMEOUT),
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            return

        z_x = tf.transform.translation.x
        z_y = tf.transform.translation.y
        q = tf.transform.rotation
        z_theta = quat_to_yaw(q.x, q.y, q.z, q.w)

        now_ns = self.get_clock().now().nanoseconds
        if self.last_ekf_time is not None:
            actual_dt = (now_ns - self.last_ekf_time) / 1e9
            if actual_dt > 0.2 or actual_dt <= 0.0:
                actual_dt = 0.05
        else:
            actual_dt = 0.05
        self.last_ekf_time = now_ns

        self.ekf.predict(actual_dt)

        if not self.seen:
            self.ekf.initialize(z_x, z_y, z_theta)
            self.seen = True
            self.x, self.y, self.theta = self.ekf.position
            self.get_logger().info(f'{ROBOT_NAME}: first detection at ({z_x:.3f}, {z_y:.3f})')
            self._start_leg(AXIS_LEGS[0])
        else:
            accepted = self.ekf.update(z_x, z_y, z_theta)
            if not accepted:
                self.get_logger().warn(
                    f'[EKF] marker_{self.marker_id} measurement rejected (outlier) '
                    f'-- using prediction only', throttle_duration_sec=1.0,
                )

        self.x, self.y, self.theta = self.ekf.position

    def _start_leg(self, leg):
        self.leg_start_x, self.leg_start_y = self.x, self.y
        self.target_x = self.x + leg['dx']
        self.target_y = self.y + leg['dy']
        self.phase = 'rotate_to_target'
        self.get_logger().info(
            f"\n--- Starting leg: {leg['label']} ---\n"
            f"  from ({self.leg_start_x:.3f}, {self.leg_start_y:.3f}) "
            f"to ({self.target_x:.3f}, {self.target_y:.3f})"
        )

    def _control_loop(self):
        if self.seen:
            self.ekf.predict(CONTROL_DT)
            self.x, self.y, self.theta = self.ekf.position

        if not self.seen:
            return

        if self.leg_index >= len(AXIS_LEGS):
            self._cmd(0.0, 0.0)
            return

        leg = AXIS_LEGS[self.leg_index]

        if self.phase == 'pause':
            now = self.get_clock().now().nanoseconds / 1e9
            if now >= self.pause_until:
                self.leg_index += 1
                if self.leg_index >= len(AXIS_LEGS):
                    self.get_logger().info('\n=== Both axis checks done. Robot stopped. ===')
                    self._cmd(0.0, 0.0)
                else:
                    self._start_leg(AXIS_LEGS[self.leg_index])
            return

        dx = self.target_x - self.x
        dy = self.target_y - self.y
        dist = math.sqrt(dx * dx + dy * dy)
        angle_to_target = math.atan2(dy, dx)
        heading_err_to_target = normalize_angle(angle_to_target - self.theta)

        if self.phase == 'rotate_to_target':
            if dist < POS_TOL:
                self.phase = 'rotate_to_heading'
                self._cmd(0.0, 0.0)
                return
            if abs(heading_err_to_target) < 0.15:
                self.phase = 'drive'
            else:
                omega = max(min(K_ANGULAR * heading_err_to_target, OMEGA_MAX), -OMEGA_MAX)
                self._cmd(0.0, omega)

        elif self.phase == 'drive':
            if dist < POS_TOL:
                self.phase = 'rotate_to_heading'
                self._cmd(0.0, 0.0)
                self.get_logger().info(f'{ROBOT_NAME}: position reached ({self.x:.3f}, {self.y:.3f})')
                return
            if abs(heading_err_to_target) > 0.8:
                self.phase = 'rotate_to_target'
                self._cmd(0.0, 0.0)
                return
            v = min(K_LINEAR * dist, V_MAX)
            omega = max(min(K_ANGULAR * heading_err_to_target, OMEGA_MAX), -OMEGA_MAX)
            self._cmd(v, omega)

        elif self.phase == 'rotate_to_heading':
            heading_err = normalize_angle(leg['heading'] - self.theta)
            if abs(heading_err) < HEADING_TOL:
                self._cmd(0.0, 0.0)
                now = self.get_clock().now().nanoseconds / 1e9
                self.pause_until = now + PAUSE_SEC
                self.phase = 'pause'
                self.get_logger().info(
                    f"\n>>> {leg['label']}: robot moved from "
                    f"({self.leg_start_x:.3f}, {self.leg_start_y:.3f}) to "
                    f"({self.x:.3f}, {self.y:.3f}) <<<\n"
                    f"    THAT direction of travel is {leg['label']}. "
                    f"Holding still for {PAUSE_SEC:.0f}s..."
                )
            else:
                omega = max(min(K_ANGULAR * heading_err, OMEGA_MAX), -OMEGA_MAX)
                self._cmd(0.0, omega)

    def _cmd(self, v: float, omega: float):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = float(v)
        msg.twist.angular.z = float(omega)
        self.pub.publish(msg)
        self.ekf.last_v = float(v)
        self.ekf.last_omega = float(omega)


def main():
    rclpy.init()
    node = CheckAxes()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._cmd(0.0, 0.0)
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
