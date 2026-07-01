#!/usr/bin/env python3.12
"""
multi_unicycle_ekf.py
=====================
Multi-Robot Unicycle Formation Controller with Extended Kalman Filter (EKF).

Fixes vs original:
  1. ArUco lost  → EKF predict-only keeps robot state alive using last commands.
  2. Spikes       → EKF fuses noisy ArUco measurements; outlier gate rejects jumps.
  3. No-stop bug  → Formation convergence detector halts all robots when reached.

EKF per robot:
  State  : [x, y, θ]   (3-dim)
  Process: unicycle kinematics  x_{k+1} = f(x_k, u_k)
  Measure: ArUco TF gives z = [x_meas, y_meas, θ_meas]  (3-dim)
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from math import sin, cos, atan2, sqrt, pi, acos, asin, degrees, radians
import numpy as np
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from geometry_msgs.msg import TwistStamped
from aruco_opencv_msgs.msg import ArucoDetection
from typing import Dict, List, Tuple
import time
from datetime import datetime
import os
from scipy.linalg import block_diag

# =============================================================================
# CONFIGURATION
# =============================================================================
ROBOT_MARKER_MAP: Dict[str, int] = {
    'bot_0': 0,
    'bot_1': 1,
    'bot_2': 2,
    'bot_3': 3,
}

WORLD_FRAME = 'ground_origin'
ARUCO_TOPIC = '/aruco_detections'
TF_TIMEOUT  = 0.05   # s

# EKF noise tuning
Q_XY    = 0.001   # process noise variance (x, y)
Q_THETA = 0.002   # process noise variance (θ)
R_XY    = 0.004   # measurement noise variance (x, y)   – camera pixel noise
R_THETA = 0.008   # measurement noise variance (θ)      – marker pose noise

# Outlier / Mahalanobis gate (chi-squared, 3 dof, 99%  = 11.34)
GATE_THRESHOLD = 11.34

# Formation convergence
FORMATION_TOL    = 0.03   # m – max positional error to call "converged"
CONVERGE_CYCLES  = 50     # consecutive control cycles under tol → stop


# =============================================================================
# SINGLE-ROBOT EKF
# =============================================================================
class RobotEKF:
    """
    Extended Kalman Filter for one unicycle robot.

    State vector:  x = [x, y, θ]^T
    Control input: u = [v, ω]^T  (last commanded velocity)
    Measurement:   z = [x_m, y_m, θ_m]^T  (from ArUco TF)

    Process model (Euler, dt seconds):
        x_{k+1} = x_k + v*cos(θ_k)*dt
        y_{k+1} = y_k + v*sin(θ_k)*dt
        θ_{k+1} = θ_k + ω*dt

    Jacobian F = ∂f/∂x:
        [1  0  -v*sin(θ)*dt]
        [0  1   v*cos(θ)*dt]
        [0  0        1      ]

    Measurement model:  h(x) = x   (identity – ArUco directly gives state)
    Jacobian H = I_3
    """

    def __init__(self, name: str, x0: float = 0.0, y0: float = 0.0, theta0: float = 0.0):
        self.name = name

        # State mean
        self.x_est = np.array([x0, y0, theta0], dtype=float)

        # State covariance (start uncertain)
        self.P = np.diag([1.0, 1.0, 0.5])

        # Process noise covariance Q
        self.Q = np.diag([Q_XY, Q_XY, Q_THETA])

        # Measurement noise covariance R
        self.R = np.diag([R_XY, R_XY, R_THETA])

        # Last commanded control
        self.last_v     = 0.0
        self.last_omega = 0.0

        # Marker initialised flag
        self.initialized = False

    # ------------------------------------------------------------------
    def initialize(self, x: float, y: float, theta: float):
        self.x_est = np.array([x, y, theta], dtype=float)
        self.P     = np.diag([0.01, 0.01, 0.01])  # tight after first obs
        self.initialized = True

    # ------------------------------------------------------------------
    def predict(self, dt: float):
        """Predict step using unicycle kinematics + last known command."""
        if not self.initialized:
            return

        x, y, theta = self.x_est
        v, omega    = self.last_v, self.last_omega

        # State prediction
        self.x_est[0] = x + v * cos(theta) * dt
        self.x_est[1] = y + v * sin(theta) * dt
        self.x_est[2] = self._wrap(theta + omega * dt)

        # Jacobian F
        F = np.array([
            [1.0, 0.0, -v * sin(theta) * dt],
            [0.0, 1.0,  v * cos(theta) * dt],
            [0.0, 0.0,  1.0               ],
        ])

        # Covariance prediction
        self.P = F @ self.P @ F.T + self.Q

    # ------------------------------------------------------------------
    def update(self, z_x: float, z_y: float, z_theta: float) -> bool:
        """
        Measurement update.
        Returns True if measurement accepted, False if outlier-gated.
        """
        if not self.initialized:
            self.initialize(z_x, z_y, z_theta)
            return True

        z    = np.array([z_x, z_y, z_theta])
        H    = np.eye(3)                   # ∂h/∂x = I

        # Innovation
        y_inn    = z - H @ self.x_est
        y_inn[2] = self._wrap(y_inn[2])   # wrap angle

        # Innovation covariance
        S = H @ self.P @ H.T + self.R

        # --- Mahalanobis gate: reject outliers (spikes) ---
        maha_sq = float(y_inn @ np.linalg.inv(S) @ y_inn)
        if maha_sq > GATE_THRESHOLD:
            return False   # outlier – skip this measurement

        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)

        # State update
        self.x_est = self.x_est + K @ y_inn
        self.x_est[2] = self._wrap(self.x_est[2])

        # Covariance update (Joseph form for numerical stability)
        I_KH   = np.eye(3) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ self.R @ K.T

        return True

    # ------------------------------------------------------------------
    @property
    def position(self) -> Tuple[float, float, float]:
        return float(self.x_est[0]), float(self.x_est[1]), float(self.x_est[2])

    @staticmethod
    def _wrap(angle: float) -> float:
        return atan2(sin(angle), cos(angle))


# =============================================================================
# ROBOT STATE (thin wrapper used by controller)
# =============================================================================
class RobotState:
    def __init__(self, name: str, marker_id: int):
        self.name           = name
        self.marker_id      = marker_id
        self.marker_tf_frame = f'marker_{marker_id}'

        self.ekf = RobotEKF(name)

        # Convenience properties (updated from EKF after every iteration)
        self.x     = 0.0
        self.y     = 0.0
        self.theta = 0.0

        self.target_v     = 0.0
        self.target_omega = 0.0

        self.pose_received    = False
        self.last_aruco_time  = None   # rclpy.time.Time

        # Trajectory logs
        self.trajectory_x     = []
        self.trajectory_y     = []
        self.trajectory_theta = []

    def sync_from_ekf(self):
        self.x, self.y, self.theta = self.ekf.position


# =============================================================================
# MAIN CONTROLLER NODE
# =============================================================================
class MultiUnicycleEKFController(Node):

    def __init__(self):
        super().__init__('aruco_ekf_formation_controller')

        self.robot_names: List[str] = list(ROBOT_MARKER_MAP.keys())
        self.num_robots: int        = len(self.robot_names)

        self.robots: Dict[str, RobotState] = {
            name: RobotState(name, ROBOT_MARKER_MAP[name])
            for name in self.robot_names
        }

        # ── Double-Angle CBF parameters ──────────────────────────────────────
        self.pstar = np.array([[0.6], [0.6], [-0.6], [0.6], [-0.6], [-0.6], [0.6], [-0.6]])

        self.H_Nset = np.array([[-1, 1, 0, 0],
                                 [0, -1, 1, 0],
                                 [0, 0, -1, 1],
                                 [1, 0, 0, -1],
                                 [0, 1, 0, -1]])
        self.Hbar_Nset = np.kron(self.H_Nset, np.eye(2))

        self.H_Mset    = np.array([[-1, 0, 1, 0]])
        self.Hbar_Mset = np.kron(self.H_Mset, np.eye(2))

        self.v_max     = 0.18
        self.omega_max = 1.50
        self.K_p_w     = 1.0
        self.length    = 0.1

        self.estar = self.Hbar_Nset @ self.pstar

        self.r     = 0.135
        self.Rs    = 0.3
        self.Rc    = 1.80
        self.Rcrit = 1.5

        self.Rc_bar   = 1 - (2 * (self.r / (self.Rc   - self.r))**2)
        self.Rs_bar   = 1 - (2 * (self.r / (self.Rs   + self.r))**2)
        self.Rcrit_bar= 1 - (2 * (self.r /  self.Rcrit        )**2)

        self.psi_min  = acos(self.Rc_bar)
        self.psi_max  = acos(self.Rs_bar)
        self.psi_crit = acos(self.Rcrit_bar)

        self.psi_star = np.array([])
        self.Kc       = np.array([])
        self.Ks       = np.array([])

        for i in range(0, len(self.estar), 2):
            e_norm = np.linalg.norm(self.estar[i:i+2])
            temp1  = 2 * asin(self.r / e_norm)
            self.psi_star = np.append(self.psi_star, temp1)
            self.Kc = np.append(self.Kc, self.Rc_bar**2  - (cos(temp1))**2)
            self.Ks = np.append(self.Ks, (cos(temp1))**2 - self.Rs_bar**2)

        self.delta = self.Rs_bar**2 - self.Rcrit_bar**2
        self.H     = self.H_Nset
        self.Hbar  = self.Hbar_Nset

        # Desired distances from pstar
        self.desired_distances = [
            np.linalg.norm(self.estar[i:i+2].flatten())
            for i in range(0, len(self.estar), 2)
        ]

        # ── Convergence detection ─────────────────────────────────────────────
        self._converge_count = 0
        self._formation_done = False

        # ── ROS2 interface ────────────────────────────────────────────────────
        cmd_qos = QoSProfile(depth=10)
        self.cmd_vel_pubs: Dict[str, any] = {
            name: self.create_publisher(TwistStamped, f'/{name}/cmd_vel', cmd_qos)
            for name in self.robot_names
        }

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        aruco_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.aruco_sub = self.create_subscription(
            ArucoDetection, ARUCO_TOPIC, self._aruco_callback, aruco_qos)

        # EKF predict + control at 50 Hz
        self.dt           = 1.0 / 50.0
        self.control_timer = self.create_timer(self.dt, self._control_loop)

        # Logging
        self.start_time      = None
        self.last_log_time   = 0.0
        self.log_interval    = 0.1
        self.trajectory_time = []
        self.distance_log    = {i: [] for i in range(self.H.shape[0])}

        self.robot_colors = {
            self.robot_names[0]: 'blue',
            self.robot_names[1]: 'red',
            self.robot_names[2]: 'green',
            self.robot_names[3]: 'orange',
        }

        self.get_logger().info(
            f'[EKF Controller] Initialized for {self.num_robots} robots.\n'
            f'  Gate threshold (Mahalanobis²): {GATE_THRESHOLD}\n'
            f'  Formation convergence tol    : {FORMATION_TOL} m'
        )

    # =========================================================================
    # ARUCO CALLBACK — EKF measurement update
    # =========================================================================
    def _aruco_callback(self, msg: ArucoDetection):
        visible_ids = {m.marker_id for m in msg.markers}

        for name, robot in self.robots.items():
            if robot.marker_id not in visible_ids:
                continue

            try:
                tf = self.tf_buffer.lookup_transform(
                    WORLD_FRAME,
                    robot.marker_tf_frame,
                    rclpy.time.Time(),
                    Duration(seconds=TF_TIMEOUT),
                )
            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                self.get_logger().warn(
                    f'TF {WORLD_FRAME}→{robot.marker_tf_frame}: {e}',
                    throttle_duration_sec=2.0)
                continue

            z_x     = tf.transform.translation.x
            z_y     = tf.transform.translation.y
            z_theta = self._quat_to_yaw(tf.transform.rotation)

            # Initialize EKF on first detection
            if not robot.pose_received:
                robot.ekf.initialize(z_x, z_y, z_theta)
                robot.pose_received   = True
                robot.last_aruco_time = self.get_clock().now()
                robot.sync_from_ekf()
                self.get_logger().info(
                    f'[{name}] First ArUco fix: ({z_x:.3f}, {z_y:.3f}, '
                    f'{degrees(z_theta):.1f}°)'
                )
                continue

            # EKF measurement update (Mahalanobis gate inside)
            accepted = robot.ekf.update(z_x, z_y, z_theta)
            robot.last_aruco_time = self.get_clock().now()

            if not accepted:
                self.get_logger().warn(
                    f'[{name}] ArUco measurement REJECTED (outlier / spike)',
                    throttle_duration_sec=1.0)

            robot.sync_from_ekf()

    # =========================================================================
    # CONTROL LOOP — EKF predict + Double-Angle CBF + convergence check
    # =========================================================================
    def _control_loop(self):
        # 1. EKF PREDICT for every robot (runs even if marker lost)
        for robot in self.robots.values():
            if robot.pose_received:
                robot.ekf.predict(self.dt)
                robot.sync_from_ekf()

        self._log_trajectories()

        if not self._all_ready():
            return

        if self._formation_done:
            return

        # 2. FORMATION CONTROL (Double-Angle CBF)
        p_internal = np.array([
            coord for name in self.robot_names
            for coord in [self.robots[name].x, self.robots[name].y]
        ])

        e_N = self.Hbar_Nset @ p_internal
        e_M = self.Hbar_Mset @ p_internal

        m    = self.Hbar_Nset.shape[0]
        mhat = self.Hbar_Mset.shape[0]

        R_psi_N = None
        xi      = np.array([])

        for i in range(0, m, 2):
            e_vec = e_N[i:i+2].reshape(2, 1)
            dij   = np.linalg.norm(e_vec)
            if dij < 1e-6:
                return

            asin_arg = float(np.clip(self.r / dij, -1.0, 1.0))
            psi_ij   = 2 * asin(asin_arg)
            gij      = e_vec / dij

            Rot_R = np.array([[cos(psi_ij/2), -sin(psi_ij/2)],
                               [sin(psi_ij/2),  cos(psi_ij/2)]])
            Rot_L = np.array([[cos(psi_ij/2),  sin(psi_ij/2)],
                               [-sin(psi_ij/2), cos(psi_ij/2)]])

            g_iRj = Rot_R @ gij;  g_iLj = Rot_L @ gij
            Pg_iRj = np.eye(2) - g_iRj @ g_iRj.T
            Pg_iLj = np.eye(2) - g_iLj @ g_iLj.T

            f_gij      = (Pg_iLj @ g_iRj) + (Pg_iRj @ g_iLj)
            f_psi_ij   = 0.5 * sin(2 * psi_ij) / (1 + cos(psi_ij))
            block_N    = f_psi_ij * f_gij
            R_psi_N    = block_N if R_psi_N is None else block_diag(R_psi_N, block_N)

            c       = int(i / 2)
            E_ij    = (cos(psi_ij))**2 - (cos(self.psi_star[c]))**2
            q       = 1.0 if E_ij > 0 else 0.0
            K       = q * self.Kc[c] + (1 - q) * self.Ks[c]
            denom   = K**2 - E_ij**2
            xi      = np.append(xi, 0.0 if abs(denom) < 1e-6 else 4 * E_ij / denom)

        R_psi_M = None;  xi_beta = np.array([])
        for i in range(0, mhat, 2):
            e_vec = e_M[i:i+2].reshape(2, 1)
            dik   = np.linalg.norm(e_vec)
            if dik < 1e-6:
                return

            asin_arg = float(np.clip(self.r / dik, -1.0, 1.0))
            psi_ik   = 2 * asin(asin_arg)
            gik      = e_vec / dik

            Rot_R = np.array([[cos(psi_ik/2), -sin(psi_ik/2)],
                               [sin(psi_ik/2),  cos(psi_ik/2)]])
            Rot_L = np.array([[cos(psi_ik/2),  sin(psi_ik/2)],
                               [-sin(psi_ik/2), cos(psi_ik/2)]])

            g_iRk = Rot_R @ gik;  g_iLk = Rot_L @ gik
            Pg_iRk = np.eye(2) - g_iRk @ g_iRk.T
            Pg_iLk = np.eye(2) - g_iLk @ g_iLk.T

            f_gik    = (Pg_iLk @ g_iRk) + (Pg_iRk @ g_iLk)
            f_psi_ik = 0.5 * sin(2 * psi_ik) / (1 + cos(psi_ik))
            block_M  = f_psi_ik * f_gik
            R_psi_M  = block_M if R_psi_M is None else block_diag(R_psi_M, block_M)

            beta_ik      = max(0.0, (cos(psi_ik))**2 - (cos(self.psi_crit))**2)
            denom_beta   = self.delta**2 - beta_ik**2
            xi_beta      = np.append(xi_beta,
                           0.0 if abs(denom_beta) < 1e-6 else 4 * beta_ik / denom_beta)

        xi      = xi.reshape(-1, 1)
        xi_beta = xi_beta.reshape(-1, 1)

        d_dt = -(self.Hbar_Nset.T @ R_psi_N @ xi)
        d_dt = d_dt.flatten()

        theta_all = np.array([self.robots[n].theta for n in self.robot_names])

        v_uni = np.array([
            2 * (d_dt[2*i] * cos(theta_all[i]) + d_dt[2*i+1] * sin(theta_all[i]))
            for i in range(self.num_robots)
        ])
        w_uni = np.array([
            (-d_dt[2*i] * sin(theta_all[i]) + d_dt[2*i+1] * cos(theta_all[i])) / self.length
            for i in range(self.num_robots)
        ])

        v_uni = np.clip(v_uni, -self.v_max,     self.v_max)
        w_uni = np.clip(w_uni, -self.omega_max, self.omega_max)

        # 3. CONVERGENCE CHECK — fix issue #3
        formation_error = self._formation_error()
        if formation_error < FORMATION_TOL:
            self._converge_count += 1
            if self._converge_count >= CONVERGE_CYCLES:
                self._formation_done = True
                self.stop_all()
                self.get_logger().info(
                    f'[EKF Controller] ✓ Formation CONVERGED '
                    f'(error={formation_error*100:.1f} cm). All robots STOPPED.')
                return
        else:
            self._converge_count = 0

        # 4. PUBLISH COMMANDS and store in EKF for next predict step
        for i, name in enumerate(self.robot_names):
            self._set_velocity(name, v_uni[i], w_uni[i])

    # =========================================================================
    # VELOCITY PUBLISHING
    # =========================================================================
    def _set_velocity(self, name: str, v: float, omega: float):
        cmd = TwistStamped()
        cmd.header.stamp    = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x  = float(v)
        cmd.twist.angular.z = float(omega)
        self.cmd_vel_pubs[name].publish(cmd)

        robot = self.robots[name]
        robot.target_v         = v
        robot.target_omega     = omega
        # Feed commanded velocity to EKF for prediction
        robot.ekf.last_v     = v
        robot.ekf.last_omega = omega

    def stop_all(self):
        for name in self.robot_names:
            self._set_velocity(name, 0.0, 0.0)
            self.robots[name].ekf.last_v     = 0.0
            self.robots[name].ekf.last_omega = 0.0

    # =========================================================================
    # HELPERS
    # =========================================================================
    def _all_ready(self) -> bool:
        return all(r.pose_received for r in self.robots.values())

    def _formation_error(self) -> float:
        """Mean positional deviation from desired formation (pstar)."""
        p = np.array([
            coord for name in self.robot_names
            for coord in [self.robots[name].x, self.robots[name].y]
        ])
        e = self.Hbar_Nset @ p - self.estar.flatten()
        return float(np.mean(np.sqrt(e[0::2]**2 + e[1::2]**2)))

    @staticmethod
    def _quat_to_yaw(q) -> float:
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return atan2(siny, cosy)

    def _log_trajectories(self):
        if self.start_time is None:
            self.start_time = time.time()
        t = time.time() - self.start_time
        if t - self.last_log_time < self.log_interval:
            return
        self.last_log_time = t
        self.trajectory_time.append(t)

        for name in self.robot_names:
            robot = self.robots[name]
            if robot.pose_received:
                robot.trajectory_x.append(robot.x)
                robot.trajectory_y.append(robot.y)
                robot.trajectory_theta.append(robot.theta)

        if self._all_ready():
            positions = np.array([[self.robots[n].x, self.robots[n].y]
                                   for n in self.robot_names])
            pairs = self._get_pairs()
            errs  = []
            for idx, (i, j) in enumerate(pairs):
                d = np.linalg.norm(positions[i] - positions[j])
                self.distance_log[idx].append(d)
                errs.append(f'e{idx+1}={d - self.desired_distances[idx]:+.3f}')
            self.get_logger().info(
                f't={t:.1f}s | Dist errors: {" | ".join(errs)} | '
                f'Form err: {self._formation_error()*100:.1f} cm | '
                f'Conv cnt: {self._converge_count}/{CONVERGE_CYCLES}',
                throttle_duration_sec=0.5,
            )

    def _get_pairs(self) -> List[Tuple[int, int]]:
        pairs = []
        for row in range(self.H.shape[0]):
            idx = np.where(self.H[row] != 0)[0]
            if len(idx) == 2:
                pairs.append((int(idx[0]), int(idx[1])))
        return pairs

    def ready_count(self) -> int:
        return sum(1 for r in self.robots.values() if r.pose_received)


# =============================================================================
# MAIN
# =============================================================================
def main(args=None):
    rclpy.init(args=args)
    controller = MultiUnicycleEKFController()

    try:
        controller.get_logger().info('Waiting for all ArUco markers…')
        timeout = 0
        while controller.ready_count() < controller.num_robots and timeout < 300:
            rclpy.spin_once(controller, timeout_sec=0.1)
            timeout += 1
            if timeout % 30 == 0:
                controller.get_logger().info(
                    f'Markers: {controller.ready_count()}/{controller.num_robots}')

        controller.get_logger().info(
            f'All {controller.ready_count()} markers detected — starting EKF formation control!')
        rclpy.spin(controller)

    except KeyboardInterrupt:
        print('\n[EKF] Ctrl+C — stopping all robots.')
    finally:
        try:
            controller.stop_all()
        except Exception:
            pass
        try:
            controller.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
