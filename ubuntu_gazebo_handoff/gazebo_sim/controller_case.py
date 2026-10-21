#!/usr/bin/env python3
"""
controller_case.py
===================
Formation controller for one Gazebo replication run of a real TurtleBot3
case (Case-1b / Case-2 / Case-3 / Case-4). New, standalone file -- does not
modify or import the existing ad hoc controller scripts in
turtlebot3_gazebo/scripts/.

Implements the bearing-based single-integrator formation law (double-angle
visual-cone parameterization), matching the exact per-case LAW VARIANT --
NOT a uniform law across cases. The authoritative source for which formula
each case actually used is experiment/multi_unicycle_controller_double_
angle_experiment_ekf.py (the script that drove the real robots), not
turtlebot3_gazebo/scripts/multi_unicycle_controller_double_angle.py (a demo
copy whose Variant 2 has an extra 10,000,000 gain the real hardware script
does not):

  law_variant 1 (Case-2, Case-3): connectivity only, K-barrier-scaled xi,
    no collision avoidance, no gain.
        d_dt = -(Hbar_Nset.T @ R_psi_N @ xi)

  law_variant 2 (Case-4): connectivity + collision avoidance, no gain --
    matches the real hardware script's commented Variant 2 exactly.
        d_dt = -(Hbar_Nset.T @ R_psi_N @ xi + Hbar_Mset.T @ R_psi_M @ xi_beta)

  law_variant 3 (Case-1b): connectivity only, but driven by the raw
    per-edge error E (not the K-barrier xi), with a 10,000,000 gain --
    identical in both the real hardware script and the demo script.
        d_dt = -10,000,000 * (Hbar_Nset.T @ R_psi_N @ E)

Parameters (r/Rs/Rc/Rcrit, v_max/omega_max/length/conv_tol, pstar, graph
edges) match UBUNTU_GAZEBO_TASK.md section 2 / `Colab with
Sushmita/sim_timesynced_cases.m` exactly -- those were never in question,
only which d_dt formula each case actually used.

ROS parameters (set via --ros-args -p from the launch file):
  case_name    (string)  default 'unknown'
  law_variant  (int)     default 1 -- 1, 2, or 3 (see above)
  log_dir      (string)  default '/tmp' -- directory to write
                          log_data.csv and control_status.json into
  conv_tol     (double)  default 0.080
  max_duration (double)  default 240.0  -- hard timeout, seconds
  log_interval (double)  default 0.1    -- CSV row cadence, seconds
"""

import json
import os
import signal
from math import sin, cos, atan2, pi, acos, asin
from typing import Dict, List, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from scipy.linalg import block_diag

ROBOT_NAMES = ['bot_0', 'bot_1', 'bot_2', 'bot_3']

# Graph edges (5 controlled pairs) and the collision-avoidance pair, in the
# exact order the real log_data.csv's d1..d5 / e1..e5 columns use.
H_NSET = np.array([
    [-1, 1, 0, 0],
    [0, -1, 1, 0],
    [0, 0, -1, 1],
    [1, 0, 0, -1],
    [0, 1, 0, -1],
])
H_MSET = np.array([[-1, 0, 1, 0]])  # bot_0 - bot_2


class RobotState:
    def __init__(self, name: str):
        self.name = name
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.odom_received = False


class CaseFormationController(Node):

    def __init__(self):
        super().__init__('formation_controller_case')

        self.declare_parameter('case_name', 'unknown')
        self.declare_parameter('law_variant', 1)
        self.declare_parameter('log_dir', '/tmp')
        self.declare_parameter('conv_tol', 0.080)
        self.declare_parameter('max_duration', 240.0)
        self.declare_parameter('log_interval', 0.1)

        self.case_name = self.get_parameter('case_name').value
        self.law_variant = int(self.get_parameter('law_variant').value)
        if self.law_variant not in (1, 2, 3):
            raise ValueError(f'law_variant must be 1, 2, or 3, got {self.law_variant}')
        self.collision_avoidance = (self.law_variant == 2)  # for logging/status only
        self.log_dir = self.get_parameter('log_dir').value
        self.conv_tol = float(self.get_parameter('conv_tol').value)
        self.max_duration = float(self.get_parameter('max_duration').value)
        self.log_interval = float(self.get_parameter('log_interval').value)

        os.makedirs(self.log_dir, exist_ok=True)

        self.robots: Dict[str, RobotState] = {n: RobotState(n) for n in ROBOT_NAMES}

        # ============================================
        # Controller parameters -- matching
        # UBUNTU_GAZEBO_TASK.md section 2 exactly.
        # ============================================
        self.pstar = np.array([[0.4], [0.3], [-0.4], [0.3], [-0.7], [-0.4], [0.7], [-0.4]])

        self.Hbar_Nset = np.kron(H_NSET, np.eye(2))
        self.Hbar_Mset = np.kron(H_MSET, np.eye(2))

        self.r = 0.135
        self.Rs = 0.16
        self.Rc = 1.6
        self.Rcrit = 0.4
        self.v_max = 0.055
        self.omega_max = 0.13
        self.length = 0.25
        self.control_rate = 50.0  # Hz

        self.dist_lower_bound = self.Rs + self.r
        self.dist_upper_bound = self.Rc - self.r

        self.estar = self.Hbar_Nset @ self.pstar

        Rc_bar = 1 - (2 * (self.r / (self.Rc - self.r)) ** 2)
        Rs_bar = 1 - (2 * (self.r / (self.Rs + self.r)) ** 2)
        Rcrit_bar = 1 - (2 * (self.r / self.Rcrit) ** 2)
        self.psi_crit = acos(Rcrit_bar)
        self.delta = Rs_bar ** 2 - Rcrit_bar ** 2

        self.psi_star = []
        self.Kc = []
        self.Ks = []
        self.desired_distances = []
        for i in range(0, len(self.estar), 2):
            e_vec = self.estar[i:i + 2].flatten()
            d_star = float(np.linalg.norm(e_vec))
            self.desired_distances.append(d_star)
            psi = 2 * asin(self.r / d_star)
            self.psi_star.append(psi)
            self.Kc.append(Rc_bar ** 2 - cos(psi) ** 2)
            self.Ks.append(cos(psi) ** 2 - Rs_bar ** 2)

        self.edge_pairs: List[Tuple[int, int]] = self._connected_pairs()

        # ============================================
        # ROS2 I/O
        # ============================================
        qos = QoSProfile(depth=10)
        self.cmd_vel_pubs = {}
        for name in ROBOT_NAMES:
            self.cmd_vel_pubs[name] = self.create_publisher(Twist, f'/{name}/cmd_vel', qos)
            self.create_subscription(Odometry, f'/{name}/odom', self._make_odom_cb(name), qos)

        self.control_timer = self.create_timer(1.0 / self.control_rate, self.control_loop)

        # ============================================
        # Logging / lifecycle state
        # ============================================
        self.start_time = None
        self.last_log_time = -1.0
        self.rows: List[list] = []
        self.should_exit = False
        self.stop_reason = None
        self.converged = False
        self.final_time = 0.0

        self.get_logger().info(
            f'Case {self.case_name}: law_variant={self.law_variant} '
            f'(collision_avoidance={self.collision_avoidance}), '
            f'v_max={self.v_max}, omega_max={self.omega_max}, length={self.length}, '
            f'Rc={self.Rc} (lower={self.dist_lower_bound:.3f}, upper={self.dist_upper_bound:.3f}), '
            f'conv_tol={self.conv_tol}, max_duration={self.max_duration}s'
        )

    # ------------------------------------------------------------------
    def _make_odom_cb(self, name):
        def cb(msg: Odometry):
            robot = self.robots[name]
            robot.x = msg.pose.pose.position.x
            robot.y = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            robot.theta = self._quat_to_yaw(q.x, q.y, q.z, q.w)
            robot.odom_received = True
        return cb

    @staticmethod
    def _quat_to_yaw(qx, qy, qz, qw):
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        return atan2(siny_cosp, cosy_cosp)

    def _connected_pairs(self):
        pairs = []
        for row in H_NSET:
            idx = np.where(row != 0)[0]
            pairs.append((int(idx[0]), int(idx[1])))
        return pairs

    def is_ready(self):
        return all(r.odom_received for r in self.robots.values())

    def _positions(self):
        return np.array([[self.robots[n].x, self.robots[n].y] for n in ROBOT_NAMES])

    def _headings(self):
        return np.array([self.robots[n].theta for n in ROBOT_NAMES])

    def set_velocity(self, name, v, omega):
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(omega)
        self.cmd_vel_pubs[name].publish(cmd)

    def stop_all(self):
        for name in ROBOT_NAMES:
            self.set_velocity(name, 0.0, 0.0)

    # ------------------------------------------------------------------
    def control_loop(self):
        if self.should_exit:
            return

        if not self.is_ready():
            return

        if self.start_time is None:
            self.start_time = self.get_clock().now()

        now = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        p = self._positions().flatten()
        theta = self._headings()

        e_N = self.Hbar_Nset @ p
        edge_dist = np.zeros(5)
        edge_err = np.zeros(5)
        for c in range(5):
            e_vec = e_N[2 * c:2 * c + 2]
            edge_dist[c] = float(np.linalg.norm(e_vec))
            edge_err[c] = edge_dist[c] - self.desired_distances[c]

        # ---- convergence / timeout stop ----
        if np.all(np.abs(edge_err) <= self.conv_tol):
            self._finish(now, converged=True, reason='converged')
            return
        if now >= self.max_duration:
            self._finish(now, converged=False, reason='timeout')
            return

        # ---- formation velocity (bearing-based double-angle law) ----
        d_dt = self._formation_velocity(p, e_N, edge_dist)

        # ---- look-ahead unicycle conversion ----
        v_uni = np.zeros(4)
        w_uni = np.zeros(4)
        for i in range(4):
            ux, uy = d_dt[2 * i], d_dt[2 * i + 1]
            v_uni[i] = 2 * (ux * cos(theta[i]) + uy * sin(theta[i]))
            w_uni[i] = (-ux * sin(theta[i]) + uy * cos(theta[i])) / self.length

        # ---- mandatory velocity clamp (real-robot limits, NOT TB3 defaults) ----
        v_uni = np.clip(v_uni, -self.v_max, self.v_max)
        w_uni = np.clip(w_uni, -self.omega_max, self.omega_max)

        for i, name in enumerate(ROBOT_NAMES):
            self.set_velocity(name, v_uni[i], w_uni[i])

        # ---- logging (subsampled to log_interval) ----
        if now - self.last_log_time >= self.log_interval:
            self.last_log_time = now
            row = [now]
            for i, name in enumerate(ROBOT_NAMES):
                row += [p[2 * i], p[2 * i + 1], theta[i]]
            row += list(edge_dist)
            row += list(edge_err)
            row += [self.dist_lower_bound, self.dist_upper_bound]
            self.rows.append(row)

    # Gain used verbatim from multi_unicycle_controller_double_angle.py's
    # commented-out "VARIANT 2" / "Variant 3" lines, for law_variant 2 and 3.
    _GIANT_GAIN = 10_000_000

    def _formation_velocity(self, p, e_N, edge_dist):
        R_psi_N = None
        xi = np.zeros(5)
        E = np.zeros(5)
        for c in range(5):
            e_vec = e_N[2 * c:2 * c + 2].reshape(2, 1)
            dij = edge_dist[c]
            asin_arg = float(np.clip(self.r / dij, -1.0, 1.0))
            psi_ij = 2 * asin(asin_arg)
            gij = e_vec / dij

            Rot_R = np.array([[cos(psi_ij / 2), -sin(psi_ij / 2)], [sin(psi_ij / 2), cos(psi_ij / 2)]])
            Rot_L = np.array([[cos(psi_ij / 2), sin(psi_ij / 2)], [-sin(psi_ij / 2), cos(psi_ij / 2)]])
            g_iRj = Rot_R @ gij
            g_iLj = Rot_L @ gij
            Pg_iRj = np.eye(2) - (g_iRj @ g_iRj.T)
            Pg_iLj = np.eye(2) - (g_iLj @ g_iLj.T)
            f_gij = (Pg_iLj @ g_iRj) + (Pg_iRj @ g_iLj)
            f_psi_ij = 0.5 * sin(2 * psi_ij) / (1 + cos(psi_ij))
            block = f_psi_ij * f_gij

            R_psi_N = block if R_psi_N is None else block_diag(R_psi_N, block)

            E_ij = (cos(psi_ij)) ** 2 - (cos(self.psi_star[c])) ** 2
            E[c] = E_ij
            K = self.Kc[c] if E_ij > 0 else self.Ks[c]
            denom = K ** 2 - E_ij ** 2
            xi[c] = 0.0 if abs(denom) < 1e-6 else 4 * E_ij / denom

        if self.law_variant == 3:
            # Variant 3 (Case-1b): raw-E-driven, no collision avoidance.
            d_dt = -self._GIANT_GAIN * (self.Hbar_Nset.T @ R_psi_N @ E.reshape(-1, 1))
            return d_dt.flatten()

        connectivity_term = self.Hbar_Nset.T @ R_psi_N @ xi.reshape(-1, 1)

        if self.law_variant == 1:
            # Variant 1 (Case-2, Case-3): connectivity only, no CA, no gain.
            d_dt = -connectivity_term
            return d_dt.flatten()

        # law_variant == 2 (Case-4): connectivity + collision avoidance, no
        # gain -- matches experiment/multi_unicycle_controller_double_angle_
        # experiment_ekf.py's commented Variant 2 exactly (that's the script
        # that actually drove the real robots; the 10,000,000 gain only
        # appears in turtlebot3_gazebo/scripts/'s demo copy of Variant 2,
        # never in the real hardware controller).
        e_M = self.Hbar_Mset @ p
        e_vec = e_M.reshape(2, 1)
        dik = float(np.linalg.norm(e_vec))
        xi_beta = 0.0
        R_psi_M = np.zeros((2, 2))
        if dik > 1e-6:
            asin_arg = float(np.clip(self.r / dik, -1.0, 1.0))
            psi_ik = 2 * asin(asin_arg)
            gik = e_vec / dik
            Rot_R = np.array([[cos(psi_ik / 2), -sin(psi_ik / 2)], [sin(psi_ik / 2), cos(psi_ik / 2)]])
            Rot_L = np.array([[cos(psi_ik / 2), sin(psi_ik / 2)], [-sin(psi_ik / 2), cos(psi_ik / 2)]])
            g_iRk = Rot_R @ gik
            g_iLk = Rot_L @ gik
            Pg_iRk = np.eye(2) - (g_iRk @ g_iRk.T)
            Pg_iLk = np.eye(2) - (g_iLk @ g_iLk.T)
            f_gik = (Pg_iLk @ g_iRk) + (Pg_iRk @ g_iLk)
            f_psi_ik = 0.5 * sin(2 * psi_ik) / (1 + cos(psi_ik))
            R_psi_M = f_psi_ik * f_gik

            beta_ik = (cos(psi_ik)) ** 2 - (cos(self.psi_crit)) ** 2 if psi_ik > self.psi_crit else 0.0
            denom_beta = self.delta ** 2 - beta_ik ** 2
            xi_beta = 0.0 if abs(denom_beta) < 1e-6 else 4 * beta_ik / denom_beta

        ca_term = self.Hbar_Mset.T @ R_psi_M @ np.array([[xi_beta]])
        d_dt = -(connectivity_term + ca_term)
        return d_dt.flatten()

    # ------------------------------------------------------------------
    def _finish(self, now, converged, reason):
        self.stop_all()
        self.should_exit = True
        self.converged = converged
        self.stop_reason = reason
        self.final_time = now
        self.get_logger().info(f'Stopping ({reason}) at t={now:.2f}s')

    def save_outputs(self):
        """Write log_data.csv (real schema, section 3) and control_status.json."""
        csv_path = os.path.join(self.log_dir, 'log_data.csv')
        header = ['time']
        for name in ROBOT_NAMES:
            header += [f'{name}_x', f'{name}_y', f'{name}_theta']
        header += ['d1', 'd2', 'd3', 'd4', 'd5']
        header += ['e1', 'e2', 'e3', 'e4', 'e5']
        header += ['lower_bound', 'upper_bound']

        import csv as csv_mod
        with open(csv_path, 'w', newline='') as f:
            writer = csv_mod.writer(f)
            writer.writerow(header)
            writer.writerows(self.rows)
        self.get_logger().info(f'Wrote {len(self.rows)} rows to {csv_path}')

        status = {
            'case': self.case_name,
            'law_variant': self.law_variant,
            'collision_avoidance': self.collision_avoidance,
            'converged': self.converged,
            'stop_reason': self.stop_reason or 'external_shutdown',
            'final_time_s': self.final_time,
            'rows_logged': len(self.rows),
            'conv_tol': self.conv_tol,
            'max_duration': self.max_duration,
        }
        tmp_path = os.path.join(self.log_dir, '.control_status.json.tmp')
        final_path = os.path.join(self.log_dir, 'control_status.json')
        with open(tmp_path, 'w') as f:
            json.dump(status, f, indent=2)
        os.rename(tmp_path, final_path)
        self.get_logger().info(f'Wrote {final_path}')


def main(args=None):
    rclpy.init(args=args)
    node = CaseFormationController()
    try:
        while rclpy.ok() and not node.should_exit:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        # Once we start cleaning up, a repeated/slow-arriving SIGINT (or the
        # orchestrator's follow-up SIGTERM) must not be able to interrupt us
        # mid-write -- that's how a prior run ended up with log_data.csv but
        # no control_status.json. Only SIGKILL can still stop this section.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        try:
            node.stop_all()
        except BaseException:
            pass
        try:
            node.save_outputs()
        except BaseException as e:
            node.get_logger().error(f'Failed to save outputs: {e}')
        try:
            node.destroy_node()
        except BaseException:
            pass
        try:
            rclpy.shutdown()
        except BaseException:
            pass


if __name__ == '__main__':
    main()
