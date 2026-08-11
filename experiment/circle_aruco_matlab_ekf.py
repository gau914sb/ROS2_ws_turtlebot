#!/us
import math
import os
from datetime import datetime

import matplotlib
matplotlib.use('Agg')   # non-interactive backend — safe without a display
import matplotlib.pyplot as plt
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
)

import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Image
from aruco_opencv_msgs.msg import ArucoDetection


# ─── tuneable defaults ────────────────────────────────────────────────────────
TARGET_MARKER_ID = 3
CENTER_X         = 0.0      # user-specified centre X (m)
CENTER_Y         = 0.0      # user-specified centre Y (m)
USE_DYNAMIC_CENTER = False  # if True, compute centre from initial position
RADIUS           = 0.3      # metres
SPEED            = 0.08     # m/s along the circle
K_ANGULAR        = 2.0      # proportional heading gain  (rad/s / rad)
K_GAIN           = 1   # lyapunov control gain (balanced for physical robot slip)
K_RADIAL         = 4.0     # radial correction gain to close the circle (increased to overpower slip drift)
OMEGA_MAX        = 2.0      # rad/s
DIRECTION        = 'ccw'    # 'ccw' or 'cw'
NUM_LAPS         = 2.0      # laps to complete (0 = infinite)
LOST_TIMEOUT     = 2.0      # s without detection → emergency stop
WORLD_FRAME      = 'ground_origin'
ARUCO_TOPIC      = '/aruco_detections'
CAMERA_TOPIC     = '/camera/camera/color/image_raw'
CMD_VEL_TOPIC    = '/cmd_vel'
TF_TIMEOUT       = 0.05
PLOT_SAVE_DIR    = os.path.expanduser('~/ros2_circle_plots')
# ─────────────────────────────────────────────────────────────────────────────



# EKF noise tuning (tuned to heavily trust camera over kinematic model to reduce phase lag)
Q_XY    = 0.05
Q_THETA = 0.1
R_XY    = 0.005
R_THETA = 0.01
GATE_THRESHOLD = 50.0  # Increased to prevent permanent track loss on sharp turns

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

def _wrap(angle: float) -> float:
    """Wrap angle to (−π, π]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class CircleArucoController(Node):
    """
    Drives a robot in a circle of a given radius using overhead ArUco feedback.

    Control law (Lyapunov-based from MATLAB):
        ω = dir_sign * (V + k_gain * ((x - cx)*cos(θ) + (y - cy)*sin(θ))) / r
        v = speed   (constant linear speed)

    The circle centre is fixed in world coordinates once the first detection
    arrives.  Heading comes from the ArUco marker quaternion (yaw in
    ground_origin frame).
    """

    def __init__(self):
        super().__init__('circle_aruco_controller')

        # ── ROS2 Parameters ───────────────────────────────────────────────────
        self.declare_parameter('target_marker_id', TARGET_MARKER_ID)
        self.declare_parameter('center_x',         CENTER_X)
        self.declare_parameter('center_y',         CENTER_Y)
        self.declare_parameter('use_dynamic_center', USE_DYNAMIC_CENTER)
        self.declare_parameter('radius',           RADIUS)
        self.declare_parameter('speed',            SPEED)
        self.declare_parameter('k_angular',        K_ANGULAR)
        self.declare_parameter('k_gain',           K_GAIN)
        self.declare_parameter('omega_max',        OMEGA_MAX)
        self.declare_parameter('direction',        DIRECTION)
        self.declare_parameter('num_laps',         NUM_LAPS)
        self.declare_parameter('lost_timeout',     LOST_TIMEOUT)
        self.declare_parameter('world_frame',      WORLD_FRAME)
        self.declare_parameter('aruco_topic',      ARUCO_TOPIC)
        self.declare_parameter('camera_topic',     CAMERA_TOPIC)
        self.declare_parameter('tf_timeout',       TF_TIMEOUT)

        self.target_marker_id = self.get_parameter('target_marker_id').value
        
        # Forcing the use of the global variables directly to ensure changes take effect immediately
        # (Bypassing ROS 2 parameter server caching)
        self.cx_param         = CENTER_X
        self.cy_param         = CENTER_Y
        self.use_dynamic_center = USE_DYNAMIC_CENTER
        self.radius           = RADIUS
        
        self.speed            = float(self.get_parameter('speed').value)
        self.k_angular        = float(self.get_parameter('k_angular').value)
        self.k_gain           = float(self.get_parameter('k_gain').value)
        self.omega_max        = float(self.get_parameter('omega_max').value)
        direction             = self.get_parameter('direction').value.strip().lower()
        self.num_laps         = float(self.get_parameter('num_laps').value)
        self.lost_timeout     = float(self.get_parameter('lost_timeout').value)
        self.world_frame      = self.get_parameter('world_frame').value
        aruco_topic           = self.get_parameter('aruco_topic').value
        camera_topic          = self.get_parameter('camera_topic').value
        self.tf_timeout_sec   = float(self.get_parameter('tf_timeout').value)

        # +1 = CCW (left turn),  −1 = CW (right turn)
        self.dir_sign = -1.0 if direction == 'cw' else 1.0

        self.marker_tf_frame = f'marker_{self.target_marker_id}'

        # ── State ─────────────────────────────────────────────────────────────
        self.phase: str                = 'WAIT'   # WAIT → CIRCLE → DONE
        self.running: bool             = True
        self.marker_ever_seen: bool    = False
        self.last_detection_time       = self.get_clock().now()
        self.last_image_time           = self.get_clock().now()

        # Circle centre in world frame (computed on first detection)
        self.cx: float | None = None
        self.cy: float | None = None
        self.target_x: float | None = None
        self.target_y: float | None = None

        # Angular progress tracking for lap counting
        self.last_angle: float | None = None   # angle of robot relative to centre
        self.accumulated_angle: float = 0.0    # total signed angle traversed (rad)
        self.target_angle: float      = abs(self.num_laps) * 2.0 * math.pi

        # ── Trajectory log ────────────────────────────────────────────────────
        self.traj_x: list   = []   # world-frame x positions (m)
        self.traj_y: list   = []   # world-frame y positions (m)
        self.traj_yaw: list = []   # heading (rad)
        self.traj_t: list   = []   # elapsed time (s)
        self._t0: float | None = None   # time of first detection

        # ── TF2 ──────────────────────────────────────────────────────────────
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── Publisher ────────────────────────────────────────────────────────
        self.cmd_pub = self.create_publisher(TwistStamped, CMD_VEL_TOPIC, 10)

        # ── QoS (aruco_opencv uses BEST_EFFORT) ──────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Subscribers ──────────────────────────────────────────────────────
        self.aruco_sub = self.create_subscription(
            ArucoDetection, aruco_topic, self._aruco_callback, sensor_qos)
        self.image_sub = self.create_subscription(
            Image, camera_topic, self._image_callback, sensor_qos)

        # ── Safety timer (10 Hz) ─────────────────────────────────────────────
        self.safety_timer = self.create_timer(0.1, self._safety_callback)
        self.ekf = RobotEKF('circle_bot')
        self.pose_received = False
        self.last_control_time = self.get_clock().now()
        self.last_traj_log_time = 0.0


        lap_str = f'{self.num_laps:.1f} lap(s)' if self.num_laps > 0 else 'infinite'
        self.get_logger().info(
            f'\n[CircleArucoController] Started.\n'
            f'  Marker ID   : {self.target_marker_id}  (TF: "{self.marker_tf_frame}")\n'
            f'  World frame : "{self.world_frame}"\n'
            f'  Radius      : {self.radius:.3f} m\n'
            f'  Center      : ({self.cx_param:.3f}, {self.cy_param:.3f}) [Dynamic: {self.use_dynamic_center}]\n'
            f'  Speed       : {self.speed:.3f} m/s\n'
            f'  K_angular   : {self.k_angular:.2f}  |  ω_max: {self.omega_max:.2f} rad/s\n'
            f'  Direction   : {direction.upper()}  (dir_sign={self.dir_sign:+.0f})\n'
            f'  Laps        : {lap_str}\n'
            f'Waiting for first ArUco detection…'
        )

    # ── Quaternion helper ─────────────────────────────────────────────────────

    @staticmethod
    def _quat_to_yaw(q) -> float:
        """Extract yaw (heading) from a quaternion."""
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    # ── Image callback ────────────────────────────────────────────────────────

    def _image_callback(self, msg: Image):
        """Track camera liveness only."""
        self.last_image_time = self.get_clock().now()

    # ── Main ArUco callback ───────────────────────────────────────────────────

    def _aruco_callback(self, msg: ArucoDetection):
        if not self.running: return
        self.last_detection_time = self.get_clock().now()
        target_visible = any(m.marker_id == self.target_marker_id for m in msg.markers)
        if not target_visible: return
        try:
            tf = self.tf_buffer.lookup_transform(
                self.world_frame, self.marker_tf_frame, rclpy.time.Time(), Duration(seconds=self.tf_timeout_sec),
            )
        except Exception as e:
            return
            
        z_x = tf.transform.translation.x
        z_y = tf.transform.translation.y
        z_yaw = self._quat_to_yaw(tf.transform.rotation)
        
        now = self.get_clock().now()
        actual_dt = (now - self.last_control_time).nanoseconds / 1e9
        self.last_control_time = now
        
        if actual_dt > 0.2 or actual_dt <= 0.0:
            actual_dt = 0.05
            
        # Predict forward by the time since last camera frame
        self.ekf.predict(actual_dt)
        
        if not self.pose_received:
            self.ekf.initialize(z_x, z_y, z_yaw)
            self.pose_received = True
        else:
            accepted = self.ekf.update(z_x, z_y, z_yaw)
            if not accepted:
                self.get_logger().warn('ArUco measurement rejected (outlier spike)!', throttle_duration_sec=1.0)
                
        # Use the EKF filtered state for everything!
        # The EKF will reject marker orientation spikes, protecting the sensitive Lyapunov law.
        cur_x, cur_y, cur_yaw = self.ekf.position

        # ── Phase WAIT: first detection → set circle centre ───────────────────
        if not self.marker_ever_seen:
            self.marker_ever_seen = True

            if self.use_dynamic_center:
                # Place the centre at `radius` distance perpendicular to
                # the robot's current heading (left for CCW, right for CW)
                centre_bearing = cur_yaw + self.dir_sign * math.pi / 2.0
                self.cx = cur_x + self.radius * math.cos(centre_bearing)
                self.cy = cur_y + self.radius * math.sin(centre_bearing)
                self.phase = 'CIRCLE'
                self.get_logger().info(
                    f'First detection at ({cur_x:.3f}, {cur_y:.3f}) m, yaw={math.degrees(cur_yaw):.1f}°\n'
                    f'Circle centre: ({self.cx:.3f}, {self.cy:.3f}) m (dynamic=True)\n'
                    f'Starting circle motion…'
                )
            else:
                self.cx = self.cx_param
                self.cy = self.cy_param
                # Compute closest point on circumference to travel to
                dx = cur_x - self.cx
                dy = cur_y - self.cy
                dist = math.hypot(dx, dy)
                if dist < 0.01:
                    self.target_x = self.cx + self.radius
                    self.target_y = self.cy
                else:
                    self.target_x = self.cx + self.radius * (dx / dist)
                    self.target_y = self.cy + self.radius * (dy / dist)
                
                self.phase = 'GOTO_CIRCUMFERENCE'
                self.get_logger().info(
                    f'First detection at ({cur_x:.3f}, {cur_y:.3f}) m, yaw={math.degrees(cur_yaw):.1f}°\n'
                    f'Circle centre: ({self.cx:.3f}, {self.cy:.3f}) m (dynamic=False)\n'
                    f'Travelling to circumference point ({self.target_x:.3f}, {self.target_y:.3f})…'
                )

            # Initialise angle tracking
            self.last_angle = math.atan2(cur_y - self.cy, cur_x - self.cx)
            self.accumulated_angle = 0.0
            self._t0 = self.get_clock().now().nanoseconds * 1e-9

            return   # wait one frame so TF is stable

        now_sec = self.get_clock().now().nanoseconds * 1e-9

        if self.phase == 'GOTO_CIRCUMFERENCE':
            tx = self.target_x - cur_x
            ty = self.target_y - cur_y
            dist_to_target = math.hypot(tx, ty)
            
            ex = self.cx - cur_x
            ey = self.cy - cur_y
            dist_to_centre = math.hypot(ex, ey)
            
            if dist_to_target < 0.05 or abs(dist_to_centre - self.radius) < 0.05:
                self.phase = 'CIRCLE'
                self.get_logger().info('Reached circumference. Switching to CIRCLE phase.')
                self.last_angle = math.atan2(cur_y - self.cy, cur_x - self.cx)
                self.accumulated_angle = 0.0
                return
                
            heading_to_target = math.atan2(ty, tx)
            e_theta = _wrap(heading_to_target - cur_yaw)
            omega = self.k_angular * e_theta
            omega = max(-self.omega_max, min(self.omega_max, omega))
            
            # Turn in place if heading error is large
            if abs(e_theta) > 0.15:  # ~8.6 degrees
                v = 0.0
            else:
                v = self.speed
                
            self._publish_cmd(v, omega)
            
            self.get_logger().info(
                f'[GOTO] Pos: ({cur_x:.3f},{cur_y:.3f}) m | '
                f'dist_to_tgt: {dist_to_target:.3f} m | '
                f'e_θ: {math.degrees(e_theta):+.1f}° | '
                f'ω: {omega:+.3f} rad/s',
                throttle_duration_sec=0.3,
            )
            return

        if self.phase != 'CIRCLE':
            return

        # ── Phase CIRCLE ──────────────────────────────────────────────────────

        # Record trajectory point ONLY during the CIRCLE phase for accurate RMS error calculation
        if now_sec - self.last_traj_log_time >= 0.1:
            self.traj_x.append(cur_x)
            self.traj_y.append(cur_y)
            self.traj_yaw.append(cur_yaw)
            self.traj_t.append(now_sec - (self._t0 or now_sec))
            self.last_traj_log_time = now_sec

        # 1. Distance to centre for logging
        ex = self.cx - cur_x
        ey = self.cy - cur_y
        dist_to_centre = math.hypot(ex, ey)

        # 2. Lyapunov-based control law (from MATLAB ODE45 simulation)
        # u = (V + k * ((x - cx)*cos(yaw) + (y - cy)*sin(yaw))) / r
        # Multiplied by dir_sign to support both CCW and CW directions.
        term = (cur_x - self.cx) * math.cos(cur_yaw) + (cur_y - self.cy) * math.sin(cur_yaw)
        u = (self.speed + self.k_gain * term) / self.radius
        
        # # CRITICAL FIX: The MATLAB control law above ONLY stabilizes heading, not radial distance!
        # # We MUST add a linear radial correction term so it actively steers back to the exact radius.
        # # This is strictly bounded to prevent the robot from commanding massive negative velocities and spinning out.
        # radial_error = dist_to_centre - self.radius
        # radial_correction = K_RADIAL * radial_error
        
        # # Bound the radial influence to prevent spin-outs if it gets too far off-track
        # radial_correction = max(-0.5, min(0.5, radial_correction))
        # u += radial_correction
        
        omega = self.dir_sign * u
        omega = max(-self.omega_max, min(self.omega_max, omega))
        
        # 3. Publish command
        self._publish_cmd(self.speed, omega)
        
        # dummy variable to avoid breaking logger below
        e_theta = 0.0

        # 6. Lap counting — accumulate signed angle traversed around centre
        current_angle = math.atan2(cur_y - self.cy, cur_x - self.cx)
        delta = _wrap(current_angle - self.last_angle)
        
        # Accumulate the angular progress (multiply by dir_sign to always be positive for forward progress)
        self.accumulated_angle += delta * self.dir_sign
        
        self.last_angle = current_angle

        laps_done = self.accumulated_angle / (2.0 * math.pi)

        self.get_logger().info(
            f'Pos: ({cur_x:.3f},{cur_y:.3f}) m | '
            f'yaw: {math.degrees(cur_yaw):.1f}° | '
            f'e_θ: {math.degrees(e_theta):+.1f}° | '
            f'ω: {omega:+.3f} rad/s | '
            f'dist_to_centre: {dist_to_centre:.3f} m | '
            f'laps: {laps_done:.3f}/{self.num_laps if self.num_laps > 0 else "∞"}',
            throttle_duration_sec=0.3,
        )

        # 7. Stop when target laps completed (if num_laps > 0)
        if self.num_laps > 0 and self.accumulated_angle >= self.target_angle:
            self._stop(
                f'Completed {laps_done:.2f} lap(s) '
                f'({math.degrees(self.accumulated_angle):.1f}°). '
                f'Circle radius ≈ {dist_to_centre:.3f} m.'
            )

    # ── Safety timer (10 Hz) ─────────────────────────────────────────────────

    def _safety_callback(self):
        if not self.running:
            return

        now = self.get_clock().now()

        # Camera liveness
        cam_age = (now - self.last_image_time).nanoseconds / 1e9
        if cam_age > 3.0:
            self.get_logger().warn(
                f'No camera image for {cam_age:.1f} s — is the camera running?',
                throttle_duration_sec=5.0,
            )

        if not self.marker_ever_seen:
            self.get_logger().info(
                f'Waiting for marker ID={self.target_marker_id} '
                f'(TF: "{self.marker_tf_frame}")…',
                throttle_duration_sec=2.0,
            )
            return

        # Lost-marker emergency stop
        marker_age = (now - self.last_detection_time).nanoseconds / 1e9
        if marker_age > self.lost_timeout:
            self._stop(
                f'EMERGENCY STOP: marker "{self.marker_tf_frame}" lost for '
                f'{marker_age:.1f} s (timeout={self.lost_timeout:.1f} s).'
            )

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _publish_cmd(self, linear_x: float, angular_z: float = 0.0):
        """Publish a stamped velocity command."""
        msg = TwistStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x  = float(linear_x)
        msg.twist.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)
        self.ekf.last_v = float(linear_x)
        self.ekf.last_omega = float(angular_z)

    def _save_plot(self):
        """
        Save a trajectory PNG showing:
          - actual robot path (solid line)
          - ideal circle (dashed)
          - start point and circle centre
          - heading arrows sampled every N points
        """
        if len(self.traj_x) < 2:
            self.get_logger().warn('Not enough trajectory data to plot.')
            return

        os.makedirs(PLOT_SAVE_DIR, exist_ok=True)
        ts  = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        path = os.path.join(PLOT_SAVE_DIR, f'circle_traj_{ts}.png')

        fig, axes = plt.subplots(1, 2, figsize=(14, 6))
        fig.suptitle(
            f'Circle Controller  |  r={self.radius:.2f} m  '
            f'v={self.speed:.3f} m/s  K={self.k_angular:.2f}  '
            f'dir={"CCW" if self.dir_sign > 0 else "CW"}',
            fontsize=13, fontweight='bold'
        )

        # ── Left: XY trajectory ───────────────────────────────────────────────
        ax = axes[0]
        xs = np.array(self.traj_x)
        ys = np.array(self.traj_y)

        # Ideal circle
        if self.cx is not None:
            theta_ideal = np.linspace(0, 2 * np.pi, 360)
            ax.plot(
                self.cx + self.radius * np.cos(theta_ideal),
                self.cy + self.radius * np.sin(theta_ideal),
                '--', color='lightgray', linewidth=1.5, label='Ideal circle'
            )
            # Highlight the center of the circle explicitly
            ax.plot(self.cx, self.cy, 'r*', markersize=20, label=f'Center ({self.cx:.2f}, {self.cy:.2f})')
            ax.annotate(f'Center\n({self.cx:.2f}, {self.cy:.2f})', 
                        xy=(self.cx, self.cy), xytext=(self.cx + 0.1, self.cy + 0.1),
                        arrowprops=dict(facecolor='red', shrink=0.05, width=1.5, headwidth=6),
                        fontsize=10, color='red', fontweight='bold',
                        bbox=dict(boxstyle='round,pad=0.3', fc='yellow', alpha=0.8))

        # Actual path (coloured by time)
        sc = ax.scatter(xs, ys, c=self.traj_t, cmap='plasma',
                        s=8, zorder=3, label='Actual path')
        fig.colorbar(sc, ax=ax, label='Time (s)')

        # Start / end markers
        ax.plot(xs[0],  ys[0],  'go', markersize=10, label='Start')
        ax.plot(xs[-1], ys[-1], 'rs', markersize=10, label='End')

        # Heading arrows (every ~20 points)
        step = max(1, len(xs) // 20)
        arrow_len = self.radius * 0.12
        for i in range(0, len(xs), step):
            ax.annotate(
                '', fontsize=0,
                xy=(xs[i] + arrow_len * math.cos(self.traj_yaw[i]),
                    ys[i] + arrow_len * math.sin(self.traj_yaw[i])),
                xytext=(xs[i], ys[i]),
                arrowprops=dict(arrowstyle='->', color='steelblue', lw=1.2)
            )

        ax.set_aspect('equal')
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title('XY Trajectory')
        ax.legend(fontsize=8, loc='best')
        ax.grid(True, alpha=0.3)

        # ── Right: radius error over time ─────────────────────────────────────
        ax2 = axes[1]
        if self.cx is not None:
            radii  = np.hypot(xs - self.cx, ys - self.cy)
            errors = radii - self.radius
            ax2.plot(self.traj_t, errors * 100, color='crimson', linewidth=1.5)
            ax2.axhline(0, color='gray', linestyle='--', linewidth=1)
            ax2.fill_between(self.traj_t, errors * 100, 0,
                             alpha=0.15, color='crimson')
            ax2.set_xlabel('Time (s)')
            ax2.set_ylabel('Radius error (cm)')
            ax2.set_title('Tracking Error  (actual − ideal radius)')
            ax2.grid(True, alpha=0.3)

            rms_err = float(np.sqrt(np.mean(errors**2))) * 100
            ax2.text(0.02, 0.96,
                     f'RMS error: {rms_err:.1f} cm',
                     transform=ax2.transAxes,
                     verticalalignment='top', fontsize=10,
                     bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.6))

        plt.tight_layout()
        fig.savefig(path, dpi=150, bbox_inches='tight')
        plt.close(fig)
        self.get_logger().info(f'[PLOT] Trajectory saved to {path}')

    def _stop(self, reason: str):
        """Zero velocity, cancel timers, destroy subscribers, save plot."""
        if not self.running:
            return
        self.running = False
        self.phase   = 'DONE'
        self._publish_cmd(0.0, 0.0)
        self._publish_cmd(0.0, 0.0)   # send twice for reliability
        self.safety_timer.cancel()
        self.destroy_subscription(self.aruco_sub)
        self.destroy_subscription(self.image_sub)
        self.get_logger().info(f'[STOPPED] {reason}')
        self._save_plot()


def main(args=None):
    rclpy.init(args=args)
    node = CircleArucoController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._publish_cmd(0.0, 0.0)
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
