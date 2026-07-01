#!/usr/bin/env python3.12

import math
import os
import signal
import sys
import threading

# ── All 4 real robots and the ArUco tracker live on domain 30 ─────────────────
os.environ.setdefault('ROS_DOMAIN_ID', '30')

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from geometry_msgs.msg import Twist
from aruco_opencv_msgs.msg import ArucoDetection

# ─── Phase labels ─────────────────────────────────────────────────────────────
PHASE_INIT                = 'INIT'
PHASE_ALIGN_TO_GOAL       = 'ALIGN_TO_GOAL'
PHASE_DRIVE_TO_GOAL       = 'DRIVE_TO_GOAL'
PHASE_ALIGN_FINAL_HEADING = 'ALIGN_FINAL_HEADING'
PHASE_RECHECK_POSITION   = 'RECHECK_POSITION'
PHASE_DONE                = 'DONE'

# ─── Global shared defaults ──────────────────────────────────────────────────
SPEED                 = 0.05    # m/s  max linear speed
SPEED_MIN             = 0.025   # m/s  min linear speed (avoid stall near goal)
DECEL_RADIUS          = 0.15    # m    start decelerating when dist < this
BRAKE_ZONE            = 0.08    # m    stop completely to shed momentum, then nudge
ANG_SPEED             = 0.10    # rad/s  max angular speed
ANG_SPEED_MIN         = 0.03    # rad/s  min angular speed (avoid stall near tol)
POS_TOL_M             = 0.04    # m  
HEADING_TOL_DEG       = 3.0     # °    tighter: was 5.0
HEADING_KP            = 1.5
MARKER_YAW_OFFSET_DEG = 0.0
ANG_DIR_SIGN          = 1
LOST_TIMEOUT          = 10.0
WORLD_FRAME           = 'ground_origin'
ARUCO_TOPIC           = '/aruco_detections'
TF_TIMEOUT            = 0.05

# ─── Per-bot configuration ────────────────────────────────────────────────────
# Each entry: (marker_id, goal_x, goal_y, final_heading_deg, cmd_vel_topic)
BOT_CONFIGS = [
    # Bot 0 — Q1 (+x, +y)
    dict(marker_id=0, goal_x= 0.20, goal_y= 0.20, final_heading_deg=  0.0,
         cmd_vel_topic='/bot_0/cmd_vel'),
    # Bot 1 — Q2 (-x, +y)
    dict(marker_id=1, goal_x=-0.20, goal_y= 0.20, final_heading_deg= 90.0,
         cmd_vel_topic='/bot_1/cmd_vel'),
    # Bot 2 — Q3 (-x, -y)
    dict(marker_id=2, goal_x=-0.20, goal_y=-0.20, final_heading_deg=180.0,
         cmd_vel_topic='/bot_2/cmd_vel'),
    # Bot 3 — Q4 (+x, -y)
    dict(marker_id=3, goal_x= 0.20, goal_y=-0.20, final_heading_deg=270.0,
         cmd_vel_topic='/bot_3/cmd_vel'),
]
# ─────────────────────────────────────────────────────────────────────────────


def _quadrant(x: float, y: float) -> str:
    if x >= 0 and y >= 0:
        return 'Q1 (+x,+y)'
    if x < 0 and y >= 0:
        return 'Q2 (-x,+y)'
    if x < 0 and y < 0:
        return 'Q3 (-x,-y)'
    return 'Q4 (+x,-y)'


def _quat_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _angle_wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class BotController(Node):
    """
    Single-robot 3-phase closed-loop pose controller.
    Identical logic to GotoPoseAruco in goto_pose_aruco.py but parameterised
    at construction time so multiple instances can co-exist in one process.
    """

    def __init__(self,
                 bot_index: int,
                 marker_id: int,
                 goal_x: float,
                 goal_y: float,
                 final_heading_deg: float,
                 cmd_vel_topic: str,
                 aruco_topic: str       = ARUCO_TOPIC,
                 world_frame: str       = WORLD_FRAME,
                 speed: float           = SPEED,
                 speed_min: float       = SPEED_MIN,
                 decel_radius: float    = DECEL_RADIUS,
                 brake_zone: float      = BRAKE_ZONE,
                 ang_speed: float       = ANG_SPEED,
                 ang_speed_min: float   = ANG_SPEED_MIN,
                 pos_tol_m: float       = POS_TOL_M,
                 heading_tol_deg: float = HEADING_TOL_DEG,
                 heading_kp: float      = HEADING_KP,
                 marker_yaw_offset_deg: float = MARKER_YAW_OFFSET_DEG,
                 ang_dir_sign: int      = ANG_DIR_SIGN,
                 lost_timeout: float    = LOST_TIMEOUT,
                 tf_timeout: float      = TF_TIMEOUT):

        node_name = f'bot{bot_index}_goto_pose'
        super().__init__(node_name)

        # Store config
        self.bot_index          = bot_index
        self.target_marker_id   = marker_id
        self.goal_x             = goal_x
        self.goal_y             = goal_y
        self.final_heading_rad  = math.radians(final_heading_deg)
        self.speed              = speed
        self.speed_min          = speed_min
        self.decel_radius       = decel_radius
        self.brake_zone         = brake_zone
        self.ang_speed          = ang_speed
        self.ang_speed_min      = ang_speed_min
        self.pos_tol_m          = pos_tol_m
        self.heading_tol_rad    = math.radians(heading_tol_deg)
        self.heading_kp         = heading_kp
        self.marker_yaw_offset_rad = math.radians(marker_yaw_offset_deg)
        self.ang_dir_sign       = float(ang_dir_sign)
        self.lost_timeout       = lost_timeout
        self.world_frame        = world_frame
        self.tf_timeout_sec     = tf_timeout
        self.marker_tf_frame    = f'marker_{marker_id}'

        # State
        self.phase: str              = PHASE_INIT
        self.running: bool           = True
        self.marker_ever_seen: bool  = False
        self.last_detection_time     = self.get_clock().now()
        self.aruco_frame_count: int  = 0
        self.aruco_target_count: int = 0
        self.last_seen_ids: list     = []

        self.start_x:   float | None = None
        self.start_y:   float | None = None
        self.start_yaw: float | None = None
        # Position when Phase 2 completed (before heading rotation drifts it)
        self.pos_at_goal_x: float | None = None
        self.pos_at_goal_y: float | None = None
        self.final_x:   float | None = None
        self.final_y:   float | None = None
        self.final_yaw: float | None = None

        # TF2
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publisher — /bot_N/cmd_vel expects TwistStamped (ROS_DOMAIN_ID=30)
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        # ArUco subscriber (BEST_EFFORT — same as aruco_opencv)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.aruco_sub = self.create_subscription(
            ArucoDetection, aruco_topic, self._aruco_callback, sensor_qos)

        # Safety watchdog at 10 Hz
        self.safety_timer = self.create_timer(0.1, self._safety_callback)

        self.get_logger().info(
            f'\n[Bot{bot_index}] GotoPoseAruco — 3-phase controller\n'
            f'  Marker ID     : {marker_id} (TF: "{self.marker_tf_frame}")\n'
            f'  Goal          : ({goal_x:.3f}, {goal_y:.3f}) m  '
            f'[{_quadrant(goal_x, goal_y)}]\n'
            f'  Final heading : {final_heading_deg:.1f}°\n'
            f'  cmd_vel topic : {cmd_vel_topic}\n'
            f'Waiting for marker ID={marker_id}…'
        )

    # ── ArUco callback (main control loop) ───────────────────────────────────

    def _aruco_callback(self, msg: ArucoDetection):
        if not self.running or self.phase == PHASE_DONE:
            return

        self.aruco_frame_count += 1
        self.last_seen_ids = sorted(m.marker_id for m in msg.markers)
        target_visible = self.target_marker_id in self.last_seen_ids
        if target_visible:
            self.aruco_target_count += 1
            self.last_detection_time = self.get_clock().now()

        self.get_logger().info(
            f'[Bot{self.bot_index}|ArUco] frame #{self.aruco_frame_count}  '
            f'visible IDs: {self.last_seen_ids}  '
            f'target(ID={self.target_marker_id}): '
            f'{"SEEN ✓" if target_visible else "NOT SEEN ✗"}',
            throttle_duration_sec=1.0,
        )

        if not target_visible:
            return

        # TF2 lookup
        try:
            tf = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.marker_tf_frame,
                rclpy.time.Time(),
                Duration(seconds=self.tf_timeout_sec),
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(
                f'[Bot{self.bot_index}] TF2 lookup failed: {e}',
                throttle_duration_sec=1.0,
            )
            return

        cur_x   = tf.transform.translation.x
        cur_y   = tf.transform.translation.y
        cur_yaw = _angle_wrap(
            _quat_to_yaw(tf.transform.rotation) + self.marker_yaw_offset_rad)

        if not self.marker_ever_seen:
            self.marker_ever_seen = True
            self.start_x   = cur_x
            self.start_y   = cur_y
            self.start_yaw = cur_yaw
            self._log_nav_plan(cur_x, cur_y, cur_yaw)
            self.phase = PHASE_ALIGN_TO_GOAL

        if self.phase == PHASE_ALIGN_TO_GOAL:
            self._phase_align_to_goal(cur_x, cur_y, cur_yaw)
        elif self.phase == PHASE_DRIVE_TO_GOAL:
            self._phase_drive_to_goal(cur_x, cur_y, cur_yaw)
        elif self.phase == PHASE_ALIGN_FINAL_HEADING:
            self._phase_align_final_heading(cur_x, cur_y, cur_yaw)
        elif self.phase == PHASE_RECHECK_POSITION:
            self._phase_recheck_position(cur_x, cur_y, cur_yaw)

    # ── Phase handlers ────────────────────────────────────────────────────────

    def _log_nav_plan(self, cur_x, cur_y, cur_yaw):
        dx      = self.goal_x - cur_x
        dy      = self.goal_y - cur_y
        dist    = math.hypot(dx, dy)
        hdg_deg = math.degrees(math.atan2(dy, dx))
        start_q = _quadrant(cur_x, cur_y)
        goal_q  = _quadrant(self.goal_x, self.goal_y)
        self.get_logger().info(
            f'\n'
            f'╔══ [Bot{self.bot_index}] NAVIGATION PLAN ══════════════════════╗\n'
            f'║  FROM : ({cur_x:.4f}, {cur_y:.4f}) m  [{start_q}]\n'
            f'║  FROM heading : {math.degrees(cur_yaw):.2f}°\n'
            f'║  TO   : ({self.goal_x:.4f}, {self.goal_y:.4f}) m  [{goal_q}]\n'
            f'║  FINAL heading: {math.degrees(self.final_heading_rad):.2f}°\n'
            f'║  Distance     : {dist:.4f} m\n'
            f'║  Turn to align: {hdg_deg:.2f}°\n'
            f'╚════════════════════════════════════════════════════╝\n'
            f'Phase 1 — ALIGN_TO_GOAL'
        )

    def _phase_align_to_goal(self, cur_x, cur_y, cur_yaw):
        dx = self.goal_x - cur_x
        dy = self.goal_y - cur_y
        desired_yaw = math.atan2(dy, dx)
        err = _angle_wrap(desired_yaw - cur_yaw)

        self.get_logger().info(
            f'[Bot{self.bot_index}|ALIGN_TO_GOAL] '
            f'err={math.degrees(err):.2f}°  '
            f'desired={math.degrees(desired_yaw):.2f}°  '
            f'cur={math.degrees(cur_yaw):.2f}°',
            throttle_duration_sec=0.3,
        )

        if abs(err) <= self.heading_tol_rad:
            self._publish_cmd(0.0, 0.0)
            self.get_logger().info(
                f'[Bot{self.bot_index}] ✓ Aligned. → Phase 2 DRIVE_TO_GOAL')
            self.phase = PHASE_DRIVE_TO_GOAL
        else:
            # Proportional angular speed with minimum to avoid stalling near tol
            raw = self.heading_kp * abs(err)
            omega = self.ang_dir_sign * math.copysign(
                max(self.ang_speed_min, min(self.ang_speed, raw)), err)
            self._publish_cmd(0.0, omega)

    def _phase_drive_to_goal(self, cur_x, cur_y, cur_yaw):
        dx   = self.goal_x - cur_x
        dy   = self.goal_y - cur_y
        dist = math.hypot(dx, dy)

        # ── Zone 1: within tolerance → snapshot position, go to Phase 3 ────────
        if dist <= self.pos_tol_m:
            self._publish_cmd(0.0, 0.0)
            self.pos_at_goal_x = cur_x   # record BEFORE rotation drift happens
            self.pos_at_goal_y = cur_y
            self.get_logger().info(
                f'[Bot{self.bot_index}] ✓ Goal reached (dist={dist*100:.1f} cm). '
                f'→ Phase 3 ALIGN_FINAL_HEADING')
            self.phase = PHASE_ALIGN_FINAL_HEADING
            return

        # ── Zone 2: brake zone → full stop, shed momentum, then nudge ─────────
        # The robot stops completely here. On the next ArUco frame it re-enters
        # this function; if dist > pos_tol_m it gets a single nudge at speed_min
        # toward the goal, then stops again. This eliminates overshoot.
        if dist < self.brake_zone:
            desired_yaw = math.atan2(dy, dx)
            heading_err = _angle_wrap(desired_yaw - cur_yaw)
            # tiny heading correction while stopped
            omega = self.ang_dir_sign * self.heading_kp * heading_err
            omega = max(-self.ang_speed_min, min(self.ang_speed_min, omega))
            # nudge at minimum speed to creep toward goal without overshoot
            self._publish_cmd(self.speed_min * 0.6, omega)
            self.get_logger().info(
                f'[Bot{self.bot_index}|BRAKE_ZONE] dist={dist*100:.1f} cm  '
                f'creeping at {self.speed_min*0.6:.3f} m/s',
                throttle_duration_sec=0.2,
            )
            return

        # ── Zone 3: decel zone → proportional speed ────────────────────────────
        if dist < self.decel_radius:
            v = self.speed_min + (self.speed - self.speed_min) * (dist / self.decel_radius)
            v = max(self.speed_min, min(self.speed, v))
        else:
            v = self.speed

        desired_yaw = math.atan2(dy, dx)
        heading_err = _angle_wrap(desired_yaw - cur_yaw)
        omega = self.ang_dir_sign * self.heading_kp * heading_err
        omega = max(-self.ang_speed, min(self.ang_speed, omega))
        self._publish_cmd(v, omega)
        self.get_logger().info(
            f'[Bot{self.bot_index}|DRIVE_TO_GOAL] '
            f'dist={dist*100:.1f} cm  v={v:.3f} m/s  '
            f'pos=({cur_x:.4f},{cur_y:.4f})  '
            f'hdg={math.degrees(cur_yaw):.2f}°',
            throttle_duration_sec=0.3,
        )

    def _phase_align_final_heading(self, cur_x, cur_y, cur_yaw):
        err = _angle_wrap(self.final_heading_rad - cur_yaw)

        self.get_logger().info(
            f'[Bot{self.bot_index}|ALIGN_FINAL_HEADING] '
            f'err={math.degrees(err):.2f}°  '
            f'desired={math.degrees(self.final_heading_rad):.2f}°  '
            f'cur={math.degrees(cur_yaw):.2f}°',
            throttle_duration_sec=0.3,
        )

        if abs(err) <= self.heading_tol_rad:
            self.final_x   = cur_x
            self.final_y   = cur_y
            self.final_yaw = cur_yaw
            # Check if rotation drift pushed us outside pos tolerance
            drift = math.hypot(cur_x - self.goal_x, cur_y - self.goal_y)
            if drift > self.pos_tol_m:
                self.get_logger().warn(
                    f'[Bot{self.bot_index}] Rotation drift = {drift*100:.1f} cm '
                    f'> {self.pos_tol_m*100:.1f} cm tol. '
                    f'→ Phase 4 RECHECK_POSITION to correct.')
                self.phase = PHASE_RECHECK_POSITION
            else:
                self._finish(cur_x, cur_y, cur_yaw)
        else:
            # Proportional angular speed with minimum to avoid stalling near tol
            raw = self.heading_kp * abs(err)
            omega = self.ang_dir_sign * math.copysign(
                max(self.ang_speed_min, min(self.ang_speed, raw)), err)
            self._publish_cmd(0.0, omega)

    def _phase_recheck_position(self, cur_x, cur_y, cur_yaw):
        """
        Phase 4: re-correct position after heading rotation caused drift.
        Re-drives to goal (using the same brake-zone logic), then
        re-aligns heading once more.
        """
        dx   = self.goal_x - cur_x
        dy   = self.goal_y - cur_y
        dist = math.hypot(dx, dy)

        self.get_logger().info(
            f'[Bot{self.bot_index}|RECHECK_POS] dist={dist*100:.1f} cm',
            throttle_duration_sec=0.3,
        )

        if dist <= self.pos_tol_m:
            self._publish_cmd(0.0, 0.0)
            self.pos_at_goal_x = cur_x
            self.pos_at_goal_y = cur_y
            self.get_logger().info(
                f'[Bot{self.bot_index}] ✓ Position re-corrected ({dist*100:.1f} cm). '
                f'→ Re-aligning heading.')
            self.phase = PHASE_ALIGN_FINAL_HEADING
            return

        # Creep toward goal (brake-zone speed — no overshoot)
        desired_yaw = math.atan2(dy, dx)
        heading_err = _angle_wrap(desired_yaw - cur_yaw)
        omega = self.ang_dir_sign * self.heading_kp * heading_err
        omega = max(-self.ang_speed_min, min(self.ang_speed_min, omega))
        self._publish_cmd(self.speed_min * 0.6, omega)

    def _finish(self, cur_x, cur_y, cur_yaw):
        """Record final pose and stop."""
        self.final_x   = cur_x
        self.final_y   = cur_y
        self.final_yaw = cur_yaw
        pos_err = math.hypot(cur_x - self.goal_x, cur_y - self.goal_y) * 100
        hdg_err = abs(math.degrees(_angle_wrap(cur_yaw - self.final_heading_rad)))
        self._stop(
            f'═══ [Bot{self.bot_index}] TASK COMPLETE ════════════════\n'
            f'  START    : ({self.start_x:.4f}, {self.start_y:.4f}) m  '
            f'[{_quadrant(self.start_x, self.start_y)}]  '
            f'hdg {math.degrees(self.start_yaw):.2f}°\n'
            f'  FINAL    : ({self.final_x:.4f}, {self.final_y:.4f}) m  '
            f'[{_quadrant(self.final_x, self.final_y)}]  '
            f'hdg {math.degrees(self.final_yaw):.2f}°\n'
            f'  Goal     : ({self.goal_x:.4f}, {self.goal_y:.4f}) m  '
            f'hdg {math.degrees(self.final_heading_rad):.2f}°\n'
            f'  Pos@goal : ({self.pos_at_goal_x:.4f}, {self.pos_at_goal_y:.4f}) m  '
            f'[before rotation]\n'
            f'  Pos err  : {pos_err:.1f} cm  Hdg err: {hdg_err:.1f}°\n'
            f'════════════════════════════════════════════════════'
        )

    # ── Safety watchdog ───────────────────────────────────────────────────────

    def _safety_callback(self):
        if not self.running or self.phase == PHASE_DONE:
            return

        if not self.marker_ever_seen:
            self.get_logger().info(
                f'[Bot{self.bot_index}] ArUco frames rcvd: {self.aruco_frame_count}  '
                f'last IDs: {self.last_seen_ids}  '
                f'| waiting for marker ID={self.target_marker_id}…',
                throttle_duration_sec=3.0,
            )
            if self.aruco_frame_count == 0:
                self.get_logger().warn(
                    f'[Bot{self.bot_index}] No ArUco frames yet — '
                    'is aruco_opencv running?',
                    throttle_duration_sec=5.0,
                )
            return

        marker_age = (
            self.get_clock().now() - self.last_detection_time
        ).nanoseconds / 1e9

        if marker_age > self.lost_timeout:
            self._stop(
                f'[Bot{self.bot_index}] EMERGENCY STOP: '
                f'marker "{self.marker_tf_frame}" lost for {marker_age:.1f} s'
            )

    # ── Helpers ───────────────────────────────────────────────────────────────

def _publish_cmd(self, linear_x: float, angular_z: float = 0.0):
        msg = Twist()
        msg.linear.x  = float(linear_x)
        msg.angular.z = float(angular_z)
        try:
            self.cmd_pub.publish(msg)
        except Exception:
            pass # Suppresses the pybind11 context error on Ctrl+C

    def _stop(self, reason: str):
        if not self.running:
            return
        self.running = False
        self.phase   = PHASE_DONE
        for _ in range(5):
            self._publish_cmd(0.0, 0.0)
        self.safety_timer.cancel()
        self.destroy_subscription(self.aruco_sub)
        self.get_logger().info(f'[STOPPED] {reason}')

    def get_result(self) -> dict:
        """Return a summary dict of this bot's navigation result."""
        if self.final_x is not None:
            pos_err_cm = math.hypot(
                self.final_x - self.goal_x,
                self.final_y - self.goal_y) * 100.0
            hdg_err_deg = abs(math.degrees(
                _angle_wrap(self.final_yaw - self.final_heading_rad)))
            status = 'DONE'
        else:
            pos_err_cm  = float('nan')
            hdg_err_deg = float('nan')
            status = 'INCOMPLETE / EMERGENCY-STOPPED'
        return dict(
            bot_index   = self.bot_index,
            marker_id   = self.target_marker_id,
            goal        = (self.goal_x, self.goal_y),
            final_pos   = (self.final_x, self.final_y),
            goal_hdg    = math.degrees(self.final_heading_rad),
            final_hdg   = math.degrees(self.final_yaw) if self.final_yaw is not None else float('nan'),
            pos_err_cm  = pos_err_cm,
            hdg_err_deg = hdg_err_deg,
            status      = status,
        )


# ─── Main ─────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)

    nodes: list[BotController] = []
    for i, cfg in enumerate(BOT_CONFIGS, start=1):
        node = BotController(
            bot_index           = i,
            marker_id           = cfg['marker_id'],
            goal_x              = cfg['goal_x'],
            goal_y              = cfg['goal_y'],
            final_heading_deg   = cfg['final_heading_deg'],
            cmd_vel_topic       = cfg['cmd_vel_topic'],
        )
        nodes.append(node)

    executor = MultiThreadedExecutor(num_threads=len(nodes) * 2)
    for node in nodes:
        executor.add_node(node)

    # Signal handler — zero-vel all bots then exit
    def _signal_handler(signum, frame):
        print(f'\n[4-Bot] Signal {signum} — emergency stop all bots!')
        for node in nodes:
            try:
                node._publish_cmd(0.0, 0.0)
                node._publish_cmd(0.0, 0.0)
                node.running = False
            except Exception:
                pass
        executor.shutdown(timeout_sec=1.0)
        sys.exit(0)

    signal.signal(signal.SIGINT,  _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    print('\n[4-Bot] All controllers started — spinning (Ctrl+C to stop all)…')

    try:
        executor.spin()
    except KeyboardInterrupt:
        print('\n[4-Bot] KeyboardInterrupt — stopping all bots.')
        for node in nodes:
            try:
                node._publish_cmd(0.0, 0.0)
            except Exception:
                pass
    except SystemExit:
        pass
    finally:
        _print_summary(nodes)
        for node in nodes:
            try:
                node._publish_cmd(0.0, 0.0)
                node.destroy_node()
            except Exception:
                pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


def _print_summary(nodes: list) -> None:
    """Print a formatted table of all bots' final position and heading errors."""
    sep = '═' * 74
    print(f'\n{sep}')
    print(f'  4-BOT NAVIGATION SUMMARY')
    print(sep)
    print(f'  {"Bot":<5} {"Marker":<8} {"Goal (x,y) m":<20}'
          f'{"Pos err":<12} {"Hdg err":<12} {"Status"}')
    print('─' * 74)
    for node in nodes:
        r = node.get_result()
        gx, gy = r['goal']
        pos_str = f"{r['pos_err_cm']:.1f} cm" if r['pos_err_cm'] == r['pos_err_cm'] else 'N/A'
        hdg_str = f"{r['hdg_err_deg']:.1f}°"  if r['hdg_err_deg'] == r['hdg_err_deg'] else 'N/A'
        print(f"  Bot{r['bot_index']:<2}  marker_{r['marker_id']:<2}  "
              f"({gx:+.2f}, {gy:+.2f})       "
              f"{pos_str:<12} {hdg_str:<12} {r['status']}")
    print(sep + '\n')


if __name__ == '__main__':
    main()
