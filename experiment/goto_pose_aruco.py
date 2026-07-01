#!/usr/bin/env python3.12
"""
goto_pose_aruco.py  —  Single-robot, 3-phase closed-loop pose controller
=========================================================================
System setup
------------
  - RealSense camera is FIXED overhead, looking down at the floor.
  - ArUco marker (ID configurable) is mounted ON TOP of the robot.
  - `ground_origin`  : fixed world frame derived from the camera.
  - `marker_N`       : the robot's live position in the world frame.

Control phases
--------------
  Phase 1 – ALIGN_TO_GOAL
      Rotate in place until the robot's heading points toward the
      goal position (within heading_tol_rad).  This minimises the
      lateral drift during the straight-line drive.

  Phase 2 – DRIVE_TO_GOAL
      Drive straight until the robot reaches (goal_x, goal_y)
      (within pos_tol_m).  A gentle heading-correction term keeps
      the robot on track during the drive.

  Phase 3 – ALIGN_TO_FINAL_HEADING
      Rotate in place until the robot faces the desired final heading
      (final_heading_deg, within heading_tol_rad).

  Then DONE — zero velocity is published once and the node shuts down.

Heading convention
------------------
  Angles are measured counter-clockwise from the +X axis of the
  `ground_origin` frame, in radians internally, degrees in parameters.
  The marker's yaw is extracted from the TF quaternion using atan2.

Quick-stop (Ctrl+C not working fix)
------------------------------------
  If Ctrl+C does not kill the process (common with ros2 run):
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped \
      "{ header: {frame_id: 'base_link'}, twist: {linear: {x: 0.0}, angular: {z: 0.0}} }"
  Or kill by PID:
    kill -SIGINT $(pgrep -f goto_pose_aruco)
  The node also registers a SIGTERM/SIGINT signal handler so that
  pressing Ctrl+C in the terminal publishes a zero-velocity command
  before exiting.

Global-frame / 4-quadrant notes
---------------------------------
  goal_x / goal_y are ABSOLUTE coordinates in the `ground_origin` world frame.
  The robot navigates from wherever it starts to those coordinates, regardless
  of which quadrant they fall in (+x+y, -x+y, -x-y, +x-y).

  Two parameters must be tuned once for your physical setup:

  marker_yaw_offset_deg (default 0.0)
      The ArUco marker may not be mounted with its own X-axis pointing in the
      same direction as the robot's forward axis.  Add the physical offset
      (degrees) here.  E.g. if the marker is rotated 90° CW on the robot body,
      set marker_yaw_offset_deg:=-90.0.

  ang_dir_sign (default +1)
      For an overhead camera the image Y-axis can point "down" in world space,
      which flips the sign of Z-rotations seen in TF.  If the robot rotates
      the WRONG direction during alignment, set ang_dir_sign:=-1.

Parameters (set via --ros-args -p name:=value)
----------------------------------------------
  target_marker_id      int    3      ArUco marker ID on the robot
  goal_x                float  1.0    goal x in world frame (m)  — any quadrant
  goal_y                float  0.0    goal y in world frame (m)  — any quadrant
  final_heading_deg     float  0.0    final heading (°, CCW from +X of world frame)
  speed                 float  0.1    linear speed (m/s)
  ang_speed             float  0.3    angular speed (rad/s)
  pos_tol_m             float  0.04   position tolerance (m)
  heading_tol_deg       float  5.0    heading tolerance (degrees)
  heading_kp            float  1.5    proportional gain for heading correction
  marker_yaw_offset_deg float  0.0    mounting offset: marker X vs robot forward (°)
  ang_dir_sign          int    1      +1 or -1: flip if robot turns wrong way
  lost_timeout          float  2.0    s without detection → emergency stop
  world_frame           str    ground_origin
  aruco_topic           str    /aruco_detections
  cmd_vel_topic         str    /cmd_vel
  tf_timeout            float  0.05   s to wait for TF lookup
"""

import math
import signal
import sys
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from geometry_msgs.msg import TwistStamped
from aruco_opencv_msgs.msg import ArucoDetection

# ─── tuneable defaults ────────────────────────────────────────────────────────
TARGET_MARKER_ID      = 1
# (x2, y2) goal — ABSOLUTE world frame.  Quadrant is set by the sign of each value:
#   goal_x > 0 → right  (+X)   |  goal_x < 0 → left  (-X)
#   goal_y > 0 → up     (+Y)   |  goal_y < 0 → down  (-Y)
# Robot starts from wherever ArUco first detects it — any quadrant, any position.
GOAL_X                = 0.2    # metres
GOAL_Y                = 0.2    # metres
FINAL_HEADING_DEG     = 90   # degrees CCW from camera +X axis
SPEED                 = 0.05    # m/s linear
ANG_SPEED             = 0.1    # rad/s angular
POS_TOL_M             = 0.04   # metres position tolerance
HEADING_TOL_DEG       = 5.0    # degrees heading tolerance
HEADING_KP            = 1.5    # proportional gain for heading correction
MARKER_YAW_OFFSET_DEG = 0.0    # mounting offset: marker X-axis vs robot forward (°)
ANG_DIR_SIGN          = 1      # +1 normal; -1 if robot rotates the wrong direction
LOST_TIMEOUT          = 2.0    # s without detection → emergency stop
WORLD_FRAME           = 'ground_origin'
ARUCO_TOPIC           = '/aruco_detections'
CMD_VEL_TOPIC         = '/cmd_vel'
TF_TIMEOUT            = 0.05   # s to block on TF lookup
# ─────────────────────────────────────────────────────────────────────────────


def _quadrant(x: float, y: float) -> str:
    """Return the quadrant label for an (x, y) point in the world frame."""
    if x >= 0 and y >= 0:
        return 'Q1 (+x,+y)'
    if x < 0 and y >= 0:
        return 'Q2 (-x,+y)'
    if x < 0 and y < 0:
        return 'Q3 (-x,-y)'
    return 'Q4 (+x,-y)'

# Phase labels
PHASE_INIT                = 'INIT'
PHASE_ALIGN_TO_GOAL       = 'ALIGN_TO_GOAL'
PHASE_DRIVE_TO_GOAL       = 'DRIVE_TO_GOAL'
PHASE_ALIGN_FINAL_HEADING = 'ALIGN_FINAL_HEADING'
PHASE_DONE                = 'DONE'


def _quat_to_yaw(q) -> float:
    """Extract yaw (rotation about Z) from a geometry_msgs quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _angle_wrap(a: float) -> float:
    """Wrap angle to [-π, π]."""
    return math.atan2(math.sin(a), math.cos(a))


class GotoPoseAruco(Node):
    """
    3-phase closed-loop goto-pose controller using overhead ArUco tracking.

    Phases:
      1. ALIGN_TO_GOAL       – rotate to face goal position
      2. DRIVE_TO_GOAL       – drive straight to goal (with heading correction)
      3. ALIGN_FINAL_HEADING – rotate to desired final heading
    """

    def __init__(self):
        super().__init__('goto_pose_aruco')

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter('target_marker_id',      TARGET_MARKER_ID)
        self.declare_parameter('goal_x',                GOAL_X)
        self.declare_parameter('goal_y',                GOAL_Y)
        self.declare_parameter('final_heading_deg',     FINAL_HEADING_DEG)
        self.declare_parameter('speed',                 SPEED)
        self.declare_parameter('ang_speed',             ANG_SPEED)
        self.declare_parameter('pos_tol_m',             POS_TOL_M)
        self.declare_parameter('heading_tol_deg',       HEADING_TOL_DEG)
        self.declare_parameter('heading_kp',            HEADING_KP)
        self.declare_parameter('marker_yaw_offset_deg', MARKER_YAW_OFFSET_DEG)
        self.declare_parameter('ang_dir_sign',          ANG_DIR_SIGN)
        self.declare_parameter('lost_timeout',          LOST_TIMEOUT)
        self.declare_parameter('world_frame',           WORLD_FRAME)
        self.declare_parameter('aruco_topic',           ARUCO_TOPIC)
        self.declare_parameter('cmd_vel_topic',         CMD_VEL_TOPIC)
        self.declare_parameter('tf_timeout',            TF_TIMEOUT)

        self.target_marker_id      = self.get_parameter('target_marker_id').value
        self.goal_x                = self.get_parameter('goal_x').value
        self.goal_y                = self.get_parameter('goal_y').value
        self.final_heading_rad     = math.radians(
            self.get_parameter('final_heading_deg').value)
        self.speed                 = self.get_parameter('speed').value
        self.ang_speed             = self.get_parameter('ang_speed').value
        self.pos_tol_m             = self.get_parameter('pos_tol_m').value
        self.heading_tol_rad       = math.radians(
            self.get_parameter('heading_tol_deg').value)
        self.heading_kp            = self.get_parameter('heading_kp').value
        self.marker_yaw_offset_rad = math.radians(
            self.get_parameter('marker_yaw_offset_deg').value)
        self.ang_dir_sign          = float(
            self.get_parameter('ang_dir_sign').value)
        self.lost_timeout          = self.get_parameter('lost_timeout').value
        self.world_frame           = self.get_parameter('world_frame').value
        aruco_topic                = self.get_parameter('aruco_topic').value
        cmd_vel_topic              = self.get_parameter('cmd_vel_topic').value
        self.tf_timeout_sec        = self.get_parameter('tf_timeout').value

        self.marker_tf_frame = f'marker_{self.target_marker_id}'

        # ── Goal sanity checks ────────────────────────────────────────────────
        # Quadrant is determined entirely by the signs of goal_x and goal_y.
        # Warn if the goal is on a quadrant boundary (axis) — the quadrant label
        # will still display correctly, but the robot will stop on the axis.
        if self.goal_x == 0.0 and self.goal_y == 0.0:
            self.get_logger().warn(
                'Goal is the world origin (0, 0) — quadrant is undefined. '
                'Set non-zero goal_x / goal_y.'
            )
        elif self.goal_x == 0.0:
            self.get_logger().warn(
                f'goal_x = 0 — goal lies ON the Y-axis (boundary between '
                f'Q1/Q2 and Q3/Q4). Robot will stop on the axis as requested.'
            )
        elif self.goal_y == 0.0:
            self.get_logger().warn(
                f'goal_y = 0 — goal lies ON the X-axis (boundary between '
                f'Q1/Q4 and Q2/Q3). Robot will stop on the axis as requested.'
            )

        # ── State ─────────────────────────────────────────────────────────────
        self.phase: str            = PHASE_INIT
        self.running: bool         = True
        self.marker_ever_seen: bool = False
        self.last_detection_time   = self.get_clock().now()

        # ArUco feedback diagnostics
        self.aruco_frame_count: int     = 0   # total ArucoDetection msgs received
        self.aruco_target_count: int    = 0   # frames where target marker was visible
        self.last_seen_ids: list        = []  # marker IDs in the most recent frame

        # Recorded start pose (set on first detection)
        self.start_x:   float | None = None
        self.start_y:   float | None = None
        self.start_yaw: float | None = None   # radians

        # Final pose (set on completion)
        self.final_x:   float | None = None
        self.final_y:   float | None = None
        self.final_yaw: float | None = None

        # ── TF2 ──────────────────────────────────────────────────────────────
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── Publisher ────────────────────────────────────────────────────────
        self.cmd_pub = self.create_publisher(TwistStamped, cmd_vel_topic, 10)

        # ── QoS: aruco_opencv publishes BEST_EFFORT ───────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Subscribers ──────────────────────────────────────────────────────
        self.aruco_sub = self.create_subscription(
            ArucoDetection,
            aruco_topic,
            self._aruco_callback,
            sensor_qos,
        )

        # ── Safety timer (10 Hz) ─────────────────────────────────────────────
        self.safety_timer = self.create_timer(0.1, self._safety_callback)

        # ── Signal handler for clean stop (Ctrl+C fix) ────────────────────────
        signal.signal(signal.SIGINT,  self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

        self.get_logger().info(
            f'\n[GotoPoseAruco] Started — GLOBAL FRAME 4-QUADRANT MODE\n'
            f'  Marker ID     : {self.target_marker_id} '
            f'(TF: "{self.marker_tf_frame}")\n'
            f'  (x1,y1) START : auto — wherever bot is first detected (any quadrant)\n'
            f'  (x2,y2) GOAL  : ({self.goal_x:.3f}, {self.goal_y:.3f}) m  '
            f'[{_quadrant(self.goal_x, self.goal_y)}]\n'
            f'  Final heading : {math.degrees(self.final_heading_rad):.1f}°\n'
            f'  Speed         : {self.speed:.3f} m/s linear, '
            f'{self.ang_speed:.3f} rad/s angular\n'
            f'  Pos tolerance : {self.pos_tol_m*100:.1f} cm\n'
            f'  Lost timeout  : {self.lost_timeout:.1f} s\n'
            f'Waiting for ArUco marker ID={self.target_marker_id}…\n'
            f'  QUICK STOP: kill -SIGINT $(pgrep -f goto_pose_aruco)'
        )

    # ── Signal handler ────────────────────────────────────────────────────────

    def _signal_handler(self, signum, frame):
        """Handle SIGINT/SIGTERM — publish zero-vel then exit."""
        self.get_logger().info(
            f'Signal {signum} received — emergency stop!')
        self._publish_cmd(0.0, 0.0)
        self._publish_cmd(0.0, 0.0)   # send twice to be sure
        self.running = False
        sys.exit(0)

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _aruco_callback(self, msg: ArucoDetection):
        """Main control loop — called on every ArucoDetection frame."""
        if not self.running or self.phase == PHASE_DONE:
            return

        # ── Feedback diagnostics (always, even if target not visible) ─────
        self.aruco_frame_count += 1
        self.last_seen_ids = sorted(m.marker_id for m in msg.markers)
        target_visible = self.target_marker_id in self.last_seen_ids
        if target_visible:
            self.aruco_target_count += 1
            self.last_detection_time = self.get_clock().now()

        self.get_logger().info(
            f'[ArUco] frame #{self.aruco_frame_count}  '
            f'visible IDs: {self.last_seen_ids}  '
            f'target(ID={self.target_marker_id}): '
            f'{"SEEN ✓" if target_visible else "NOT SEEN ✗"}',
            throttle_duration_sec=1.0,
        )

        if not target_visible:
            return

        # ── Get robot pose via TF2 ────────────────────────────────────────
        try:
            tf = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.marker_tf_frame,
                rclpy.time.Time(),
                Duration(seconds=self.tf_timeout_sec),
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(
                f'TF2 "{self.world_frame}" → "{self.marker_tf_frame}" failed: {e}',
                throttle_duration_sec=1.0,
            )
            return

        cur_x   = tf.transform.translation.x
        cur_y   = tf.transform.translation.y
        # Apply marker mounting offset so that cur_yaw = robot's true forward
        # heading in the world frame, regardless of marker orientation on body.
        cur_yaw = _angle_wrap(
            _quat_to_yaw(tf.transform.rotation) + self.marker_yaw_offset_rad
        )

        # ── First detection: current position = (x1,y1), start immediately ──────
        if not self.marker_ever_seen:
            self.marker_ever_seen = True
            self.start_x   = cur_x
            self.start_y   = cur_y
            self.start_yaw = cur_yaw
            self._log_nav_plan(cur_x, cur_y, cur_yaw)
            self.phase = PHASE_ALIGN_TO_GOAL

        # ── Dispatch ───────────────────────────────────────────────────────────
        if self.phase == PHASE_ALIGN_TO_GOAL:
            self._phase_align_to_goal(cur_x, cur_y, cur_yaw)
        elif self.phase == PHASE_DRIVE_TO_GOAL:
            self._phase_drive_to_goal(cur_x, cur_y, cur_yaw)
        elif self.phase == PHASE_ALIGN_FINAL_HEADING:
            self._phase_align_final_heading(cur_x, cur_y, cur_yaw)

    # ── Phase handlers ────────────────────────────────────────────────────────

    def _log_nav_plan(self, cur_x: float, cur_y: float, cur_yaw: float):
        """Print the (x1,y1)→(x2,y2) navigation plan box."""
        dx      = self.goal_x - cur_x
        dy      = self.goal_y - cur_y
        dist    = math.hypot(dx, dy)
        hdg_deg = math.degrees(math.atan2(dy, dx))
        start_q = _quadrant(cur_x, cur_y)
        goal_q  = _quadrant(self.goal_x, self.goal_y)
        same_q  = start_q == goal_q
        self.get_logger().info(
            f'\n'
            f'╔══ NAVIGATION PLAN ═══════════════════════════════════╗\n'
            f'║  (x1,y1) FROM : ({cur_x:.4f}, {cur_y:.4f}) m  [{start_q}]\n'
            f'║  FROM heading : {math.degrees(cur_yaw):.2f}°  (CCW from camera +X)\n'
            f'║  (x2,y2) TO   : ({self.goal_x:.4f}, {self.goal_y:.4f}) m  [{goal_q}]\n'
            f'║  FINAL heading: {math.degrees(self.final_heading_rad):.2f}°\n'
            f'║  Distance     : {dist:.4f} m\n'
            f'║  Turn to align: {hdg_deg:.2f}°\n'
            f'║  Quadrants    : '
            f'{"same (" + start_q + ")" if same_q else start_q + " → " + goal_q}\n'
            f'╚══════════════════════════════════════════════════════╝\n'
            f'Phase 1 — ALIGN_TO_GOAL: rotating to face [{goal_q}] goal…'
        )

    def _phase_align_to_goal(self, cur_x: float, cur_y: float, cur_yaw: float):
        """
        Phase 1: rotate in place until heading points toward (goal_x, goal_y).
        """
        dx = self.goal_x - cur_x
        dy = self.goal_y - cur_y
        desired_yaw = math.atan2(dy, dx)
        err = _angle_wrap(desired_yaw - cur_yaw)

        self.get_logger().info(
            f'[ALIGN_TO_GOAL] heading err={math.degrees(err):.2f}°  '
            f'desired={math.degrees(desired_yaw):.2f}°  '
            f'current={math.degrees(cur_yaw):.2f}°',
            throttle_duration_sec=0.3,
        )

        if abs(err) <= self.heading_tol_rad:
            self._publish_cmd(0.0, 0.0)
            self.get_logger().info(
                f'✓ Heading aligned ({math.degrees(err):.2f}°).\n'
                f'Phase 2 — DRIVE_TO_GOAL: driving to '
                f'({self.goal_x:.3f}, {self.goal_y:.3f}) m…'
            )
            self.phase = PHASE_DRIVE_TO_GOAL
        else:
            # Proportional magnitude clipped to ang_speed; sign corrected for camera
            omega = self.ang_dir_sign * math.copysign(
                min(self.ang_speed, self.heading_kp * abs(err)), err)
            self._publish_cmd(0.0, omega)

    def _phase_drive_to_goal(self, cur_x: float, cur_y: float, cur_yaw: float):
        """
        Phase 2: drive straight to (goal_x, goal_y).
        A small heading-correction term steers the robot back on track.
        """
        dx  = self.goal_x - cur_x
        dy  = self.goal_y - cur_y
        dist = math.hypot(dx, dy)

        self.get_logger().info(
            f'[DRIVE_TO_GOAL] dist={dist:.4f} m  '
            f'pos=({cur_x:.4f}, {cur_y:.4f})  '
            f'heading={math.degrees(cur_yaw):.2f}°',
            throttle_duration_sec=0.3,
        )

        if dist <= self.pos_tol_m:
            self._publish_cmd(0.0, 0.0)
            self.get_logger().info(
                f'✓ Goal reached! dist={dist:.4f} m\n'
                f'Phase 3 — ALIGN_FINAL_HEADING: rotating to '
                f'{math.degrees(self.final_heading_rad):.1f}°…'
            )
            self.phase = PHASE_ALIGN_FINAL_HEADING
        else:
            # Heading correction: steer toward the goal angle (all quadrants via atan2)
            desired_yaw = math.atan2(dy, dx)
            heading_err = _angle_wrap(desired_yaw - cur_yaw)
            omega = self.ang_dir_sign * self.heading_kp * heading_err
            omega = max(-self.ang_speed, min(self.ang_speed, omega))
            self._publish_cmd(self.speed, omega)

    def _phase_align_final_heading(self, cur_x: float, cur_y: float, cur_yaw: float):
        """
        Phase 3: rotate in place to reach final_heading_rad.
        """
        err = _angle_wrap(self.final_heading_rad - cur_yaw)

        self.get_logger().info(
            f'[ALIGN_FINAL_HEADING] heading err={math.degrees(err):.2f}°  '
            f'desired={math.degrees(self.final_heading_rad):.2f}°  '
            f'current={math.degrees(cur_yaw):.2f}°',
            throttle_duration_sec=0.3,
        )

        if abs(err) <= self.heading_tol_rad:
            self.final_x   = cur_x
            self.final_y   = cur_y
            self.final_yaw = cur_yaw
            self._stop(
                f'═══ TASK COMPLETE (global frame) ════════════════════\n'
                f'  INITIAL position : ({self.start_x:.4f}, {self.start_y:.4f}) m  '
                f'[{_quadrant(self.start_x, self.start_y)}]\n'
                f'  INITIAL heading  : {math.degrees(self.start_yaw):.2f}°\n'
                f'  FINAL   position : ({self.final_x:.4f}, {self.final_y:.4f}) m  '
                f'[{_quadrant(self.final_x, self.final_y)}]\n'
                f'  FINAL   heading  : {math.degrees(self.final_yaw):.2f}°\n'
                f'  Goal was         : ({self.goal_x:.4f}, {self.goal_y:.4f}) m  '
                f'heading {math.degrees(self.final_heading_rad):.2f}°  '
                f'[{_quadrant(self.goal_x, self.goal_y)}]\n'
                f'  Position error   : '
                f'{math.hypot(cur_x-self.goal_x, cur_y-self.goal_y)*100:.1f} cm\n'
                f'  Heading error    : {math.degrees(err):.2f}°\n'
                f'════════════════════════════════════════════════════'
            )
        else:
            omega = self.ang_dir_sign * math.copysign(
                min(self.ang_speed, self.heading_kp * abs(err)), err)
            self._publish_cmd(0.0, omega)

    # ── Safety timer ─────────────────────────────────────────────────────────

    def _safety_callback(self):
        """10 Hz watchdog — emergency stop if marker lost > lost_timeout s."""
        if not self.running or self.phase == PHASE_DONE:
            return

        if not self.marker_ever_seen:
            # Report ArUco pipeline health before first target detection
            self.get_logger().info(
                f'[ArUco feedback] frames received: {self.aruco_frame_count}  '
                f'last visible IDs: {self.last_seen_ids}  '
                f'| Waiting for target marker ID={self.target_marker_id} '
                f'(TF: "{self.marker_tf_frame}")…',
                throttle_duration_sec=3.0,
            )
            if self.aruco_frame_count == 0:
                self.get_logger().warn(
                    'No ArUco frames received yet — '
                    'check that aruco_opencv is running and publishing to '
                    f'"{self.get_parameter("aruco_topic").value}"',
                    throttle_duration_sec=5.0,
                )
            return

        marker_age = (
            self.get_clock().now() - self.last_detection_time
        ).nanoseconds / 1e9

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

    def _stop(self, reason: str):
        """Zero velocity, cancel timer, destroy subscribers, mark done."""
        if not self.running:
            return
        self.running = False
        self.phase   = PHASE_DONE
        # Publish zero velocity several times for reliability
        for _ in range(5):
            self._publish_cmd(0.0, 0.0)
        self.safety_timer.cancel()
        self.destroy_subscription(self.aruco_sub)
        self.get_logger().info(f'[STOPPED] {reason}')


def main(args=None):
    rclpy.init(args=args)
    node = GotoPoseAruco()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt — stopping robot.')
        node._publish_cmd(0.0, 0.0)
        node._publish_cmd(0.0, 0.0)
    except SystemExit:
        pass
    finally:
        if node.running:
            node._publish_cmd(0.0, 0.0)
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
