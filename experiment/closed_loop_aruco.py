#!/usr/bin/env python3.12
"""
Closed-Loop ArUco Mover  —  Overhead camera, marker on robot
=============================================================
System setup
------------
  - RealSense camera is FIXED overhead, looking down at the floor.
  - ArUco marker (ID=0) is mounted ON TOP of the robot.
  - `ground_origin`  : fixed world frame derived from the camera.
  - `marker_0`       : the robot's live position in the world frame.
                       Updated every detection frame by aruco_opencv.

  TF tree:
      (world)
        camera_color_optical_frame  [fixed overhead camera]
          └─ ground_origin           [fixed world reference]
               └─ marker_0           [robot position, live]

Control logic
-------------
  1. Wait for first detection → record robot start position (x0, y0).
  2. Every subsequent detection → compute displacement from start:
         travelled = sqrt((x − x0)² + (y − y0)²)
  3. While travelled < target_distance → publish forward velocity.
  4. When travelled ≥ target_distance → stop.
  5. Emergency stop if marker lost for > lost_timeout seconds.

Topics
------
  Subscribed:
    /aruco_detections              (aruco_opencv_msgs/ArucoDetection)
    /camera/camera/color/image_raw (sensor_msgs/Image)  [liveness only]
  Published:
    /cmd_vel                       (geometry_msgs/TwistStamped)

ROS2 parameters
---------------
  target_marker_id  int    0       ArUco ID of the marker on the robot
  target_distance   float  0.5     metres to travel from start position
  speed             float  0.1     m/s forward command
  lost_timeout      float  2.0     s without detection → emergency stop
  world_frame       str    ground_origin
  aruco_topic       str    /aruco_detections
  camera_topic      str    /camera/camera/color/image_raw
  tf_timeout        float  0.05    s to wait for TF lookup
"""

import math
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
TARGET_DISTANCE  = 0.5       # metres to travel from start
SPEED            = 0.1       # m/s
LOST_TIMEOUT     = 2.0       # s without detection → emergency stop
WORLD_FRAME      = 'ground_origin'   # fixed overhead-camera world frame
ARUCO_TOPIC      = '/aruco_detections'
CAMERA_TOPIC     = '/camera/camera/color/image_raw'
CMD_VEL_TOPIC    = '/cmd_vel'
TF_TIMEOUT       = 0.05      # s to block on TF lookup
# ─────────────────────────────────────────────────────────────────────────────


class ClosedLoopArucoMover(Node):
    """
    Moves the robot a fixed distance using overhead-camera ArUco tracking.

    The marker is on the robot.  The camera is fixed overhead.
    ground_origin → marker_0  gives the robot's (x, y) in world coordinates.
    We stop when the robot's displacement from its start position reaches
    target_distance.
    """

    def __init__(self):
        super().__init__('closed_loop_aruco_mover')

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter('target_marker_id', TARGET_MARKER_ID)
        self.declare_parameter('target_distance',  TARGET_DISTANCE)
        self.declare_parameter('speed',            SPEED)
        self.declare_parameter('lost_timeout',     LOST_TIMEOUT)
        self.declare_parameter('world_frame',      WORLD_FRAME)
        self.declare_parameter('aruco_topic',      ARUCO_TOPIC)
        self.declare_parameter('camera_topic',     CAMERA_TOPIC)
        self.declare_parameter('tf_timeout',       TF_TIMEOUT)

        self.target_marker_id = self.get_parameter('target_marker_id').value
        self.target_distance  = self.get_parameter('target_distance').value
        self.speed            = self.get_parameter('speed').value
        self.lost_timeout     = self.get_parameter('lost_timeout').value
        self.world_frame      = self.get_parameter('world_frame').value
        aruco_topic           = self.get_parameter('aruco_topic').value
        camera_topic          = self.get_parameter('camera_topic').value
        self.tf_timeout_sec   = self.get_parameter('tf_timeout').value

        # TF frame name aruco_opencv uses for this marker ID
        self.marker_tf_frame = f'marker_{self.target_marker_id}'

        # ── State ─────────────────────────────────────────────────────────────
        # Robot start position in world frame (set on first detection)
        self.start_x: float | None = None
        self.start_y: float | None = None

        self.travelled: float      = 0.0
        self.running: bool         = True
        self.marker_ever_seen: bool = False
        self.last_detection_time   = self.get_clock().now()
        self.last_image_time       = self.get_clock().now()

        # Position-stale watchdog: detect frozen TF or physically blocked robot
        self.last_moved_x: float   = 0.0
        self.last_moved_y: float   = 0.0
        self.last_move_time        = self.get_clock().now()
        self.STALE_MOVE_THRESH     = 0.002   # metres — min change to count as "moved"
        self.STALE_TIMEOUT         = 3.0     # seconds without movement → warn/stop

        # ── TF2 ──────────────────────────────────────────────────────────────
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── Publisher ────────────────────────────────────────────────────────
        self.cmd_pub = self.create_publisher(TwistStamped, CMD_VEL_TOPIC, 10)

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
        self.image_sub = self.create_subscription(
            Image,
            camera_topic,
            self._image_callback,
            sensor_qos,
        )

        # ── Safety timer (10 Hz) ─────────────────────────────────────────────
        self.safety_timer = self.create_timer(0.1, self._safety_callback)

        self.get_logger().info(
            f'\n[ClosedLoopArucoMover] Started.\n'
            f'  Setup               : overhead camera, marker on robot\n'
            f'  Tracking marker ID  : {self.target_marker_id} '
            f'(TF frame: "{self.marker_tf_frame}")\n'
            f'  World frame         : "{self.world_frame}"\n'
            f'  Target travel       : {self.target_distance:.3f} m\n'
            f'  Forward speed       : {self.speed:.3f} m/s\n'
            f'  Lost timeout        : {self.lost_timeout:.1f} s\n'
            f'  ArUco topic         : {aruco_topic}\n'
            f'  Camera topic        : {camera_topic}\n'
            f'Waiting for first marker detection…'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _image_callback(self, msg: Image):
        """Track camera liveness."""
        self.last_image_time = self.get_clock().now()

    def _aruco_callback(self, msg: ArucoDetection):
        """
        Main control loop — runs on every ArucoDetection frame.

        ground_origin → marker_0  gives the robot's (x, y) in the world.
        We measure displacement from the start position to know how far
        the robot has actually moved.
        """
        if not self.running:
            return

        self.last_detection_time = self.get_clock().now()

        # ── 1. Is the target marker in this frame? ────────────────────────
        target_visible = any(
            m.marker_id == self.target_marker_id for m in msg.markers
        )
        if not target_visible:
            return   # marker absent — safety timer handles timeout

        # ── 2. Get robot's current world position via TF2 ─────────────────
        #  world_frame (ground_origin) → marker_tf_frame (marker_0)
        #  translation.x, .y  =  robot's XY position in world
        try:
            tf = self.tf_buffer.lookup_transform(
                self.world_frame,      # fixed world frame (overhead camera)
                self.marker_tf_frame,  # robot's current position
                rclpy.time.Time(),     # Time(0) = use latest available
                Duration(seconds=self.tf_timeout_sec),
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(
                f'TF2 "{self.world_frame}" → "{self.marker_tf_frame}" failed: {e}',
                throttle_duration_sec=1.0,
            )
            return

        cur_x = tf.transform.translation.x
        cur_y = tf.transform.translation.y

        # ── 3. First detection: record start position ─────────────────────
        if not self.marker_ever_seen:
            self.marker_ever_seen = True
            self.start_x = cur_x
            self.start_y = cur_y
            self.last_moved_x = cur_x
            self.last_moved_y = cur_y
            self.last_move_time = self.get_clock().now()
            self.get_logger().info(
                f'Robot start position in "{self.world_frame}":\n'
                f'  x={cur_x:.4f} m,  y={cur_y:.4f} m\n'
                f'  Target travel: {self.target_distance:.4f} m\n'
                f'Starting motion.'
            )

        # ── 4. Compute displacement from start (= distance travelled) ─────
        dx = cur_x - self.start_x
        dy = cur_y - self.start_y
        self.travelled = math.sqrt(dx * dx + dy * dy)

        self.get_logger().info(
            f'Pos: ({cur_x:.4f}, {cur_y:.4f}) m  |  '
            f'Travelled: {self.travelled:.4f} / {self.target_distance:.4f} m  |  '
            f'Remaining: {max(self.target_distance - self.travelled, 0.0):.4f} m'
        )

        # ── 5. Stop check ─────────────────────────────────────────────────
        if self.travelled >= self.target_distance:
            self._stop(
                f'Target reached! '
                f'Travelled {self.travelled:.4f} m '
                f'(target {self.target_distance:.4f} m).\n'
                f'  Start : ({self.start_x:.4f}, {self.start_y:.4f}) m\n'
                f'  End   : ({cur_x:.4f}, {cur_y:.4f}) m'
            )
            return

        # ── 6. Position-stale watchdog ────────────────────────────────────
        # Track whether the position is actually changing while we command velocity.
        # If frozen for STALE_TIMEOUT seconds the robot is blocked or TF is stale.
        pos_change = math.sqrt(
            (cur_x - self.last_moved_x) ** 2 +
            (cur_y - self.last_moved_y) ** 2
        )
        if pos_change > self.STALE_MOVE_THRESH:
            # Position changed — reset watchdog
            self.last_moved_x = cur_x
            self.last_moved_y = cur_y
            self.last_move_time = self.get_clock().now()
        else:
            stale_age = (self.get_clock().now() - self.last_move_time).nanoseconds / 1e9
            if stale_age > self.STALE_TIMEOUT:
                self._stop(
                    f'POSITION FROZEN for {stale_age:.1f} s at '
                    f'({cur_x:.4f}, {cur_y:.4f}) m — robot is physically '
                    f'blocked or outside camera FOV.\n'
                    f'  Travelled so far: {self.travelled:.4f} m.\n'
                    f'  Move robot back into camera view and restart.'
                )
                return

        # ── 7. Publish forward velocity ───────────────────────────────────
        self._publish_cmd(self.speed)

    # ── Safety timer ─────────────────────────────────────────────────────────

    def _safety_callback(self):
        """
        10 Hz watchdog:
          - Camera liveness warning
          - Emergency stop if marker lost > lost_timeout seconds
          - Remind user while waiting for first detection
        """
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
                f'Waiting for ArUco marker ID={self.target_marker_id} '
                f'(TF frame: "{self.marker_tf_frame}")…',
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

    def _stop(self, reason: str):
        """Zero velocity, cancel timer, destroy subscribers."""
        if not self.running:
            return
        self.running = False
        self._publish_cmd(0.0)
        self.safety_timer.cancel()
        self.destroy_subscription(self.aruco_sub)
        self.destroy_subscription(self.image_sub)
        self.get_logger().info(f'[STOPPED] {reason}')


def main(args=None):
    rclpy.init(args=args)
    node = ClosedLoopArucoMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
