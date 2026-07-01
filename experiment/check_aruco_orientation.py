#!/usr/bin/env python3.12
"""
check_aruco_orientation.py  —  ArUco TF orientation diagnostic
==============================================================
Reads the ground_origin → marker_N TF transform and prints:

  1. Raw position (x, y, z)
  2. Raw quaternion (qx, qy, qz, qw)
  3. Roll / Pitch / Yaw (RPY) — all three, not just yaw
  4. The marker's X, Y, Z axes expressed as unit vectors in the world frame
  5. Which world-frame direction each marker axis points (UP/DOWN/LEFT/RIGHT/etc.)
  6. The "yaw from Z-rotation" used by goto_pose_aruco.py
  7. Alternative yaw candidates (from X-rotation, Y-rotation) for diagnosis

RViz axis colours reminder
--------------------------
  Red   = X axis
  Green = Y axis
  Blue  = Z axis

Your reported RViz view: red UP, green RIGHT, blue DOWN
  → Marker X points UP (vertical), marker Y points RIGHT (horizontal),
    marker Z points DOWN (into the floor / away from camera).

  If the robot's FORWARD direction aligns with one of the in-plane axes (Y or −Y
  in your case), `atan2` on the Z quaternion gives the WRONG heading.
  This script shows which angle to use.

Usage
-----
  python3 experiment/check_aruco_orientation.py \\
    --ros-args -p marker_id:=1

  Then physically rotate the robot slowly and watch which angle tracks correctly.

Parameters
----------
  marker_id    int   1      ArUco marker ID to inspect
  world_frame  str   ground_origin
  rate_hz      float 2.0    print rate (Hz)
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from aruco_opencv_msgs.msg import ArucoDetection


def _quat_to_rpy(q):
    """Return (roll, pitch, yaw) in radians from a geometry_msgs quaternion."""
    qx, qy, qz, qw = q.x, q.y, q.z, q.w

    # Roll (rotation about X)
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (rotation about Y)
    sinp = 2.0 * (qw * qy - qz * qx)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    # Yaw (rotation about Z)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def _quat_rotate_vec(q, v):
    """Rotate a 3-vector v by quaternion q.  Returns (rx, ry, rz)."""
    qx, qy, qz, qw = q.x, q.y, q.z, q.w
    vx, vy, vz = v
    # q * (vx,vy,vz,0) * q_conjugate
    # Using: v' = v + 2*qw*(q_xyz × v) + 2*(q_xyz × (q_xyz × v))
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    rx = vx + qw * tx + (qy * tz - qz * ty)
    ry = vy + qw * ty + (qz * tx - qx * tz)
    rz = vz + qw * tz + (qx * ty - qy * tx)
    return rx, ry, rz


def _dir_label(vec, threshold=0.7):
    """
    Describe which world-frame direction a unit vector mostly points toward.
    World frame: X=camera +X, Y=camera +Y, Z=up (if ground_origin is floor-level).
    """
    x, y, z = vec
    labels = []
    if z > threshold:
        labels.append('UP (+Z)')
    elif z < -threshold:
        labels.append('DOWN (-Z)')
    if x > threshold:
        labels.append('+X (camera right?)')
    elif x < -threshold:
        labels.append('-X (camera left?)')
    if y > threshold:
        labels.append('+Y (camera down?)')
    elif y < -threshold:
        labels.append('-Y (camera up?)')
    return ', '.join(labels) if labels else f'oblique ({x:.2f},{y:.2f},{z:.2f})'


class ArucoOrientationChecker(Node):

    def __init__(self):
        super().__init__('aruco_orientation_checker')

        self.declare_parameter('marker_id',   0)
        self.declare_parameter('world_frame', 'ground_origin')
        self.declare_parameter('rate_hz',     2.0)

        self.marker_id   = self.get_parameter('marker_id').value
        self.world_frame = self.get_parameter('world_frame').value
        rate_hz          = self.get_parameter('rate_hz').value

        self.marker_frame = f'marker_{self.marker_id}'
        self.frame_count  = 0
        self.target_seen  = 0

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.aruco_sub = self.create_subscription(
            ArucoDetection,
            '/aruco_detections',
            self._aruco_cb,
            sensor_qos,
        )

        self.timer = self.create_timer(1.0 / rate_hz, self._print_cb)

        self.get_logger().info(
            f'\n[ArUco Orientation Checker]\n'
            f'  Marker ID   : {self.marker_id} (TF: "{self.marker_frame}")\n'
            f'  World frame : "{self.world_frame}"\n'
            f'  Print rate  : {rate_hz:.1f} Hz\n'
            f'\n'
            f'  RViz axis colours: RED=X  GREEN=Y  BLUE=Z\n'
            f'  Your observation : RED(X) UP, GREEN(Y) RIGHT, BLUE(Z) DOWN\n'
            f'    → Marker X is VERTICAL, Y is in-plane horizontal\n'
            f'    → Yaw from Z-rotation may NOT equal robot heading!\n'
            f'\n'
            f'  Slowly rotate the robot and watch which angle tracks correctly.\n'
        )

    def _aruco_cb(self, msg: ArucoDetection):
        self.frame_count += 1
        if any(m.marker_id == self.marker_id for m in msg.markers):
            self.target_seen += 1

    def _print_cb(self):
        # Attempt TF lookup
        try:
            tf = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.marker_frame,
                rclpy.time.Time(),
                Duration(seconds=0.1),
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(
                f'TF "{self.world_frame}" → "{self.marker_frame}" not available: {e}\n'
                f'  ArUco frames received: {self.frame_count}  '
                f'target visible: {self.target_seen}',
                throttle_duration_sec=3.0,
            )
            return

        t = tf.transform.translation
        q = tf.transform.rotation

        # ── RPY ──────────────────────────────────────────────────────────
        roll, pitch, yaw = _quat_to_rpy(q)

        # ── Marker axis vectors in world frame ───────────────────────────
        # marker_X in world = where the red axis points
        mx_world = _quat_rotate_vec(q, (1.0, 0.0, 0.0))
        # marker_Y in world = where the green axis points
        my_world = _quat_rotate_vec(q, (0.0, 1.0, 0.0))
        # marker_Z in world = where the blue axis points
        mz_world = _quat_rotate_vec(q, (0.0, 0.0, 1.0))

        # ── Alternative yaw candidates ───────────────────────────────────
        # "Yaw from X" = atan2 of where marker-X projects onto world-XY plane
        yaw_from_x = math.atan2(mx_world[1], mx_world[0])
        # "Yaw from Y" = atan2 of where marker-Y projects onto world-XY plane
        yaw_from_y = math.atan2(my_world[1], my_world[0])
        # Standard yaw from Z quaternion (what goto_pose_aruco uses)
        yaw_from_z_quat = yaw   # = the standard yaw from RPY

        # ── Is any axis mostly vertical? ─────────────────────────────────
        x_is_vertical = abs(mx_world[2]) > 0.8
        y_is_vertical = abs(my_world[2]) > 0.8
        z_is_vertical = abs(mz_world[2]) > 0.8

        # ── Build report ─────────────────────────────────────────────────
        sep = '─' * 60
        report = (
            f'\n{sep}\n'
            f'[ArUco ID={self.marker_id}]  '
            f'frames total={self.frame_count}  target_seen={self.target_seen}\n'
            f'{sep}\n'
            f'POSITION in "{self.world_frame}":\n'
            f'  x={t.x:+.4f}  y={t.y:+.4f}  z={t.z:+.4f}  (metres)\n'
            f'\n'
            f'QUATERNION:\n'
            f'  qx={q.x:+.4f}  qy={q.y:+.4f}  qz={q.z:+.4f}  qw={q.w:+.4f}\n'
            f'\n'
            f'RPY (degrees):\n'
            f'  Roll  (X-rot): {math.degrees(roll):+8.2f}°\n'
            f'  Pitch (Y-rot): {math.degrees(pitch):+8.2f}°\n'
            f'  Yaw   (Z-rot): {math.degrees(yaw):+8.2f}°  '
            f'← used by goto_pose_aruco\n'
            f'\n'
            f'MARKER AXES in world frame (RViz colours: R=X G=Y B=Z):\n'
            f'  RED   (marker-X): ({mx_world[0]:+.3f},{mx_world[1]:+.3f},{mx_world[2]:+.3f})'
            f'  → {_dir_label(mx_world)}'
            f'{"  ⚠ VERTICAL" if x_is_vertical else ""}\n'
            f'  GREEN (marker-Y): ({my_world[0]:+.3f},{my_world[1]:+.3f},{my_world[2]:+.3f})'
            f'  → {_dir_label(my_world)}'
            f'{"  ⚠ VERTICAL" if y_is_vertical else ""}\n'
            f'  BLUE  (marker-Z): ({mz_world[0]:+.3f},{mz_world[1]:+.3f},{mz_world[2]:+.3f})'
            f'  → {_dir_label(mz_world)}'
            f'{"  ⚠ VERTICAL" if z_is_vertical else ""}\n'
            f'\n'
            f'YAW CANDIDATES (rotate robot slowly to identify correct one):\n'
            f'  Yaw from Z-quat (current code) : {math.degrees(yaw_from_z_quat):+8.2f}°\n'
            f'  Yaw from marker-X projected XY : {math.degrees(yaw_from_x):+8.2f}°\n'
            f'  Yaw from marker-Y projected XY : {math.degrees(yaw_from_y):+8.2f}°\n'
        )

        # ── Diagnosis ────────────────────────────────────────────────────
        report += f'\nDIAGNOSIS:\n'
        if x_is_vertical:
            report += (
                f'  ⚠ Marker X (red) is VERTICAL → "yaw from Z-quat" is NOT\n'
                f'    the heading angle. The robot forward axis is likely marker-Y.\n'
                f'  ✓ Use "yaw_from_y" = {math.degrees(yaw_from_y):+.2f}°\n'
                f'  ✓ Set marker_yaw_offset_deg:=-90.0 in goto_pose_aruco to fix.\n'
            )
        elif y_is_vertical:
            report += (
                f'  ⚠ Marker Y (green) is VERTICAL → robot forward is likely marker-X.\n'
                f'  ✓ "yaw from Z-quat" (current code) should be correct.\n'
                f'  ✓ Set marker_yaw_offset_deg:=0.0 (default).\n'
            )
        elif z_is_vertical:
            report += (
                f'  ✓ Marker Z (blue) is vertical (normal for flat-floor marker).\n'
                f'  ✓ "yaw from Z-quat" IS the correct heading.\n'
                f'  ✓ marker_yaw_offset_deg:=0.0 (default) is correct.\n'
            )
        else:
            report += (
                f'  ? No axis is clearly vertical — marker may be tilted.\n'
                f'    Check that the marker is flat on the robot top surface.\n'
            )

        report += f'{sep}\n'
        self.get_logger().info(report)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoOrientationChecker()
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
