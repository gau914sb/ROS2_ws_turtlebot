#!/usr/bin/env python3.12
"""
Ground-Origin Calibration  —  camera_color_optical_frame → ground_origin
==========================================================================
Calibrates the static transform between the RealSense camera's optical
frame and your world/ground frame ("ground_origin") by using ONE robot's
ArUco marker as a physical reference, instead of measuring camera
height/tilt by hand.

How it works
------------
  1. You place the calibration robot's marker flat on the floor at the
     exact point you want as the world origin, facing whatever direction
     you want to be world "+X" / heading 0.
  2. This script looks up camera_color_optical_frame → marker_<id>
     directly (no ground_origin needed yet), collects N samples while the
     marker sits still, and averages them (translation: mean; rotation:
     proper rotation-group mean via scipy) to smooth out ArUco/TF jitter.
  3. It publishes that averaged transform as a STATIC transform
     camera_color_optical_frame → ground_origin (so ground_origin now
     means exactly "wherever the calibration marker was"), and saves it to
     a YAML file for reuse in later sessions.
  4. The script keeps running afterward — static transforms only reach
     late-joining nodes (aruco_tracker, rviz2, your controller scripts)
     while the publishing node is alive, so leave this terminal open for
     the whole experiment.

Reuse without recalibrating
----------------------------
  If the camera hasn't physically moved since your last calibration, skip
  the sampling step and just republish the saved result:

      python3.12 experiment/calibrate_ground_origin.py --load

Usage
-----
  # First-time calibration using bot_3's marker (ROBOT_MARKER_MAP['bot_3'] = 3):
  python3.12 experiment/calibrate_ground_origin.py --marker-id 3

  # Use a different marker / robot as the calibration reference:
  python3.12 experiment/calibrate_ground_origin.py --marker-id 1

  # More samples for a noisier camera, or fewer for a quick recalibration:
  python3.12 experiment/calibrate_ground_origin.py --marker-id 3 --samples 100

  # Reload the last saved calibration instead of resampling:
  python3.12 experiment/calibrate_ground_origin.py --load

Once this is running, aruco_tracker's `output_frame` parameter and every
other script's WORLD_FRAME ('ground_origin') will resolve correctly.
"""

import argparse
import os
import time
from datetime import datetime, timezone
from typing import Optional

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from geometry_msgs.msg import TransformStamped
from aruco_opencv_msgs.msg import ArucoDetection

# =============================================================================
# CONFIGURATION
# =============================================================================

CAMERA_FRAME = 'camera_color_optical_frame'   # published by realsense2_camera
WORLD_FRAME  = 'ground_origin'                # what every other script looks up
ARUCO_TOPIC  = '/aruco_detections'
TF_TIMEOUT   = 0.05

DEFAULT_MARKER_ID   = 3     # bot_3 by default — change with --marker-id
DEFAULT_NUM_SAMPLES = 50    # ~1-2s of samples at typical aruco_tracker rates

DEFAULT_SAVE_PATH = os.path.expanduser(
    '~/turtlebot3_simulations/calibration/ground_origin_calibration.yaml'
)


# =============================================================================
# Node
# =============================================================================

class GroundOriginCalibrator(Node):
    """
    Collects N samples of camera_color_optical_frame -> marker_<id>,
    averages them, and publishes/saves the result as the static
    camera_color_optical_frame -> ground_origin transform.
    """

    def __init__(self, marker_id: int, num_samples: int):
        super().__init__('ground_origin_calibrator')
        self.marker_id = marker_id
        self.num_samples = num_samples
        self.marker_frame = f'marker_{marker_id}'

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        self.sampling_active = False
        self.done = False
        self._samples = []   # list of (x, y, z, qx, qy, qz, qw)
        self._last_progress_print = 0

        aruco_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(ArucoDetection, ARUCO_TOPIC, self._aruco_cb, aruco_qos)

    # ── Sampling ────────────────────────────────────────────────────────

    def _aruco_cb(self, msg: ArucoDetection):
        if not self.sampling_active or self.done:
            return

        visible = {m.marker_id for m in msg.markers}
        if self.marker_id not in visible:
            return

        try:
            tf = self.tf_buffer.lookup_transform(
                CAMERA_FRAME, self.marker_frame,
                rclpy.time.Time(), Duration(seconds=TF_TIMEOUT),
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            return

        t = tf.transform.translation
        q = tf.transform.rotation
        self._samples.append((t.x, t.y, t.z, q.x, q.y, q.z, q.w))

        if len(self._samples) - self._last_progress_print >= 5:
            self._last_progress_print = len(self._samples)
            self.get_logger().info(f'Calibration samples: {len(self._samples)}/{self.num_samples}')

        if len(self._samples) >= self.num_samples:
            self.done = True

    # ── Averaging + publish + save ───────────────────────────────────────

    def compute_transform(self) -> TransformStamped:
        """Average the collected samples into one TransformStamped."""
        arr = np.array(self._samples)  # shape (N, 7)
        mean_xyz = arr[:, 0:3].mean(axis=0)

        quats = arr[:, 3:7]  # (N, 4) as [x, y, z, w]
        mean_rot = Rotation.from_quat(quats).mean()
        mean_quat = mean_rot.as_quat()  # [x, y, z, w]

        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = CAMERA_FRAME
        tf_msg.child_frame_id = WORLD_FRAME
        tf_msg.transform.translation.x = float(mean_xyz[0])
        tf_msg.transform.translation.y = float(mean_xyz[1])
        tf_msg.transform.translation.z = float(mean_xyz[2])
        tf_msg.transform.rotation.x = float(mean_quat[0])
        tf_msg.transform.rotation.y = float(mean_quat[1])
        tf_msg.transform.rotation.z = float(mean_quat[2])
        tf_msg.transform.rotation.w = float(mean_quat[3])
        return tf_msg

    def publish_static(self, tf_msg: TransformStamped):
        self.static_broadcaster.sendTransform(tf_msg)
        self.get_logger().info(
            f'Publishing static transform {tf_msg.header.frame_id} -> {tf_msg.child_frame_id} '
            f'(xyz=[{tf_msg.transform.translation.x:.4f}, '
            f'{tf_msg.transform.translation.y:.4f}, '
            f'{tf_msg.transform.translation.z:.4f}])'
        )

    def save_yaml(self, tf_msg: TransformStamped, save_path: str):
        t = tf_msg.transform.translation
        q = tf_msg.transform.rotation
        rpy_deg = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz', degrees=True)

        data = {
            'parent_frame': tf_msg.header.frame_id,
            'child_frame': tf_msg.child_frame_id,
            'calibration_marker_id': self.marker_id,
            'num_samples': len(self._samples),
            'translation': {'x': float(t.x), 'y': float(t.y), 'z': float(t.z)},
            'rotation_quaternion': {
                'x': float(q.x), 'y': float(q.y), 'z': float(q.z), 'w': float(q.w)
            },
            'rotation_rpy_deg': {
                'roll': float(rpy_deg[0]), 'pitch': float(rpy_deg[1]), 'yaw': float(rpy_deg[2])
            },
            'calibrated_at': datetime.now(timezone.utc).isoformat(),
        }

        os.makedirs(os.path.dirname(save_path), exist_ok=True)
        with open(save_path, 'w') as f:
            yaml.safe_dump(data, f, sort_keys=False)
        self.get_logger().info(f'Calibration saved to {save_path}')

    # ── Reuse a saved calibration without resampling ─────────────────────

    def load_and_publish(self, save_path: str):
        with open(save_path, 'r') as f:
            data = yaml.safe_load(f)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = data.get('parent_frame', CAMERA_FRAME)
        tf_msg.child_frame_id = data.get('child_frame', WORLD_FRAME)
        tr = data['translation']
        rot = data['rotation_quaternion']
        tf_msg.transform.translation.x = float(tr['x'])
        tf_msg.transform.translation.y = float(tr['y'])
        tf_msg.transform.translation.z = float(tr['z'])
        tf_msg.transform.rotation.x = float(rot['x'])
        tf_msg.transform.rotation.y = float(rot['y'])
        tf_msg.transform.rotation.z = float(rot['z'])
        tf_msg.transform.rotation.w = float(rot['w'])

        self.get_logger().info(
            f'Loaded calibration from {save_path} '
            f'(calibrated {data.get("calibrated_at", "unknown time")}, '
            f'marker_{data.get("calibration_marker_id", "?")})'
        )
        self.publish_static(tf_msg)


# =============================================================================
# Main
# =============================================================================

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--marker-id', type=int, default=DEFAULT_MARKER_ID,
                         help=f'ArUco marker ID to use as the calibration reference '
                              f'(default: {DEFAULT_MARKER_ID}, i.e. bot_3)')
    parser.add_argument('--samples', type=int, default=DEFAULT_NUM_SAMPLES,
                         help=f'Number of TF samples to average (default: {DEFAULT_NUM_SAMPLES})')
    parser.add_argument('--save-path', type=str, default=DEFAULT_SAVE_PATH,
                         help=f'Where to save/load the calibration YAML (default: {DEFAULT_SAVE_PATH})')
    parser.add_argument('--load', action='store_true',
                         help='Skip sampling — load and republish the previously saved calibration')
    args = parser.parse_args()

    rclpy.init()
    node = GroundOriginCalibrator(marker_id=args.marker_id, num_samples=args.samples)

    try:
        if args.load:
            node.load_and_publish(args.save_path)
            node.get_logger().info(
                'Republishing saved calibration. Leave this running — '
                'other nodes need this process alive to receive ground_origin.'
            )
            rclpy.spin(node)
            return

        marker_name = f'marker_{args.marker_id}'
        print(
            f'\n[Calibration] Place the calibration marker ({marker_name}) flat on the '
            f'floor at your desired world origin.\n'
            f'  - Its facing direction will become world +X / heading 0.\n'
            f'  - Keep it perfectly still once placed.\n'
        )
        input('Press Enter when it is in position... ')

        node.get_logger().info(f'Collecting {args.samples} samples of {marker_name}...')
        node.sampling_active = True

        start_time = time.time()
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.time() - start_time > 30.0:
                node.get_logger().error(
                    f'Timed out waiting for {marker_name} — is it visible to the camera '
                    f'and is aruco_tracker running?'
                )
                return

        tf_msg = node.compute_transform()
        node.publish_static(tf_msg)
        node.save_yaml(tf_msg, args.save_path)

        node.get_logger().info(
            'Calibration complete. ground_origin is now live. '
            'Leave this running — other nodes need this process alive to receive it.'
        )
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
