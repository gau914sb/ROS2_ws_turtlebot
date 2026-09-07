#!/usr/bin/env python3.12
"""
display_all_distances.py  —  Live inter-robot distance monitor
=================================================================
Standalone ROS2 node that reads each robot's (x, y) straight from the
ground_origin → marker_N TF frames (same ArUco feedback path as
multi_unicycle_controller_double_angle_experiment_ekf.py) and prints /
plots the distance between EVERY pair of robots — not just the 5
formation edges used by the controller.

For 4 robots that's all C(4,2) = 6 pairs: (bot_0,bot_1), (bot_0,bot_2),
(bot_0,bot_3), (bot_1,bot_2), (bot_1,bot_3), (bot_2,bot_3).

No EKF / control loop here — this is a passive monitor, so it uses the
raw TF measurement directly.

Terminal output flags each pair as:
  OK      — between lower_bound (Rs+r) and upper_bound (Rc-r)
  TOO CLOSE  — below lower_bound (collision risk)
  TOO FAR    — above upper_bound (connectivity risk)

Usage
-----
  python3 experiment/display_all_distances.py
  python3 experiment/display_all_distances.py --ros-args -p rate_hz:=10.0
  python3 experiment/display_all_distances.py --ros-args -p enable_plot:=false

Parameters
----------
  rate_hz       float  5.0    how often to sample/print/update the plot
  window_sec    float  30.0   seconds of history shown in the live plot
  enable_plot   bool   true   show a live matplotlib plot alongside printing
"""

import itertools
import threading
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from aruco_opencv_msgs.msg import ArucoDetection

# =============================================================================
# ROBOT ↔ MARKER MAPPING — same as multi_unicycle_controller_double_angle_experiment_ekf.py
# =============================================================================
ROBOT_MARKER_MAP = {
    'bot_0': 0,
    'bot_1': 1,
    'bot_2': 2,
    'bot_3': 3,
}

WORLD_FRAME = 'ground_origin'
ARUCO_TOPIC = '/aruco_detections'
TF_TIMEOUT  = 0.05

# Radius parameters — same values as multi_unicycle_controller_double_angle_experiment_ekf.py
r  = 0.135  # Agent radius
Rs = 0.16   # Safety radius
Rc = 1.6    # Connectivity radius

LOWER_BOUND = Rs + r  # Minimum allowed center-to-center distance (collision/safety)
UPPER_BOUND = Rc - r  # Maximum allowed center-to-center distance (connectivity)

PAIR_COLORS = ['blue', 'red', 'green', 'orange', 'purple', 'brown',
               'cyan', 'magenta', 'olive', 'teal']


class DistanceMonitor(Node):

    def __init__(self):
        super().__init__('inter_bot_distance_monitor')

        self.declare_parameter('rate_hz', 5.0)
        self.declare_parameter('window_sec', 30.0)
        self.declare_parameter('enable_plot', True)

        self.rate_hz     = float(self.get_parameter('rate_hz').value)
        self.window_sec  = float(self.get_parameter('window_sec').value)
        self.enable_plot = bool(self.get_parameter('enable_plot').value)

        self.robot_names = list(ROBOT_MARKER_MAP.keys())
        self.pairs = list(itertools.combinations(self.robot_names, 2))

        self.positions = {name: None for name in self.robot_names}       # (x, y)
        self.pose_received = {name: False for name in self.robot_names}

        # History buffers for plotting, one deque of (t, dist) per pair
        maxlen = max(10, int(self.window_sec * self.rate_hz))
        self.history = {pair: deque(maxlen=maxlen) for pair in self.pairs}
        self.data_lock = threading.Lock()
        self.start_time = None

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        aruco_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.aruco_sub = self.create_subscription(
            ArucoDetection, ARUCO_TOPIC, self._aruco_callback, aruco_qos,
        )

        self.timer = self.create_timer(1.0 / self.rate_hz, self._sample_callback)

        self.get_logger().info(
            f'[Distance Monitor] robots={self.robot_names}  pairs={len(self.pairs)}  '
            f'rate={self.rate_hz:.1f}Hz  lower_bound(Rs+r)={LOWER_BOUND:.3f}m  '
            f'upper_bound(Rc-r)={UPPER_BOUND:.3f}m'
        )

    # ------------------------------------------------------------------
    # Update each robot's (x, y) from its marker TF whenever it's visible
    # ------------------------------------------------------------------
    def _aruco_callback(self, msg: ArucoDetection):
        visible_ids = {m.marker_id for m in msg.markers}

        for name, marker_id in ROBOT_MARKER_MAP.items():
            if marker_id not in visible_ids:
                continue

            try:
                tf = self.tf_buffer.lookup_transform(
                    WORLD_FRAME,
                    f'marker_{marker_id}',
                    rclpy.time.Time(),
                    Duration(seconds=TF_TIMEOUT),
                )
            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                self.get_logger().warn(
                    f'TF lookup {WORLD_FRAME}→marker_{marker_id} failed: {e}',
                    throttle_duration_sec=2.0,
                )
                continue

            t = tf.transform.translation
            self.positions[name] = (t.x, t.y)
            self.pose_received[name] = True

    # ------------------------------------------------------------------
    # Compute + log + record all pairwise distances at rate_hz
    # ------------------------------------------------------------------
    def _sample_callback(self):
        if self.start_time is None:
            self.start_time = self.get_clock().now()

        now = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        lines = []
        with self.data_lock:
            for a, b in self.pairs:
                pos_a, pos_b = self.positions[a], self.positions[b]
                if pos_a is None or pos_b is None:
                    continue

                dx = pos_a[0] - pos_b[0]
                dy = pos_a[1] - pos_b[1]
                dist = (dx * dx + dy * dy) ** 0.5

                self.history[(a, b)].append((now, dist))

                if dist < LOWER_BOUND:
                    flag = 'TOO CLOSE'
                elif dist > UPPER_BOUND:
                    flag = 'TOO FAR'
                else:
                    flag = 'OK'
                lines.append(f'{a}-{b}={dist:.3f}m [{flag}]')

        if lines:
            self.get_logger().info(f't={now:6.1f}s | ' + '  '.join(lines))
        else:
            self.get_logger().info('Waiting for robot poses...', throttle_duration_sec=2.0)


def _run_live_plot(node: DistanceMonitor):
    """Runs on the main thread while rclpy spins in a background thread."""
    fig, ax = plt.subplots(figsize=(12, 7))
    lines = {}
    for idx, pair in enumerate(node.pairs):
        color = PAIR_COLORS[idx % len(PAIR_COLORS)]
        (line,) = ax.plot([], [], '-', color=color, linewidth=2,
                           label=f'{pair[0]}-{pair[1]}')
        lines[pair] = line

    ax.axhline(y=LOWER_BOUND, color='red', linestyle=':', linewidth=1.8,
               label=f'Lower bound Rs+r = {LOWER_BOUND:.3f}m')
    ax.axhline(y=UPPER_BOUND, color='black', linestyle=':', linewidth=1.8,
               label=f'Upper bound Rc-r = {UPPER_BOUND:.3f}m')

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Distance (m)')
    ax.set_title('Live Inter-Robot Distances (all pairs)')
    ax.legend(loc='upper right', fontsize=8, ncol=2)
    ax.grid(True, alpha=0.3)
    ax.set_ylim(0, UPPER_BOUND * 1.2)

    def update(_frame):
        with node.data_lock:
            snapshot = {pair: list(node.history[pair]) for pair in node.pairs}

        t_max = 0.0
        for pair, series in snapshot.items():
            if not series:
                continue
            ts, ds = zip(*series)
            lines[pair].set_data(ts, ds)
            t_max = max(t_max, ts[-1])

        if t_max > 0:
            ax.set_xlim(max(0.0, t_max - node.window_sec), t_max + 0.5)
        return list(lines.values())

    anim = FuncAnimation(fig, update, interval=int(1000 / node.rate_hz), blit=False)
    plt.tight_layout()
    plt.show()
    return anim


def main(args=None):
    rclpy.init(args=args)
    node = DistanceMonitor()

    if node.enable_plot:
        spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        spin_thread.start()
        try:
            _run_live_plot(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            try:
                rclpy.shutdown()
            except Exception:
                pass
    else:
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
