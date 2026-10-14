#!/usr/bin/env python3
"""
camera_recorder.py
===================
Subscribes to the two documentation camera image topics (bridged from Gazebo
via camera_bridge.yaml) and writes each to its own MP4 file with OpenCV --
this is the "no screen-recording" requirement from UBUNTU_GAZEBO_TASK.md
section 4(c): the video comes from actual camera sensors in the simulation,
not a capture of the GUI.

ROS parameters:
  top_topic    (string) default '/cam_top/image'
  iso_topic    (string) default '/cam_iso/image'
  top_output   (string) required -- full path to camera_top.mp4
  iso_output   (string) required -- full path to camera_iso.mp4
  fps          (double) default 15.0
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraRecorder(Node):

    def __init__(self):
        super().__init__('camera_recorder')

        self.declare_parameter('top_topic', '/cam_top/image')
        self.declare_parameter('iso_topic', '/cam_iso/image')
        self.declare_parameter('top_output', '/tmp/camera_top.mp4')
        self.declare_parameter('iso_output', '/tmp/camera_iso.mp4')
        self.declare_parameter('fps', 15.0)

        self.fps = float(self.get_parameter('fps').value)
        self.bridge = CvBridge()

        qos = QoSProfile(depth=10)
        self.writers = {}
        self.frame_counts = {'top': 0, 'iso': 0}

        top_topic = self.get_parameter('top_topic').value
        iso_topic = self.get_parameter('iso_topic').value
        self.outputs = {
            'top': self.get_parameter('top_output').value,
            'iso': self.get_parameter('iso_output').value,
        }

        self.create_subscription(Image, top_topic, lambda msg: self._cb('top', msg), qos)
        self.create_subscription(Image, iso_topic, lambda msg: self._cb('iso', msg), qos)

        self.get_logger().info(
            f'Recording {top_topic} -> {self.outputs["top"]}, '
            f'{iso_topic} -> {self.outputs["iso"]} at {self.fps} fps'
        )

    def _cb(self, key, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if key not in self.writers:
            h, w = frame.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.writers[key] = cv2.VideoWriter(self.outputs[key], fourcc, self.fps, (w, h))
            self.get_logger().info(f'{key}: opened writer {self.outputs[key]} ({w}x{h})')
        self.writers[key].write(frame)
        self.frame_counts[key] += 1

    def close(self):
        for key, writer in self.writers.items():
            writer.release()
            self.get_logger().info(f'{key}: wrote {self.frame_counts[key]} frames to {self.outputs[key]}')


def main(args=None):
    rclpy.init(args=args)
    node = CameraRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
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
