#!/usr/bin/env python3
"""Finds coloured blocks in the base camera and locates them in the base frame.

With one camera, distance comes from geometry: the pixel defines a ray, and the ray is
intersected with a horizontal plane at the object's mid height. Publishes JSON on
object/detections, and a marked-up picture on object/debug_image.
"""
import json
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

# HSV ranges (OpenCV hue is 0-179). Tune these for your real lighting.
COLORS = {
    'red': [((0, 110, 60), (9, 255, 255)), ((170, 110, 60), (179, 255, 255))],
    'green': [((40, 90, 50), (85, 255, 255))],
    'blue': [((100, 150, 140), (128, 255, 255))],     # bright only: the gripper fingers are a dark blue
}
DRAW = {'red': (0, 0, 255), 'green': (0, 200, 0), 'blue': (255, 80, 0)}


def quat_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        self.declare_parameter('object_mid_height', 0.02)
        self.declare_parameter('min_area', 60)
        self.declare_parameter('camera_frame', 'camera_optical_frame')
        self.declare_parameter('fallback_hfov', 1.085)
        self.h = self.get_parameter('object_mid_height').value
        self.min_area = self.get_parameter('min_area').value
        self.frame = self.get_parameter('camera_frame').value
        self.K = None; self.bridge = CvBridge()
        self.tf_buf = Buffer(); self.tf = TransformListener(self.tf_buf, self)
        self.pub = self.create_publisher(String, 'object/detections', 10)
        self.dbg = self.create_publisher(Image, 'object/debug_image', 2)
        self.create_subscription(CameraInfo, 'camera/camera_info', self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, 'camera/image_raw', self.on_image, qos_profile_sensor_data)

    def on_info(self, msg):
        if self.K is None and msg.k[0] > 0:
            self.K = np.array(msg.k).reshape(3, 3)

    def on_image(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warning(f'bad image: {e}', throttle_duration_sec=5.0)
            return
        K = self.K
        if K is None:     # a real USB camera may publish no calibration: use its nominal field of view
            f = img.shape[1] / 2 / np.tan(self.get_parameter('fallback_hfov').value / 2)
            K = np.array([[f, 0, img.shape[1] / 2], [0, f, img.shape[0] / 2], [0, 0, 1.0]])
        try:
            t = self.tf_buf.lookup_transform('base_link', self.frame, Time())
        except Exception:
            return
        R = quat_matrix(t.transform.rotation)
        c = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
        hsv = cv2.cvtColor(cv2.GaussianBlur(img, (5, 5), 0), cv2.COLOR_BGR2HSV)
        found = []
        for name, ranges in COLORS.items():
            mask = np.zeros(hsv.shape[:2], np.uint8)
            for lo, hi in ranges:
                mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < self.min_area:
                    continue
                M = cv2.moments(cnt)
                u, v = M['m10'] / M['m00'], M['m01'] / M['m00']
                ray = R @ np.linalg.inv(K) @ np.array([u, v, 1.0])
                if ray[2] >= -1e-3:     # at or above the horizon: not on the floor
                    continue
                s = (self.h - c[2]) / ray[2]
                p = c + s * ray
                found.append(dict(color=name, x=round(float(p[0]), 4), y=round(float(p[1]), 4), area=round(float(area), 1)))
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(img, (x, y), (x + w, y + h), DRAW[name], 2)
                cv2.putText(img, f'{name} {p[0]:.2f},{p[1]:.2f}', (x, max(y - 5, 12)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, DRAW[name], 1)
        self.pub.publish(String(data=json.dumps(found)))
        out = self.bridge.cv2_to_imgmsg(img, 'bgr8'); out.header = msg.header
        self.dbg.publish(out)


def main():
    rclpy.init()
    node = ObjectDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
