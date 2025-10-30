#!/usr/bin/env python3

from typing import Optional, Tuple

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, Bool

class VizualizeDetection:
    def __init__(self):
        # -------- viz params --------
        self.viz_enable = rospy.get_param("~viz_enable", True)
        self.viz_use_compressed = rospy.get_param("~viz_use_compressed", self.use_compressed)
        self.viz_input_topic = rospy.get_param("~viz_topic",
                                         "~out/compressed" if self.viz_use_compressed else "~out/image_raw")
        self.viz_traffic_light = rospy.get_param("~viz_traffic_light", True)
        self.viz_stop_line = rospy.get_param("~viz_stop_line", "/stop_sequence/stop_line")

        self.viz_line_color_bgr = tuple(int(x) for x in rospy.get_param("~viz_line_color_bgr", [0, 0, 255]))  # RED
        self.viz_line_thickness = int(rospy.get_param("~viz_line_thickness", 4))
        self.viz_text = rospy.get_param("~viz_text", "STOP LINE")
        self.viz_text_scale = float(rospy.get_param("~viz_text_scale", 0.8))
        self.viz_text_thickness = int(rospy.get_param("~viz_text_thickness", 2))

        # viz publisher
        self.viz_pub = None
        if self.viz_enable:
            if self.viz_use_compressed:
                self.viz_pub = rospy.Publisher(self.viz_topic, CompressedImage, queue_size=1)
            else:
                self.viz_pub = rospy.Publisher(self.viz_topic, Image, queue_size=1)
            rospy.loginfo("\033[1m[visual] viz publish: %s (compressed=%s)\033[0m",
                          rospy.resolve_name(self.viz_topic), self.viz_use_compressed)
            

    # ---------- overlay + publish ----------
    def draw_overlay(self, frame: np.ndarray,
                     stop_line: Optional[Tuple[float, float, float, float, float]]) -> np.ndarray:
        h, w = frame.shape[:2]
        canvas = frame.copy()
        if stop_line is not None:
            x1, y1, x2, y2, dst = stop_line
            # 안전 클램프
            x1 = int(np.clip(x1, 0, w - 1)); y1 = int(np.clip(y1, 0, h - 1))
            x2 = int(np.clip(x2, 0, w - 1)); y2 = int(np.clip(y2, 0, h - 1))
            # 선
            cv2.line(canvas, (x1, y1), (x2, y2),
                     self.viz_line_color_bgr, self.viz_line_thickness, cv2.LINE_AA)
            # 라벨(중앙 위)
            tx, ty = int((x1 + x2) / 2), int((y1 + y2) / 2) - 10
            cv2.putText(canvas, self.viz_text, (tx, max(0, ty)),
                        cv2.FONT_HERSHEY_SIMPLEX, self.viz_text_scale,
                        self.viz_line_color_bgr, self.viz_text_thickness, cv2.LINE_AA)
        else:
            cv2.putText(canvas, "NO STOP LINE", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2, cv2.LINE_AA)
        return canvas

    def publish_viz(self, image_bgr: np.ndarray, header=None) -> None:
        if self.viz_pub is None:
            return
        if self.viz_use_compressed:
            msg = CompressedImage()
            if header is not None:
                msg.header = header
            msg.format = "jpeg"
            msg.data = np.array(cv2.imencode(".jpg", image_bgr)[1]).tobytes()
            self.viz_pub.publish(msg)
        else:
            msg = self.bridge.cv2_to_imgmsg(image_bgr, "bgr8")
            if header is not None:
                msg.header = header
            self.viz_pub.publish(msg)

        # (NEW) visualize and publish overlay
        # if self.viz_enable and self.viz_pub is not None:
        #     out = self.draw_overlay(frame, stop_line)  # draw on full frame
        #     self.publish_viz(out, header)