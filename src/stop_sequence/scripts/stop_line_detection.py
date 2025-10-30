#!/usr/bin/env python3

from typing import Optional, Tuple

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

class StopLineDetection:

    def __init__(self) -> None:
        rospy.init_node("stop_line_detection", anonymous=False)

        self.bridge = CvBridge()

        # -------- input params --------
        self.input_topic = rospy.get_param("~input_topic", "/camera/image_raw")
        self.use_compressed = rospy.get_param("~use_compressed", False)
        self.roi_y_ratio = float(rospy.get_param("~roi_y_ratio", 0.55))
        self.stop_roi_ratio = float(rospy.get_param("~stop_roi_ratio", 0.25))
        self.stop_line_pixels_per_meter = float(
            rospy.get_param("~stop_line_pixels_per_meter", 45.0)
        )
        self.stop_line_min_aspect = float(
            rospy.get_param("~stop_line_min_aspect", 2.5)
        )
        self.stop_line_min_width = float(
            rospy.get_param("~stop_line_min_width", 80.0)
        )
        self.output_topic = rospy.get_param("~output_topic", "/perception/stop_line")

        # -------- viz params (NEW) --------
        self.viz_enable = rospy.get_param("~viz_enable", True)  # NEW
        self.viz_use_compressed = rospy.get_param("~viz_use_compressed", self.use_compressed)  # NEW
        # 기본 토픽명: private 네임스페이스 기준 ~out/...
        self.viz_topic = rospy.get_param("~viz_topic",
                                         "~out/compressed" if self.viz_use_compressed else "~out/image_raw")  # NEW
        self.viz_line_color_bgr = tuple(int(x) for x in rospy.get_param("~viz_line_color_bgr", [0, 0, 255]))  # RED  # NEW
        self.viz_line_thickness = int(rospy.get_param("~viz_line_thickness", 4))  # NEW
        self.viz_text = rospy.get_param("~viz_text", "STOP LINE")  # NEW
        self.viz_text_scale = float(rospy.get_param("~viz_text_scale", 0.8))  # NEW
        self.viz_text_thickness = int(rospy.get_param("~viz_text_thickness", 2))  # NEW

        # -------- subs --------
        if self.use_compressed:
            self.sub = rospy.Subscriber(
                self.input_topic, CompressedImage, self.compressed_cb, queue_size=1
            )
        else:
            self.sub = rospy.Subscriber(
                self.input_topic, Image, self.image_cb, queue_size=1
            )
        rospy.loginfo(
            "\033[1m[stop_line] subscribe: %s (compressed=%s)\033[0m",
            self.input_topic,
            self.use_compressed,
        )

        # -------- pubs --------
        self.stop_pub = rospy.Publisher(
            self.output_topic, Float32MultiArray, queue_size=1
        )
        rospy.loginfo("\033[1m[stop_line] publish: %s \033[0m", self.output_topic)

        # (NEW) viz publisher
        self.viz_pub = None  # NEW
        if self.viz_enable:  # NEW
            if self.viz_use_compressed:
                self.viz_pub = rospy.Publisher(self.viz_topic, CompressedImage, queue_size=1)
            else:
                self.viz_pub = rospy.Publisher(self.viz_topic, Image, queue_size=1)
            rospy.loginfo("\033[1m[stop_line] viz publish: %s (compressed=%s)\033[0m",
                          rospy.resolve_name(self.viz_topic), self.viz_use_compressed)

    def compressed_cb(self, msg: CompressedImage) -> None:
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            rospy.logwarn("[stop_line] JPEG decode failed.")  # CHANGED tag
            return
        self.handle_frame(frame, header=msg.header)  # pass header for stamp/frame_id

    def image_cb(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as exc:
            rospy.logwarn("[stop_line] cv_bridge error: %s", exc)  # CHANGED tag
            return
        self.handle_frame(frame, header=msg.header)

    def handle_frame(self, frame: np.ndarray, header=None) -> None:  # CHANGED: header param
        roi_color, roi_y = self.extract_roi(frame, self.roi_y_ratio)

        stop_line = self.detect_stop_line(frame, roi_y)

        self.publish_stop_line(stop_line)

        # (NEW) visualize and publish overlay
        if self.viz_enable and self.viz_pub is not None:
            out = self.draw_overlay(frame, stop_line)  # draw on full frame
            self.publish_viz(out, header)

    @staticmethod
    def extract_roi(frame: np.ndarray, roi_ratio: float) -> Tuple[np.ndarray, int]:
        h = frame.shape[0]
        roi_y = int(h * roi_ratio)
        return frame[roi_y:, :], roi_y

    @staticmethod
    def threshold_white(hsv: np.ndarray) -> np.ndarray:
        lower_white = np.array([0, 0, 200], dtype=np.uint8)
        upper_white = np.array([180, 70, 255], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower_white, upper_white)
        return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8), 2)

    def detect_stop_line(
        self, frame: np.ndarray, roi_y: int
    ) -> Optional[Tuple[float, float, float, float, float]]:
        h, _ = frame.shape[:2]
        start_y = max(int(h * (1.0 - self.stop_roi_ratio)), roi_y)
        roi = frame[start_y:, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = self.threshold_white(hsv)
        mask = cv2.GaussianBlur(mask, (5, 5), 0)
        _, thresh = cv2.threshold(mask, 200, 255, cv2.THRESH_BINARY)
        contours, _hier = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best_line = None
        best_width = 0.0
        for contour in contours:
            rect = cv2.minAreaRect(contour)
            (cx, cy), (width, height), angle = rect
            if width < height:
                width, height = height, width
                angle += 90.0

            if height < 5:
                continue
            if abs(angle) > 20.0:
                continue
            aspect = width / float(max(height, 1e-3))
            if aspect < self.stop_line_min_aspect:
                continue
            if width < self.stop_line_min_width:
                continue
            if width < best_width:
                continue
            best_width = width
            x1 = cx - width / 2.0
            x2 = cx + width / 2.0
            y_world = cy + start_y
            distance_pixels = float(h - y_world)
            px_per_meter = max(self.stop_line_pixels_per_meter, 1e-3)
            distance_m = max(0.0, distance_pixels / px_per_meter)
            best_line = (float(x1), float(y_world), float(x2), float(y_world), distance_m)

        return best_line



    def publish_stop_line(
        self, stop_line: Optional[Tuple[float, float, float, float, float]]
    ) -> None:
        array = Float32MultiArray()
        if stop_line is not None:
            array.data = [stop_line[0], stop_line[1], stop_line[2], stop_line[3], stop_line[4]]
            dim = MultiArrayDimension()
            dim.label = "stop_line[x1,y1,x2,y2,distance]"
            dim.size = 1
            dim.stride = 5
            field_dim = MultiArrayDimension()
            field_dim.label = "fields"
            field_dim.size = 5
            field_dim.stride = 1
            array.layout.dim = [dim, field_dim]
        self.stop_pub.publish(array)

    # ---------- NEW: overlay + publish ----------
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
    # ---------- NEW end ----------

    def spin(self) -> None:
        rospy.spin()


def main() -> None:
    StopLineDetection().spin()


if __name__ == "__main__":
    main()
