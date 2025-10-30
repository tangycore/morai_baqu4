#!/usr/bin/env python3

from typing import Optional, Tuple

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, Bool

class StopLineDetection:

    def __init__(self) -> None:
        rospy.init_node("stop_line_detection", anonymous=False)

        self.bridge = CvBridge()

        # -------- input params --------
        self.enabled = bool(rospy.get_param("~enabled_on_start", False))

        self.image_sub = None  # image subscriber placeholder
        self._manage_input_sub(self.enabled)
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
        self.output_bool_topic = rospy.get_param("~output_bool_topic", "/perception/stop_decision")

        self.enable_sub = rospy.Subscriber("/stop_sequence/enable", Bool, self._on_enable, queue_size=1)

        # -------- pubs --------
        self.stop_line_pub = rospy.Publisher(
            self.output_topic, Float32MultiArray, queue_size=1
        )
        self.stop_decision_pub = rospy.Publisher(
            self.output_bool_topic, int, queue_size=1
        )
        rospy.loginfo("\033[1m[stop_line] publish: %s, %s\033[0m", self.output_topic, self.output_bool_topic)
    
    def _on_enable(self, msg: Bool) -> None:
        if msg.data != self.enabled:
            self.enabled = msg.data
            self._manage_input_sub(self.enabled)
            rospy.loginfo(f"[stop_line] enabled -> {self.enabled}")

    def _manage_input_sub(self, turn_on: bool) -> None:
        rospy.loginfo(f"[stop_line] managing input sub -> {turn_on}")
        if turn_on and self.image_sub is None:
            # -------- subs --------
            if self.use_compressed:
                self.image_sub = rospy.Subscriber(
                    self.input_topic, CompressedImage, self.compressed_cb, queue_size=1
                )
            else:
                self.image_sub = rospy.Subscriber(
                    self.input_topic, Image, self.image_cb, queue_size=1
                )
            rospy.loginfo(
                "\033[1m[stop_line] subscribe: %s (compressed=%s)\033[0m",
                self.input_topic,
                self.use_compressed,
            )
        elif (not turn_on) and self.image_sub is not None:
            self.image_sub.unregister()
            self.image_sub = None
            rospy.loginfo("[stop_line] image subscription STOPPED")

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
        self.publish_stop_decision(stop_line)

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
        self.stop_line_pub.publish(array)

    def publish_stop_decision(
        self, stop_line: Optional[Tuple[float, float, float, float, float]]
    ) -> None:
        stop = stop_line is not None and stop_line[4] > 1.0  # 1 meter threshold
        if stop:
            rospy.logwarn("[stop_line] STOP line detected within 1 meter.")
        self.stop_decision_pub.publish(Bool(data=stop))

    def spin(self) -> None:
        rospy.spin()


def main() -> None:
    StopLineDetection().spin()


if __name__ == "__main__":
    main()
