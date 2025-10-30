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

        self.input_topic = rospy.get_param("~input_topic", "/camera/image_raw")
        self.use_compressed = rospy.get_param("~use_compressed", False)
        self.roi_y_ratio = float(rospy.get_param("~roi_y_ratio", 0.55))
        self.stop_roi_ratio = float(rospy.get_param("~stop_roi_ratio", 0.25))
        self.output_topic = rospy.get_param("~output_topic", "/perception/stop_line")

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
        self.stop_pub = rospy.Publisher(
            self.output_topic, Float32MultiArray, queue_size=1
        )
        rospy.loginfo(
            "\033[1m[stop_line] publish: %s \033[0m",
            self.output_topic
        )

    def compressed_cb(self, msg: CompressedImage) -> None:
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            rospy.logwarn("[lane_marking] JPEG decode failed.")
            return
        self.handle_frame(frame)

    def image_cb(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as exc:  # pragma: no cover
            rospy.logwarn("[lane_marking] cv_bridge error: %s", exc)
            return
        self.handle_frame(frame)

    def handle_frame(self, frame: np.ndarray) -> None:
        roi_color, roi_y = self.extract_roi(frame, self.roi_y_ratio)

        stop_line = self.detect_stop_line(frame, roi_y)

        self.publish_stop_line(stop_line)

    @staticmethod
    def extract_roi(frame: np.ndarray, roi_ratio: float) -> Tuple[np.ndarray, int]:
        h = frame.shape[0]
        roi_y = int(h * roi_ratio)
        return frame[roi_y:, :], roi_y

    @staticmethod
    def threshold_white(hsv: np.ndarray) -> np.ndarray:
        """밝은 흰색 픽셀 마스크."""
        lower_white = np.array([0, 0, 200], dtype=np.uint8)
        upper_white = np.array([180, 70, 255], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower_white, upper_white)
        return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8), 2)

    def detect_stop_line(
        self, frame: np.ndarray, roi_y: int
    ) -> Optional[Tuple[float, float, float, float]]:
        """영상 하단에서 정지선(수평 흰선) 탐지."""
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
            if width < best_width:
                continue
            best_width = width
            x1 = cx - width / 2.0
            x2 = cx + width / 2.0
            y_world = cy + start_y
            best_line = (float(x1), float(y_world), float(x2), float(y_world))

        return best_line

    def publish_stop_line(self, stop_line: Optional[Tuple[float, float, float, float]]) -> None:
        array = Float32MultiArray()
        if stop_line is not None:
            array.data = [stop_line[0], stop_line[1], stop_line[2], stop_line[3]]
            dim = MultiArrayDimension()
            dim.label = "stop_line[x1,y1,x2,y2]"
            dim.size = 1
            dim.stride = 4
            field_dim = MultiArrayDimension()
            field_dim.label = "fields"
            field_dim.size = 4
            field_dim.stride = 1
            array.layout.dim = [dim, field_dim]
        self.stop_pub.publish(array)

    def spin(self) -> None:
        rospy.spin()


def main() -> None:
    StopLineDetection().spin()


if __name__ == "__main__":
    main()