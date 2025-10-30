#!/usr/bin/env python3

from typing import Optional, Tuple
from enum import Enum

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, Bool

class StopState(Enum):
    """정지 상태 머신"""
    IDLE = 0           # 비활성화 (초록불)
    SEARCHING = 1      # 정지선 찾는 중 (빨간불, 아직 정지선 못 찾음)
    DETECTED = 2       # 정지선 감지! (latched - 이 상태에서는 계속 정지)
    STOPPED = 3        # 완전히 정지함 (속도 0)

class StopLineDetection:

    def __init__(self) -> None:
        rospy.init_node("stop_line_detection", anonymous=False)

        self.bridge = CvBridge()

        # -------- State Machine --------
        self.state = StopState.IDLE
        self.stop_line_detected_once = False  # Latch flag
        self.detection_time = None  # 정지선 최초 감지 시간
        self.min_stop_duration = rospy.Duration(3.0)  # 최소 정지 시간 3초

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
            self.output_bool_topic, Bool, queue_size=1
        )
        rospy.loginfo("\033[1m[stop_line] publish: %s, %s\033[0m", self.output_topic, self.output_bool_topic)
    
    def _on_enable(self, msg: Bool) -> None:
        """신호등 상태 콜백: True=빨간불, False=초록불"""
        if msg.data and not self.enabled:
            # 빨간불로 변경 → SEARCHING 상태로
            self.enabled = True
            self.state = StopState.SEARCHING
            self.stop_line_detected_once = False
            self._manage_input_sub(True)
            rospy.loginfo("[stop_line] 🔴 RED LIGHT - Start searching for stop line")
        
        elif not msg.data and self.enabled:
            # 초록불로 변경 → IDLE 상태로 리셋
            self.enabled = False
            self.state = StopState.IDLE
            self.stop_line_detected_once = False
            self.detection_time = None
            self._manage_input_sub(False)
            rospy.loginfo("[stop_line] 🟢 GREEN LIGHT - Reset state")

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
            rospy.logwarn("[stop_line] JPEG decode failed.")
            return
        self.handle_frame(frame, header=msg.header)

    def image_cb(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as exc:
            rospy.logwarn("[stop_line] cv_bridge error: %s", exc)
            return
        self.handle_frame(frame, header=msg.header)

    def handle_frame(self, frame: np.ndarray, header=None) -> None:
        roi_color, roi_y = self.extract_roi(frame, self.roi_y_ratio)

        stop_line = self.detect_stop_line(frame, roi_y)

        # 🔥 State Machine 로직
        self.update_state(stop_line)

        # Publish
        self.publish_stop_line(stop_line)
        self.publish_stop_decision()

    def update_state(self, stop_line: Optional[Tuple[float, float, float, float, float]]) -> None:
        """State Machine 업데이트"""
        
        if self.state == StopState.IDLE:
            # 비활성화 상태 - 아무것도 안 함
            pass
        
        elif self.state == StopState.SEARCHING:
            # 정지선 찾는 중
            if stop_line is not None and stop_line[4] < 3.0:  # 3m 이내
                # 정지선 발견! → DETECTED 상태로
                self.state = StopState.DETECTED
                self.stop_line_detected_once = True
                self.detection_time = rospy.Time.now()
                rospy.logwarn("[stop_line] 🛑 STOP LINE DETECTED! Entering LATCHED state")
        
        elif self.state == StopState.DETECTED:
            # 정지선 감지됨 (Latched) - 이 상태에서는 계속 정지!
            # 정지선이 안 보여도 상태 유지
            rospy.loginfo_throttle(1.0, "[stop_line] 🛑 DETECTED state - maintaining stop")
            
            # 선택: 일정 시간 후 자동으로 STOPPED 상태로 전환
            if self.detection_time and (rospy.Time.now() - self.detection_time) > self.min_stop_duration:
                self.state = StopState.STOPPED
                rospy.loginfo("[stop_line] ⏸️ Minimum stop duration reached - STOPPED state")
        
        elif self.state == StopState.STOPPED:
            # 완전히 정지 완료 - 초록불 올 때까지 대기
            rospy.loginfo_throttle(2.0, "[stop_line] ⏸️ STOPPED - waiting for green light")

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

    def publish_stop_decision(self) -> None:
        """🔥 State Machine 기반 정지 결정"""
        # DETECTED 또는 STOPPED 상태면 무조건 정지!
        should_stop = self.state in [StopState.DETECTED, StopState.STOPPED]
        
        if should_stop:
            rospy.logwarn_throttle(1.0, f"[stop_line] 🛑 STOP (state={self.state.name})")
        
        self.stop_decision_pub.publish(Bool(data=should_stop))

    def spin(self) -> None:
        rospy.spin()


def main() -> None:
    StopLineDetection().spin()


if __name__ == "__main__":
    main()