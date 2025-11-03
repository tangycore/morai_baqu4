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
    DETECTED = 2       # 정지선 감지! (latched)
    STOPPED = 3        # 완전히 정지함

class StopLineDetection:

    def __init__(self) -> None:
        rospy.init_node("stop_line_detection", anonymous=False)

        self.bridge = CvBridge()

        # -------- State Machine --------
        self.state = StopState.IDLE
        self.stop_line_detected_once = False
        self.detection_time = None
        self.min_stop_duration = rospy.Duration(3.0)
        
        # 🔥 최근 감지된 정지선 (latching용)
        self.last_detected_stop_line = None
        self.frames_since_detection = 0
        self.max_frames_without_detection = 10  # 10프레임까지는 유지

        # -------- input params --------
        self.enabled = bool(rospy.get_param("~enabled_on_start", False))
        self.image_sub = None
        self._manage_input_sub(self.enabled)
        self.input_topic = rospy.get_param("~input_topic", "/camera/image_raw")
        self.use_compressed = rospy.get_param("~use_compressed", False)

        # 🔥 정지선 감지 파라미터 (완화됨)
        self.roi_y_ratio = float(rospy.get_param("~roi_y_ratio", 0.4))  # 0.55 → 0.4 (더 넓게)
        self.stop_roi_ratio = float(rospy.get_param("~stop_roi_ratio", 0.5))  # 0.25 → 0.5 (화면 절반)
        self.stop_line_pixels_per_meter = float(
            rospy.get_param("~stop_line_pixels_per_meter", 45.0)
        )
        self.stop_line_min_aspect = float(
            rospy.get_param("~stop_line_min_aspect", 1.5)  # 2.5 → 1.5 (완화)
        )
        self.stop_line_min_width = float(
            rospy.get_param("~stop_line_min_width", 50.0)  # 80 → 50 (완화)
        )
        self.output_topic = rospy.get_param("~output_topic", "/perception/stop_line")
        self.output_bool_topic = rospy.get_param("~output_bool_topic", "/perception/stop_decision")

        # 🔥 디버그 시각화
        self.debug_enable = rospy.get_param("~debug_enable", True)
        self.debug_image_pub = None
        if self.debug_enable:
            self.debug_image_pub = rospy.Publisher(
                "/stop_line/debug_image", Image, queue_size=1
            )

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
            self.last_detected_stop_line = None
            self.frames_since_detection = 0
            self._manage_input_sub(True)
            rospy.logwarn("[stop_line] 🔴 RED LIGHT - Start searching for stop line")
        
        elif not msg.data and self.enabled:
            # 초록불로 변경 → IDLE 상태로 리셋
            self.enabled = False
            self.state = StopState.IDLE
            self.stop_line_detected_once = False
            self.detection_time = None
            self.last_detected_stop_line = None
            self.frames_since_detection = 0
            self._manage_input_sub(False)
            rospy.logwarn("[stop_line] 🟢 GREEN LIGHT - Reset state")

    def _manage_input_sub(self, turn_on: bool) -> None:
        rospy.loginfo(f"[stop_line] managing input sub -> {turn_on}")
        if turn_on and self.image_sub is None:
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
        # 정지선 감지
        stop_line, debug_img = self.detect_stop_line_with_debug(frame)

        # 🔥 감지 정보 로그
        if stop_line is not None:
            rospy.loginfo(f"[stop_line] 📏 Detected at {stop_line[4]:.2f}m")
            self.last_detected_stop_line = stop_line
            self.frames_since_detection = 0
        else:
            self.frames_since_detection += 1
            rospy.logdebug(f"[stop_line] ❌ Not detected (frames: {self.frames_since_detection})")

        # State Machine 업데이트
        self.update_state(stop_line)

        # Publish
        self.publish_stop_line(stop_line if stop_line else self.last_detected_stop_line)
        self.publish_stop_decision()

        # 🔥 디버그 이미지 발행
        if self.debug_enable and self.debug_image_pub and debug_img is not None:
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(debug_img, "bgr8")
                debug_msg.header = header if header else rospy.Header()
                self.debug_image_pub.publish(debug_msg)
            except Exception as e:
                rospy.logwarn(f"Failed to publish debug image: {e}")

    def update_state(self, stop_line: Optional[Tuple[float, float, float, float, float]]) -> None:
        """State Machine 업데이트"""
        
        if self.state == StopState.IDLE:
            pass
        
        elif self.state == StopState.SEARCHING:
            # 정지선 찾는 중
            if stop_line is not None and stop_line[4] < 5.0:  # 🔥 5m 이내
                # 정지선 발견!
                self.state = StopState.DETECTED
                self.stop_line_detected_once = True
                self.detection_time = rospy.Time.now()
                rospy.logwarn(f"[stop_line] 🛑 STOP LINE DETECTED at {stop_line[4]:.2f}m!")
        
        elif self.state == StopState.DETECTED:
            # 🔥 Latched state - 정지선이 안 보여도 일정 프레임 동안 유지
            if self.frames_since_detection > self.max_frames_without_detection:
                # 너무 오래 안 보이면 SEARCHING으로 복귀 (재감지)
                rospy.logwarn(f"[stop_line] ⚠️ Lost stop line for {self.frames_since_detection} frames")
                # self.state = StopState.SEARCHING  # 선택: 재검색 or 계속 유지
            
            rospy.loginfo_throttle(1.0, f"[stop_line] 🛑 DETECTED (lost frames: {self.frames_since_detection})")
            
            # 일정 시간 후 STOPPED로 전환
            if self.detection_time and (rospy.Time.now() - self.detection_time) > self.min_stop_duration:
                self.state = StopState.STOPPED
                rospy.loginfo("[stop_line] ⏸️ STOPPED state")
        
        elif self.state == StopState.STOPPED:
            rospy.loginfo_throttle(2.0, "[stop_line] ⏸️ STOPPED - waiting for green")

    def detect_stop_line_with_debug(
        self, frame: np.ndarray
    ) -> Tuple[Optional[Tuple[float, float, float, float, float]], Optional[np.ndarray]]:
        """정지선 감지 + 디버그 이미지 생성"""
        h, w = frame.shape[:2]
        
        # 🔥 ROI 설정 (화면 하단 50%)
        start_y = int(h * (1.0 - self.stop_roi_ratio))
        roi = frame[start_y:, :]
        
        # Debug 이미지 초기화
        debug_img = frame.copy()
        cv2.rectangle(debug_img, (0, start_y), (w, h), (0, 255, 0), 2)
        cv2.putText(debug_img, "ROI", (10, start_y + 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        # HSV 변환 및 흰색 검출
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        # 🔥 흰색 범위 완화
        lower_white = np.array([0, 0, 180], dtype=np.uint8)  # 200 → 180
        upper_white = np.array([180, 80, 255], dtype=np.uint8)  # 70 → 80
        mask = cv2.inRange(hsv, lower_white, upper_white)
        
        # 모폴로지 연산
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        mask = cv2.GaussianBlur(mask, (7, 7), 0)
        _, thresh = cv2.threshold(mask, 150, 255, cv2.THRESH_BINARY)  # 200 → 150
        
        # Contour 찾기
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # 🔥 디버그: 모든 contour 그리기
        debug_roi = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)
        cv2.drawContours(debug_roi, contours, -1, (0, 255, 0), 2)
        debug_img[start_y:, :] = cv2.addWeighted(debug_img[start_y:, :], 0.5, debug_roi, 0.5, 0)
        
        best_line = None
        best_width = 0.0
        best_score = 0.0
        
        rospy.logdebug(f"[stop_line] Found {len(contours)} contours")
        
        for idx, contour in enumerate(contours):
            # Bounding rect
            rect = cv2.minAreaRect(contour)
            (cx, cy), (width, height), angle = rect
            
            # Width가 더 길도록
            if width < height:
                width, height = height, width
                angle += 90.0
            
            # 🔥 조건 완화
            if height < 3:  # 5 → 3
                continue
            if abs(angle) > 30.0:  # 20 → 30
                continue
            
            aspect = width / float(max(height, 1e-3))
            area = width * height
            
            # 🔥 디버그 로그
            rospy.logdebug(f"  Contour {idx}: w={width:.1f}, h={height:.1f}, "
                          f"aspect={aspect:.2f}, angle={angle:.1f}°")
            
            if aspect < self.stop_line_min_aspect:
                rospy.logdebug(f"    → Rejected: aspect {aspect:.2f} < {self.stop_line_min_aspect}")
                continue
            if width < self.stop_line_min_width:
                rospy.logdebug(f"    → Rejected: width {width:.1f} < {self.stop_line_min_width}")
                continue
            
            # 점수 계산 (넓이 + 가로 길이)
            score = area + width * 2
            
            if score > best_score:
                best_score = score
                best_width = width
                
                # 좌표 계산
                x1 = cx - width / 2.0
                x2 = cx + width / 2.0
                y_world = cy + start_y
                distance_pixels = float(h - y_world)
                px_per_meter = max(self.stop_line_pixels_per_meter, 1e-3)
                distance_m = max(0.0, distance_pixels / px_per_meter)
                
                best_line = (float(x1), float(y_world), float(x2), float(y_world), distance_m)
                
                # 🔥 디버그: 최적 라인 그리기
                cv2.line(debug_img, (int(x1), int(y_world)), (int(x2), int(y_world)), 
                        (0, 0, 255), 3)
                cv2.putText(debug_img, f"STOP: {distance_m:.2f}m", 
                           (int(x1), int(y_world) - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                
                rospy.loginfo(f"  → BEST: w={width:.1f}, aspect={aspect:.2f}, "
                             f"dist={distance_m:.2f}m, score={score:.0f}")
        
        if best_line is None:
            cv2.putText(debug_img, "NO STOP LINE", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 3)
        
        return best_line, debug_img

    @staticmethod
    def extract_roi(frame: np.ndarray, roi_ratio: float) -> Tuple[np.ndarray, int]:
        h = frame.shape[0]
        roi_y = int(h * roi_ratio)
        return frame[roi_y:, :], roi_y

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
        """State Machine 기반 정지 결정"""
        should_stop = self.state in [StopState.DETECTED, StopState.STOPPED]
        
        if should_stop:
            dist_str = f"{self.last_detected_stop_line[4]:.2f}m" if self.last_detected_stop_line else "N/A"
            rospy.logwarn_throttle(1.0, f"[stop_line] 🛑 STOP (state={self.state.name}, dist={dist_str})")
        
        self.stop_decision_pub.publish(Bool(data=should_stop))

    def spin(self) -> None:
        rospy.spin()


def main() -> None:
    StopLineDetection().spin()


if __name__ == "__main__":
    main()