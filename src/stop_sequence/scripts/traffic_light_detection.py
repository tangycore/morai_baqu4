import cv_bridge
import cv2
import numpy as np
import rospkg
import rospy
import sensor_msgs.msg as sensor_msg
import vision_msgs.msg as vision_msg
import std_msgs.msg as std_msg
from ultralytics import YOLO


class TrafficLightDetection:
    def __init__(self):
        # ROS1 노드 초기화
        rospy.init_node("tracker_node", anonymous=False)

        # ==== Parameters (ROS1: ~네임스페이스) ====
        self.model_name        = rospy.get_param("~model", "custom.pt")
        self.input_topic       = rospy.get_param("~input_topic", "/image_jpeg/compressed")
        self.result_topic      = rospy.get_param("~result_topic", "/perception/result")
        self.result_image_topic= rospy.get_param("~result_image_topic", "/perception/image_raw")

        self.conf_thres        = float(rospy.get_param("~conf_thres", 0.8))
        self.iou_thres         = float(rospy.get_param("~iou_thres", 0.45))
        self.max_det           = int(rospy.get_param("~max_det", 300))
        self.classes           = rospy.get_param("~classes", list(range(4)))  # 예: [0,1,2]
        self.tracker           = rospy.get_param("~tracker", "bytetrack.yaml")
        self.device            = rospy.get_param("~device", "mps")  # Linux GPU면 'cuda' 또는 0 권장

        self.result_conf       = bool(rospy.get_param("~result_conf", True))
        self.result_line_width = int(rospy.get_param("~result_line_width", 1))
        self.result_font_size  = int(rospy.get_param("~result_font_size", 1))
        self.result_labels     = bool(rospy.get_param("~result_labels", True))
        self.result_boxes      = bool(rospy.get_param("~result_boxes", True))

        rospack = rospkg.RosPack()
        path = rospack.get_path('stop_sequence')
        self.model = YOLO(f"{path}/models/{self.model_name}")
        self.model.fuse()

        # ==== Bridge ====
        self.bridge = cv_bridge.CvBridge()

        # ==== Pub/Sub ====
        self.sub = rospy.Subscriber(self.input_topic, sensor_msg.CompressedImage,
                                    self.image_callback, queue_size=1)
        self.pub_stop_enable = rospy.Publisher("/stop_sequence/enable", std_msg.Bool, queue_size=1, latch=True)
        self.result_pub = rospy.Publisher(self.result_topic,
                                          vision_msg.Detection2DArray, queue_size=1)
        self.result_image_pub = rospy.Publisher(self.result_image_topic,
                                                sensor_msg.Image, queue_size=1)

        rospy.loginfo("\033[1m[traffic_light] started -> Model: %s\033[0m", self.model_name)

    
    def image_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            cv_image = cv2.resize(cv_image, (640, 640), interpolation=cv2.INTER_LINEAR)
        except Exception as e:
            rospy.logerr("cv_bridge conversion failed: %s", str(e))
            return
        
        # Yolov8 Tracking
        try:
            result = self.model.track(
                source=cv_image,
                conf=self.conf_thres,
                iou=self.iou_thres,
                max_det=self.max_det,
                classes=self.classes,
                tracker=self.tracker,
                device=self.device,
                verbose=False,
                retina_masks=True
            )
        except Exception as e:
            rospy.logerr("YOLOv8 tracking failed: %s", str(e))
            return
        
        if not result:
            return

        try:
            result_msg = self.create_detections(result)
            result_image_msg = self.create_image(result)

            if hasattr(result_msg, "header"):
                result_msg.header = msg.header
            if hasattr(result_image_msg, "header"):
                result_image_msg.header = msg.header

            self.result_pub.publish(result_msg)
            self.result_image_pub.publish(result_image_msg)

            self.on_inference_result(result_msg)
        except Exception as e:
            rospy.logerr("Result publishing failed: %s", str(e))

    def create_detections(self, result):
        detections_msg = vision_msg.Detection2DArray()

        b = result[0].boxes
        if b is None or b.xywh is None:
            return detections_msg

        bounding_box = b.xywh
        classes = b.cls
        confidence_score = b.conf
        names = result[0].names if hasattr(result[0], "names") else {}

        for bbox, cls, conf in zip(bounding_box, classes, confidence_score):
            detection = vision_msg.Detection2D()

            # center: Pose2D(x,y,theta)
            detection.bbox.center.x = float(bbox[0])
            detection.bbox.center.y = float(bbox[1])
            detection.bbox.center.theta = 0.0
            detection.bbox.size_x = float(bbox[2])
            detection.bbox.size_y = float(bbox[3])

            hypothesis = vision_msg.ObjectHypothesisWithPose()
            label = names.get(int(cls), str(int(cls)))
            score = float(conf)

            if hasattr(hypothesis, "id"):
                hypothesis.id = label
            if hasattr(hypothesis, "score"):
                hypothesis.score = score

            detection.results.append(hypothesis)
            detections_msg.detections.append(detection)

        return detections_msg

    def create_image(self, result):
        plotted_image = result[0].plot(
            conf=self.result_conf,
            line_width=self.result_line_width,
            font_size=self.result_font_size,
            labels=self.result_labels,
            boxes=self.result_boxes,
        )
        image_msg = self.bridge.cv2_to_imgmsg(
            plotted_image, encoding="bgr8"
        )
        return image_msg
    
    def on_inference_result(self, result):
        red_detected = self._is_red(result)
        self.pub_stop_enable.publish(std_msg.Bool(data=red_detected))

    def _is_red(self, results: vision_msg.Detection2DArray) -> bool:
        ids = {getattr(h, "id", None) for d in results.detections for h in d.results}
        return "red" in ids or "red-arrow" in ids
    
    def spin(self):
        rospy.spin()


def main():
    node = TrafficLightDetection()
    node.spin()


if __name__ == '__main__':
    main()