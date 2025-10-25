#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Obstacle Converter Node
Converts vision_msgs/Detection3DArray to planning-friendly obstacle list
with Frenet coordinates and velocity information
"""

import rospy
import numpy as np
from vision_msgs.msg import Detection3DArray
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Header
from nav_msgs.msg import Path
import tf.transformations as tf_trans

class ObstacleConverter:
    def __init__(self):
        rospy.init_node('obstacle_converter', anonymous=True)
        
        # Subscriber
        self.detection_sub = rospy.Subscriber(
            '/cluster_result', 
            Detection3DArray, 
            self.detection_callback, 
            queue_size=1
        )
        
        # Publisher (planning 모듈에서 subscribe)
        self.obstacle_pub = rospy.Publisher(
            '/obstacles', 
            PoseArray,  # 간단하게 PoseArray 사용 (나중에 custom msg로 변경 가능)
            queue_size=1
        )
        
        # 이전 프레임 정보 (속도 계산용)
        self.prev_detections = {}
        self.prev_time = None
        
        rospy.loginfo("Obstacle Converter Node initialized")
        
    def detection_callback(self, msg):
        """Detection3DArray를 obstacle 정보로 변환"""
        
        current_time = msg.header.stamp.to_sec()
        
        obstacle_array = PoseArray()
        obstacle_array.header = msg.header
        
        for det in msg.detections:
            # ID 추출 (tracker가 할당한 stable ID)
            obj_id = det.results[0].id
            
            # 위치 정보
            x = det.bbox.center.position.x
            y = det.bbox.center.position.y
            z = det.bbox.center.position.z
            
            # 크기 정보
            length = det.bbox.size.x
            width = det.bbox.size.y
            height = det.bbox.size.z
            
            # Orientation (quaternion)
            qx = det.bbox.center.orientation.x
            qy = det.bbox.center.orientation.y
            qz = det.bbox.center.orientation.z
            qw = det.bbox.center.orientation.w
            
            # Yaw 계산
            euler = tf_trans.euler_from_quaternion([qx, qy, qz, qw])
            yaw = euler[2]
            
            # 속도 추정 (이전 프레임과 비교)
            vx, vy = 0.0, 0.0
            if obj_id in self.prev_detections and self.prev_time is not None:
                dt = current_time - self.prev_time
                if dt > 0.01:  # 최소 시간 간격
                    prev_x, prev_y = self.prev_detections[obj_id]
                    vx = (x - prev_x) / dt
                    vy = (y - prev_y) / dt
            
            # 현재 위치 저장 (다음 프레임용)
            self.prev_detections[obj_id] = (x, y)
            
            # Pose 메시지 생성 (임시로 position과 orientation만 사용)
            # 나중에 custom message로 변경하여 velocity, size 등 추가
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = z
            pose.orientation.x = qx
            pose.orientation.y = qy
            pose.orientation.z = qz
            pose.orientation.w = qw
            
            obstacle_array.poses.append(pose)
            
            rospy.logdebug(
                f"ID={obj_id}: pos=({x:.2f},{y:.2f}), "
                f"vel=({vx:.2f},{vy:.2f}), size=({length:.2f}x{width:.2f})"
            )
        
        self.prev_time = current_time
        self.obstacle_pub.publish(obstacle_array)
        rospy.loginfo(f"Published {len(obstacle_array.poses)} obstacles")

if __name__ == '__main__':
    try:
        converter = ObstacleConverter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
