#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sys
from geometry_msgs.msg import Point32, PoseStamped
from nav_msgs.msg import Odometry, Path
from morai_msgs.msg import EgoVehicleStatus
from vision_msgs.msg import Detection3DArray  # ★ 추가
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np

from planning_pkg.frenet import world2frenet, frenet2world, find_closest_waypoint
from planning_pkg.planner import (
    generate_opt_path,
    generate_velocity_keeping_trajectories_in_frenet,
    frenet_paths_to_world, 
    check_valid_path
)
from planning_pkg.config import (
    DESIRED_LAT_POS,
    FINAL_DESIRED_SPEED,
    LOCAL_REF_WINDOW_M,
    LOCAL_REF_BACKWARD_M,
    LAT_ACCEL_MAX,
)
from planning_pkg.msg import PlanVelocityInfo

import math

class VehicleState:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.heading = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.v = 0.0
        self.ax = 0.0
        self.ay = 0.0
        self.az = 0.0
        self.a = 0.0

    def update_from_msg(self, msg):
        self.heading = math.radians(msg.heading)
        self.vx = msg.velocity.x / 3.6
        self.vy = msg.velocity.y / 3.6
        self.vz = msg.velocity.z / 3.6
        self.v = math.sqrt(self.vx**2 + self.vy**2 + self.vz**2)
        self.ax = msg.acceleration.x / 3.6
        self.ay = msg.acceleration.y / 3.6
        self.az = msg.acceleration.z / 3.6
        self.a = math.sqrt(self.ax**2 + self.ay**2 + self.az**2)

    def __str__(self):
        return f"(x={self.x:.3f}, y={self.y:.3f}, heading={math.degrees(self.heading):.2f}°, v={self.v:.3f} m/s)"

class LocalPath:
    def __init__(self):
        rospy.init_node('local_path', anonymous=True)
        
        # Publishers
        self.opt_local_path_pub = rospy.Publisher('/opt_path', Path, queue_size=1)
        self.valid_local_path_pub = rospy.Publisher('/valid_local_path', Path, queue_size=1)
        self.frenet_path_pub = rospy.Publisher('/all_frenet_path', Path, queue_size=1)
        self.plan_velocity_info_pub = rospy.Publisher('/plan_velocity_info', PlanVelocityInfo, queue_size=1)

        # Path messages
        self.opt_local_path_msg = Path()
        self.opt_local_path_msg.header.frame_id = '/map'
        self.valid_local_path_msg = Path()
        self.valid_local_path_msg.header.frame_id = '/map'
        self.frenet_path_msg = Path()
        self.frenet_path_msg.header.frame_id = '/map'
        self.plan_velocity_info_msg = PlanVelocityInfo()
        
        # Subscribers
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/global_path", Path, self.global_path_callback)
        rospy.Subscriber("/morai/competition_status", EgoVehicleStatus, self.status_callback)
        rospy.Subscriber("/cluster_result", Detection3DArray, self.cluster_callback)  # ★ 추가

        # State variables
        self.ego_state = VehicleState()
        self.global_path = None
        self.obstacles = []  # ★ 추가
        
        # Reference line
        self.refer_xlist = []
        self.refer_ylist = []
        self.refer_slist = []
        self.path_resolution = 0.5
        self.ref_window_m = LOCAL_REF_WINDOW_M
        self.ref_backward_m = LOCAL_REF_BACKWARD_M
        self.ref_window_points = 0
        self.ref_backward_points = 0
        self.segment_start_idx = 0
        self.segment_end_idx = 0
        self.closest_wp_idx = 0
        self.segment_xlist = np.array([])
        self.segment_ylist = np.array([])
        self.segment_slist = np.array([])
        self.segment_refresh_margin_points = 10
        
        # Planning state
        self.prev_opt_target_speed = 0.0
        self.s0, self.s1, self.s2 = 0, 0, 0
        self.d0, self.d1, self.d2 = 0, 0, 0
        self.no_valid_path_cnt = 0
        
        # Configuration
        self.ego_filter_radius = 2.0  # ★ Ego vehicle 필터링 반경
        
        self.setup_reference_line()

    def status_callback(self, msg):
        self.is_status = True
        self.ego_state.update_from_msg(msg)

    def odom_callback(self, msg):
        odom_quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        _, _, self.ego_state.heading = euler_from_quaternion(odom_quaternion)
        self.ego_state.x = msg.pose.pose.position.x
        self.ego_state.y = msg.pose.pose.position.y

    def global_path_callback(self, msg):
        if self.global_path is None:
            self.global_path = msg

    def cluster_callback(self, msg):
        # rospy.logwarn("="*60)
        # rospy.logwarn(f"[CLUSTER_CALLBACK] Received {len(msg.detections)} detections")
        # rospy.logwarn(f"[CLUSTER_CALLBACK] ego_state: x={self.ego_state.x:.2f}, y={self.ego_state.y:.2f}, heading={self.ego_state.heading:.2f}")
        
        self.obstacles = []
        
        cos_h = math.cos(self.ego_state.heading)
        sin_h = math.sin(self.ego_state.heading)
        
        for i, detection in enumerate(msg.detections):
            x_base = detection.bbox.center.position.x
            y_base = detection.bbox.center.position.y
            
            dist = np.hypot(x_base, y_base)
            
            # rospy.logwarn(f"  Det {i}: x_base={x_base:.2f}, y_base={y_base:.2f}, dist={dist:.2f}")
            
            if dist <= self.ego_filter_radius:
                # rospy.logwarn(f"    → FILTERED (ego)")
                continue

            # 2. Guardrail filtering (size/ratio check)
            length = detection.bbox.size.x
            width = detection.bbox.size.y
            height = detection.bbox.size.z

            # Too long -> guardrail
            if length > 10.0:
                # rospy.logwarn(f"    -> FILTERED (too long, length={length:.1f}m)")
                continue
            
            # Aspect ratio check
            aspect_ratio = length / (width + 1e-6)
            if aspect_ratio > 8.0:
                # rospy.logwarn(f"    -> FILTERED (elongated, ratio={aspect_ratio:.1f})")
                continue
            
            # 변환
            x_map = self.ego_state.x + x_base * cos_h - y_base * sin_h
            y_map = self.ego_state.y + x_base * sin_h + y_base * cos_h
            
            # rospy.logwarn(f"    → x_map={x_map:.2f}, y_map={y_map:.2f}")
            
            obstacle = {
                'object': type('obj', (), {
                    'x': x_map,
                    'y': y_map,
                    'width': detection.bbox.size.y,
                    'height': detection.bbox.size.x
                })()
            }
            
            self.obstacles.append(obstacle)
        
        # rospy.logwarn(f"[CLUSTER_CALLBACK] Final obstacles: {len(self.obstacles)}")
        # rospy.logwarn("="*60)

    def setup_reference_line(self):
        while not self.global_path:
            rospy.logdebug_once("Waiting global path")
        rospy.loginfo("find global path")
        
        total = len(self.global_path.poses)
        for i, pose in enumerate(self.global_path.poses):
            self.refer_xlist.append(pose.pose.position.x)
            self.refer_ylist.append(pose.pose.position.y)

            progress = (i + 1) / total
            bar = '=' * int(progress * 40)
            sys.stdout.write(f"\rProgress: [{bar:<40}] {progress*100:6.2f}%")
            sys.stdout.flush()

        sys.stdout.write("\n")
        rospy.loginfo("Reference line setup complete")

        self.refer_xlist = np.array(self.refer_xlist, dtype=float)
        self.refer_ylist = np.array(self.refer_ylist, dtype=float)

        diffs = np.hypot(np.diff(self.refer_xlist), np.diff(self.refer_ylist))
        if diffs.size > 0:
            sample_count = min(200, diffs.size)
            self.path_resolution = float(np.mean(diffs[:sample_count]))
        
        self.ref_window_points = int(np.ceil(self.ref_window_m / max(self.path_resolution, 1e-3)))
        self.ref_backward_points = int(np.ceil(self.ref_backward_m / max(self.path_resolution, 1e-3)))
        self.segment_refresh_margin_points = max(10, int(0.1 * (self.ref_window_points + self.ref_backward_points)))

        rospy.loginfo(f"refer frenet_s setting start.")

        self.refer_slist = []
        total = len(self.refer_xlist)
        for i, (rx, ry) in enumerate(zip(self.refer_xlist, self.refer_ylist)):
            s, _ = world2frenet(rx, ry, self.refer_xlist, self.refer_ylist, 0.0)
            self.refer_slist.append(s)

            progress = (i + 1) / total
            bar_len = 40
            bar = '=' * int(progress * bar_len)
            sys.stdout.write(f"\rProgress: [{bar:<{bar_len}}] {progress*100:6.2f}%")
            sys.stdout.flush()

        sys.stdout.write("\n✅ Frenet conversion complete!\n")

        self.refer_slist = np.array(self.refer_slist, dtype=float)

        rospy.loginfo(f"refer frenet_s setting done.")
        self.update_reference_segment(0)
        self.closest_wp_idx = 0
        self.s0, self.d0 = world2frenet(
            self.ego_state.x, self.ego_state.y, 
            self.segment_xlist, self.segment_ylist,
            self.segment_slist[0] if self.segment_slist.size else 0.0
        )
        rospy.loginfo("All setup done!")

    def update_reference_segment(self, center_idx):
        if self.refer_xlist.size == 0:
            return
        
        total_points = self.ref_window_points + self.ref_backward_points
        start_idx = max(center_idx - self.ref_backward_points, 0)
        end_idx = start_idx + total_points

        if end_idx >= len(self.refer_xlist):
            end_idx = len(self.refer_xlist) - 1
            start_idx = max(end_idx - total_points, 0)

        self.segment_start_idx = start_idx
        self.segment_end_idx = end_idx
        self.segment_xlist = self.refer_xlist[start_idx:end_idx + 1]
        self.segment_ylist = self.refer_ylist[start_idx:end_idx + 1]
        self.segment_slist = self.refer_slist[start_idx:end_idx + 1]
    
    def frenet_path_to_msg(self, path):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()

        for x, y, yaw in zip(path.xlist, path.ylist, path.yawlist):
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            q = tf.transformations.quaternion_from_euler(0, 0, yaw)
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]

            path_msg.poses.append(pose)

        return path_msg
    
    def update_frenet_state(self):
        if self.global_path is None or self.segment_xlist.size < 2:
            rospy.logwarn("[LocalPath] Reference segment not ready.")
            return

        local_idx = find_closest_waypoint(
            self.ego_state.x,
            self.ego_state.y,
            self.segment_xlist,
            self.segment_ylist
        )

        global_idx = self.segment_start_idx + local_idx
        self.closest_wp_idx = global_idx

        if (self.segment_end_idx - global_idx) < self.segment_refresh_margin_points:
            self.update_reference_segment(global_idx)
            local_idx = find_closest_waypoint(
                self.ego_state.x,
                self.ego_state.y,
                self.segment_xlist,
                self.segment_ylist
            )
            global_idx = self.segment_start_idx + local_idx
            self.closest_wp_idx = global_idx

        initial_s = self.segment_slist[0] if self.segment_slist.size else 0.0
        self.s0, self.d0 = world2frenet(
            self.ego_state.x,
            self.ego_state.y,
            self.segment_xlist,
            self.segment_ylist,
            initial_s
        )

        idx = global_idx

        if idx < len(self.refer_xlist) - 1:
            dx = self.refer_xlist[idx + 1] - self.refer_xlist[idx]
            dy = self.refer_ylist[idx + 1] - self.refer_ylist[idx]
        else:
            dx = self.refer_xlist[idx] - self.refer_xlist[idx - 1]
            dy = self.refer_ylist[idx] - self.refer_ylist[idx - 1]

        theta_r = math.atan2(dy, dx)

        if 1 <= idx < len(self.refer_xlist) - 1:
            x1, y1 = self.refer_xlist[idx - 1], self.refer_ylist[idx - 1]
            x2, y2 = self.refer_xlist[idx], self.refer_ylist[idx]
            x3, y3 = self.refer_xlist[idx + 1], self.refer_ylist[idx + 1]

            a = math.hypot(x2 - x1, y2 - y1)
            b = math.hypot(x3 - x2, y3 - y2)
            c = math.hypot(x3 - x1, y3 - y1)
            s = (a + b + c) / 2.0
            area = math.sqrt(max(s * (s - a) * (s - b) * (s - c), 0))
            kappa_r = 4 * area / (a * b * c + 1e-9)
        else:
            kappa_r = 0.0

        theta_x = self.ego_state.heading
        vx = self.ego_state.vx
        vy = self.ego_state.vy
        ax = self.ego_state.ax
        ay = self.ego_state.ay

        delta_theta = theta_x - theta_r

        self.s1 = vx * math.cos(delta_theta) + vy * math.sin(delta_theta)
        self.d1 = -vx * math.sin(delta_theta) + vy * math.cos(delta_theta)
        self.s2 = ax * math.cos(delta_theta) + ay * math.sin(delta_theta) + self.s1 * self.d1 * kappa_r
        self.d2 = -ax * math.sin(delta_theta) + ay * math.cos(delta_theta) - self.s1**2 * kappa_r

        rospy.loginfo(f"[LocalPath] Frenet updated: s0={self.s0:.2f}, s1={self.s1:.2f}, d0={self.d0:.2f}")

    def generate_local_path(self):
        if self.ego_state is None or self.global_path is None:
            rospy.logerr("Check ego_state and global_path.")
            return
        if self.segment_xlist.size < 2:
            rospy.logwarn("[LocalPath] Reference segment not ready for world conversion.")
            return
        
        rospy.loginfo(f"ego status: {self.ego_state.x}, {self.ego_state.y} | frenet: {self.s0}, {self.d0}")
        
        # 궤적 생성
        fplist = generate_velocity_keeping_trajectories_in_frenet(
            (self.d0, self.d1, self.d2, 0, 0), 
            (self.s0, self.s1, self.s2, 0), 
            DESIRED_LAT_POS, 
            FINAL_DESIRED_SPEED
        )
        
        # World 좌표로 변환
        fplist = frenet_paths_to_world(
            fplist, 
            self.segment_xlist, 
            self.segment_ylist, 
            self.segment_slist
        )

        # Frenet path 시각화
        frenet_paths = Path()
        frenet_paths.header.frame_id = "map"
        frenet_paths.header.stamp = rospy.Time.now()
        for path in fplist:
            frenet_path = self.frenet_path_to_msg(path)
            frenet_paths.poses.extend(frenet_path.poses)
        self.frenet_path_msg = frenet_paths

        # ★ 장애물을 고려한 유효성 검사
        fplist = check_valid_path(fplist, self.obstacles)

        if not fplist:
            rospy.logwarn("No valid path.")
            self.no_valid_path_cnt += 1
            self.plan_velocity_info_msg.current_speed = self.ego_state.v 
            self.plan_velocity_info_msg.target_speed = self.prev_opt_target_speed if self.no_valid_path_cnt <= 3 else 0
            self.plan_velocity_info_msg.timestamp = rospy.Time.now().to_sec()
            return
        
        # Valid path 시각화
        valid_paths = Path()
        valid_paths.header.frame_id = "map"
        valid_paths.header.stamp = rospy.Time.now()
        for path in fplist:
            valid_path = self.frenet_path_to_msg(path)
            valid_paths.poses.extend(valid_path.poses)
        self.valid_local_path_msg = valid_paths
        
        # 최적 경로 선택
        opt_path = generate_opt_path(fplist)
        self.no_valid_path_cnt = 0
        
        # 목표 속도 계산
        target_v_idx = np.argmin(np.abs(opt_path.s0 - (self.s0 + 4)))
        
        self.plan_velocity_info_msg.current_speed = self.ego_state.v
        
        curvature = abs(opt_path.kappa[target_v_idx]) if len(opt_path.kappa) > target_v_idx else 0.0
        if curvature > 1e-6:
            curvature_speed_limit = math.sqrt(max(LAT_ACCEL_MAX / curvature, 0.0))
        else:
            curvature_speed_limit = FINAL_DESIRED_SPEED
        
        target_speed = min(opt_path.s1[target_v_idx], curvature_speed_limit, FINAL_DESIRED_SPEED)
        self.plan_velocity_info_msg.target_speed = max(target_speed, 0.0)
        self.plan_velocity_info_msg.timestamp = rospy.Time.now().to_sec()
        
        self.opt_local_path_msg = self.frenet_path_to_msg(opt_path)
        self.prev_opt_target_speed = self.plan_velocity_info_msg.target_speed
        
    def publish_path(self):
        rate = rospy.Rate(5)  # 10Hz
        while not rospy.is_shutdown():
            self.opt_local_path_msg.header.stamp = rospy.Time.now()
            self.update_frenet_state()
            self.generate_local_path()
            self.frenet_path_pub.publish(self.frenet_path_msg)
            self.valid_local_path_pub.publish(self.valid_local_path_msg)
            self.opt_local_path_pub.publish(self.opt_local_path_msg)
            self.plan_velocity_info_pub.publish(self.plan_velocity_info_msg)
            rate.sleep()

if __name__ == "__main__":
    try:
        node = LocalPath()
        node.publish_path()
    except rospy.ROSInterruptException:
        pass