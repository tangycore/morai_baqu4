#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, os
import rospkg
from math import cos,sin,pi,sqrt,pow,atan2
from geometry_msgs.msg import Point,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry,Path
from morai_msgs.msg import CtrlCmd
import numpy as np
import tf
from tf.transformations import euler_from_quaternion,quaternion_from_euler


from control.pid import PIDController
from control.pure_pursuit import PurePursuit

from planning_pkg.msg import PlanVelocityInfo

'''
TODO
1. planning에서 계획된 속도같은거 msg 만들어서 보내주기.
2. pid + pure_pursuit 연동
    - pid를 일단 해보고 -> pure_pursuit에 같이 엮어서 -> 차량 제어
'''

class EgoControlPub :
    def __init__(self):
        rospy.init_node('ego_control_pub', anonymous=True)
        rospy.Subscriber("/opt_path", Path, self.path_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/plan_velocity_info", PlanVelocityInfo, self.plan_velocity_callback)

        self.ctrl_cmd_pub = rospy.Publisher('/ctrl_cmd',CtrlCmd, queue_size=1)
        self.ctrl_cmd_msg = CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType = 1

        self.current_postion = Point()
        self.planned_velocity = PlanVelocityInfo()

        self.path = None
        self.last_opt_path_time = rospy.Time.now()
        self.opt_path_timeout = rospy.Duration(0.1) 
        self.current_speed = 0
        self.target_speed = 0
        self.is_path = False
        self.is_odom = False

        # PID 게인을 m/s 기준으로 재튜닝
        self.pid = PIDController(1.08, 0.252, 1.08)
        self.pure_pursuit = PurePursuit(3.0, np.deg2rad(40), 0.5, 5)

    def ctrl_pub(self):
        rate = rospy.Rate(50) # 50Hz

        while not rospy.is_shutdown():
            # timeout 검사
            if self.is_odom and self.is_path and self.path is not None:
                # if rospy.Time.now() - self.last_opt_path_time > self.opt_path_timeout:
                #     rospy.logwarn_throttle(2.0, "[WARN] Using previous path (no new /opt_path)")
                # else:
                rospy.loginfo_throttle(2.0, "[INFO] Using latest path")

                curr_ego_state = (self.current_postion.x, self.current_postion.y, self.vehicle_yaw, self.current_speed)
                steer_angle = self.pure_pursuit.steer_control(curr_ego_state, self.path)
                # 계획 속도를 10m/s(≈36km/h)로 제한해 PID 과도 응답 방지
                self.target_speed = min(10, self.target_speed)
                print(f"steer: {steer_angle}")
                accel, brake, aout = self.pid.accel_control(self.current_speed, self.target_speed, 0.02)

                rospy.loginfo(f"{self.current_speed * 3.6:.2f}, {self.target_speed * 3.6:.2f} | "
                            f"steer : {np.rad2deg(steer_angle):.2f}, accel: {accel:.3f}, brake: {brake:.3f}, aout: {aout:.3f}")

                self.ctrl_cmd_msg.accel = accel
                self.ctrl_cmd_msg.brake = brake
                self.ctrl_cmd_msg.steering = steer_angle
                self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
            else:
                if not self.is_odom:
                    rospy.logwarn_throttle(2.0, "[2] can't subscribe '/odom' topic...")

            rate.sleep()


    def path_callback(self, msg):
        self.is_path=True
        self.path = msg
        self.pid.intergral = 0.0
        self.pid.prev_error = 0.0
        self.target_speed_idx = 0
        self.last_opt_path_time = rospy.Time.now()

    def odom_callback(self, msg):
        self.is_odom=True
        odom_quaternion=(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
        _,_,self.vehicle_yaw=euler_from_quaternion(odom_quaternion)
        self.current_postion.x=msg.pose.pose.position.x
        self.current_postion.y=msg.pose.pose.position.y

    def plan_velocity_callback(self, msg):
        self.current_speed = msg.current_speed
        # planning 노드에서 전달한 목표 속도는 m/s 단위로 그대로 사용
        self.target_speed = msg.target_speed

if __name__ == '__main__':
    try:
        ego_control = EgoControlPub()
        ego_control.ctrl_pub()
    except rospy.ROSInterruptException:
        ego_control.ctrl_cmd_msg.accel = 0
        ego_control.ctrl_cmd_msg.brake = 0
        ego_control.ctrl_cmd_msg.steering = 0
        ego_control.ctrl_cmd_pub.publish(ego_control.ctrl_cmd_msg)
