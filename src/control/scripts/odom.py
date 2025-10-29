#!/usr/bin/env python3
import rospy
from morai_msgs.msg import GPSMessage, EgoVehicleStatus, CtrlCmd
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from pyproj import Proj
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np

class GPSIMUParser:
    def __init__(self):
        rospy.init_node('odom', anonymous=True)
        
        # Subscribers
        self.gps_sub = rospy.Subscriber("/gps", GPSMessage, self.navsat_callback)
        self.imu_sub = rospy.Subscriber("/imu", Imu, self.imu_callback)
        self.status_sub = rospy.Subscriber("/morai/competition_status", 
                                          EgoVehicleStatus, self.status_callback)
        self.ctrl_sub = rospy.Subscriber("/ctrl_cmd", CtrlCmd, self.ctrl_callback)
        
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)
        
        # States
        self.x, self.y, self.psi = 0.0, 0.0, 0.0
        self.vx, self.delta = 0.0, 0.0
        self.L = 3.0  # wheelbase
        
        # GPS 관련
        self.lat, self.lon = 0.0, 0.0
        self.e_o, self.n_o = 0.0, 0.0
        self.gps_is_valid = False  #  GPS가 유효한지 (0이 아닌지)
        self.gps_ever_received = False  # GPS를 한 번이라도 받았는지
        
        # Flags
        self.is_imu = False
        self.is_vx = False
        
        self.proj_UTM = Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)
        
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = 'odom'
        self.odom_msg.child_frame_id = 'base_link'
        
        # 고정 주파수 및 dt
        self.ODOM_HZ = 50.0
        self.dt = 1.0 / self.ODOM_HZ  # 0.02초
        
        rate = rospy.Rate(self.ODOM_HZ)  # 50Hz
        
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            
            if self.is_imu:
                if self.gps_is_valid:
                    #  GPS 정상 (값이 0이 아님): GPS 기반 위치
                    self.update_from_gps()
                    rospy.loginfo_throttle(5.0, "[Odom] GPS OK")
                    
                elif self.gps_ever_received and self.is_vx:
                    #  GPS Blackout (값이 0): Dead Reckoning
                    self.dead_reckoning(self.dt)
                    rospy.logwarn_throttle(1.0, 
                        f"[Odom] GPS BLACKOUT (lat={self.lat}, lon={self.lon})! "
                        f"DR (v={self.vx:.2f}, δ={np.rad2deg(self.delta):.1f}°)")
                
                else:
                    rospy.logwarn_throttle(2.0, "[Odom] Waiting for GPS initial fix...")
                
                # Odometry 발행
                self.publish_odom(current_time)
            
            rate.sleep()
    
    def navsat_callback(self, gps_msg):
        """
        GPS 메시지 수신
        음영 구역에서는 lat=0, lon=0으로 들어옴!
        """
        self.lat = gps_msg.latitude
        self.lon = gps_msg.longitude
        self.e_o = gps_msg.eastOffset
        self.n_o = gps_msg.northOffset
        
        #  GPS 유효성 체크: lat, lon이 둘 다 0이면 무효
        if self.lat != 0.0 or self.lon != 0.0:
            self.gps_is_valid = True
            self.gps_ever_received = True
        else:
            self.gps_is_valid = False
    
    def update_from_gps(self):
        """GPS로부터 위치 업데이트"""
        # 이미 navsat_callback에서 유효성 체크했으므로 바로 사용
        xy_zone = self.proj_UTM(self.lon, self.lat)
        self.x = xy_zone[0] - self.e_o
        self.y = xy_zone[1] - self.n_o
    
    def imu_callback(self, data):
        """IMU (Yaw)"""
        if data.orientation.w != 0:
            _, _, self.psi = euler_from_quaternion([
                data.orientation.x, 
                data.orientation.y,
                data.orientation.z, 
                data.orientation.w
            ])
            self.is_imu = True
            
            # Orientation 저장
            self.odom_msg.pose.pose.orientation.x = data.orientation.x
            self.odom_msg.pose.pose.orientation.y = data.orientation.y
            self.odom_msg.pose.pose.orientation.z = data.orientation.z
            self.odom_msg.pose.pose.orientation.w = data.orientation.w
    
    def status_callback(self, msg):
        """차량 속도"""
        self.vx = np.hypot(msg.velocity.x, msg.velocity.y)
        self.is_vx = True
    
    def ctrl_callback(self, msg):
        """조향각"""
        self.delta = msg.steering
    
    def dead_reckoning(self, dt):
        """
        교수님 방법: Bicycle Model
        x_k+1 = x_k + Vx·cos(ψ + δ)·Δt
        y_k+1 = y_k + Vx·sin(ψ + δ)·Δt
        ψ_k+1 = ψ_k + (Vx/L)·tan(δ)·Δt
        """
        # 위치 업데이트
        heading = self.psi + self.delta
        self.x += self.vx * np.cos(heading) * dt
        self.y += self.vx * np.sin(heading) * dt
        
        # Yaw 업데이트
        if abs(self.delta) > 1e-4:
            self.psi += (self.vx / self.L) * np.tan(self.delta) * dt
            self.psi = np.arctan2(np.sin(self.psi), np.cos(self.psi))  # 정규화
            
            # Orientation 업데이트
            q = quaternion_from_euler(0, 0, self.psi)
            self.odom_msg.pose.pose.orientation.x = q[0]
            self.odom_msg.pose.pose.orientation.y = q[1]
            self.odom_msg.pose.pose.orientation.z = q[2]
            self.odom_msg.pose.pose.orientation.w = q[3]
    
    def publish_odom(self, stamp):
        """Odometry 발행"""
        self.odom_msg.header.stamp = stamp
        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        self.odom_msg.pose.pose.position.z = 0.0
        self.odom_pub.publish(self.odom_msg)

if __name__ == '__main__':
    try:
        GPS_IMU_parser = GPSIMUParser()
    except rospy.ROSInterruptException:
        pass