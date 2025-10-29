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
        self.beta = 0.0  # ✅ sideslip angle (EgoVehicleStatus에서 받음)
        
        # Vehicle parameters
        self.L = 2.5  # wheelbase
        self.lf = 1.375  # front axle to CG (55%)
        self.lr = 1.125  # rear axle to CG (45%)
        
        # GPS 관련
        self.lat, self.lon = 0.0, 0.0
        self.e_o, self.n_o = 0.0, 0.0
        self.gps_is_valid = False
        self.gps_ever_received = False
        
        # Flags
        self.is_imu = False
        self.is_vx = False
        self.is_beta = False  # ✅ sideslip 수신 여부
        
        self.proj_UTM = Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)
        
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = 'odom'
        self.odom_msg.child_frame_id = 'base_link'
        
        # 고정 주파수 및 dt
        self.ODOM_HZ = 50.0
        self.dt = 1.0 / self.ODOM_HZ
        self.last_update_time = None
        
        rate = rospy.Rate(self.ODOM_HZ)
        
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            if self.last_update_time is None:
                dt = self.dt
            else:
                dt = (current_time - self.last_update_time).to_sec()
                dt = np.clip(dt, 0.0, 0.1)
            
            if self.is_imu:
                if self.gps_is_valid:
                    # GPS 정상: GPS + IMU 그대로 사용
                    self.update_from_gps()
                    rospy.loginfo_throttle(2.0, "[Odom] GPS OK")
                    
                elif self.gps_ever_received and self.is_vx and self.is_beta:
                    # GPS Blackout: Dead Reckoning with Sideslip from EgoVehicleStatus
                    self.dead_reckoning(dt)
                    rospy.logwarn_throttle(1.0, 
                        f"[Odom] GPS BLACKOUT! DR (v={self.vx:.2f}m/s, β={np.rad2deg(self.beta):.1f}°)")
                
                else:
                    rospy.logwarn_throttle(2.0, "[Odom] Waiting for GPS initial fix...")
                
                # Odometry 발행
                self.publish_odom(current_time)
                self.last_update_time = current_time
            
            rate.sleep()
    
    def navsat_callback(self, gps_msg):
        """GPS 메시지 수신 (음영구간: lat=0, lon=0)"""
        lat = gps_msg.latitude
        lon = gps_msg.longitude
        valid_fix = not (np.isclose(lat, 0.0) and np.isclose(lon, 0.0))

        if valid_fix:
            self.lat = lat
            self.lon = lon
            self.e_o = gps_msg.eastOffset
            self.n_o = gps_msg.northOffset
            self.gps_is_valid = True
            self.gps_ever_received = True
        else:
            self.gps_is_valid = False
    
    def update_from_gps(self):
        """GPS 정상 구간: GPS로 위치 업데이트"""
        xy_zone = self.proj_UTM(self.lon, self.lat)
        self.x = xy_zone[0] - self.e_o
        self.y = xy_zone[1] - self.n_o
    
    def imu_callback(self, data):
        """IMU에서 yaw 그대로 받음 (필터 없음)"""
        q_in = [
            data.orientation.x,
            data.orientation.y,
            data.orientation.z,
            data.orientation.w
        ]
        
        if np.linalg.norm(q_in) > 1e-3:
            _, _, self.psi = euler_from_quaternion(q_in)
            
            # Orientation 저장
            self.odom_msg.pose.pose.orientation.x = q_in[0]
            self.odom_msg.pose.pose.orientation.y = q_in[1]
            self.odom_msg.pose.pose.orientation.z = q_in[2]
            self.odom_msg.pose.pose.orientation.w = q_in[3]
            
            self.is_imu = True
    
    def status_callback(self, msg):
        """
        EgoVehicleStatus에서 속도 및 sideslip angle 수신
        """
        # 1. 속도 (kph → m/s)
        speed_kph = np.sqrt(msg.velocity.x**2 + msg.velocity.y**2 + msg.velocity.z**2)
        if np.isfinite(speed_kph):
            self.vx = speed_kph * (1000.0 / 3600.0)
            self.is_vx = True
        
        # 2. Sideslip angle (타이어 4개 평균 또는 후륜 평균)
        # Option 1: 전체 평균
        # beta_avg = (msg.side_slip_angle_fl + msg.side_slip_angle_fr + 
        #             msg.side_slip_angle_rl + msg.side_slip_angle_rr) / 4.0
        
        # Option 2: 후륜 평균 (CG가 뒤쪽에 가까우므로)
        beta_avg = (msg.side_slip_angle_rl + msg.side_slip_angle_rr) / 2.0
        
        # Option 3: 가중 평균 (더 정확함)
        # beta_front = (msg.side_slip_angle_fl + msg.side_slip_angle_fr) / 2.0
        # beta_rear = (msg.side_slip_angle_rl + msg.side_slip_angle_rr) / 2.0
        # beta_avg = (self.lr * beta_front + self.lf * beta_rear) / (self.lf + self.lr)
        
        if np.isfinite(beta_avg):
            self.beta = beta_avg
            self.is_beta = True
    
    def ctrl_callback(self, msg):
        """조향각 (라디안)"""
        self.delta = msg.steering
    
    def dead_reckoning(self, dt):
        """
        GPS Blackout 전용: EgoVehicleStatus의 sideslip angle 직접 사용
        
        교수님 수식:
        - Ẋ = v·cos(ψ + β)
        - Ẏ = v·sin(ψ + β)
        - ψ̇ = (v·cos(β))/(ℓf + ℓr) · tan(δ)
        - β는 EgoVehicleStatus에서 받음 ✅
        """
        
        # 1. Sideslip angle은 이미 status_callback에서 받음
        beta = self.beta
        
        # 2. 실제 이동 방향 = ψ + β
        heading = self.psi + beta
        
        # 3. 위치 업데이트
        dx = self.vx * np.cos(heading) * dt
        dy = self.vx * np.sin(heading) * dt
        
        self.x += dx
        self.y += dy
        
        # 4. Yaw rate 계산 및 업데이트
        if abs(self.delta) > 1e-4 and abs(self.vx) > 0.1:
            yaw_rate = (self.vx * np.cos(beta) / (self.lf + self.lr)) * np.tan(self.delta)
            dpsi = yaw_rate * dt
            
            self.psi += dpsi
            self.psi = np.arctan2(np.sin(self.psi), np.cos(self.psi))
            
            # Orientation 업데이트
            q = quaternion_from_euler(0, 0, self.psi)
            self.odom_msg.pose.pose.orientation.x = q[0]
            self.odom_msg.pose.pose.orientation.y = q[1]
            self.odom_msg.pose.pose.orientation.z = q[2]
            self.odom_msg.pose.pose.orientation.w = q[3]
        
        # 디버그 로그
        rospy.logwarn_throttle(0.5, 
            f"[DR] v={self.vx:.2f}m/s, δ={np.rad2deg(self.delta):.2f}°, "
            f"β={np.rad2deg(beta):.2f}° (from Status), ψ={np.rad2deg(self.psi):.2f}°, "
            f"heading(ψ+β)={np.rad2deg(heading):.2f}°")
    
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