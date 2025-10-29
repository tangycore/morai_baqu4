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
        self.last_imu_time = None
        
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = 'odom'
        self.odom_msg.child_frame_id = 'base_link'
        
        # 고정 주파수 및 dt
        self.ODOM_HZ = 30.0
        self.dt = 1.0 / self.ODOM_HZ  # 0.02초
        self.last_update_time = None
        
        rate = rospy.Rate(self.ODOM_HZ)  # 50Hz
        
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            if self.last_update_time is None:
                dt = self.dt
            else:
                dt = (current_time - self.last_update_time).to_sec()
                dt = np.clip(dt, 0.0, 0.1)  # overly large gap 보호
            
            if self.is_imu:
                if self.gps_is_valid:
                    #  GPS 정상 (값이 0이 아님): GPS 기반 위치
                    self.update_from_gps()
                    rospy.loginfo_throttle(1.0, "[Odom] GPS OK")
                    
                elif self.gps_ever_received and self.is_vx:
                    #  GPS Blackout (값이 0): Dead Reckoning
                    self.dead_reckoning(dt)
                    rospy.logwarn_throttle(1.0, 
                        f"[Odom] GPS BLACKOUT (lat={self.lat}, lon={self.lon})! "
                        f"DR (v={self.vx:.2f}, δ={np.rad2deg(self.delta):.1f}°)")
                
                else:
                    rospy.logwarn_throttle(2.0, "[Odom] Waiting for GPS initial fix...")
                
                # Odometry 발행
                self.publish_odom(current_time)
                self.last_update_time = current_time
            
            rate.sleep()
    
    def navsat_callback(self, gps_msg):
        """
        GPS 메시지 수신
        음영 구역에서는 lat=0, lon=0으로 들어옴!
        """
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
        """GPS로부터 위치 업데이트"""
        # 이미 navsat_callback에서 유효성 체크했으므로 바로 사용
        xy_zone = self.proj_UTM(self.lon, self.lat)
        self.x = xy_zone[0] - self.e_o
        self.y = xy_zone[1] - self.n_o
    
    def imu_callback(self, data):
        """IMU (Yaw)"""
        stamp = data.header.stamp if data.header.stamp != rospy.Time() else rospy.Time.now()
        if self.last_imu_time is None:
            dt = 0.0
        else:
            dt = (stamp - self.last_imu_time).to_sec()
            dt = np.clip(dt, 0.0, 0.05)
        self.last_imu_time = stamp

        yaw_rate = data.angular_velocity.z
        if np.isfinite(yaw_rate):
            self.psi = self.normalize_angle(self.psi + yaw_rate * dt)

        q_in = [
            data.orientation.x,
            data.orientation.y,
            data.orientation.z,
            data.orientation.w
        ]
        quat_norm = np.linalg.norm(q_in)
        if quat_norm > 1e-3:
            _, _, yaw_meas = euler_from_quaternion(q_in)
            yaw_error = self.normalize_angle(yaw_meas - self.psi)
            self.psi = self.normalize_angle(self.psi + 0.05 * yaw_error)

        q_out = quaternion_from_euler(0, 0, self.psi)
        self.odom_msg.pose.pose.orientation.x = q_out[0]
        self.odom_msg.pose.pose.orientation.y = q_out[1]
        self.odom_msg.pose.pose.orientation.z = q_out[2]
        self.odom_msg.pose.pose.orientation.w = q_out[3]

        self.is_imu = True
    
    def status_callback(self, msg):
        """차량 속도"""
        speed_kph = np.sqrt(msg.velocity.x**2 + msg.velocity.y**2 + msg.velocity.z**2)
        if np.isfinite(speed_kph):
            self.vx = speed_kph * (1000.0 / 3600.0)  # kph -> m/s
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
        # IMU yaw(ψ)를 그대로 사용해 위치 적분 (조향각은 차량 방향이 아님)
        heading = self.psi
        self.x += self.vx * np.cos(heading) * dt
        self.y += self.vx * np.sin(heading) * dt
    
    def publish_odom(self, stamp):
        """Odometry 발행"""
        self.odom_msg.header.stamp = stamp
        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        self.odom_msg.pose.pose.position.z = 0.0
        self.odom_pub.publish(self.odom_msg)

    @staticmethod
    def normalize_angle(angle):
        return np.arctan2(np.sin(angle), np.cos(angle))

if __name__ == '__main__':
    try:
        GPS_IMU_parser = GPSIMUParser()
    except rospy.ROSInterruptException:
        pass
