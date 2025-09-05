# baqu4_morai_ws
2025 대학생 창작 모빌리티 경진대회 시뮬레이션 부문


1. MORAI 센서 설정  
   - LiDAR: UDP로 Connect  
   - IMU: ROS로 Connect

2. LiDAR 실행
```bash
roslaunch baqu4_tf lidar.launch

2. 토픽 확인
```bash
rostopic hz /front_lidar/velodyne_points
rostopic echo -n1 /imu

