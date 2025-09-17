# baqu4_morai_ws
2025 대학생 창작 모빌리티 경진대회 시뮬레이션 부문


1. MORAI 센서 설정  
   - LiDAR: UDP
   - IMU, CAM, GPS: ROS
```bash
roslaunch rosbridge_server rosbridge_websocket.launch
```

2. TF 발행
```bash
roslaunch baqu4_tf baqu4_tf.launch
```

