# baqu4_morai_ws
2025 대학생 창작 모빌리티 경진대회 시뮬레이션 부문


0. 포트번호 설정 
   - CollisionData - 9122 (Destination) 
   - Competition Vehicle Status - 9121 (Destination)
   - Ego Ctrl Cmd - 9123 (Host)

1. MORAI 네트워크 설정  
   - CollisionData, Competition Vehicle Status, LiDAR: UDP
   - IMU, CAM, GPS: ROS
```bash
roslaunch rosbridge_server rosbridge_websocket.launch
```

2. 실행권한 주기 (scripts)
```bash
chmod +x ~/baqu4_morai_ws/src/해당 패키지/scripts/*.py
```

3. 시뮬레이터 실행

4. TF/노드/네트워크 키기
```bash
roslaunch baqu4_tf baqu4_tf.launch
roslaunch baqu4_udp final_mission.launch || roslaunch baqu4_udp ai_mission.launch
roslaunch control control.launch
roslaunch planning_pkg planning.launch
roslaunch lidar_clustering lidar_clustering_tracking.launch
```

5. 차량 제어 시작
```bash
roscd control
cd scripts && python3 ego_control_pub.py
```

push after git sync

---
### Perception
 - [바로가기](./src/perception)
### Planning
 - [바로가기](./src/planning)
### Control
- [바로가기](./src/control)
### Localization
- [바로가기](https://github.com/hku-mars/FAST_LIO/tree/7cc4175de6f8ba2edf34bab02a42195b141027e9)
