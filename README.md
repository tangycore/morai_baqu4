# baqu4_morai_ws
2025 대학생 창작 모빌리티 경진대회 시뮬레이션 부문


0. 포트번호 설정 (Destination Only)
   - CollisionData - 9122 
   - Competition Vehicle Status - 9121

1. MORAI 네트워크 설정  
   - CollisionData, Competition Vehicle Status, LiDAR: UDP
   - Ego Ctrl Cmd, IMU, CAM, GPS: ROS
```bash
roslaunch rosbridge_server rosbridge_websocket.launch
```

2. 실행권한 주기 (scripts)
```bash
chmod +x ~/baqu4_morai_ws/src/해당 패키지/scripts/*.py
```


3. TF/노드/네트워크 키기
```bash
roslaunch baqu4_tf baqu4_tf.launch
roslaunch baqu4_udp baqu4_udp.launch
roslaunch control control.launch
roslaunch planning_pkg planning.launch
```

push after git sync

---

### Planning
 - [바로가기](./src/planning)
### Control
- [바로가기](./src/control)
