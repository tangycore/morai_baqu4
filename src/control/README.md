# Control

## Overview
- 종방향 제어 : PID control
- 횡방향 제어 : Pure Pursuit
- 20ms 마다 발행

## Data Flow
- 입력: `/opt_path`, `/plan_velocity_info`, `/odom`
- 처리:
  - 로컬 경로 기반 Pure Pursuit 조향 계산
  - PID 기반 가감속 명령 생성
  - 안전 범위 클램프 및 제어 메시지 구성
- 출력: `/ctrl_cmd` (Accel, Brake, Steering)

## Structure
- `control/scripts/ego_control_pub.py` – 제어 노드 엔트리, ROS 통신 및 주기 loop
- `control/src/control/pure_pursuit.py` – 횡방향 제어 유틸
- `control/src/control/pid.py` – 종방향 PID 
- `control/launch/control.launch` – odom topic 발행

## Configuration
- 각 제어 파라미터:
  - Pure Pursuit 생성자에 넘겨줌
    - PurePursuit(`wheel_base`, `max_steer`, `K_LD`, `min_ld`)
    - 최소 lfd 설정후, 속도(kph)에 비례하여 사용 (`K_LD` * `curr_kph`)
  - PID: 생성자에 넘겨줌 (m/s를 기준으로 하고 있음)
    - PIDController(`p_gain`, `i_gain`, `d_gain`)
    - example: `self.pid =  PIDController(1.08, 0.252, 1.08)`
    - class 내부에 최대가속 상수가 존재 (출력이 0~1 사이로 나와야 해서 설정함.)

## Running
- `/odom`, `/opt_path` 토픽이 켜져 있어야함.
- `python3 ego_control_pub.py` 실행

## Todo
- 현재 코너 주행시 속도가 너무 빠름
  1. steer angle에 따라 accel 혹은 break 조절
  2. 아니면 planning 할때 곡률에서 대한 속도 제한 추가 (현재는 그냥 일정한 속도 제한을 하고 있음)
