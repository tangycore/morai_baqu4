# Planning
> [optimal trajectories in a frenet frame (polynomial) 에 대한 자세한 설명 링크](https://blog.phan.kr/posts/Optimal-Trajectories-in-a-Frenet-Frame/)
## Overview
- local path 생성 후 발행
- optimal trajectories in a frenet frame (polynomial)
- 200ms 마다 발행
- m/s를 기준으로 하고 있음.

## Data Flow
- 입력: `/global_path`, `/odom`, `/Ego_topic`, …
- 처리 단계:
  - 참조선 전처리
  - Frenet 궤적 생성
  - 유효 경로 선택
- 출력: `/opt_path`, `/valid_local_path`, `/plan_velocity_info`

## Structure
- `scripts`
  - `global_path.py` – global path 발행 노드
  - `local_path.py` – local path 발행 노드

- `src/planning_pkg`
  - `planner.py` – 궤적 생성
  - `config.py` - 다양한 파라미터 (planning time, weights, max [speed, accel, kappa], etc..)
  - `frenet.py` – Frenet ↔ World 변환
  - `polynomial.py` – 다항식생성
  - `frenet_path.py` – 생성된 경로를 담을 class

- `msg/PlanVelocityInfo.msg` – 제어기 연동 메시지 (current speed, target speed)

## Configuration
- 파라미터 설명 파일 주석 참고 (`planning/src/planning_pkg/config.py`)
- 속도/가속도 제한, 비용 계수 튜닝 planning에서 사용하는 대부분의 파라미터 설정 가능

## Running
1. `roslaunch control control.launch` # odom topic 발행 먼저해야됨.
2. `roslaunch planning_pkg planning.launch`

## Todo
- 모드 변경 기능 추가 (현재 속도 유지 기능만 되어 있음)
  - 정지 모드, 속도 유지 모드
  - Finate state machine으로 하면 될 거 같은 느낌 (신호등, 종료지점)
- 장애물 정보 받아와야함 
- 음역 구역 진입시 odom 대체 처리(?)

