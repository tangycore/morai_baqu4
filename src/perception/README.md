# Perception – LiDAR Clustering with Multi-Object Tracking

## Overview
- Velodyne 포인트클라우드에서 DBSCAN 기반 객체 클러스터 추출 + Multi-Object Tracking
- ROI 필터링 후 KD-tree 가속 DBSCAN으로 클러스터링
- IOU 기반 Data Association으로 안정적인 객체 ID 할당
- 옵션(고급): Kalman Filter를 통한 노이즈 제거 및 속도 추정
- 결과는 바운딩 박스(stable ID 포함), 마커, 클러스터 포인트로 발행
- Planning 모듈 연동을 위한 obstacle converter 

## New Features (Tracking 추가) 

### Multi-Object Tracking
- **Stable Object ID**: 같은 객체는 프레임 간 동일한 ID 유지
- **IOU Matching**: IOU 기반 detection-track 매칭
- **Lost Track Management**: N 프레임 미감지 시 자동 삭제
- **Velocity Estimation**: 위치 변화로 속도 계산

### Two Tracking Options
1. **Simple Tracker** (기본)
   - IOU 기반 매칭
   - 간단한 위치 차분으로 속도 계산
   - Kalman 방식보다 빠름

2. **Kalman+IOU Tracker** (고급)
   - IOU 매칭 + Kalman Filter
   - 노이즈 대비 강화
   - State: [x, y, vx, vy]
   - 노이즈 많음 에서 유리

3. **PointPillars + Tracker** 
   - 딥러닝 활용, 추후 고려

### Obstacle Converter (선택)
- Detection3DArray → Planning 포맷 변환
- Frenet 좌표 변환 준비

## Data Flow

### 입력
- `/front_lidar/velodyne_points` 또는 설정한 `lidar_topic` (sensor_msgs/PointCloud2)
- TF 변환 정보(`/tf`)로 `output_frame` 기준 좌표 변환

### 처리 단계
1. **Ego Vehicle Filtering**: Bounding box 기반으로 자차 영역 제거
2. **ROI Filtering**: 거리 및 관심 영역 필터링 (x, y, z, max_distance)
3. **DBSCAN Clustering**: KD-tree 기반 밀도기반 클러스터링
4. **Bounding Box Generation**: 각 클러스터의 3D 박스 추정
5. **Multi-Object Tracking**: IOU 매칭으로 stable ID 할당 및 속도 추정
6. **Visualization**: Marker 및 텍스트 생성 (ID 표시)

### 출력
- `/cluster_result` (vision_msgs/Detection3DArray)
  - `results[0].id`: Stable object ID (tracking 적용 시)
  - `bbox`: 3D bounding box (center, orientation, size)
- `/cluster_markers` (visualization_msgs/MarkerArray)
  - Green boxes with stable ID labels
- `/cluster_cloud` (sensor_msgs/PointCloud2)
  - 모든 클러스터 포인트 합본
- `/obstacles` (PoseArray, obstacle_converter 사용 시)
  - Planning 모듈 연동용 (속도 정보 포함)

## Structure

### 디렉터리
```
src/perception/
├── src/
│   ├── clustering.cxx                    # Clustering Only (트래킹 미포함)
│   └── clustering_with_tracking.cxx      # Clustering + Tracking (권장)
├── include/
│   ├── lidar_clustering/
│   │   ├── lidar_cluster_node.h          # 클래스 선언 및 파라미터 구조체
│   │   ├── simple_tracker.h              # Simple IOU tracker (기본)
│   │   └── kalman_iou_tracker.h          # Kalman + IOU tracker (고급))
│   └── dbscan_kdtree/
│       ├── DBSCAN_kdtree.h               # KD-tree 기반 DBSCAN
│       ├── DBSCAN_simple.h               # 기본 DBSCAN
│       └── DBSCAN_precomp.h              # 최적화 DBSCAN
├── scripts/
│   └── obstacle_converter.py             # Planning 연동용 변환 노드
├── launch/
│   ├── lidar_clustering.launch           # DBSCAN withou Tracking
│   └── lidar_clustering_tracking.launch  # Tracking 버전 런치
├── CMakeLists.txt
├── package.xml
└── README.md
```

### 주요 구성

#### Core Components
- **LidarClusterNode** (`lidar_cluster_node.h`): 메인 클래스 선언
  - `ClusteringParams`: 클러스터링 파라미터 구조체
  - `FilteringParams`: ROI 필터링 파라미터 구조체
  - `EgoVehicleBounds`: 자차 영역 정의 구조체
- **DBSCAN Clustering**: KD-tree 탐색으로 이웃 검색 가속
- **Multi-Object Tracking**: IOU 기반 detection-track 매칭
- **Kalman Filter**: 옵션으로 노이즈 제거 및 속도 smoothing

#### Implementation
- **Ego Vehicle Filtering**: `EgoVehicleBounds::contains()` inline 함수로 자차 포인트 제거
- **Parameter Loading**: `loadParameters()` 함수로 ROS 파라미터 로드
- **Bounding Box**: PCA 기반 회전 박스 추정
- **Visualization**: 3D 박스 + ID 텍스트 마커 생성

## Topics

### Subscribed
- `lidar_topic` (기본 `/front_lidar/velodyne_points`)
  - Type: sensor_msgs/PointCloud2

### Published
- `cluster_result_topic` (기본 `/cluster_result`)
  - Type: vision_msgs/Detection3DArray
  - **중요**: `results[0].id`에 stable object ID 저장
- `/cluster_markers`
  - Type: visualization_msgs/MarkerArray
  - Green boxes with "ID X" labels
- `/cluster_cloud`
  - Type: sensor_msgs/PointCloud2
  - 모든 클러스터의 포인트 합본
- `/obstacles` (obstacle_converter 사용 시)
  - Type: geometry_msgs/PoseArray (임시)
  - Planning 모듈 연동용

## Parameters

### 입력/출력
- `lidar_topic` (string): 포인트클라우드 입력 토픽
  - 기본값: `/front_lidar/velodyne_points`
- `output_frame` (string): 결과 프레임
  - 기본값: `base_link`
  - 모든 결과는 이 프레임 기준으로 변환됨
- `cluster_result_topic` (string): 결과 토픽 이름
  - 기본값: `cluster_result`

### 클러스터링
- `cluster_tolerance` (float): DBSCAN epsilon [m]

- `min_cluster_size` (int): 최소 포인트 수

- `max_cluster_size` (int): 최대 포인트 수

### ROI 필터링 (output_frame 기준)
- `max_distance` (float): 원거리 컷오프 [m]

- `min_x` (float): x축 하한 [m]
  - Ego vehicle filtering으로 자차는 별도 제거됨

- `min_y`, `max_y` (float): y축 범위 [m]

- `min_z`, `max_z` (float): z축 범위 [m]

### Ego Vehicle Bounds (선택적)
자차 영역을 커스터마이즈하려면 launch 파일에 추가:
- `ego_min_x`, `ego_max_x` (float): 자차 x 범위 [m]
  - 기본값: [-1.0, 4.7]
- `ego_min_y`, `ego_max_y` (float): 자차 y 범위 [m]
  - 기본값: [-1.0, 1.0]
- `ego_min_z`, `ego_max_z` (float): 자차 z 범위 [m]
  - 기본값: [-0.5, 2.0]

### Tracking (코드 내부 파라미터)

**Simple Tracker** (`simple_tracker.h`):
- `iou_threshold_`: 0.3 (IOU 매칭 임계값)
- `max_lost_frames_`: 5 (몇 프레임 미감지 시 삭제)
- `history_size_`: 5 (속도 계산용 히스토리)

**Kalman Tracker** (`kalman_iou_tracker.h`):
- Process Noise: 1e-2 (동역학 모델 - 불확실)
- Measurement Noise: 1e-1 (LiDAR 측정 노이즈 - 불확실)
- State: [x, y, vx, vy]
- Measurement: [x, y]

## Usage

### 기본 실행 (Simple Tracker)
```bash
# Tracking 포함 버전 (권장)
roslaunch lidar_clustering lidar_clustering_tracking.launch

# Tracking 미포함 버전
roslaunch lidar_clustering lidar_clustering.launch
```

### Planning 연동 
```bash
# obstacle_converter 실행
rosrun lidar_clustering obstacle_converter.py

# Planning 모듈에서 subscribe
rostopic echo /obstacles
```


## Advanced: Switching to Kalman Filter

### 현재: Simple Tracker (기본)
```cpp
// src/clustering_with_tracking.cxx
#include "lidar_clustering/simple_tracker.h"
SimpleTracker tracker_;
```

### 변경 시: Kalman+IOU Tracker
1. **코드 수정** (`src/clustering_with_tracking.cxx`):
```cpp
// 헤더 변경
#include "lidar_clustering/kalman_iou_tracker.h"

// 멤버 변수 변경 (lidar_cluster_node.h에서)
KalmanIOUTracker tracker_;
```


.
