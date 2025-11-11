# Perception – LiDAR Clustering with Multi-Object Tracking

## Overview
- DBSCAN 기반 객체 클러스터 추출 + Multi-Object Tracking
- 결과는 바운딩 박스(stable ID 포함), 마커, 클러스터 포인트로 발행

### Multi-Object Tracking
- **Stable Object ID**: 같은 객체는 프레임 간 동일한 ID 유지
- **IOU Matching**: IOU 기반 detection-track 매칭
- **Lost Track Management**: N 프레임 미검출 시 자동 삭제
- **Velocity Estimation**: 위치 변화로 속도 계산

### Tracking 
 **Tracker** (기본)
   - IOU 기반 매칭
   - 간단한 위치 차분으로 속도 계산

## Data Flow

### 입력
- `/front_lidar/velodyne_points` 또는 설정한 `lidar_topic` (sensor_msgs/PointCloud2)
- TF 변환 정보(`/tf`)로 `output_frame` 기준 좌표 변환

### 처리 단계
1. **Ego Vehicle Filtering**: Bounding box 기반으로 자차 영역 제거
2. **ROI Filtering**: 거리 및 관심 영역 ㅋ필터링 (x, y, z, max_distance)
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

## Structure

### 디렉터리
```
src/perception/
├── src/
│   ├── clustering_with_tracking.cxx   # Clustering + Tracking 메인 노드
│   └── tracker.cxx                    # Tracker 클래스 구현부 (IOU 기반)
├── include/
│   ├── lidar_clustering/
│   │   ├── lidar_cluster_node.h       # LidarClusterNode 선언부
│   │   └── tracker.h                  # Tracker 클래스 선언부
│   └── dbscan_kdtree/
│       ├── DBSCAN_simple.h            # 기본 DBSCAN 구현
│       ├── DBSCAN_kdtree.h            # KD-tree 가속 DBSCAN
│       └── DBSCAN_precomp.h           # 사전계산 최적화 버전
├── launch/
│   └── lidar_clustering_tracking.launch # Clustering + Tracking
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

## Topics

### Subscribed
- `lidar_topic` (기본 `/front_lidar/velodyne_points`)
  - Type: sensor_msgs/PointCloud2

### Published
- `cluster_result_topic` (기본 `/cluster_result`)
  - Type: vision_msgs/Detection3DArray
  - `results[0].id`에 stable object ID 저장
- `/cluster_markers`
  - Type: visualization_msgs/MarkerArray
  - Green boxes with "ID X" labels
- `/cluster_cloud`
  - Type: sensor_msgs/PointCloud2
  - 모든 클러스터의 포인트 합본

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

## Usage

### 기본 실행 Tracker)
```bash
roslaunch lidar_clustering lidar_clustering_tracking.launch
```





.
