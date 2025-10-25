# Perception – LiDAR Clustering

## Overview
- Velodyne 포인트클라우드에서 DBSCAN 기반 객체 클러스터 추출
- ROI 필터링 후 KD-tree 가속 DBSCAN으로 클러스터링
- 결과는 바운딩 박스, 마커, 합쳐진 클러스터 포인트로 발행
- 주행용 단순 클러스터링(클래스 분류 없음), 좌표계는 `output_frame` 기준

## Data Flow
- 입력
  - `/front_lidar/velodyne_points` 또는 설정한 `lidar_topic` (sensor_msgs/PointCloud2)
  - TF 변환 정보(`/tf`)로 `output_frame` 기준 좌표 변환
- 처리 단계
  - 거리 및 ROI 필터링 (x, y, z, max_distance)
  - KD-tree 기반 DBSCAN 클러스터링
  - 클러스터별 3D 박스 추정 및 시각화 메시지 생성
- 출력
  - `/cluster_result` (vision_msgs/Detection3DArray) — 파라미터 `cluster_result_topic`로 변경 가능
  - `/cluster_markers` (visualization_msgs/MarkerArray)
  - `/cluster_cloud` (sensor_msgs/PointCloud2, 모든 클러스터 포인트 합본)

## Structure
- 디렉터리
  - `src/perception/src/clustering.cxx` — ROS1 노드 본체
  - `src/perception/include/`
    - `clustering_cxx/` — 래퍼 헬퍼, 파라미터 정의
      - `clustering.hxx`, `parameters.hxx`
    - `dbscan_kdtree/` — DBSCAN 구현
      - `DBSCAN_kdtree.h`, `DBSCAN_simple.h`, `DBSCAN_precomp.h`
    - `visualization.h` — 시각화 유틸
  - `src/perception/launch/lidar_clustering.launch` — 런치 파일
  - `src/perception/CMakeLists.txt`, `package.xml` — ROS 패키지 메타
- 주요 구성
  - 포인트 필터링 → TF 변환 → DBSCAN → 결과 메시지 생성/발행
  - DBSCAN은 KD-tree 탐색으로 이웃 검색 가속

## Topics
- Subscribed
  - `lidar_topic` (기본 `/front_lidar/velodyne_points`)
- Published
  - `cluster_result_topic` (기본 `/cluster_result`, 타입 Detection3DArray)
  - `/cluster_markers` (MarkerArray)
  - `/cluster_cloud` (PointCloud2)

## Parameters
- 입력/출력
  - `lidar_topic` (string) 포인트클라우드 입력 토픽
  - `output_frame` (string) 결과 프레임, 기본 `base_link`
  - `cluster_result_topic` (string) 결과 토픽 이름
- 클러스터링
  - `cluster_tolerance` (double) 이웃 반경 eps [m]
  - `min_cluster_size` (int) 최소 포인트 수
  - `max_cluster_size` (int) 최대 포인트 수
- ROI 필터링 (`output_frame` 기준)
  - `max_distance` (double) 원거리 컷오프 [m]
  - `min_x` (double) x 하한 [m]
  - `min_y`, `max_y` (double) y 범위 [m]
  - `min_z`, `max_z` (double) z 범위 [m]

## Configuration
- 런치 파일에서 파라미터를 조정합니다.
- 권장 시작값
  - `cluster_tolerance`: 0.6
  - `min_cluster_size`: 20
  - `max_cluster_size`: 8000
- ROI 예시
  - `max_distance`: 60.0
  - `min_x`: 3.0 (에고 주변 반사 제거)
  - `min_y`: -20.0, `max_y`: 20.0
  - `min_z`: 0.0, `max_z`: 3.5