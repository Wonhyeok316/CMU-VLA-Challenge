# Exploration Manager

ROS 패키지로, traversable_area PointCloud2 메시지를 받아서 Occupancy Grid를 생성하고 Inflation을 적용하는 기능을 제공합니다.

## 기능

- **PointCloud2 처리**: `/traversable_area` 토픽에서 PointCloud2 메시지를 구독
- **Occupancy Grid 생성**: PointCloud2 데이터를 2D 그리드로 변환
- **Inflation 적용**: 장애물 주변에 안전 마진을 추가
- **실시간 발행**: 원본 그리드와 Inflated 그리드를 각각 발행

## 토픽

### Subscribers
- `/traversable_area` (sensor_msgs/PointCloud2): 주행 가능한 영역의 포인트 클라우드

### Publishers
- `/exploration/occupancy_grid` (nav_msgs/OccupancyGrid): 원본 occupancy grid
- `/exploration/inflated_grid` (nav_msgs/OccupancyGrid): inflation이 적용된 grid

## 파라미터

`config/exploration_params.yaml` 파일에서 설정 가능:

- `grid_resolution`: 그리드 셀 크기 (미터, 기본값: 0.1)
- `grid_width`: 그리드 너비 (셀 개수, 기본값: 100)
- `grid_height`: 그리드 높이 (셀 개수, 기본값: 100)
- `grid_origin_x`: 그리드 원점 X 좌표 (미터, 기본값: -5.0)
- `grid_origin_y`: 그리드 원점 Y 좌표 (미터, 기본값: -5.0)
- `inflation_radius`: Inflation 반지름 (미터, 기본값: 0.5)

## 사용법

### 빌드
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 실행
```bash
roslaunch exploration_manager exploration_manager.launch
```

### RViz에서 시각화
launch 파일에 RViz가 포함되어 있어서 자동으로 실행됩니다. 다음 토픽들을 시각화할 수 있습니다:
- `/exploration/occupancy_grid`: 원본 그리드
- `/exploration/inflated_grid`: Inflated 그리드

## 의존성

- ROS Noetic
- Python 3
- OpenCV (cv2)
- NumPy
- SciPy
- PCL ROS

## 설치

필요한 Python 패키지 설치:
```bash
pip3 install opencv-python scipy numpy
```

## 주의사항

- PointCloud2의 모든 점은 z=0 평면에 있다고 가정합니다
- 그리드 크기와 해상도는 메모리 사용량에 영향을 줍니다
- Inflation 반지름은 로봇의 크기와 안전 마진을 고려하여 설정하세요 