#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from scipy.ndimage import binary_dilation
import cv2
from collections import deque, defaultdict
import json
import os

class GraphBuilder:
    def __init__(self):
        rospy.init_node('graph_builder', anonymous=True)

        # Grid 파라미터
        self.grid_resolution = rospy.get_param('~grid_resolution', 0.1)
        self.grid_width = rospy.get_param('~grid_width', 100)
        self.grid_height = rospy.get_param('~grid_height', 100)
        self.inflation_radius = rospy.get_param('~inflation_radius', 0.1)
        self.grid_origin_x = rospy.get_param('~grid_origin_x', -5.0)
        self.grid_origin_y = rospy.get_param('~grid_origin_y', -5.0)

        # Grid 데이터
        self.occupancy_grid = np.zeros((self.grid_height, self.grid_width), dtype=np.uint8)
        self.inflated_grid = np.zeros((self.grid_height, self.grid_width), dtype=np.uint8)
        self.explored_grid = np.zeros((self.grid_height, self.grid_width), dtype=np.uint8)
        
        # 그래프 데이터
        self.traversable_graph = defaultdict(list)
        self.frontiers = []
        self.current_robot_pos = None
        
        # Publishers
        self.grid_pub = rospy.Publisher('/exploration/occupancy_grid', OccupancyGrid, queue_size=1)
        self.inflated_grid_pub = rospy.Publisher('/exploration/inflated_grid', OccupancyGrid, queue_size=1)
        self.traversable_graph_pub = rospy.Publisher('/exploration/traversable_graph', OccupancyGrid, queue_size=1)
        self.graph_data_pub = rospy.Publisher('/exploration/graph_data', OccupancyGrid, queue_size=1)
        
        # Subscribers
        self.traversable_sub = rospy.Subscriber('/traversable_area', PointCloud2, self.traversable_callback)
        self.state_estimation_sub = rospy.Subscriber('/state_estimation', Odometry, self.state_estimation_callback)
        
        # Timers
        self.publish_timer = rospy.Timer(rospy.Duration(0.1), self.publish_grid)
        self.graph_publish_timer = rospy.Timer(rospy.Duration(0.5), self.publish_traversable_graph)
        self.data_save_timer = rospy.Timer(rospy.Duration(2.0), self.save_graph_data)
        
        # 4방향 이동 (상, 하, 좌, 우)
        self.directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        
        rospy.loginfo("Graph Builder initialized")
        rospy.loginfo(f"Grid resolution: {self.grid_resolution}m")
        rospy.loginfo(f"Grid size: {self.grid_width}x{self.grid_height}")
        rospy.loginfo(f"Inflation radius: {self.inflation_radius}m")

    def world_to_grid(self, x, y):
        """월드 좌표를 그리드 좌표로 변환"""
        grid_x = int((x - self.grid_origin_x) / self.grid_resolution)
        grid_y = int((y - self.grid_origin_y) / self.grid_resolution)
        return grid_x, grid_y

    def grid_to_world(self, grid_x, grid_y):
        """그리드 좌표를 월드 좌표로 변환"""
        world_x = self.grid_origin_x + grid_x * self.grid_resolution
        world_y = self.grid_origin_y + grid_y * self.grid_resolution
        return world_x, world_y
    
    def is_valid_grid_coord(self, grid_x, grid_y):
        """그리드 좌표가 유효한지 확인"""
        return 0 <= grid_x < self.grid_width and 0 <= grid_y < self.grid_height
    
    def state_estimation_callback(self, msg):
        """State estimation을 통한 로봇 위치 업데이트"""
        self.current_robot_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        
        if self.current_robot_pos:
            grid_x, grid_y = self.world_to_grid(self.current_robot_pos[0], self.current_robot_pos[1])
            if self.is_valid_grid_coord(grid_x, grid_y):
                self.explored_grid[grid_y, grid_x] = 1
                
                # 주변 영역도 explored로 마킹 (로봇 크기 고려)
                robot_radius_cells = int(0.3 / self.grid_resolution)
                for dx in range(-robot_radius_cells, robot_radius_cells + 1):
                    for dy in range(-robot_radius_cells, robot_radius_cells + 1):
                        nx, ny = grid_x + dx, grid_y + dy
                        if self.is_valid_grid_coord(nx, ny):
                            self.explored_grid[ny, nx] = 1
    
    def traversable_callback(self, msg):
        """통행 가능한 포인트 처리"""
        try:
            points = self.pointcloud2_to_array(msg)
            self.occupancy_grid.fill(0)
            
            # 각 grid 셀에 점의 개수를 카운트
            grid_counts = np.zeros((self.grid_height, self.grid_width), dtype=np.int32)
            
            for point in points:
                if abs(point[2]) < 0.1:  # z축 높이가 낮은 포인트만
                    grid_x, grid_y = self.world_to_grid(point[0], point[1])
                    if self.is_valid_grid_coord(grid_x, grid_y):
                        grid_counts[grid_y, grid_x] += 1
            
            # 최소 점 개수 임계값 적용 (노이즈 필터링)
            min_points_threshold = 3
            self.occupancy_grid = np.where(grid_counts >= min_points_threshold, 100, 0)
            
            self.apply_inflation()
            self.build_traversable_graph()
            self.update_frontiers()
            
            rospy.loginfo(f"Processed {len(points)} points from traversable_points")
        except Exception as e:
            rospy.logerr(f"Error processing traversable points: {e}")
    
    def pointcloud2_to_array(self, cloud_msg):
        """PointCloud2 메시지를 numpy 배열로 변환"""
        points = []
        
        fields = cloud_msg.fields
        point_step = cloud_msg.point_step
        data = cloud_msg.data
        
        x_idx = y_idx = z_idx = -1
        for i, field in enumerate(fields):
            if field.name == 'x':
                x_idx = field.offset
            elif field.name == 'y':
                y_idx = field.offset
            elif field.name == 'z':
                z_idx = field.offset
        
        if x_idx == -1 or y_idx == -1 or z_idx == -1:
            rospy.logerr("Required fields (x, y, z) not found in PointCloud2")
            return []
        
        for i in range(cloud_msg.width * cloud_msg.height):
            offset = i * point_step
            
            x = np.frombuffer(data[offset + x_idx:offset + x_idx + 4], dtype=np.float32)[0]
            y = np.frombuffer(data[offset + y_idx:offset + y_idx + 4], dtype=np.float32)[0]
            z = np.frombuffer(data[offset + z_idx:offset + z_idx + 4], dtype=np.float32)[0]
            
            points.append([x, y, z])
        
        return np.array(points)
    
    def apply_inflation(self):
        """장애물 주변에 inflation 적용"""
        obstacle_mask = (self.occupancy_grid == 0)
        
        inflation_pixels = int(self.inflation_radius / self.grid_resolution)
        inflated_mask = binary_dilation(obstacle_mask, iterations=inflation_pixels)
        
        self.inflated_grid = np.where(inflated_mask, 100, 0)
    
    def build_traversable_graph(self):
        """4방향 인접 그래프 생성"""
        self.traversable_graph.clear()
        
        for y in range(self.grid_height):
            for x in range(self.grid_width):
                if self.inflated_grid[y, x] == 0:  # 통행 가능한 셀
                    node = (x, y)
                    neighbors = []
                    
                    # 4방향 이웃 확인
                    for dx, dy in self.directions:
                        nx, ny = x + dx, y + dy
                        if self.is_valid_grid_coord(nx, ny) and self.inflated_grid[ny, nx] == 0:
                            neighbors.append((nx, ny))
                    
                    if neighbors:
                        self.traversable_graph[node] = neighbors
    
    def update_frontiers(self):
        """Frontier 업데이트"""
        self.frontiers = []
        visited = set()
        
        # 디버깅 정보 추가
        explored_count = np.sum(self.explored_grid)
        inflated_occupied = np.sum(self.inflated_grid == 100)
        inflated_free = np.sum(self.inflated_grid == 0)
        
        rospy.loginfo(f"Grid status - Explored: {explored_count}, Inflated occupied: {inflated_occupied}, Inflated free: {inflated_free}")
        
        for y in range(self.grid_height):
            for x in range(self.grid_width):
                if (x, y) in visited:
                    continue
                
                if self.is_frontier_cell(x, y):
                    frontier = self.grow_frontier(x, y, visited)
                    if len(frontier) > 3:  # 최소 크기 조건
                        self.frontiers.append(frontier)
        
        rospy.loginfo(f"Found {len(self.frontiers)} frontiers")
    
    def is_frontier_cell(self, x, y):
        """셀이 frontier인지 확인"""
        if not self.is_valid_grid_coord(x, y) or self.inflated_grid[y, x] != 0:
            return False
        
        if self.explored_grid[y, x] == 1:
            for dx, dy in self.directions:
                nx, ny = x + dx, y + dy
                if (self.is_valid_grid_coord(nx, ny) and 
                    self.explored_grid[ny, nx] == 0 and 
                    self.inflated_grid[ny, nx] == 0):
                    return True
        return False
    
    def grow_frontier(self, start_x, start_y, visited):
        """Frontier 영역을 BFS로 확장"""
        frontier = []
        queue = deque([(start_x, start_y)])
        
        while queue:
            x, y = queue.popleft()
            if (x, y) in visited:
                continue
            
            visited.add((x, y))
            if self.is_frontier_cell(x, y):
                frontier.append((x, y))
                
                for dx, dy in self.directions:
                    nx, ny = x + dx, y + dy
                    if (self.is_valid_grid_coord(nx, ny) and 
                        (nx, ny) not in visited and 
                        self.is_frontier_cell(nx, ny)):
                        queue.append((nx, ny))
        
        return frontier
    
    def publish_grid(self, event):
        """Occupancy grid 발행"""
        # 기본 occupancy grid
        grid_msg = OccupancyGrid()
        grid_msg.header.frame_id = "map"
        grid_msg.header.stamp = rospy.Time.now()
        grid_msg.info.resolution = self.grid_resolution
        grid_msg.info.width = self.grid_width
        grid_msg.info.height = self.grid_height
        grid_msg.info.origin.position.x = self.grid_origin_x
        grid_msg.info.origin.position.y = self.grid_origin_y
        grid_msg.info.origin.orientation.w = 1.0
        grid_msg.data = self.occupancy_grid.flatten().tolist()
        self.grid_pub.publish(grid_msg)
        
        # Inflated grid
        inflated_msg = OccupancyGrid()
        inflated_msg.header.frame_id = "map"
        inflated_msg.header.stamp = rospy.Time.now()
        inflated_msg.info.resolution = self.grid_resolution
        inflated_msg.info.width = self.grid_width
        inflated_msg.info.height = self.grid_height
        inflated_msg.info.origin.position.x = self.grid_origin_x
        inflated_msg.info.origin.position.y = self.grid_origin_y
        inflated_msg.info.origin.orientation.w = 1.0
        inflated_msg.data = self.inflated_grid.flatten().tolist()
        self.inflated_grid_pub.publish(inflated_msg)
    
    def publish_traversable_graph(self, event):
        """Traversable graph 발행"""
        graph_grid = np.zeros((self.grid_height, self.grid_width), dtype=np.uint8)
        
        # 그래프 노드들을 grid에 표시
        for node in self.traversable_graph:
            x, y = node
            if self.is_valid_grid_coord(x, y):
                graph_grid[y, x] = 50  # 그래프 노드는 중간 값으로 표시
        
        graph_msg = OccupancyGrid()
        graph_msg.header.frame_id = "map"
        graph_msg.header.stamp = rospy.Time.now()
        graph_msg.info.resolution = self.grid_resolution
        graph_msg.info.width = self.grid_width
        graph_msg.info.height = self.grid_height
        graph_msg.info.origin.position.x = self.grid_origin_x
        graph_msg.info.origin.position.y = self.grid_origin_y
        graph_msg.info.origin.orientation.w = 1.0
        graph_msg.data = graph_grid.flatten().tolist()
        self.traversable_graph_pub.publish(graph_msg)
    
    def save_graph_data(self, event):
        """그래프 데이터를 /tmp에 저장"""
        try:
            graph_data = {
                'grid_resolution': self.grid_resolution,
                'grid_width': self.grid_width,
                'grid_height': self.grid_height,
                'grid_origin_x': self.grid_origin_x,
                'grid_origin_y': self.grid_origin_y,
                'traversable_graph': dict(self.traversable_graph),
                'frontiers': self.frontiers,
                'explored_grid': self.explored_grid.tolist(),
                'inflated_grid': self.inflated_grid.tolist(),
                'occupancy_grid': self.occupancy_grid.tolist(),
                'current_robot_pos': self.current_robot_pos,
                'timestamp': rospy.Time.now().to_sec()
            }
            
            # /tmp 디렉토리에 저장
            file_path = '/tmp/graph_data.json'
            with open(file_path, 'w') as f:
                json.dump(graph_data, f, indent=2)
            
            rospy.loginfo(f"Graph data saved to {file_path}")
            rospy.loginfo(f"Graph nodes: {len(self.traversable_graph)}, Frontiers: {len(self.frontiers)}")
            
        except Exception as e:
            rospy.logerr(f"Error saving graph data: {e}")

if __name__ == '__main__':
    try:
        graph_builder = GraphBuilder()
        rospy.loginfo("Graph Builder started. Graph data will be saved to /tmp/graph_data.json")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 