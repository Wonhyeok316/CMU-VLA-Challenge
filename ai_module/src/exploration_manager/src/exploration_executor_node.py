#!/usr/bin/env python3

import rospy
import numpy as np
import json
import os
import time
from collections import defaultdict
import heapq
import math
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

class ExplorationExecutor:
    def __init__(self):
        rospy.init_node('exploration_executor', anonymous=True)
        
        # 탐색 상태
        self.exploration_active = True
        self.current_waypoint = None
        self.waypoint_reached_threshold = 0.3  # 미터
        self.current_robot_pos = None
        
        # Publishers
        self.waypoint_pub = rospy.Publisher('/waypoint_with_heading', PoseStamped, queue_size=1)
        
        # Subscribers
        self.state_estimation_sub = rospy.Subscriber('/state_estimation', Odometry, self.state_estimation_callback)
        
        # 그래프 데이터
        self.traversable_graph = defaultdict(list)
        self.frontiers = []
        self.grid_resolution = 0.1
        self.grid_origin_x = -5.0
        self.grid_origin_y = -5.0
        
        # 탐색 결과 저장
        self.exploration_results = {
            'visited_frontiers': [],
            'path_length': 0,
            'exploration_time': 0,
            'waypoints': [],
            'exploration_percentage': 0
        }
        
        rospy.loginfo("Exploration Executor initialized")
        
        # 그래프 데이터 로드 및 탐색 시작
        self.load_graph_data()
        self.start_exploration()
    
    def state_estimation_callback(self, msg):
        """로봇 위치 업데이트"""
        self.current_robot_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
    
    def load_graph_data(self):
        """저장된 그래프 데이터 로드"""
        try:
            file_path = '/tmp/graph_data.json'
            
            if not os.path.exists(file_path):
                rospy.logerr(f"Graph data file not found: {file_path}")
                rospy.loginfo("Waiting for graph data to be generated...")
                return False
            
            with open(file_path, 'r') as f:
                graph_data = json.load(f)
            
            # 데이터 로드
            self.grid_resolution = graph_data['grid_resolution']
            self.grid_width = graph_data['grid_width']
            self.grid_height = graph_data['grid_height']
            self.grid_origin_x = graph_data['grid_origin_x']
            self.grid_origin_y = graph_data['grid_origin_y']
            self.traversable_graph = defaultdict(list, graph_data['traversable_graph'])
            self.frontiers = graph_data['frontiers']
            self.current_robot_pos = graph_data['current_robot_pos']
            
            rospy.loginfo(f"Graph data loaded successfully")
            rospy.loginfo(f"Graph nodes: {len(self.traversable_graph)}")
            rospy.loginfo(f"Frontiers: {len(self.frontiers)}")
            rospy.loginfo(f"Robot position: {self.current_robot_pos}")
            
            return True
            
        except Exception as e:
            rospy.logerr(f"Error loading graph data: {e}")
            return False
    
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
    
    def a_star_pathfinding(self, start, goal):
        """A* 알고리즘으로 경로 찾기"""
        if start not in self.traversable_graph or goal not in self.traversable_graph:
            return None
        
        open_set = [(0, start)]
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        
        while open_set:
            current_f, current = heapq.heappop(open_set)
            
            if current == goal:
                return self.reconstruct_path(came_from, current)
            
            for neighbor in self.traversable_graph[current]:
                tentative_g = g_score[current] + 1
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        return None
    
    def heuristic(self, a, b):
        """맨해튼 거리 휴리스틱"""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])
    
    def reconstruct_path(self, came_from, current):
        """경로 재구성"""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path[::-1]
    
    def find_best_frontier(self):
        """가장 좋은 frontier 찾기"""
        if not self.frontiers or not self.current_robot_pos:
            return None
        
        robot_grid_x, robot_grid_y = self.world_to_grid(self.current_robot_pos[0], self.current_robot_pos[1])
        robot_pos = (robot_grid_x, robot_grid_y)
        
        best_frontier = None
        best_score = float('inf')
        
        for frontier in self.frontiers:
            # frontier의 중심점 계산
            center_x = sum(p[0] for p in frontier) / len(frontier)
            center_y = sum(p[1] for p in frontier) / len(frontier)
            center = (int(center_x), int(center_y))
            
            # A*로 경로 찾기
            path = self.a_star_pathfinding(robot_pos, center)
            if path:
                # 점수 계산: 경로 길이 + frontier 크기의 역수
                path_length = len(path)
                frontier_size = len(frontier)
                score = path_length + 10.0 / frontier_size
                
                if score < best_score:
                    best_score = score
                    best_frontier = center
        
        return best_frontier
    
    def is_waypoint_reached(self):
        """waypoint에 도달했는지 확인"""
        if not self.current_waypoint or not self.current_robot_pos:
            return False
        
        distance = math.sqrt(
            (self.current_robot_pos[0] - self.current_waypoint[0])**2 +
            (self.current_robot_pos[1] - self.current_waypoint[1])**2
        )
        return distance < self.waypoint_reached_threshold
    
    def publish_waypoint(self, x, y):
        """waypoint 발행"""
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "map"
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation.w = 1.0
        
        self.waypoint_pub.publish(pose_msg)
    
    def start_exploration(self):
        """완전 탐색 시작"""
        rospy.loginfo("Starting complete exploration...")
        
        # 그래프 데이터가 로드될 때까지 대기
        while not self.load_graph_data() and not rospy.is_shutdown():
            rospy.sleep(2.0)
        
        if rospy.is_shutdown():
            return
        
        # 탐색 시작
        start_time = time.time()
        visited_frontiers = []
        total_path_length = 0
        
        while self.exploration_active and not rospy.is_shutdown():
            if not self.frontiers:
                rospy.loginfo("No more frontiers to explore!")
                break
            
            target = self.find_best_frontier()
            if target:
                world_x, world_y = self.grid_to_world(target[0], target[1])
                self.publish_waypoint(world_x, world_y)
                self.current_waypoint = (world_x, world_y)
                
                # 탐색 결과에 추가
                self.exploration_results['waypoints'].append({
                    'x': world_x,
                    'y': world_y,
                    'grid_x': target[0],
                    'grid_y': target[1],
                    'timestamp': rospy.Time.now().to_sec()
                })
                
                rospy.loginfo(f"New waypoint: ({world_x:.2f}, {world_y:.2f})")
                
                # waypoint 도달 대기
                rate = rospy.Rate(1.0)
                while not self.is_waypoint_reached() and not rospy.is_shutdown():
                    rate.sleep()
                
                visited_frontiers.append(target)
                rospy.loginfo(f"Reached frontier at ({world_x:.2f}, {world_y:.2f})")
                
                # 경로 길이 계산
                if len(self.exploration_results['waypoints']) > 1:
                    prev_waypoint = self.exploration_results['waypoints'][-2]
                    distance = math.sqrt(
                        (world_x - prev_waypoint['x'])**2 + 
                        (world_y - prev_waypoint['y'])**2
                    )
                    total_path_length += distance
                
            else:
                rospy.loginfo("No frontier found for waypoint")
                break
            
            rospy.sleep(0.1)
        
        # 탐색 완료
        exploration_time = time.time() - start_time
        self.exploration_results['exploration_time'] = exploration_time
        self.exploration_results['path_length'] = total_path_length
        self.exploration_results['visited_frontiers'] = len(visited_frontiers)
        
        # 결과 저장
        self.save_exploration_results()
        
        rospy.loginfo("Complete exploration finished!")
        rospy.loginfo(f"Exploration time: {exploration_time:.2f} seconds")
        rospy.loginfo(f"Total path length: {total_path_length:.2f} meters")
        rospy.loginfo(f"Visited frontiers: {len(visited_frontiers)}")
    
    def save_exploration_results(self):
        """탐색 결과를 /tmp에 저장"""
        try:
            results = {
                'exploration_results': self.exploration_results,
                'timestamp': rospy.Time.now().to_sec(),
                'graph_info': {
                    'total_nodes': len(self.traversable_graph),
                    'total_frontiers': len(self.frontiers),
                    'visited_frontiers': self.exploration_results['visited_frontiers']
                }
            }
            
            file_path = '/tmp/exploration_results.json'
            with open(file_path, 'w') as f:
                json.dump(results, f, indent=2)
            
            rospy.loginfo(f"Exploration results saved to {file_path}")
            
        except Exception as e:
            rospy.logerr(f"Error saving exploration results: {e}")

if __name__ == '__main__':
    try:
        exploration_executor = ExplorationExecutor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 