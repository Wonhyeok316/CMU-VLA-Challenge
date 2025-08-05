#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import math

class GraphVisualizer:
    def __init__(self):
        rospy.init_node('graph_visualizer', anonymous=True)
        
        # Subscribers
        self.traversable_graph_sub = rospy.Subscriber(
            '/exploration/traversable_graph', 
            OccupancyGrid, 
            self.traversable_graph_callback
        )
        
        # Publishers
        self.graph_markers_pub = rospy.Publisher(
            '/exploration/graph_visualization', 
            MarkerArray, 
            queue_size=1
        )
        
        # Graph 데이터
        self.graph_nodes = []
        self.graph_edges = []
        self.grid_resolution = 0.1
        self.grid_origin_x = -5.0
        self.grid_origin_y = -5.0
        
        # 시각화 설정
        self.node_radius = 0.05  # 노드 반지름 (미터)
        self.edge_width = 0.02   # 엣지 두께 (미터)
        self.node_color = ColorRGBA(0.0, 0.8, 1.0, 0.8)  # 하늘색
        self.edge_color = ColorRGBA(1.0, 1.0, 0.0, 0.6)  # 노란색
        
        rospy.loginfo("Graph Visualizer initialized")
        
        # 주기적 업데이트
        self.update_timer = rospy.Timer(rospy.Duration(0.5), self.update_visualization)
    
    def traversable_graph_callback(self, msg):
        """Traversable graph 데이터 처리"""
        try:
            # Grid 파라미터 업데이트
            self.grid_resolution = msg.info.resolution
            self.grid_origin_x = msg.info.origin.position.x
            self.grid_origin_y = msg.info.origin.position.y
            
            # Grid 데이터에서 그래프 노드 추출
            grid_data = np.array(msg.data).reshape(msg.info.height, msg.info.width)
            
            # 그래프 노드 찾기 (값이 50인 셀들)
            node_positions = np.where(grid_data == 50)
            
            self.graph_nodes = []
            for y, x in zip(node_positions[0], node_positions[1]):
                world_x, world_y = self.grid_to_world(x, y)
                self.graph_nodes.append((world_x, world_y))
            
            # 간단한 엣지 생성 (4방향 인접 노드들)
            self.graph_edges = []
            for i, node1 in enumerate(self.graph_nodes):
                for j, node2 in enumerate(self.graph_nodes):
                    if i != j:
                        # 거리 계산
                        dist = math.sqrt((node1[0] - node2[0])**2 + (node1[1] - node2[1])**2)
                        # 인접한 노드들만 엣지로 연결 (거리가 grid_resolution의 1.5배 이내)
                        if dist <= self.grid_resolution * 1.5:
                            self.graph_edges.append((node1, node2))
            
            rospy.loginfo(f"Graph updated: {len(self.graph_nodes)} nodes, {len(self.graph_edges)} edges")
            
        except Exception as e:
            rospy.logerr(f"Error processing traversable graph: {e}")
    
    def grid_to_world(self, grid_x, grid_y):
        """그리드 좌표를 월드 좌표로 변환"""
        world_x = self.grid_origin_x + grid_x * self.grid_resolution
        world_y = self.grid_origin_y + grid_y * self.grid_resolution
        return world_x, world_y
    
    def update_visualization(self, event):
        """그래프 시각화 업데이트"""
        if not self.graph_nodes:
            return
        
        marker_array = MarkerArray()
        
        # 노드 마커 생성
        node_marker = Marker()
        node_marker.header.frame_id = "map"
        node_marker.header.stamp = rospy.Time.now()
        node_marker.ns = "graph_nodes"
        node_marker.id = 0
        node_marker.type = Marker.SPHERE_LIST
        node_marker.action = Marker.ADD
        
        # 노드 스타일 설정
        node_marker.scale.x = self.node_radius * 2
        node_marker.scale.y = self.node_radius * 2
        node_marker.scale.z = self.node_radius * 2
        node_marker.color = self.node_color
        
        # 노드 위치 추가
        for node in self.graph_nodes:
            point = Point()
            point.x = node[0]
            point.y = node[1]
            point.z = 0.1  # 약간 위에 표시
            node_marker.points.append(point)
        
        marker_array.markers.append(node_marker)
        
        # 엣지 마커 생성
        edge_marker = Marker()
        edge_marker.header.frame_id = "map"
        edge_marker.header.stamp = rospy.Time.now()
        edge_marker.ns = "graph_edges"
        edge_marker.id = 1
        edge_marker.type = Marker.LINE_LIST
        edge_marker.action = Marker.ADD
        
        # 엣지 스타일 설정
        edge_marker.scale.x = self.edge_width
        edge_marker.color = self.edge_color
        
        # 엣지 위치 추가
        for edge in self.graph_edges:
            # 시작점
            start_point = Point()
            start_point.x = edge[0][0]
            start_point.y = edge[0][1]
            start_point.z = 0.05
            edge_marker.points.append(start_point)
            
            # 끝점
            end_point = Point()
            end_point.x = edge[1][0]
            end_point.y = edge[1][1]
            end_point.z = 0.05
            edge_marker.points.append(end_point)
        
        marker_array.markers.append(edge_marker)
        
        # 마커 발행
        self.graph_markers_pub.publish(marker_array)
    
    def create_info_marker(self):
        """그래프 정보를 표시하는 텍스트 마커"""
        info_marker = Marker()
        info_marker.header.frame_id = "map"
        info_marker.header.stamp = rospy.Time.now()
        info_marker.ns = "graph_info"
        info_marker.id = 2
        info_marker.type = Marker.TEXT_VIEW_FACING
        info_marker.action = Marker.ADD
        
        # 위치 설정 (화면 좌상단)
        info_marker.pose.position.x = self.grid_origin_x + 2.0
        info_marker.pose.position.y = self.grid_origin_y + 2.0
        info_marker.pose.position.z = 1.0
        
        # 텍스트 설정
        info_marker.text = f"Graph Nodes: {len(self.graph_nodes)}\nGraph Edges: {len(self.graph_edges)}"
        info_marker.scale.z = 0.2  # 텍스트 크기
        info_marker.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)  # 흰색
        
        return info_marker

if __name__ == '__main__':
    try:
        graph_visualizer = GraphVisualizer()
        rospy.loginfo("Graph Visualizer started. Check RViz for visualization.")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 