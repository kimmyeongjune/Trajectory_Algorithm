import osmnx as ox
import networkx as nx
import queue
import math
import heapq

map_graph = ox.graph_from_place('Berkeley, California', network_type='drive')

# 가장 가까운 노드 찾기
origin = ox.distance.nearest_nodes(map_graph, X= -80 , Y= 30)
destination = list(map_graph.nodes())[-1]

# 최단 경로 계산
shortest_path = nx.shortest_path(map_graph, origin, destination, weight='length')
fig, ax = ox.plot_graph_route(map_graph, shortest_path)

import heapq

# 주어진 그래프, 출발 노드 키, 목표 노드 키를 기반으로 최단 경로를 찾는 다익스트라 알고리즘
def dijkstras_search(origin_key, goal_key, graph):
    # 우선순위 큐(힙)를 초기화
    open_queue = []
    heapq.heappush(open_queue, (0, origin_key))  # (거리, 노드) 형태로 추가

    # 최단 거리와 경로 추적을 위한 딕셔너리 초기화
    distances = {origin_key: 0.0}
    predecessors = {}

    # 목표를 찾기 전까지 큐를 순회
    while open_queue:
        # 우선순위 큐에서 가장 짧은 거리의 노드를 꺼냄
        current_distance, current_node = heapq.heappop(open_queue)

        # 목표 노드에 도달하면 경로 반환
        if current_node == goal_key:
            return get_path(origin_key, goal_key, predecessors)

        # 인접 노드를 탐색하여 최단 거리 갱신
        for neighbor, edge_data in graph[current_node].items():
            edge_length = edge_data[0].get('length', 1)  # 가중치 (기본값 1)
            new_distance = current_distance + edge_length

            # 더 짧은 경로가 발견된 경우 큐와 거리, 선행 노드 정보 업데이트
            if neighbor not in distances or new_distance < distances[neighbor]:
                distances[neighbor] = new_distance
                predecessors[neighbor] = current_node
                heapq.heappush(open_queue, (new_distance, neighbor))

    # 목표 노드를 찾지 못한 경우
    raise ValueError("Goal not found in search.")

# 경로 생성 함수
def get_path(origin_key, goal_key, predecessors):
    path = []
    current = goal_key
    while current != origin_key:
        path.append(current)
        current = predecessors[current]
    path.append(origin_key)
    path.reverse()
    return path

# 예제 사용
map_graph = ox.graph_from_place('Berkeley, California', network_type='drive')
origin = ox.distance.nearest_nodes(map_graph, X= -80 , Y= 30)
destination = list(map_graph.nodes())[-1]

# 다익스트라 알고리즘으로 최단 경로 찾기
path = dijkstras_search(origin, destination, map_graph)

# 경로 시각화
fig, ax = ox.plot_graph_route(map_graph, path)