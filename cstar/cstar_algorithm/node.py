import math
from typing import Tuple, Set

class Node:
    """Node class for the Rapidly Covering Graph (RCG)"""
    def __init__(self, position: Tuple[int, int], node_id: int):
        self.position = position              # Tọa độ của node trong grid (r, c)
        self.id = node_id                     # ID duy nhất của node
        self.neighbors = set()                # Tập các node kết nối trực tiếp (cạnh)
        self.coverage_set = set()             # Tập các ô mà node này có thể bao phủ (bán kính cảm biến)
        self.is_frontier = False              # True nếu node là frontier (có thể bao phủ vùng mới)
        self.parent = None                    # Node cha (dùng trong tìm đường)
        self.g_cost = float('inf')            # Chi phí g (A* cost từ start → node này)
        self.h_cost = 0                       # Chi phí heuristic (ước lượng đến đích)
        self.f_cost = float('inf')            # Tổng chi phí f = g + h (dùng cho heapq)
        
    def add_neighbor(self, neighbor_id: int):
        """Thêm node lân cận (hàng xóm) vào danh sách kết nối"""
        self.neighbors.add(neighbor_id)
        
    def __lt__(self, other):
        """So sánh node theo chi phí f_cost (cho heapq sắp xếp)"""
        return self.f_cost < other.f_cost