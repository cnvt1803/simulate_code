import math
from typing import Tuple, Set


class Node:
    """Node class for the Rapidly Covering Graph (RCG)"""

    def __init__(self, position: Tuple[int, int], node_id: int):
        # Tọa độ của node trong grid (r, c)
        self.position = position
        self.id = node_id                     # ID duy nhất của node
        self.neighbors = set()                # Tập các node kết nối trực tiếp (cạnh)
        # Tập các ô mà node này có thể bao phủ (bán kính cảm biến)
        self.coverage_set = set()
        # True nếu node là frontier (có thể bao phủ vùng mới)
        self.is_frontier = False
        self.parent = None                    # Node cha (dùng trong tìm đường)
        # Chi phí g (A* cost từ start → node này)
        self.g_cost = float('inf')
        # Chi phí heuristic (ước lượng đến đích)
        self.h_cost = 0
        # Tổng chi phí f = g + h (dùng cho heapq)
        self.f_cost = float('inf')
        self.state = "Op"   # "Op" hoặc "Cl"

    def add_neighbor(self, neighbor_id: int):
        """Thêm node lân cận (hàng xóm) vào danh sách kết nối"""
        self.neighbors.add(neighbor_id)

    def __lt__(self, other):
        """So sánh node theo chi phí f_cost (cho heapq sắp xếp)"""
        return self.f_cost < other.f_cost
