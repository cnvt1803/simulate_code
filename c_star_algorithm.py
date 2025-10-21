"""
C* Algorithm Implementation for Coverage Path Planning
Thuật toán lập kế hoạch đường đi để bao phủ toàn bộ môi trường (Coverage Path Planning)
Dựa trên bài báo:
"C*: A Coverage Path Planning Algorithm for Unknown Environments using Rapidly Covering Graphs"
tác giả: Zongyuan Shen, James P. Wilson, Shalabh Gupta
"""

import heapq
import math
import random
import numpy as np
from typing import List, Tuple, Set, Dict, Optional
from collections import defaultdict

# --- Các hằng số thể hiện trạng thái của mỗi ô trong bản đồ ---
FREE_UNCOVERED = 0   # Ô trống, chưa được bao phủ
OBSTACLE = 1         # Ô là chướng ngại vật
COVERED = 2          # Ô đã được bao phủ
FRONTIER = 3         # Ô thuộc vùng biên (chưa được bao phủ nhưng gần vùng đã bao phủ)

# --- Hướng di chuyển cho robot (8 hướng: 4 chính + 4 chéo) ---
DIR8 = [
    (-1, -1), (-1, 0), (-1, 1),   # lên trái, lên, lên phải
    (0, -1),           (0, 1),    # trái, phải
    (1, -1),  (1, 0),  (1, 1)     # xuống trái, xuống, xuống phải
]
DIR4 = [
            (-1, 0),    # lên trái, lên, lên phải
    (0, -1),           (0, 1),    # trái, phải
             (1, 0),      # xuống trái, xuống, xuống phải
]
def snap_to_grid(p, grid_step):
    """Làm tròn vị trí về lưới sampling step"""
    r, c = p
    return (max(0, r // grid_step * grid_step),
            max(0, c // grid_step * grid_step))


# ===============================================================
# 1️⃣ CLASS Node - Đại diện cho một "nút" trong đồ thị RCG
# ===============================================================
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


# ===============================================================
# 2️⃣ CLASS RapidlyCoveringGraph (RCG)
#     Là đồ thị được mở rộng dần khi robot phát hiện thêm vùng mới.
# ===============================================================
class RapidlyCoveringGraph:
    """Rapidly Covering Graph (RCG) for C* algorithm"""
    def __init__(self, grid_size: Tuple[int, int], sensing_radius: int = 2):
        self.grid_size = grid_size            # Kích thước bản đồ (số hàng, số cột)
        self.sensing_radius = sensing_radius  # Bán kính cảm biến của robot
        self.nodes = {}                       # Từ điển chứa các node: {id: Node}
        self.node_counter = 0                 # Đếm số node đã tạo (dùng để cấp ID)
        self.grid_to_nodes = defaultdict(set) # Ánh xạ ô (r,c) → tập các node ở vị trí đó
        
    def add_node(self, position: Tuple[int, int]) -> int:
        """Thêm một node mới vào đồ thị"""
        node_id = self.node_counter
        self.node_counter += 1
        
        node = Node(position, node_id)
        # Tính toán vùng bao phủ (coverage set) quanh node này
        node.coverage_set = self.get_coverage_area(position)
        
        # Lưu node vào danh sách
        self.nodes[node_id] = node
        self.grid_to_nodes[position].add(node_id)
        
        # Kết nối node mới này với các node gần đó
        self.connect_node(node_id)
        
        return node_id
    
    def get_coverage_area(self, position: Tuple[int, int]) -> Set[Tuple[int, int]]:
        """Trả về tập hợp các ô nằm trong phạm vi cảm biến của node"""
        r, c = position
        coverage = set()
        
        # Quét quanh node trong phạm vi bán kính sensing_radius
        for dr in range(-self.sensing_radius, self.sensing_radius + 1):
            for dc in range(-self.sensing_radius, self.sensing_radius + 1):
                nr, nc = r + dr, c + dc
                # Nếu nằm trong bản đồ và khoảng cách <= bán kính → thêm vào coverage
                if (0 <= nr < self.grid_size[0] and 
                    0 <= nc < self.grid_size[1] and
                    math.sqrt(dr*dr + dc*dc) <= self.sensing_radius):
                    coverage.add((nr, nc))
        
        return coverage
    
    def connect_node(self, node_id: int):
        """Kết nối node với các node gần nó trong đồ thị"""
        node = self.nodes[node_id]
        r, c = node.position
        
        # Bán kính tìm kiếm node lân cận mở rộng gấp đôi bán kính cảm biến
        search_radius = self.sensing_radius * 2
        for dr in range(-search_radius, search_radius + 1):
            for dc in range(-search_radius, search_radius + 1):
                nr, nc = r + dr, c + dc
                # Nếu có node khác gần vị trí này
                if (nr, nc) in self.grid_to_nodes:
                    for other_id in self.grid_to_nodes[(nr, nc)]:
                        if other_id != node_id:
                            distance = math.sqrt(dr*dr + dc*dc)
                            # Kết nối nếu khoảng cách nhỏ hơn bán kính tìm kiếm
                            if distance <= search_radius:
                                node.add_neighbor(other_id)
                                self.nodes[other_id].add_neighbor(node_id)
    
    def find_frontier_nodes(self,
                        covered_cells: Set[Tuple[int, int]],
                        obstacles: Set[Tuple[int, int]],
                        observed_cells: Set[Tuple[int, int]],
                        use_8_adj: bool = False) -> List[int]:
        """
        Frontier node nếu vị trí node free và KỀ CẬN:
        - unknown (ô chưa observed), và/hoặc
        - obstacle, và/hoặc
        - boundary.
        Đồng thời loại các node đã COVERED hoặc chính ô hiện tại (lọc ở phía CStar nếu cần).
        """
        frontier_nodes = []
        adj = DIR8 if use_8_adj else DIR4

        for node_id, node in self.nodes.items():
            r, c = node.position
            # ngoài bản đồ hoặc là obstacle → không phải frontier
            if not (0 <= r < self.grid_size[0] and 0 <= c < self.grid_size[1]):
                node.is_frontier = False
                continue
            if (r, c) in obstacles:
                node.is_frontier = False
                continue
            if (r, c) in covered_cells:  # quan trọng: không coi ô đã đi qua là frontier
                node.is_frontier = False
                continue

            touches_unknown = False
            touches_obstacle = False
            touches_boundary = False

            for dr, dc in adj:
                nr, nc = r + dr, c + dc
                if not (0 <= nr < self.grid_size[0] and 0 <= nc < self.grid_size[1]):
                    touches_boundary = True
                    continue
                ncell = (nr, nc)
                if ncell in obstacles:
                    touches_obstacle = True
                if ncell not in observed_cells:
                    touches_unknown = True

            is_frontier = touches_unknown or touches_obstacle or touches_boundary
            node.is_frontier = is_frontier
            if is_frontier:
                frontier_nodes.append(node_id)

        return frontier_nodes



        # === [C* Add-on] ===
    def _in_bounds(self, r, c):
        return 0 <= r < self.grid_size[0] and 0 <= c < self.grid_size[1]

    def _lap_index(self, pos, lap_dir='vertical', w=None):
        if w is None: w = self.sensing_radius
        r, c = pos
        return (c // w) if lap_dir == 'vertical' else (r // w)

    def _remove_edge(self, u, v):
        if u in self.nodes and v in self.nodes:
            self.nodes[u].neighbors.discard(v)
            self.nodes[v].neighbors.discard(u)

    def _add_edge(self, u, v):
        if u in self.nodes and v in self.nodes and u != v:
            self.nodes[u].add_neighbor(v)
            self.nodes[v].add_neighbor(u)

    def rewire_as_cstar(self, new_node_id: int, sampling_step: int, lap_dir: str = 'vertical'):
        """Chỉ nối: (1) 2 hàng xóm gần nhất cùng lap, (2) vài nút gần ở lap kề"""
        w = sampling_step
        node = self.nodes[new_node_id]
        pos = node.position
        lap_of = {nid: self._lap_index(n.position, lap_dir, w) for nid, n in self.nodes.items()}
        my_lap = lap_of[new_node_id]

        # Xóa hết kết nối auto cũ (nếu có)
        for nb in list(node.neighbors):
            self._remove_edge(new_node_id, nb)

        # (1) 2 hàng xóm gần nhất cùng lap
        same_lap = [nid for nid, n in self.nodes.items() if lap_of[nid] == my_lap and nid != new_node_id]
        same_lap.sort(key=lambda nid: abs(self.nodes[nid].position[0]-pos[0]) +
                                      abs(self.nodes[nid].position[1]-pos[1]))
        for nid in same_lap[:2]:
            self._add_edge(new_node_id, nid)

        # (2) các nút ở lap kề trong phạm vi √2*w
        limit = math.sqrt(2)*w + 1e-6
        side_laps = [my_lap-1, my_lap+1]
        near = []
        for nid, n in self.nodes.items():
            if nid == new_node_id: continue
            if lap_of[nid] in side_laps:
                d = math.hypot(n.position[0]-pos[0], n.position[1]-pos[1])
                if d <= limit:
                    near.append((d, nid))
        near.sort()
        for _, nid in near[:3]:
            self._add_edge(new_node_id, nid)

    def prune(self, covered_cells: set, obstacles: set, lap_dir: str = 'vertical', w: int | None = None):
        """Cắt tỉa RCG: giữ node/cạnh thiết yếu, gộp node thẳng hàng."""
        if w is None: w = self.sensing_radius
        lap_of = {nid: self._lap_index(n.position, lap_dir, w) for nid, n in self.nodes.items()}
        essential = set()

        # --- A) giữ node chạm unknown/obstacle hoặc ở đầu/cuối lap
        for nid, node in self.nodes.items():
            r, c = node.position
            touch = False
            for dr in range(-w, w+1):
                for dc in range(-w, w+1):
                    rr, cc = r+dr, c+dc
                    if not self._in_bounds(rr, cc): continue
                    if (rr, cc) in obstacles:
                        touch = True
                        break
                    if (rr, cc) not in covered_cells and (rr, cc) not in obstacles:
                        touch = True
                        break
                if touch: break
            if touch: essential.add(nid)
            else:
                # check lap end
                dirs = [(-w,0),(w,0)] if lap_dir=='vertical' else [(0,-w),(0,w)]
                for dr,dc in dirs:
                    rr,cc=r+dr,c+dc
                    if (not self._in_bounds(rr,cc)) or ((rr,cc) in obstacles):
                        essential.add(nid)
                        break

        # --- B) loại cạnh dư giữa lap kề: giữ 1 cạnh gần obstacle nhất
        to_remove = set()
        for nid, node in self.nodes.items():
            groups = {}
            for nb in node.neighbors:
                if lap_of[nb] == lap_of[nid]: continue
                groups.setdefault(lap_of[nb], []).append(nb)
            for _, lst in groups.items():
                if len(lst) <= 1: continue
                def closeness(x):
                    (rx,cx)=self.nodes[x].position
                    best = min((abs(rx-ro)+abs(cx-co)) for ro,co in obstacles) if obstacles else 1e9
                    return best
                best=min(lst,key=closeness)
                for x in lst:
                    if x!=best: to_remove.add(tuple(sorted((nid,x))))

        for u,v in to_remove:
            self._remove_edge(u,v)

        # --- C) gộp node bậc 2 cùng lap (A-X-B)
        merged_add, merged_del, del_nodes = set(), set(), set()
        def same_lap(a,b): return lap_of[a]==lap_of[b]
        for nid, node in list(self.nodes.items()):
            if nid in essential: continue
            if len(node.neighbors)==2:
                a,b=list(node.neighbors)
                if same_lap(nid,a) and same_lap(nid,b):
                    merged_add.add(tuple(sorted((a,b))))
                    merged_del.add(tuple(sorted((a,nid))))
                    merged_del.add(tuple(sorted((nid,b))))
                    del_nodes.add(nid)
        for u,v in merged_del: self._remove_edge(u,v)
        for u,v in merged_add: self._add_edge(u,v)
        for nid in del_nodes:
            for nb in list(self.nodes[nid].neighbors): self._remove_edge(nid,nb)
            pos=self.nodes[nid].position
            self.grid_to_nodes[pos].discard(nid)
            if not self.grid_to_nodes[pos]: self.grid_to_nodes.pop(pos,None)
            self.nodes.pop(nid,None)

# ===============================================================
# 3️⃣ CLASS CStar — thuật toán C* chính
# ===============================================================
class CStar:
    """C* Coverage Path Planning Algorithm"""
    
    def __init__(self, initial_grid, start_pos: Tuple[int, int], sensing_radius: int = 2):
        # Sao chép grid (để không ảnh hưởng đến bản gốc)
        self.grid = [row[:] for row in initial_grid]
        self.rows, self.cols = len(initial_grid), len(initial_grid[0])
        self.start_pos = start_pos
        self.sensing_radius = sensing_radius
        
        # Trạng thái thuật toán
        self.current_pos = start_pos            # Vị trí hiện tại của robot
        self.covered_cells = set()              # Các ô đã bao phủ
        self.obstacles = set()                  # Các ô là chướng ngại vật
        self.observed_cells = set()             # Các ô đã được sensor quan sát
        self.path = [start_pos]                 # Danh sách các bước di chuyển
        self.rcg = RapidlyCoveringGraph((self.rows, self.cols), sensing_radius)
        self.observed_cells: Set[Tuple[int,int]] = set()

        # Callback cho visualization (nếu có)
        self.on_step_callback = None
        self.on_rcg_update_callback = None
        self.on_path_update_callback = None
        
        # Khởi tạo danh sách chướng ngại vật
        self.initialize_obstacles()
        
        # Bộ đếm thống kê
        self.total_nodes_generated = 0
        self.total_path_length = 0
        
    def initialize_obstacles(self):
        """Duyệt grid và lưu lại tất cả ô chướng ngại vật"""
        for r in range(self.rows):
            for c in range(self.cols):
                if self.grid[r][c] == OBSTACLE:
                    self.obstacles.add((r, c))
    
    def set_callbacks(self, step_callback=None, rcg_update_callback=None, path_update_callback=None):
        """Đăng ký các hàm callback để cập nhật GUI (Pygame)"""
        self.on_step_callback = step_callback
        self.on_rcg_update_callback = rcg_update_callback
        self.on_path_update_callback = path_update_callback
    
    def is_valid_position(self, pos: Tuple[int, int]) -> bool:
        """Kiểm tra xem một vị trí có hợp lệ (trong bản đồ & không phải obstacle)"""
        r, c = pos
        return (0 <= r < self.rows and 
                0 <= c < self.cols and 
                (r, c) not in self.obstacles)
    
    def get_sensing_area(self, pos: Tuple[int, int]) -> Set[Tuple[int, int]]:
        """Trả về vùng cảm biến (set các ô trong bán kính sensing_radius quanh pos)"""
        r, c = pos
        sensing_area = set()
        
        for dr in range(-self.sensing_radius, self.sensing_radius + 1):
            for dc in range(-self.sensing_radius, self.sensing_radius + 1):
                nr, nc = r + dr, c + dc
                if (0 <= nr < self.rows and 
                    0 <= nc < self.cols and
                    math.sqrt(dr*dr + dc*dc) <= self.sensing_radius):
                    sensing_area.add((nr, nc))
        
        return sensing_area
    
    def cover_cells(self, pos: Tuple[int, int]):
        """Đánh dấu đúng ô robot đi qua là COVERED (không quét theo bán kính)."""
        r, c = pos
        if (r, c) not in self.obstacles and 0 <= r < self.rows and 0 <= c < self.cols:
            self.covered_cells.add((r, c))
            self.grid[r][c] = COVERED
    def update_observed(self, pos: Tuple[int, int]):
        """Đánh dấu các ô trong tầm cảm biến là 'đã observed' (chỉ để xác định unknown)."""
        r0, c0 = pos
        for dr in range(-self.sensing_radius, self.sensing_radius + 1):
            for dc in range(-self.sensing_radius, self.sensing_radius + 1):
                nr, nc = r0 + dr, c0 + dc
                if 0 <= nr < self.rows and 0 <= nc < self.cols:
                    if math.sqrt(dr*dr + dc*dc) <= self.sensing_radius:
                        self.observed_cells.add((nr, nc))

    def is_frontier_cell(self, cell: Tuple[int, int], use_8_adj: bool = False) -> bool:
        """Ô free là frontier nếu kề cận unknown và/hoặc obstacle/boundary."""
        r, c = cell
        if not (0 <= r < self.rows and 0 <= c < self.cols):
            return False
        if cell in self.obstacles:
            return False
        adj = DIR8 if use_8_adj else DIR4
        touches_unknown = touches_obstacle = touches_boundary = False
        for dr, dc in adj:
            nr, nc = r + dr, c + dc
            if not (0 <= nr < self.rows and 0 <= nc < self.cols):
                touches_boundary = True
                continue
            ncell = (nr, nc)
            if ncell in self.obstacles:
                touches_obstacle = True
            if ncell not in self.observed_cells:
                touches_unknown = True
        return touches_unknown or touches_obstacle or touches_boundary

    
    def sample_random_configuration(self) -> Tuple[int, int]:
        """Lấy ngẫu nhiên một vị trí hợp lệ (dùng để mở rộng RCG)"""
        while True:
            r = random.randint(0, self.rows - 1)
            c = random.randint(0, self.cols - 1)
            if self.is_valid_position((r, c)):
                return (r, c)
    
    def find_nearest_node(self, pos: Tuple[int, int]) -> Optional[int]:
        """Tìm node gần nhất với pos, ưu tiên các node cùng 'lap' (đúng tinh thần C*)"""
        if not self.rcg.nodes:
            return None

        # Xác định lap mục tiêu của vị trí pos
        # Dùng bước w = sensing_radius và hướng 'vertical' (đổi thành 'horizontal' nếu bạn sweep ngang)
        w = self.sensing_radius
        target_lap = self.rcg._lap_index(pos, lap_dir='vertical', w=w)

        # Tách ứng viên thành 2 nhóm: cùng lap và khác lap
        same_lap_ids: list[int] = []
        other_ids: list[int] = []
        for nid, n in self.rcg.nodes.items():
            lap_n = self.rcg._lap_index(n.position, lap_dir='vertical', w=w)
            if lap_n == target_lap:
                same_lap_ids.append(nid)
            else:
                other_ids.append(nid)

        # Hàm phụ tìm gần nhất trong 1 danh sách ứng viên (không cần NumPy)
        def nearest_from(candidates: list[int]) -> Optional[int]:
            if not candidates:
                return None
            best_id = None
            best_d = float('inf')
            px, py = pos
            for nid in candidates:
                nx, ny = self.rcg.nodes[nid].position
                d = math.hypot(px - nx, py - ny)
                if d < best_d:
                    best_d = d
                    best_id = nid
            return best_id

        # Ưu tiên cùng lap; nếu không có thì mới xét tất cả node còn lại
        nearest_same = nearest_from(same_lap_ids)
        if nearest_same is not None:
            return nearest_same

        return nearest_from(other_ids)

        
    def path_exists(self, start: Tuple[int,int], goal: Tuple[int,int], allow_diagonals: bool = False) -> bool:
        """Kiểm tra tồn tại đường đi (A*), chọn 4-hướng hoặc 8-hướng tùy allow_diagonals."""
        if not self.is_valid_position(start) or not self.is_valid_position(goal):
            return False

        directions = DIR8 if allow_diagonals else DIR4
        open_set = [(0, start)]
        closed = set()
        g = {start: 0}

        # heuristic: Manhattan cho 4 hướng, Euclid cho 8 hướng
        def h_cost(a,b):
            if allow_diagonals:
                return math.hypot(a[0]-b[0], a[1]-b[1])
            else:
                return abs(a[0]-b[0]) + abs(a[1]-b[1])

        while open_set:
            _, cur = heapq.heappop(open_set)
            if cur == goal:
                return True
            if cur in closed:
                continue
            closed.add(cur)
            for dr, dc in directions:
                nxt = (cur[0]+dr, cur[1]+dc)
                if not self.is_valid_position(nxt) or nxt in closed:
                    continue
                # cost bước: 1 cho thẳng, sqrt(2) cho chéo
                step = 1.0 if (dr == 0 or dc == 0) else math.sqrt(2)
                tentative = g[cur] + step
                if nxt not in g or tentative < g[nxt]:
                    g[nxt] = tentative
                    f = tentative + h_cost(nxt, goal)
                    heapq.heappush(open_set, (f, nxt))
        return False
        
    def find_path_to_node(self, start: Tuple[int,int], goal_node_id: int) -> List[Tuple[int,int]]:
        goal = self.rcg.nodes[goal_node_id].position
        allow_diag = (goal in self.covered_cells)  # backtracking thì cho chéo
        return self.a_star_path(start, goal, allow_diagonals=allow_diag)

    
    def a_star_path(self, start: Tuple[int,int], goal: Tuple[int,int], allow_diagonals: bool = False) -> List[Tuple[int,int]]:
        """A* trả về đường đi; chỉ 4 hướng (mặc định). Cho chéo khi backtracking."""
        if not self.is_valid_position(start) or not self.is_valid_position(goal):
            return []

        directions = DIR8 if allow_diagonals else DIR4
        open_set = [(0, start)]
        closed = set()
        came = {}
        g = {start: 0}

        def h_cost(a,b):
            if allow_diagonals:
                return math.hypot(a[0]-b[0], a[1]-b[1])
            else:
                return abs(a[0]-b[0]) + abs(a[1]-b[1])

        while open_set:
            _, cur = heapq.heappop(open_set)
            if cur == goal:
                path = [cur]
                while cur in came:
                    cur = came[cur]
                    path.append(cur)
                return path[::-1]
            if cur in closed:
                continue
            closed.add(cur)
            for dr, dc in directions:
                nxt = (cur[0]+dr, cur[1]+dc)
                if not self.is_valid_position(nxt) or nxt in closed:
                    continue
                step = 1.0 if (dr == 0 or dc == 0) else math.sqrt(2)
                tentative = g[cur] + step
                if nxt not in g or tentative < g[nxt]:
                    came[nxt] = cur
                    g[nxt] = tentative
                    f = tentative + h_cost(nxt, goal)
                    heapq.heappush(open_set, (f, nxt))
        return []
    
    def is_coverage_complete(self) -> bool:
        """Kiểm tra xem bản đồ đã được bao phủ đủ hay chưa"""
        total_free_cells = 0
        covered_free_cells = 0
        
        # Đếm số ô tự do và số ô đã bao phủ
        for r in range(self.rows):
            for c in range(self.cols):
                if (r, c) not in self.obstacles:
                    total_free_cells += 1
                    if (r, c) in self.covered_cells:
                        covered_free_cells += 1
        
        if total_free_cells == 0:
            return True  # không có vùng tự do nào
        
        coverage_ratio = covered_free_cells / total_free_cells
        
        # Ngưỡng dừng động (adaptive threshold):
        # Càng nhiều chướng ngại vật → ngưỡng dừng càng thấp.
        obstacle_ratio = len(self.obstacles) / (self.rows * self.cols)
        
        if obstacle_ratio > 0.3:      # Môi trường dày đặc vật cản
            threshold = 0.90
        elif obstacle_ratio > 0.15:   # Môi trường trung bình
            threshold = 0.95
        else:                         # Môi trường thoáng
            threshold = 0.98
        
        # Nếu tỷ lệ bao phủ >= ngưỡng → hoàn tất
        return coverage_ratio >= threshold

    
    def expand_rcg(self, max_iterations: int = 100) -> Optional[int]:
        """Mở rộng RCG theo phong cách RRT, ưu tiên (bias) về vùng chưa bao phủ"""
        for iteration in range(max_iterations):
            # 80% thời gian: lấy mẫu gần các ô chưa bao phủ; 20%: hoàn toàn ngẫu nhiên
            if random.random() < 0.8:
                q_rand = self.sample_near_uncovered_areas()
            else:
                q_rand = self.sample_random_configuration()
            
            # Tìm node gần nhất trong RCG so với vị trí vừa lấy mẫu
            nearest_id = self.find_nearest_node(q_rand)
            
            if nearest_id is None:
                # Trường hợp RCG chưa có node nào → thêm node đầu tiên tại vị trí robot hiện tại
                node_id = self.rcg.add_node(self.current_pos)
                self.total_nodes_generated += 1
                return node_id
            
            nearest_node = self.rcg.nodes[nearest_id]
            
            # Bước "tiến dần" từ node gần nhất về phía điểm q_rand (không nhảy thẳng)
            direction = (q_rand[0] - nearest_node.position[0], 
                         q_rand[1] - nearest_node.position[1])
            distance = math.sqrt(direction[0]**2 + direction[1]**2)
            
            if distance > 0:
                # Đặt kích thước bước hợp lý (thường không vượt quá bán kính cảm biến)
                step_size = min(self.sensing_radius, distance)
                nx = nearest_node.position[0] + max(1, round((direction[0]/distance)*step_size))
                ny = nearest_node.position[1] + max(1, round((direction[1]/distance)*step_size))
                new_pos = snap_to_grid((nx, ny), grid_step=self.sensing_radius)

                
                # Loại bỏ nếu vị trí mới là obstacle hoặc ra ngoài bản đồ
                if not self.is_valid_position(new_pos):
                    continue
                    
                # Kiểm tra có đường đi an toàn (A*) từ node gần nhất → vị trí mới không
                if self.path_exists(nearest_node.position, new_pos, allow_diagonals=False):
                    # Thêm node mới vào RCG
                    new_node_id = self.rcg.add_node(new_pos)
                    self.total_nodes_generated += 1
                    
                    # Chỉ chấp nhận nếu node này có thể bao phủ thêm các ô mới
                    new_coverage = self.rcg.get_coverage_area(new_pos)
                    can_cover_new = any(
                        cell not in self.covered_cells and cell not in self.obstacles
                        for cell in new_coverage
                    )
                    
                    if can_cover_new:
                        w = self.sensing_radius
                        self.rcg.rewire_as_cstar(new_node_id, sampling_step=w, lap_dir='vertical')
                        self.rcg.prune(covered_cells=self.covered_cells, obstacles=self.obstacles,
                                    lap_dir='vertical', w=w)
                        return new_node_id

        
        # Không mở rộng thêm được node "có ích"
        return None
    
    def sample_near_uncovered_areas(self) -> Tuple[int, int]:
        """Lấy mẫu vị trí ngẫu nhiên nhưng có thiên hướng gần các ô chưa bao phủ"""
        # Gom tất cả các ô: chưa bao phủ & không phải chướng ngại vật
        uncovered_cells = []
        for r in range(self.rows):
            for c in range(self.cols):
                if ((r, c) not in self.covered_cells and 
                    (r, c) not in self.obstacles):
                    uncovered_cells.append((r, c))
        
        if uncovered_cells:
            # Chọn ngẫu nhiên một ô mục tiêu chưa bao phủ
            target_cell = random.choice(uncovered_cells)
            # Thêm nhiễu nhỏ quanh ô mục tiêu để không bám cứng vào một ô duy nhất
            noise_r = random.randint(-self.sensing_radius, self.sensing_radius)
            noise_c = random.randint(-self.sensing_radius, self.sensing_radius)
            sample_pos = (
                max(0, min(self.rows - 1, target_cell[0] + noise_r)),
                max(0, min(self.cols - 1, target_cell[1] + noise_c))
            )
            # Trả về mẫu nếu hợp lệ
            if self.is_valid_position(sample_pos):
                return sample_pos
        
        # Nếu không có ô nào chưa bao phủ (hoặc không tìm được vị trí hợp lệ) → rơi về ngẫu nhiên thuần
        return self.sample_random_configuration()
    
    def select_best_frontier_node(self, frontier_nodes: List[int]) -> Optional[int]:
        """Chọn frontier tốt nhất khi coverage = ô robot sẽ đi qua (1 ô/node)."""
        if not frontier_nodes:
            return None
        
        best_node_id = None
        best_score = -float('inf')
        
        for node_id in frontier_nodes:
            node = self.rcg.nodes[node_id]

            # Lợi ích bao phủ: 1 nếu ô node chưa đi qua, ngược lại 0
            cell_uncovered = (
                node.position not in self.covered_cells and
                node.position not in self.obstacles and
                0 <= node.position[0] < self.rows and
                0 <= node.position[1] < self.cols
            )
            new_coverage = 1 if cell_uncovered else 0
            
            # Chi phí đường đi (A*)
            path_to_node = self.find_path_to_node(self.current_pos, node_id)
            path_cost = len(path_to_node) if path_to_node else float('inf')
            
            # Kết nối + khám phá (giữ nguyên)
            connectivity_bonus = len(node.neighbors) * 0.1
            distance = math.sqrt(
                (self.current_pos[0] - node.position[0])**2 +
                (self.current_pos[1] - node.position[1])**2
            )
            exploration_bonus = min(distance * 0.05, 2.0)

            if path_cost > 0 and path_cost != float('inf'):
                coverage_score   = new_coverage * 0.6               # 0 hoặc 0.6
                efficiency_score = (new_coverage / path_cost) * 0.3  # 0 nếu đã covered
                exploration_score = exploration_bonus * 0.1
                score = coverage_score + efficiency_score + exploration_score + connectivity_bonus
            else:
                score = -1
            
            if score > best_score:
                best_score = score
                best_node_id = node_id
        
        return best_node_id

        # ====== SENSOR-LOCAL FRONTIER DETECTION & TARGETING ======

    def detect_frontier_cells_from_sensor(self, pos: Tuple[int,int]) -> List[Tuple[int,int]]:
        """Tìm các ô frontier trong tầm cảm biến (loại ô hiện tại, obstacle, đã cover)."""
        r0, c0 = pos
        cells = []
        for dr in range(-self.sensing_radius, self.sensing_radius + 1):
            for dc in range(-self.sensing_radius, self.sensing_radius + 1):
                nr, nc = r0 + dr, c0 + dc
                if not (0 <= nr < self.rows and 0 <= nc < self.cols):
                    continue
                if math.sqrt(dr*dr + dc*dc) > self.sensing_radius:
                    continue
                cell = (nr, nc)
                if cell == pos or cell in self.obstacles or cell in self.covered_cells:
                    continue
                if self.is_frontier_cell(cell, use_8_adj=False):
                    cells.append(cell)
        # ưu tiên gần trước
        cells.sort(key=lambda p: abs(p[0]-r0) + abs(p[1]-c0))
        return cells

    def select_best_frontier_cell(self, frontier_cells: List[Tuple[int,int]]) -> Optional[Tuple[int,int]]:
        """Chọn ô frontier có A* 4-hướng ngắn nhất."""
        best_cell, best_cost = None, float('inf')
        for cell in frontier_cells:
            path = self.a_star_path(self.current_pos, cell, allow_diagonals=False)
            if path:
                cost = len(path)
                if cost < best_cost:
                    best_cost = cost
                    best_cell = cell
        return best_cell


    def run(self) -> Tuple[List[Tuple[int, int]], Dict]:
        """Chạy vòng lặp chính của thuật toán C*"""
        print("--- Starting C* Algorithm ---")
        iteration = 0
        max_iterations = 1000

        # Bao phủ & quan sát tại vị trí khởi đầu
        self.cover_cells(self.current_pos)             # chỉ ô robot đi qua
        self.update_observed(self.current_pos)         # cập nhật ô đã quan sát

        while not self.is_coverage_complete() and iteration < max_iterations:
            iteration += 1
            print(f"\n--- Iteration {iteration} ---")

            # Cập nhật trực quan nếu có GUI
            if self.on_step_callback:
                self.on_step_callback(self.grid, self.current_pos, self.path, iteration)

            # ===== 1) ƯU TIÊN: đi tới ô frontier mà SENSOR đang thấy (4 hướng) =====
            local_frontiers = self.detect_frontier_cells_from_sensor(self.current_pos)
            # bỏ ô hiện tại & ô đã cover
            local_frontiers = [c for c in local_frontiers
                            if c != self.current_pos and c not in self.covered_cells]

            target_cell = self.select_best_frontier_cell(local_frontiers)
            if target_cell is not None:
                path_to_cell = self.a_star_path(self.current_pos, target_cell, allow_diagonals=False)
                if path_to_cell and len(path_to_cell) > 1:
                    for pos in path_to_cell[1:]:
                        self.current_pos = pos
                        self.path.append(pos)
                        self.cover_cells(pos)
                        self.update_observed(self.current_pos)
                        self.total_path_length += 1
                        if self.on_step_callback:
                            self.on_step_callback(self.grid, self.current_pos, self.path, iteration)

                    covered_count = len(self.covered_cells)
                    total_free = sum(1 for r in range(self.rows) for c in range(self.cols)
                                    if (r, c) not in self.obstacles)
                    print(f"[Local] Coverage: {covered_count}/{total_free} ({covered_count/total_free*100:.1f}%)")
                    # Ưu tiên local xong thì sang vòng lặp kế tiếp
                    if self.on_rcg_update_callback:
                        self.on_rcg_update_callback(self.rcg)
                    continue

            # ===== 2) RCG FRONTIER (toàn cục) =====
            frontier_nodes = self.rcg.find_frontier_nodes(
                self.covered_cells, self.obstacles, self.observed_cells, use_8_adj=False
            )
            print(f"Found {len(frontier_nodes)} frontier nodes")

            # Lọc bỏ frontier không hợp lệ: vị trí hiện tại, đã cover, hoặc không có đường 4 hướng
            valid_frontiers = []
            for nid in frontier_nodes:
                pos = self.rcg.nodes[nid].position
                if pos == self.current_pos:
                    continue
                if pos in self.covered_cells:
                    continue
                path_chk = self.a_star_path(self.current_pos, pos, allow_diagonals=False)
                if path_chk and len(path_chk) > 1:
                    valid_frontiers.append(nid)

            frontier_nodes = valid_frontiers

            # Nếu không có frontier hợp lệ → cố gắng mở rộng RCG
            if not frontier_nodes:
                print("No valid frontier nodes, expanding RCG...")
                new_node_id = self.expand_rcg()
                if new_node_id is None:
                    print("Expand failed, trying more aggressive sampling...")
                    new_node_id = self.expand_rcg(max_iterations=500)
                if new_node_id is not None:
                    pos = self.rcg.nodes[new_node_id].position
                    # chỉ giữ nếu có đường 4 hướng tới node mới
                    if self.a_star_path(self.current_pos, pos, allow_diagonals=False):
                        frontier_nodes = [new_node_id]
                        print(f"Added new node {new_node_id} at {pos}")
                if not frontier_nodes:
                    # Không thể mở rộng thêm → thoát vòng
                    break

            # Chọn frontier tốt nhất (giữ nguyên hàm chấm điểm của bạn)
            best_node_id = self.select_best_frontier_node(frontier_nodes)
            if best_node_id is None:
                print("No valid frontier node found")
                break

            best_node = self.rcg.nodes[best_node_id]
            print(f"Selected frontier node {best_node_id} at {best_node.position}")

            # Lập kế hoạch đường đi (A*) tới node đó
            # Lưu ý: khám phá → 4 hướng; backtracking (nếu mục tiêu đã cover) → 8 hướng
            allow_diag = (best_node.position in self.covered_cells)
            path_to_node = self.a_star_path(self.current_pos, best_node.position, allow_diagonals=allow_diag)
            if not path_to_node or len(path_to_node) <= 1:
                print(f"No path found to node {best_node_id}")
                continue

            print(f"Path length: {len(path_to_node)}")

            # Thực thi hành trình (mỗi bước cover + observed)
            for pos in path_to_node[1:]:
                self.current_pos = pos
                self.path.append(pos)
                self.cover_cells(pos)
                self.update_observed(self.current_pos)
                self.total_path_length += 1

                if self.on_step_callback:
                    self.on_step_callback(self.grid, self.current_pos, self.path, iteration)

            if self.on_rcg_update_callback:
                self.on_rcg_update_callback(self.rcg)

            # Log tiến độ
            covered_count = len(self.covered_cells)
            total_free = sum(1 for r in range(self.rows) for c in range(self.cols)
                            if (r, c) not in self.obstacles)
            print(f"Coverage: {covered_count}/{total_free} ({covered_count/total_free*100:.1f}%)")

        # ===== 3) PHA QUÉT CUỐI (BOUSTROPHEDON) — nếu còn ô trống =====
        remaining_free = any(
            (r, c) not in self.obstacles and (r, c) not in self.covered_cells
            for r in range(self.rows) for c in range(self.cols)
        )
        if remaining_free:
            print("\n--- Starting final boustrophedon sweep ---")
            self.final_boustrophedon_sweep()

        # ==== TÍNH KẾT QUẢ CUỐI (sau khi sweep nếu có) ====
        final_coverage = len(self.covered_cells)
        total_free_cells = sum(
            1 for r in range(self.rows) for c in range(self.cols)
            if (r, c) not in self.obstacles
        )
        coverage_percentage = (final_coverage / total_free_cells) * 100 if total_free_cells else 100.0

        results = {
            'iterations': iteration,
            'total_path_length': self.total_path_length,
            'coverage_percentage': coverage_percentage,
            'nodes_generated': self.total_nodes_generated,
            'final_coverage': final_coverage,
            'total_free_cells': total_free_cells
        }

        print(f"\n--- C* Algorithm Completed ---")
        print(f"Iterations: {iteration}")
        print(f"Path length: {self.total_path_length}")
        print(f"Coverage: {coverage_percentage:.1f}%")
        print(f"Nodes generated: {self.total_nodes_generated}")

        return self.path, results


    def _row_free_segments(self, r: int) -> List[Tuple[int, int]]:
        """Trả về các đoạn [c_start, c_end] liên tục (free, không obstacle) trên hàng r."""
        segs = []
        c = 0
        while c < self.cols:
            # bỏ qua obstacle
            while c < self.cols and (r, c) in self.obstacles:
                c += 1
            if c >= self.cols:
                break
            start = c
            # đi qua đoạn free liên tục
            while c < self.cols and (r, c) not in self.obstacles:
                c += 1
            end = c - 1
            segs.append((start, end))
        return segs

    def _boustrophedon_order(self) -> List[Tuple[int, int]]:
        """
        Sinh thứ tự quét boustrophedon toàn cục:
        - Duyệt từng hàng.
        - Mỗi hàng chia thành các đoạn free liên tục (không obstacle).
        - Hàng chẵn: trái->phải, hàng lẻ: phải->trái.
        Trả về danh sách các ô theo thứ tự quét.
        """
        path_cells = []
        for r in range(self.rows):
            segs = self._row_free_segments(r)
            if not segs:
                continue
            # gom tất cả cell free theo thứ tự zigzag
            if r % 2 == 0:
                # trái -> phải
                for (c0, c1) in segs:
                    for c in range(c0, c1 + 1):
                        path_cells.append((r, c))
            else:
                # phải -> trái
                for (c0, c1) in reversed(segs):
                    for c in range(c1, c0 - 1, -1):
                        path_cells.append((r, c))
        return path_cells

    def _nearest_index(self, cells: List[Tuple[int, int]], src: Tuple[int, int]) -> int:
        """Tìm chỉ số cell gần 'src' nhất để bắt đầu sweep cho đỡ tốn đường nối."""
        if not cells:
            return 0
        best_i, best_d = 0, float('inf')
        sr, sc = src
        for i, (r, c) in enumerate(cells):
            d = abs(sr - r) + abs(sc - c)  # L1 đủ nhanh và ổn
            if d < best_d:
                best_d, best_i = d, i
        return best_i

    def final_boustrophedon_sweep(self):
        """
        Pha quét cuối: đi theo đường boustrophedon,
        với mỗi cell chưa được cover, dùng A* nối sang rồi cover.
        """
        sweep_cells = self._boustrophedon_order()
        if not sweep_cells:
            return

        # Bắt đầu quét từ ô gần nhất vị trí hiện tại để giảm đường nối
        start_i = self._nearest_index(sweep_cells, self.current_pos)
        ordered = sweep_cells[start_i:] + sweep_cells[:start_i]

        for target in ordered:
            # Bỏ qua nếu ô này đã được cover bởi bán kính cảm biến trước đó
            if target in self.covered_cells:
                continue

            # Tìm đường đi A* tới ô target
            path = self.a_star_path(self.current_pos, target, allow_diagonals=False)
            if not path:
                continue  # có thể bị chắn kín – bỏ qua

            # Thực thi di chuyển và cover dọc đường
            for pos in path[1:]:
                if not self.is_valid_position(pos):
                    break
                self.current_pos = pos
                self.path.append(pos)
                self.cover_cells(pos)
                self.update_observed(self.current_pos)
                self.total_path_length += 1
                if self.on_step_callback:
                    # Giữ cho GUI hiển thị sống động trong pha quét cuối
                    self.on_step_callback(self.grid, self.current_pos, self.path, -1)  # -1: phase sweep

        # Quét xong: gọi cập nhật RCG nếu cần vẽ lại
        if self.on_rcg_update_callback:
            self.on_rcg_update_callback(self.rcg)
# ====================== PHẦN DÙNG THỬ (CONSOLE) ======================

def create_test_environment(size: int = 30) -> List[List[int]]:
    """Tạo môi trường kiểm thử đơn giản với vài obstacle hình chữ nhật"""
    grid = [[FREE_UNCOVERED for _ in range(size)] for _ in range(size)]
    
    # Định nghĩa một số obstacle theo (r1, c1, r2, c2)
    obstacles = [
        (5, 5, 8, 8),
        (15, 20, 18, 25),
        (25, 10, 28, 15),
        (10, 25, 15, 28)
    ]
    
    # Đánh dấu các ô obstacle
    for start_r, start_c, end_r, end_c in obstacles:
        for r in range(start_r, min(end_r + 1, size)):
            for c in range(start_c, min(end_c + 1, size)):
                if 0 <= r < size and 0 <= c < size:
                    grid[r][c] = OBSTACLE
    
    return grid

def run_console_test():
    """Chạy C* ở chế độ console (không GUI) để kiểm chứng nhanh"""
    print("=== C* Algorithm Console Test ===")
    
    # Tạo bản đồ 20x20 có chướng ngại vật
    grid = create_test_environment(20)
    start_pos = (0, 0)
    
    # Khởi tạo C*
    c_star = CStar(grid, start_pos, sensing_radius=2)
    
    # Chạy thuật toán
    path, results = c_star.run()
    
    # In kết quả
    print(f"\nFinal Results:")
    print(f"Path length: {len(path)}")  # số bước đi (bao gồm cả lặp lại ô)
    print(f"Coverage: {results['coverage_percentage']:.1f}%")
    print(f"RCG nodes: {results['nodes_generated']}")

if __name__ == "__main__":
    run_console_test()
