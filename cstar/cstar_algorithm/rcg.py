import math
from collections import defaultdict
from typing import Dict, Set, Tuple, List, Optional
from .node import Node
from .constants import DIR4, DIR8

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
