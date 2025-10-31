import math
from collections import defaultdict
from typing import Dict, Set, Tuple, List, Optional
from .node import Node
from .constants import DIR4, DIR8


class RapidlyCoveringGraph:
    """Rapidly Covering Graph (RCG) for C* algorithm"""

    def __init__(self, grid_size: Tuple[int, int], sensing_radius: int = 2):
        # Kích thước bản đồ (số hàng, số cột)
        self.grid_size = grid_size
        self.sensing_radius = sensing_radius  # Bán kính cảm biến của robot
        self.nodes = {}                       # Từ điển chứa các node: {id: Node}
        # Đếm số node đã tạo (dùng để cấp ID)
        self.node_counter = 0
        # Ánh xạ ô (r,c) → tập các node ở vị trí đó
        self.grid_to_nodes = defaultdict(set)

    # RapidlyCoveringGraph.add_node

    # rcg.py
    def add_node(self, position: Tuple[int, int],
                 sampling_step: Optional[int] = None,
                 lap_dir: str = 'vertical',
                 obstacles: Optional[Set[Tuple[int, int]]] = None) -> int:
        w = sampling_step if sampling_step is not None else max(
            1, self.sensing_radius)

        # Nếu đã có node tại vị trí này → vẫn rewire lại (để cập nhật cạnh)
        if position in self.grid_to_nodes and self.grid_to_nodes[position]:
            existing = max(self.grid_to_nodes[position])
            self.rewire_as_cstar(existing, sampling_step=w,
                                 lap_dir=lap_dir, obstacles=obstacles or set())
            return existing

        node_id = self.node_counter
        self.node_counter += 1
        node = Node(position, node_id)
        node.coverage_set = self.get_coverage_area(position)
        node.state = "Op"
        self.nodes[node_id] = node
        self.grid_to_nodes[position].add(node_id)

        self.rewire_as_cstar(node_id, sampling_step=w,
                             lap_dir=lap_dir, obstacles=obstacles or set())
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
        if w is None or w <= 0:
            w = max(1, self.sensing_radius)
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

    def _los_free(self, a: Tuple[int, int], b: Tuple[int, int], obstacles: Set[Tuple[int, int]]) -> bool:
        """Bresenham LOS check: True nếu đoạn ab không cắt obstacle."""
        (r1, c1), (r2, c2) = a, b
        dr = abs(r2 - r1)
        dc = abs(c2 - c1)
        sr = 1 if r1 < r2 else -1
        sc = 1 if c1 < c2 else -1
        err = dr - dc
        r, c = r1, c1
        while True:
            if (r, c) in obstacles:
                return False
            if (r, c) == (r2, c2):
                break
            e2 = 2*err
            if e2 > -dc:
                err -= dc
                r += sr
            if e2 < dr:
                err += dr
                c += sc
        return True

    def rewire_as_cstar(self, new_node_id: int, sampling_step: int, lap_dir: str = 'vertical',
                        obstacles: Optional[Set[Tuple[int, int]]] = None):
        import math
        w = sampling_step
        node = self.nodes[new_node_id]
        pos = node.position
        obstacles = obstacles or set()

        lap_of = {nid: self._lap_index(n.position, lap_dir, w)
                  for nid, n in self.nodes.items()}
        my_lap = lap_of[new_node_id]

        # clear current edges of the new node
        for nb in list(node.neighbors):
            self._remove_edge(new_node_id, nb)

        # ---- (1) Kết nối trong CÙNG LAP: lấy 1 nút phía 'trước' & 1 nút phía 'sau' theo trục quét
        same_ids = [nid for nid in self.nodes if lap_of[nid]
                    == my_lap and nid != new_node_id]

        def pick_up_down():
            up, down = None, None
            r0, c0 = pos
            # ưu tiên cùng cột (vertical) hoặc cùng hàng (horizontal)
            if lap_dir == 'vertical':
                ups = [nid for nid in same_ids if self.nodes[nid].position[1]
                       == c0 and self.nodes[nid].position[0] < r0]
                downs = [nid for nid in same_ids if self.nodes[nid].position[1]
                         == c0 and self.nodes[nid].position[0] > r0]
                if ups:
                    up = min(ups,   key=lambda i: abs(
                        self.nodes[i].position[0]-r0))
                if downs:
                    down = min(downs, key=lambda i: abs(
                        self.nodes[i].position[0]-r0))
                # nếu không có cùng cột, cho phép lệch 1 ô cột
                if not up:
                    ups = [
                        nid for nid in same_ids if self.nodes[nid].position[0] < r0]
                    if ups:
                        up = min(ups, key=lambda i: (
                            abs(self.nodes[i].position[1]-c0), abs(self.nodes[i].position[0]-r0)))
                if not down:
                    downs = [
                        nid for nid in same_ids if self.nodes[nid].position[0] > r0]
                    if downs:
                        down = min(downs, key=lambda i: (
                            abs(self.nodes[i].position[1]-c0), abs(self.nodes[i].position[0]-r0)))
            else:
                lefts = [nid for nid in same_ids if self.nodes[nid].position[0]
                         == r0 and self.nodes[nid].position[1] < c0]
                rights = [nid for nid in same_ids if self.nodes[nid].position[0]
                          == r0 and self.nodes[nid].position[1] > c0]
                left = min(lefts,  key=lambda i: abs(
                    self.nodes[i].position[1]-c0)) if lefts else None
                right = min(rights, key=lambda i: abs(
                    self.nodes[i].position[1]-c0)) if rights else None
                return left, right
            return up, down

        a, b = pick_up_down()
        if a is not None:
            self._add_edge(new_node_id, a)
        if b is not None:
            self._add_edge(new_node_id, b)

        # ---- (2) Kết nối LAP KỀ: mỗi phía giữ 1 cạnh trong phạm vi √2·w và có LOS
        limit = math.sqrt(2)*w + 1e-6
        for side in (my_lap-1, my_lap+1):
            best = None
            best_d = float('inf')
            for nid, n in self.nodes.items():
                if lap_of[nid] != side or nid == new_node_id:
                    continue
                d = math.hypot(n.position[0]-pos[0], n.position[1]-pos[1])
                if d <= limit and self._los_free(pos, n.position, obstacles):
                    if d < best_d:
                        best_d, best = d, nid
            if best is not None:
                self._add_edge(new_node_id, best)

    def _is_frontier_cell(self, pos: Tuple[int, int],
                          observed_cells: Set[Tuple[int, int]],
                          obstacles: Set[Tuple[int, int]], w: int) -> bool:
        r, c = pos
        touches_unknown = touches_obstacle = touches_boundary = False
        for dr in range(-w, w+1):
            for dc in range(-w, w+1):
                rr, cc = r+dr, c+dc
                if not self._in_bounds(rr, cc):
                    touches_boundary = True
                    continue
                if (rr, cc) in obstacles:
                    touches_obstacle = True
                elif (rr, cc) not in observed_cells:
                    touches_unknown = True
        return touches_unknown or touches_obstacle or touches_boundary
    # rcg.py — thêm vào class

    def _lap_center_coord(self, idx: int, lap_dir: str, w: int) -> int:
        if w <= 0:
            w = max(1, self.sensing_radius)
        if lap_dir == 'vertical':
            return min(self.grid_size[1]-1, max(0, idx*w + w//2))
        else:
            return min(self.grid_size[0]-1, max(0, idx*w + w//2))

    def generate_frontier_samples(
        self,
        observed_cells,
        obstacles,
        newly_discovered,
        w: int,
        lap_dir: str = 'vertical',
        delta: int = 1
    ) -> List[int]:
        # sampling front: vùng mới quan sát, free
        Fi = {cell for cell in newly_discovered
              if cell in observed_cells and cell not in obstacles}
        if not Fi:
            return []

        # === NEW: helper khử "bụi" node gần kề (Poisson-disk 1D theo L1) ===
        def _too_close_to_existing(pos: Tuple[int, int], min_d: int) -> bool:
            r, c = pos
            for nid, n in self.nodes.items():
                rr, cc = n.position
                if abs(rr - r) + abs(cc - c) < min_d:
                    return True
            return False

        # nhóm cells theo lap
        buckets = defaultdict(list)
        for (r, c) in Fi:
            idx = self._lap_index((r, c), lap_dir, w)
            buckets[idx].append((r, c))

        created: List[int] = []
        # === NEW: khoảng cách tối thiểu giữa các sample ===
        min_sep = max(1, w // 2)

        for idx, cells in buckets.items():
            # cột/hàng trung tâm của dải lap idx
            fix = self._lap_center_coord(idx, lap_dir, w)

            if lap_dir == 'vertical':
                cells.sort(key=lambda x: x[0])  # quét theo r
                last_pick: Optional[int] = None
                for (r, _c) in cells:
                    c = fix
                    if not self._in_bounds(r, c):
                        continue

                    # giữ khoảng cách ít nhất delta*w theo hướng quét
                    if (last_pick is not None) and (abs(r - last_pick) < delta * w):
                        continue

                    cand = (r, c)

                    # === NEW: chỉ nhận nếu là frontier + không quá sát node có sẵn + mở thêm unknown
                    if not self._is_frontier_cell(cand, observed_cells, obstacles, w):
                        continue
                    if _too_close_to_existing(cand, min_sep):
                        continue
                    cov = self.get_coverage_area(cand)
                    opens_unknown = any(
                        (cell not in observed_cells) and (cell not in obstacles) for cell in cov
                    )
                    if not opens_unknown:
                        continue

                    nid = self.add_node(cand, sampling_step=w,
                                        lap_dir=lap_dir, obstacles=obstacles)
                    self.nodes[nid].is_frontier = True
                    created.append(nid)
                    last_pick = r

            else:
                cells.sort(key=lambda x: x[1])  # quét theo c
                last_pick: Optional[int] = None
                for (_r, c) in cells:
                    r = fix
                    if not self._in_bounds(r, c):
                        continue

                    if (last_pick is not None) and (abs(c - last_pick) < delta * w):
                        continue

                    cand = (r, c)

                    if not self._is_frontier_cell(cand, observed_cells, obstacles, w):
                        continue
                    if _too_close_to_existing(cand, min_sep):
                        continue
                    cov = self.get_coverage_area(cand)
                    opens_unknown = any(
                        (cell not in observed_cells) and (cell not in obstacles) for cell in cov
                    )
                    if not opens_unknown:
                        continue

                    nid = self.add_node(cand, sampling_step=w,
                                        lap_dir=lap_dir, obstacles=obstacles)
                    self.nodes[nid].is_frontier = True
                    created.append(nid)
                    last_pick = c

        return created

    def prune(
        self,
        covered_cells: Set[Tuple[int, int]],
        obstacles: Set[Tuple[int, int]],
        lap_dir: str = "vertical",
        w: Optional[int] = None,
        k_edge_samples: int = 5,
    ) -> None:
        """
        Cắt tỉa RCG theo spirit của C*:
        - A) Giữ node/cạnh thiết yếu (essential).
        - B) Giảm cạnh giữa các lap kề: với cùng một lap-kề, chỉ giữ 1 cạnh
            "gần obstacle/unknown nhất" (theo khoảng cách nhỏ nhất dọc theo cạnh).
        - C) Gộp node bậc-2 thẳng hàng trong cùng lap (A-X-B) để làm đồ thị thưa.

        Params
        ------
        covered_cells: tập các ô đã cover (để suy ra unknown = free mà chưa cover).
        obstacles: tập các ô obstacle.
        lap_dir: 'vertical' hoặc 'horizontal' (xác định cách tạo laps).
        w: chiều rộng lap (sampling step); mặc định lấy sensing_radius.
        k_edge_samples: số điểm nội suy trên cạnh để đo độ gần obstacle/unknown.
        """
        if w is None:
            w = max(1, self.sensing_radius)

        # ---------- Helper nội bộ ----------
        def is_frontier_like(pos: Tuple[int, int]) -> bool:
            """Node được xem là 'chạm unknown/obstacle/boundary' trong lân cận w."""
            r, c = pos
            touches_unknown = touches_obstacle = touches_boundary = False
            for dr in range(-w, w + 1):
                for dc in range(-w, w + 1):
                    rr, cc = r + dr, c + dc
                    if not self._in_bounds(rr, cc):
                        touches_boundary = True
                        continue
                    if (rr, cc) in obstacles:
                        touches_obstacle = True
                    elif (rr, cc) not in covered_cells:
                        # free & chưa cover → xem như unknown (ở mức planner)
                        touches_unknown = True
            return touches_unknown or touches_obstacle or touches_boundary

        def is_lap_end(nid: int) -> bool:
            """Node ở 'đầu/cuối lap' nếu bước ±w theo phương dọc-lap đi ra ngoài/đụng obstacle."""
            r, c = self.nodes[nid].position
            dirs = [(-w, 0), (w, 0)
                    ] if lap_dir == "vertical" else [(0, -w), (0, w)]
            for dr, dc in dirs:
                rr, cc = r + dr, c + dc
                if not self._in_bounds(rr, cc) or (rr, cc) in obstacles:
                    return True
            return False

        def edge_frontier_closeness(u: int, v: int) -> float:
            """
            Độ 'gần' obstacle/unknown của cạnh (u, v):
            - Lấy k điểm nội suy trên đoạn thẳng, trả về min khoảng cách Manhattan
            tới obstacle hoặc tới cell 'unknown' (free mà chưa cover).
            - Giá trị càng nhỏ → cạnh càng 'thiết yếu' để nối giữa hai lap.
            """
            (r1, c1) = self.nodes[u].position
            (r2, c2) = self.nodes[v].position
            best = float("inf")

            for t in range(k_edge_samples + 2):  # +2 để gồm cả 2 đầu mút
                alpha = t / (k_edge_samples + 1)
                rr = round(r1 + alpha * (r2 - r1))
                cc = round(c1 + alpha * (c2 - c1))
                # nếu ngoài map, coi như rất gần boundary
                if not self._in_bounds(rr, cc):
                    return 0.0
                # khoảng cách tới obstacle
                if obstacles:
                    d_obs = min(abs(rr - ro) + abs(cc - co)
                                for (ro, co) in obstacles)
                else:
                    d_obs = float("inf")
                # khoảng cách tới unknown (free chưa cover)
                # (ta tìm cell free gần nhất không thuộc covered_cells)
                d_unk = float("inf")
                # quét trong vuông nhỏ quanh điểm để tránh O(N)
                radius = min(3, w)  # nhanh/thực dụng
                for dr in range(-radius, radius + 1):
                    for dc in range(-radius, radius + 1):
                        r0, c0 = rr + dr, cc + dc
                        if not self._in_bounds(r0, c0):
                            d_unk = 0
                            continue
                        if (r0, c0) not in obstacles and (r0, c0) not in covered_cells:
                            d_unk = min(d_unk, abs(dr) + abs(dc))
                best = min(best, min(d_obs, d_unk))
            return best

        def same_lap(a: int, b: int) -> bool:
            return lap_of[a] == lap_of[b]

        # ---------- Chuẩn bị ----------
        if not self.nodes:
            return

        lap_of: Dict[int, int] = {
            nid: self._lap_index(n.position, lap_dir=lap_dir, w=w)
            for nid, n in self.nodes.items()
        }

        # ---------- (A) Chọn ESSENTIAL NODES ----------
        essential: Set[int] = set()
        for nid, node in self.nodes.items():
            keep = False

            # A1) chạm unknown/obstacle/boundary
            if is_frontier_like(node.position):
                keep = True

            # A2) là end-node của lap
            if not keep and is_lap_end(nid):
                keep = True

            # A3) (mềm) node nối sang lap kề bằng cạnh 'thiết yếu' sẽ được giữ ở bước (B)
            # => chưa đánh dấu ở đây, để (B) quyết cạnh trước, sau đó thêm node endpoint của cạnh giữ lại.

            if keep:
                essential.add(nid)

        # ---------- (B) GIẢM CẠNH GIỮA LAP KỀ ----------
        # Với mỗi node, nhóm các neighbor theo lap kề → nếu >1 cạnh tới cùng một lap,
        # giữ cạnh có edge_frontier_closeness nhỏ nhất, xóa phần còn lại.
        to_remove_edges: Set[Tuple[int, int]] = set()
        to_keep_edges: Set[Tuple[int, int]] = set()

        for u, node in self.nodes.items():
            # nhóm neighbor khác-lap theo lap_id
            groups: Dict[int, List[int]] = {}
            for v in node.neighbors:
                if same_lap(u, v):
                    continue
                groups.setdefault(lap_of[v], []).append(v)

            for _, lst in groups.items():
                if len(lst) == 1:
                    v = lst[0]
                    edge = tuple(sorted((u, v)))
                    to_keep_edges.add(edge)
                    continue

                # Chọn 1 cạnh 'gần obstacle/unknown' nhất
                scored = []
                for v in lst:
                    edge_score = edge_frontier_closeness(u, v)
                    scored.append((edge_score, v))
                scored.sort(key=lambda x: x[0])

                # giữ cạnh tốt nhất
                best_v = scored[0][1]
                to_keep_edges.add(tuple(sorted((u, best_v))))

                # các cạnh còn lại → xóa
                for _, v in scored[1:]:
                    to_remove_edges.add(tuple(sorted((u, v))))

        # Thực thi xóa cạnh giữa các lap kề theo lựa chọn trên
        for (a, b) in to_remove_edges:
            self._remove_edge(a, b)

        # Sau khi quyết cạnh, thêm endpoint của các cạnh được giữ vào essential
        for (a, b) in to_keep_edges:
            essential.add(a)
            essential.add(b)

        # ---------- (C) GỘP NODE BẬC-2 CÙNG LAP (A–X–B) ----------
        merged_add: Set[Tuple[int, int]] = set()
        merged_del: Set[Tuple[int, int]] = set()
        delete_nodes: Set[int] = set()

        # Duyệt snapshot vì sẽ chỉnh sửa self.nodes
        for x, node in list(self.nodes.items()):
            if x in essential:
                continue
            # chỉ gộp node bậc-2
            if len(node.neighbors) != 2:
                continue
            a, b = list(node.neighbors)
            # yêu cầu cùng lap
            if not (same_lap(x, a) and same_lap(x, b)):
                continue

            # (mềm) kiểm tra 'thẳng hàng' theo lap_dir:
            ra, ca = self.nodes[a].position
            rx, cx = node.position
            rb, cb = self.nodes[b].position
            collinear = (
                (lap_dir == "vertical" and ca == cx == cb) or
                (lap_dir == "horizontal" and ra == rx == rb)
            )
            if not collinear:
                # Nếu không hoàn toàn thẳng hàng, vẫn có thể gộp để thưa graph (tuỳ chọn).
                # Ở đây ta yêu cầu collinear để an toàn.
                continue

            # Đánh dấu gộp: xóa (a-x) & (x-b), thêm (a-b), xóa node x
            merged_del.add(tuple(sorted((a, x))))
            merged_del.add(tuple(sorted((x, b))))
            merged_add.add(tuple(sorted((a, b))))
            delete_nodes.add(x)

        # Thực thi gộp node
        for (u, v) in merged_del:
            self._remove_edge(u, v)
        for (u, v) in merged_add:
            self._add_edge(u, v)

        for nid in delete_nodes:
            # dọn map phụ
            pos = self.nodes[nid].position
            for nb in list(self.nodes[nid].neighbors):
                self._remove_edge(nid, nb)
            self.grid_to_nodes[pos].discard(nid)
            if not self.grid_to_nodes[pos]:
                self.grid_to_nodes.pop(pos, None)
            self.nodes.pop(nid, None)

        # ---------- (D) (tuỳ chọn) Xóa node cô lập không essential ----------
        isolated: List[int] = []
        for nid, node in self.nodes.items():
            if nid in essential:
                continue
            if not node.neighbors:
                isolated.append(nid)
        for nid in isolated:
            pos = self.nodes[nid].position
            self.grid_to_nodes[pos].discard(nid)
            if not self.grid_to_nodes[pos]:
                self.grid_to_nodes.pop(pos, None)
            self.nodes.pop(nid, None)
