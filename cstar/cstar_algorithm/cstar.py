import heapq
import math
import random
from typing import List, Tuple, Set, Dict, Optional
from collections import deque
from .constants import FREE_UNCOVERED, OBSTACLE, COVERED, FRONTIER, DIR4, DIR8
from .rcg import RapidlyCoveringGraph
from .utils import snap_to_grid


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
        self.prev_observed: Set[Tuple[int, int]] = set()

        # Callback cho visualization (nếu có)
        self.on_step_callback = None
        self.on_rcg_update_callback = None
        self.on_path_update_callback = None

        # Khởi tạo danh sách chướng ngại vật
        self.initialize_obstacles()

        # Bộ đếm thống kê
        self.total_nodes_generated = 0
        self.total_path_length = 0
        self.lap_dir = 'vertical'
        self.w = max(1, sensing_radius)
        self.sweep_sign = -1  # bắt đầu quét lên;

        self.next_shift_left = True  # lần đầu ưu tiên dịch sang luống "trái"

        self.sealed_endcaps: Set[Tuple[Tuple[int, int], int]
                                 ] = set()  # (position, sweep_sign)

        # (from_lap, to_lap, iter_when, covered_before, pathlen_before)
        self.last_shift = None
        self.shift_oscillations = 0
        # {(lapA, lapB)} với lapA<lapB
        self.banned_corridors: Set[Tuple[int, int]] = set()
        self.forward_since_shift = 0
        self.min_forward_after_shift = max(2, self.w)  # hysteresis tối thiểu

    def _ban_edge(self, a: int, b: int):
        if not hasattr(self, "_banned_edges"):
            self._banned_edges: Set[Tuple[int, int]] = set()
        self._banned_edges.add(tuple(sorted((a, b))))

    def _edge_banned(self, a: int, b: int) -> bool:
        return hasattr(self, "_banned_edges") and (tuple(sorted((a, b))) in self._banned_edges)

    def _lap_pair(self, a: int, b: int) -> Tuple[int, int]:
        return (a, b) if a < b else (b, a)

    def _diag_ok(self, cur: Tuple[int, int], nxt: Tuple[int, int]) -> bool:
        """Cho phép bước chéo chỉ khi không cắt hai ô góc (chống lách góc)."""
        dr = nxt[0] - cur[0]
        dc = nxt[1] - cur[1]
        if abs(dr) + abs(dc) != 2:  # không phải bước chéo
            return True
        a = (cur[0] + dr, cur[1])      # ô theo hàng
        b = (cur[0], cur[1] + dc)      # ô theo cột
        return (a not in self.obstacles) and (b not in self.obstacles)

    def _lap_index_of(self, pos: Tuple[int, int]) -> int:
        return self.rcg._lap_index(pos, lap_dir=self.lap_dir, w=self.w)

    def _next_cell_on_lap(self, pos: Tuple[int, int]) -> Optional[Tuple[int, int]]:
        r, c = pos
        if self.lap_dir == 'vertical':
            cand = (r + self.sweep_sign, c)
        else:
            cand = (r, c + self.sweep_sign)
        return cand if self.is_valid_position(cand) else None

    def _shift_to_adjacent_lap(self, left_first: bool = True) -> Optional[Tuple[int, int]]:
        """Dịch ngang sang luống kề ±w mà KHÔNG dùng A*."""
        r, c = self.current_pos
        w = self.w
        opts = ([-w, +w] if left_first else [+w, -w])
        for delta in opts:
            if self.lap_dir == 'vertical':
                cand = (r, c + delta)
            else:
                cand = (r + delta, c)
            if self.is_valid_position(cand):
                return cand
        return None

    from collections import deque

    def _cover_step(self, nxt: Tuple[int, int], iter_tag: int):
        """
        Thực hiện 1 bước (1 ô) theo hướng quét hiện tại.
        - Cập nhật vị trí, path, COVERED, observed, sampling-front.
        - Cộng tổng chiều dài đường đi.
        - Đếm 'tiến thẳng sau shift' để áp dụng hysteresis chống dao động.
        - Callback cập nhật frame nếu có.
        """
        prev = self.current_pos
        self.current_pos = nxt
        self.path.append(nxt)

        # Cập nhật cover/observe + sampling
        self.cover_cells(nxt)
        self.update_observed(nxt)
        self._sampling_front_step(lap_dir=self.lap_dir)

        # Cập nhật tổng độ dài đường đi
        self.total_path_length += 1
        if prev is not None:
            if self.lap_dir == 'vertical':
                self.forward_since_shift += 1 if (
                    self.current_pos[1] == prev[1] and self.current_pos[0] != prev[0]) else 0
            else:
                self.forward_since_shift += 1 if (
                    self.current_pos[0] == prev[0] and self.current_pos[1] != prev[1]) else 0

        # Callback vẽ bước
        if self.on_step_callback:
            self.on_step_callback(
                self.grid, self.current_pos, self.path, iter_tag)

    def _advance_along_lap_until_block(self, max_steps: int = 999, iter_tag: int = -2) -> int:
        """
        Đi thẳng theo luống hiện tại (boustrophedon) cho đến khi:
        - đụng biên/obstacle, hoặc
        - phát hiện dao động ABAB, hoặc
        - 'stall' (không tăng cover đủ lâu), hoặc
        - vượt quá max_steps.
        Trả về: số bước đã đi.
        """
        steps = 0
        recent = deque(maxlen=8)   # phát hiện ABAB
        stall_window = 10          # số bước cho phép không tăng cover
        stall_left = stall_window
        covered_prev = len(self.covered_cells)

        while steps < max_steps:
            prev_pos = self.current_pos
            prev_lap = self._lap_index_of(prev_pos)

            nxt = self._next_cell_on_lap(prev_pos)
            if nxt is None or not self.is_valid_position(nxt):
                break

            # Thực hiện 1 bước trên luống
            self._cover_step(nxt, iter_tag=iter_tag)
            steps += 1

            # Nếu vô tình sang luống khác (hiếm), đảo chiều để giữ BM
            new_lap = self._lap_index_of(self.current_pos)
            if new_lap != prev_lap:
                self.sweep_sign *= -1

            # (1) Phát hiện mẫu ABAB: p[-1]==p[-3] & p[-2]==p[-4]
            recent.append(self.current_pos)
            if len(recent) >= 4 and recent[-1] == recent[-3] and recent[-2] == recent[-4]:
                break

            # (2) Phát hiện 'stall' (không tăng coverage trong 1 cửa sổ)
            covered_now = len(self.covered_cells)
            if covered_now > covered_prev:
                stall_left = stall_window
                covered_prev = covered_now
            else:
                stall_left -= 1
                if stall_left <= 0:
                    break

        # === Anti-rectangle (chống lặp hình chữ nhật giữa 2 luống kề) ===
        # Ý tưởng: nếu vừa shift qua hành lang (from_lap -> to_lap) rồi lại quay về gần như ngay,
        # mà không tăng cover đáng kể → 'cấm' hành lang đó tạm thời để buộc backtrack/đổi hướng.
        if self.last_shift and steps > 0:
            from_lap, to_lap, pathlen_when, covered_before, pathlen_before = self.last_shift
            cur_lap = self._lap_index_of(self.current_pos)

            def _lap_pair(a: int, b: int) -> Tuple[int, int]:
                return (a, b) if a < b else (b, a)

            # Quay lại 'hành lang' cũ? (to_lap -> from_lap)
            if _lap_pair(from_lap, to_lap) == _lap_pair(to_lap, cur_lap):
                gained_cover = len(self.covered_cells) - covered_before
                advanced = self.total_path_length - pathlen_before

                # Ngưỡng 'không tiến triển': phủ thêm < w/2 và tổng bước < 3w
                if gained_cover < max(1, self.w // 2) and advanced < 3 * self.w:
                    corridor = _lap_pair(from_lap, to_lap)
                    # Khởi tạo set nếu chưa có (phòng trường hợp quên khai báo)
                    if not hasattr(self, "banned_corridors"):
                        self.banned_corridors = set()
                    if not hasattr(self, "sealed_endcaps"):
                        self.sealed_endcaps = set()

                    self.banned_corridors.add(corridor)
                    # Ép niêm phong 'đầu mút' hiện tại để vòng sau không tiếp tục lặp
                    self.sealed_endcaps.add(
                        (self.current_pos, self.sweep_sign))
                    # Đảo chiều nhẹ để thoát pocket
                    self.sweep_sign *= -1
                    # Reset dấu vết shift để không trigger nhiều lần
                    self.last_shift = None

        return steps

    def _pick_backtrack_seed(self) -> Optional[Tuple[int, int]]:
        """
        Chọn 'seed' để backtrack (đích của A*) theo ưu tiên:
        1) Frontier node RCG cùng luống hiện tại (ưu tiên gần nhất theo L1).
        2) Frontier node bất kỳ (gần nhất theo L1).
        3) Nếu không có → cố mở rộng RCG rồi lấy node mới.
        """
        w = self.w
        lap_here = self._lap_index_of(self.current_pos)
        frontier = self.rcg.find_frontier_nodes(
            self.covered_cells, self.obstacles, self.observed_cells, use_8_adj=False
        )

        def l1(p, q): return abs(p[0]-q[0]) + abs(p[1]-q[1])

        same_lap = [nid for nid in frontier
                    if self.rcg._lap_index(self.rcg.nodes[nid].position, self.lap_dir, w) == lap_here]
        pool = same_lap or frontier
        if pool:
            pool.sort(key=lambda nid: l1(
                self.current_pos, self.rcg.nodes[nid].position))
            return self.rcg.nodes[pool[0]].position

        # Không có frontier → cố gắng mở rộng RCG quanh vùng chưa cover
        new_id = self.alg1_expand_rcg(max_iterations=200, lap_dir=self.lap_dir)
        if new_id is not None:
            return self.rcg.nodes[new_id].position
        return None

    def _backtrack_once(self, iter_tag: int) -> bool:
        """
        CHỈ ở đây mới dùng A*: quay về 'seed' rồi tiếp tục lane-scan.
        Trả True nếu backtrack đi được ít nhất 1 bước.
        """
        seed = self._pick_backtrack_seed()
        if seed is None or seed == self.current_pos:
            return False
        path = self.a_star_path(
            self.current_pos, seed, allow_diagonals=False,
            prefer_axis=self.lap_dir, prefer_sign=self.sweep_sign
        )
        if not path or len(path) <= 1:
            return False
        # thực thi đường A* (backtrack)
        for pos in path[1:]:
            # ví dụ iter_tag=-3 để phân biệt backtrack
            self._cover_step(pos, iter_tag=iter_tag)
        return True

    def _try_shift_to_adjacent_lap(self, left_first: bool, iter_tag: int = -2) -> bool:
        r, c = self.current_pos
        w = self.w
        deltas = ([-w, +w] if left_first else [+w, -w])

        cur_lap = self._lap_index_of(self.current_pos)

        for delta in deltas:
            # Hysteresis: nếu vừa shift, không cho shift ngược lại ngay
            if self.forward_since_shift < self.min_forward_after_shift and self.last_shift:
                prev_from, prev_to, *_ = self.last_shift
                # đang cố quay về prev_from?
                target_lap = cur_lap + (-1 if delta < 0 else +1)
                if target_lap == prev_from and cur_lap == prev_to:
                    continue

            # Bỏ qua hành lang đã cấm
            target_lap = cur_lap + (-1 if delta < 0 else +1)
            if self._lap_pair(cur_lap, target_lap) in self.banned_corridors:
                continue

            path = []
            if self.lap_dir == 'vertical':
                sgn = 1 if delta > 0 else -1
                for i in range(1, abs(delta) + 1):
                    cand = (r, c + sgn * i)
                    if not self.is_valid_position(cand):
                        path = []
                        break
                    path.append(cand)
            else:
                sgn = 1 if delta > 0 else -1
                for i in range(1, abs(delta) + 1):
                    cand = (r + sgn * i, c)
                    if not self.is_valid_position(cand):
                        path = []
                        break
                    path.append(cand)

            if not path:
                continue

            # Thực thi shift
            covered_before = len(self.covered_cells)
            pathlen_before = self.total_path_length
            prev_lap = self._lap_index_of(self.current_pos)

            for p in path:
                self._cover_step(p, iter_tag=iter_tag)

            new_lap = self._lap_index_of(self.current_pos)
            if new_lap != prev_lap:
                self.sweep_sign *= -1

            # Ghi nhận shift
            self.last_shift = (
                prev_lap, new_lap, self.total_path_length, covered_before, pathlen_before)
            self.forward_since_shift = 0  # reset đếm tiến thẳng sau shift

            # toggle ưu tiên trái/phải lần sau
            self.next_shift_left = not self.next_shift_left
            return True

        return False

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

    def a_star_path(self, start, goal, allow_diagonals=False,
                    prefer_axis: Optional[str] = None, prefer_sign: int = 0) -> List[Tuple[int, int]]:
        if not self.is_valid_position(start) or not self.is_valid_position(goal):
            return []

        dirs = DIR8 if allow_diagonals else DIR4

        # --- cost có bias luống ---
        lateral_penalty = 0.6     # phạt đi "lệch luống"
        forward_bonus = 0.05    # thưởng nhỏ khi đi đúng hướng quét

        def step_cost(a, b):
            dr, dc = b[0]-a[0], b[1]-a[1]
            base = 1.0 if (dr == 0 or dc == 0) else math.sqrt(2)
            if prefer_axis == 'vertical':
                lat = abs(dc)       # lệch luống là thay đổi cột
                fwd = (dr > 0) - (dr < 0)
            elif prefer_axis == 'horizontal':
                lat = abs(dr)       # lệch luống là thay đổi hàng
                fwd = (dc > 0) - (dc < 0)
            else:
                lat, fwd = 0, 0
            cost = base + lateral_penalty * lat
            if prefer_axis and prefer_sign != 0 and fwd == prefer_sign:
                cost -= forward_bonus
            return cost

        # --- heuristic khớp bias (weighted Manhattan/Octile) ---
        def h(a, b):
            dx = abs(a[0]-b[0])
            dy = abs(a[1]-b[1])
            if allow_diagonals:
                base = (dx + dy) + (math.sqrt(2) - 2) * min(dx, dy)
            else:
                base = dx + dy
            # phạt lệch luống trong ước lượng (nhẹ)
            if prefer_axis == 'vertical':
                return base + 0.3 * dy
            elif prefer_axis == 'horizontal':
                return base + 0.3 * dx
            return base

        open_heap = []
        g = {start: 0.0}
        parent = {}
        eps = 1e-3

        def push(n):
            gx = g[n]
            hx = h(n, goal)
            vx = (n[0]-start[0], n[1]-start[1])
            vgoal = (goal[0]-start[0], goal[1]-start[1])
            cross = abs(vx[0]*vgoal[1] - vx[1]*vgoal[0])
            fx = gx + hx + eps * cross
            heapq.heappush(open_heap, (fx, hx, n))

        push(start)
        closed = set()

        while open_heap:
            _, _, cur = heapq.heappop(open_heap)
            if cur in closed:
                continue
            if cur == goal:
                path = [cur]
                while cur in parent:
                    cur = parent[cur]
                    path.append(cur)
                path.reverse()
                return self.smooth_path(path)
            closed.add(cur)

            for dr, dc in dirs:
                nxt = (cur[0]+dr, cur[1]+dc)
                if not self.is_valid_position(nxt) or nxt in closed:
                    continue
                if not self._diag_ok(cur, nxt):
                    continue
                cand_g = g[cur] + step_cost(cur, nxt)
                if cand_g < g.get(nxt, float('inf')):
                    g[nxt] = cand_g
                    parent[nxt] = cur
                    push(nxt)
        return []

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
        if not self.rcg.nodes:
            return None
        w = self.w
        lap_dir = self.lap_dir

        target_lap = self.rcg._lap_index(pos, lap_dir=lap_dir, w=w)
        same_lap_ids, other_ids = [], []
        for nid, n in self.rcg.nodes.items():
            lap_n = self.rcg._lap_index(n.position, lap_dir=lap_dir, w=w)
            (same_lap_ids if lap_n == target_lap else other_ids).append(nid)

        def nearest_from(cands: list[int]) -> Optional[int]:
            if not cands:
                return None
            px, py = pos
            return min(cands, key=lambda nid: math.hypot(
                px - self.rcg.nodes[nid].position[0],
                py - self.rcg.nodes[nid].position[1]
            ))

        return nearest_from(same_lap_ids) or nearest_from(other_ids)

    def _run_length_in_adjacent_lap(self, delta: int, lookahead: int) -> tuple[int, int]:
        """
        Ước lượng độ dài chạy được sau khi dịch sang luống kề (cột + delta nếu quét dọc),
        theo hướng quét hiện tại. Trả về (run_len, unknown_gain).
        """
        r, c = self.current_pos
        if self.lap_dir == 'vertical':
            cand = (r, c + delta)
        else:
            cand = (r + delta, c)
        if not self.is_valid_position(cand):
            return (0, 0)

        run_len = 0
        unknown_gain = 0
        rr, cc = cand
        step = self.sweep_sign

        for _ in range(lookahead):
            if not self.is_valid_position((rr, cc)):
                break
            # tăng điểm nếu ô chưa observed/covered (hấp dẫn hơn)
            if (rr, cc) not in self.observed_cells and (rr, cc) not in self.obstacles:
                unknown_gain += 1
            if (rr, cc) in self.obstacles:
                break
            run_len += 1
            if self.lap_dir == 'vertical':
                rr += step
            else:
                cc += step

        return (run_len, unknown_gain)

    def _choose_shift_side(self) -> Optional[Tuple[Tuple[int, int], bool]]:
        """
        Chọn luống kề tốt hơn để dịch (±w) với các ràng buộc:
        - Tránh 'hành lang' đã cấm trong banned_corridors (chống lặp hình chữ nhật).
        - Hysteresis: sau khi vừa shift, bắt buộc tiến thẳng >= min_forward_after_shift
        mới cho phép shift trở lại hành lang cũ.
        Ưu tiên theo (run_len, unknown_gain, is_left).
        Trả về: (candidate_pos, next_is_left) hoặc None nếu không có ứng viên hợp lệ.
        """
        w = self.w
        r, c = self.current_pos
        cur_lap = self._lap_index_of(self.current_pos)

        # Hai hướng: trái rồi phải theo định nghĩa ±w
        deltas = (-w, +w)

        opts = []
        for delta in deltas:
            # Lap mục tiêu nếu dịch theo delta
            target_lap = cur_lap + (-1 if delta < 0 else +1)

            # 1) Tránh hành lang đã cấm
            lap_pair = (cur_lap, target_lap) if cur_lap < target_lap else (
                target_lap, cur_lap)
            if hasattr(self, "banned_corridors") and lap_pair in self.banned_corridors:
                continue

            # 2) Hysteresis: nếu vừa shift, không cho shift ngược lại ngay
            if getattr(self, "last_shift", None) and self.forward_since_shift < getattr(self, "min_forward_after_shift", max(2, w)):
                prev_from, prev_to, *_ = self.last_shift
                # Đang ở prev_to và muốn quay về prev_from?
                if (cur_lap == prev_to) and (target_lap == prev_from):
                    continue

            # 3) Xây dựng candidate position (điểm đích sau dịch ngang ±w)
            if self.lap_dir == 'vertical':
                cand = (r, c + delta)
            else:
                cand = (r + delta, c)

            if not self.is_valid_position(cand):
                continue

            # 4) Ước lượng độ dài chạy được + gain unknown sau khi dịch
            run_len, unknown_gain = self._run_length_in_adjacent_lap(
                delta, lookahead=6*w)

            # 5) Ưu tiên: run_len > unknown_gain > trái
            priority = (run_len, unknown_gain, 1 if delta < 0 else 0)
            opts.append((priority, cand, delta < 0))  # is_left = delta<0

        if not opts:
            return None

        # Chọn theo priority lớn nhất
        opts.sort()
        _prio, cand, is_left = opts[-1]
        return (cand, is_left)

    def path_exists(self, start: Tuple[int, int], goal: Tuple[int, int], allow_diagonals: bool = False) -> bool:
        """Kiểm tra tồn tại đường đi (A*), chọn 4-hướng hoặc 8-hướng tùy allow_diagonals."""
        if not self.is_valid_position(start) or not self.is_valid_position(goal):
            return False

        directions = DIR8 if allow_diagonals else DIR4
        open_set = [(0, start)]
        closed = set()
        g = {start: 0}

        # heuristic: Manhattan cho 4 hướng, Euclid cho 8 hướng
        def h_cost(a, b):
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
                if not self._diag_ok(cur, nxt):
                    continue
                # cost bước: 1 cho thẳng, sqrt(2) cho chéo
                step = 1.0 if (dr == 0 or dc == 0) else math.sqrt(2)
                tentative = g[cur] + step
                if nxt not in g or tentative < g[nxt]:
                    g[nxt] = tentative
                    f = tentative + h_cost(nxt, goal)
                    heapq.heappush(open_set, (f, nxt))
        return False

    def find_path_to_node(self, start: Tuple[int, int], goal_node_id: int) -> List[Tuple[int, int]]:
        goal = self.rcg.nodes[goal_node_id].position
        allow_diag = (goal in self.covered_cells)  # backtracking thì cho chéo
        return self.a_star_path(start, goal, allow_diagonals=allow_diag)

    def _is_blocked(self, r: int, c: int) -> bool:
        return not (0 <= r < self.rows and 0 <= c < self.cols) or (r, c) in self.obstacles

    def _los(self, a: Tuple[int, int], b: Tuple[int, int]) -> bool:
        """Line-of-sight giữa a và b (Bresenham), cấm đi xuyên ô obstacle."""
        (x0, y0), (x1, y1) = a, b
        dx = abs(x1 - x0)
        sx = 1 if x0 < x1 else -1
        dy = -abs(y1 - y0)
        sy = 1 if y0 < y1 else -1
        err = dx + dy
        x, y = x0, y0
        while True:
            if self._is_blocked(x, y):
                return False
            if (x, y) == (x1, y1):
                return True
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x += sx
            if e2 <= dx:
                err += dx
                y += sy

    def _reachable_uncovered_exists(self) -> bool:
        """BFS từ vị trí hiện tại: còn ô free-chưa-cover nào tới được không?"""
        start = self.current_pos
        if not self.is_valid_position(start):
            return False
        seen = {start}
        q = deque([start])
        while q:
            r, c = q.popleft()
            if (r, c) not in self.obstacles and (r, c) not in self.covered_cells:
                if (r, c) != start:
                    return True
            for dr, dc in DIR4:
                nr, nc = r + dr, c + dc
                if (0 <= nr < self.rows and 0 <= nc < self.cols and
                    (nr, nc) not in self.obstacles and
                        (nr, nc) not in seen):
                    seen.add((nr, nc))
                    q.append((nr, nc))
        return False

    def _nearest_reachable_uncovered(self) -> Optional[Tuple[int, int]]:
        """Tìm ô free-chưa-cover gần nhất CÓ THỂ đi tới (BFS lớp)."""
        start = self.current_pos
        if not self.is_valid_position(start):
            return None
        seen = {start}
        q = deque([start])
        while q:
            r, c = q.popleft()
            if (r, c) not in self.obstacles and (r, c) not in self.covered_cells:
                if (r, c) != start:
                    return (r, c)
            for dr, dc in DIR4:
                nr, nc = r + dr, c + dc
                if (0 <= nr < self.rows and 0 <= nc < self.cols and
                    (nr, nc) not in self.obstacles and
                        (nr, nc) not in seen):
                    seen.add((nr, nc))
                    q.append((nr, nc))
        return None

    def smooth_path(self, path: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
        """String-pulling: co đường về các đoạn nhìn-thấy-nhau (LOS)."""
        if not path:
            return path
        out = [path[0]]
        i = 0
        j = 1
        while j < len(path):
            # kéo j xa nhất sao cho i -- j còn LOS
            while j < len(path) and self._los(out[-1], path[j]):
                j += 1
            # j đã vượt qua điểm hợp lệ cuối cùng → chốt j-1
            out.append(path[j-1])
            i = j-1
            j = i+1
        # bỏ bớt nếu thừa điểm trùng
        if len(out) >= 2 and out[-1] == out[-2]:
            out.pop()
        return out

    def _sampling_front_step(self, lap_dir: str = 'vertical'):
        newly = self.observed_cells - self.prev_observed
        self.prev_observed = set(self.observed_cells)

        if not newly:
            return

        # Chỉ sinh mẫu khi thực sự quan sát thêm đáng kể
        min_new = max(3, (self.w * self.w) // 2)
        if len(newly) < min_new:
            return

        w = max(1, self.sensing_radius)

        # thưa mẫu hơn: delta=2 để giảm mật độ node
        ids = self.rcg.generate_frontier_samples(
            observed_cells=self.observed_cells,
            obstacles=self.obstacles,
            newly_discovered=newly,
            w=w,
            lap_dir=lap_dir,
            delta=2,
        )
        # cắt số node thêm mỗi bước
        if len(ids) > 20:
            ids = ids[:20]

        self.total_nodes_generated += len(ids)
        if ids:
            # prune nhẹ nhưng không phải mỗi bước
            if (self.total_path_length % 30) == 0:
                self.rcg.prune(self.covered_cells, self.obstacles,
                               lap_dir=lap_dir, w=w, k_edge_samples=1)
                if self.on_rcg_update_callback:
                    self.on_rcg_update_callback(self.rcg)

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

    def alg1_expand_rcg(self, max_iterations: int = 100, lap_dir: str = 'vertical') -> Optional[int]:
        """
        Mở rộng RCG theo tinh thần C*:
        - Bias lấy mẫu gần vùng chưa quan sát/chưa phủ.
        - Tiến một bước kích thước w (sensing_radius) theo hướng về q_rand.
        - Thêm node bằng RCG.add_node(..., sampling_step=w, lap_dir=lap_dir) → rewire C*.
        - Chỉ chấp nhận node nếu có thể bao phủ/thấy thêm vùng mới; nếu không thì rollback.
        - Prune nhẹ sau khi nhận node.
        """
        w = max(1, self.sensing_radius)

        # Nếu RCG chưa có gì → seed tại vị trí hiện tại
        if not self.rcg.nodes:
            nid0 = self.rcg.add_node(
                self.current_pos, sampling_step=w, lap_dir=lap_dir, obstacles=self.obstacles)
            self.total_nodes_generated += 1
            return nid0

        for _ in range(max_iterations):
            # --- 1) Propose q_rand (70% gần vùng chưa phủ/unknown, 30% random) ---
            if random.random() < 0.7:
                # gần vùng chưa bao phủ (free & chưa covered) + nhiễu nhỏ
                q_rand = self.sample_near_uncovered_areas()
            else:
                q_rand = self.sample_random_configuration()

            # --- 2) Tìm nearest node trên RCG (ưu tiên cùng lap) ---
            nearest_id = self.find_nearest_node(q_rand)
            if nearest_id is None:
                # an toàn, nhưng bình thường sẽ không vào đây vì đã seed ở trên
                nid0 = self.rcg.add_node(
                    self.current_pos, sampling_step=w, lap_dir=lap_dir, obstacles=self.obstacles)
                self.total_nodes_generated += 1
                return nid0

            nearest_node = self.rcg.nodes[nearest_id]
            px, py = nearest_node.position
            rx, ry = q_rand

            # --- 3) Bước tiến một đoạn ≤ w hướng về q_rand (discrete, tối thiểu 1 ô) ---
            dx, dy = rx - px, ry - py
            dist = math.hypot(dx, dy)
            if dist == 0:
                continue

            step = min(w, dist)
            nx = px + int(round((dx / dist) * step))
            ny = py + int(round((dy / dist) * step))

            # snap mịn theo lưới ô (1 ô) để A* dễ nối
            from .utils import snap_to_grid
            new_pos = snap_to_grid((nx, ny), grid_step=1)

            # Bỏ nếu ra ngoài/obstacle
            if not self.is_valid_position(new_pos):
                continue

            # Bỏ nếu không có đường đi 4-hướng từ nearest → new_pos
            if not self.path_exists(nearest_node.position, new_pos, allow_diagonals=False):
                continue

            # --- 4) Thêm node (đã tự rewire C* trong add_node) ---
            new_node_id = self.rcg.add_node(
                new_pos, sampling_step=w, lap_dir=lap_dir, obstacles=self.obstacles)
            self.total_nodes_generated += 1

            # --- 5) Đánh giá "có ích": có mở thêm vùng mới không?
            cov = self.rcg.get_coverage_area(new_pos)

            # (a) mới quan sát (unknown → observed) hoặc (b) mới cover
            opens_unknown = any((cell not in self.observed_cells) and (cell not in self.obstacles)
                                for cell in cov)
            adds_coverage = any((cell not in self.covered_cells) and (cell not in self.obstacles)
                                for cell in cov)

            if opens_unknown or adds_coverage:
                # Giữ node → prune nhẹ để RCG thưa gọn
                self.rcg.prune(
                    covered_cells=self.covered_cells,
                    obstacles=self.obstacles,
                    lap_dir=lap_dir,
                    w=w
                )
                return new_node_id
            else:
                # --- 6) Rollback node vô ích để không bẩn đồ thị ---
                node = self.rcg.nodes.get(new_node_id)
                if node is not None:
                    for nb in list(node.neighbors):
                        self.rcg._remove_edge(new_node_id, nb)
                    pos = node.position
                    self.rcg.grid_to_nodes[pos].discard(new_node_id)
                    if not self.rcg.grid_to_nodes[pos]:
                        self.rcg.grid_to_nodes.pop(pos, None)
                    self.rcg.nodes.pop(new_node_id, None)

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

    def select_best_frontier_node(self, frontier_nodes: List[int], lap_dir: str = 'vertical', w: int = 1) -> Optional[int]:
        if not frontier_nodes:
            return None

        last_vec = None
        if len(self.path) >= 2:
            (r1, c1), (r2, c2) = self.path[-2], self.path[-1]
            last_vec = (r2 - r1, c2 - c1)

        def turn_penalty(to_pos):
            if not last_vec:
                return 0.0
            v = (to_pos[0]-self.current_pos[0], to_pos[1]-self.current_pos[1])
            if v == (0, 0):
                return 0.0
            dot = last_vec[0]*v[0] + last_vec[1]*v[1]
            la = math.hypot(*last_vec)
            lb = math.hypot(*v)
            cosang = dot/(la*lb) if la*lb > 0 else 1.0
            return (1 - cosang)

        best_id, best_score = None, -1e18
        cur_lap = self.rcg._lap_index(self.current_pos, lap_dir=lap_dir, w=w)

        for nid in frontier_nodes:
            pos = self.rcg.nodes[nid].position
            lap = self.rcg._lap_index(pos, lap_dir=lap_dir, w=w)

            new_cov = 1 if (
                pos not in self.covered_cells and pos not in self.obstacles) else 0

            path = self.a_star_path(
                self.current_pos, pos,
                allow_diagonals=False,
                prefer_axis=lap_dir,
                prefer_sign=self.sweep_sign
            )
            cost = len(path) if path else float('inf')
            if not path:
                continue

            dr = abs(pos[0]-self.current_pos[0])
            dc = abs(pos[1]-self.current_pos[1])
            forward = dr if lap_dir == 'vertical' else dc

            same_lap_bonus = 1.0 if lap == cur_lap else (
                0.3 if lap in (cur_lap-1, cur_lap+1) else -1.0)
            far_pen = 0.5 if (abs(dr)+abs(dc) > 4*w) else 0.0
            tpen = turn_penalty(pos)

            score = (
                2.0 * new_cov
                + 1.2 * same_lap_bonus
                + 0.2 * forward
                - 0.8 * (cost) * 0.05
                - 0.6 * tpen
                - 0.8 * far_pen
                + 0.05 * len(self.rcg.nodes[nid].neighbors)
            )

            if score > best_score:
                best_score, best_id = score, nid

        return best_id

        # ====== SENSOR-LOCAL FRONTIER DETECTION & TARGETING ======

    def detect_frontier_cells_from_sensor(self, pos: Tuple[int, int]) -> List[Tuple[int, int]]:
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

    def select_best_frontier_cell(self, frontier_cells: List[Tuple[int, int]]) -> Optional[Tuple[int, int]]:
        """Chọn ô frontier có A* 4-hướng ngắn nhất."""
        best_cell, best_cost = None, float('inf')
        for cell in frontier_cells:
            path = self.a_star_path(
                self.current_pos, cell, allow_diagonals=False)
            if path:
                cost = len(path)
                if cost < best_cost:
                    best_cost = cost
                    best_cell = cell
        return best_cell

    def close_and_maybe_link(self, nid: int, next_is_left: bool, w: int):
        if nid not in self.rcg.nodes:
            return
        node = self.rcg.nodes[nid]
        node.state = "Cl"
        r, c = node.position

        if next_is_left:
            if self.lap_dir == 'vertical':
                candidates = [(r - w, c), (r + w, c)]
            else:
                candidates = [(r, c - w), (r, c + w)]

            created = []
            for rr, cc in candidates:
                if 0 <= rr < self.rows and 0 <= cc < self.cols and (rr, cc) not in self.obstacles:
                    new_id = self.rcg.add_node(
                        self.current_pos, sampling_step=w, lap_dir=self.lap_dir, obstacles=self.obstacles)
                    self.total_nodes_generated += 1
                    created.append(new_id)

            if created:
                self.rcg.prune(self.covered_cells, self.obstacles,
                               lap_dir=self.lap_dir, w=w)
                if self.on_rcg_update_callback:
                    self.on_rcg_update_callback(self.rcg)
    # === NEW: tiện ích trạng thái Open/Closed cho node RCG ===

    def _ensure_node_state(self, nid: int):
        n = self.rcg.nodes[nid]
        if not hasattr(n, "state"):
            n.state = "Op"  # mặc định là Open khi mới thêm

    def _neighbors_LUDR(self, nid: int, lap_dir: str, w: int):
        """
        Trả về tuple (nL, nU, nD, nR) là hàng xóm 'trái, trên, dưới, phải' của node theo định nghĩa C*:
        - 'Up/Down' là cùng lap, tiến/lùi theo sweep.
        - 'Left/Right' là lap kề ±w.
        Ưu tiên chọn node gần nhất đúng hướng (theo L1) nếu có nhiều.
        """
        node = self.rcg.nodes[nid]
        r, c = node.position
        # phân loại theo lap
        cur_lap = self.rcg._lap_index((r, c), lap_dir=lap_dir, w=w)
        same_lap = []
        left_lap = []
        right_lap = []
        for nb_id in node.neighbors:
            nb = self.rcg.nodes[nb_id]
            lr = self.rcg._lap_index(nb.position, lap_dir=lap_dir, w=w)
            if lr == cur_lap:
                same_lap.append(nb_id)
            elif lr == cur_lap - 1:
                left_lap.append(nb_id)
            elif lr == cur_lap + 1:
                right_lap.append(nb_id)

        def nearest(ids, keydir):
            if not ids:
                return None
            if lap_dir == 'vertical':
                # 'Up' là r' < r ; 'Down' là r' > r
                if keydir == "U":
                    cand = [i for i in ids if self.rcg.nodes[i].position[0] < r]
                    return min(cand, key=lambda i: abs(self.rcg.nodes[i].position[0]-r)) if cand else None
                if keydir == "D":
                    cand = [i for i in ids if self.rcg.nodes[i].position[0] > r]
                    return min(cand, key=lambda i: abs(self.rcg.nodes[i].position[0]-r)) if cand else None
            else:
                # horizontal laps
                if keydir == "U":  # 'Up' hiểu là c' < c
                    cand = [i for i in ids if self.rcg.nodes[i].position[1] < c]
                    return min(cand, key=lambda i: abs(self.rcg.nodes[i].position[1]-c)) if cand else None
                if keydir == "D":  # 'Down' hiểu là c' > c
                    cand = [i for i in ids if self.rcg.nodes[i].position[1] > c]
                    return min(cand, key=lambda i: abs(self.rcg.nodes[i].position[1]-c)) if cand else None
            return None

        nU = nearest(same_lap, "U")
        nD = nearest(same_lap, "D")
        # 'Left/Right' lấy node gần nhất theo L1 trên lap kề

        def nearest_L1(ids):
            if not ids:
                return None
            return min(ids, key=lambda i: abs(self.rcg.nodes[i].position[0]-r)+abs(self.rcg.nodes[i].position[1]-c))
        nL = nearest_L1(left_lap)
        nR = nearest_L1(right_lap)

        def filt(x):
            return None if (x is None or self._edge_banned(nid, x)) else x
        nL, nU, nD, nR = filt(nL), filt(nU), filt(nD), filt(nR)
        return nL, nU, nD, nR
    # === NEW: chọn goal node theo C* ===

    def alg1_pick_next_node_LUDR(self, cur_id: int) -> Optional[int]:
        lap_dir, w = self.lap_dir, self.w
        nL, nU, nD, nR = self._neighbors_LUDR(cur_id, lap_dir, w)

        def is_open(nid):
            if nid is None:
                return False
            self._ensure_node_state(nid)
            return self.rcg.nodes[nid].state == "Op"

        # Ưu tiên L, U, D, R
        if is_open(nL):
            return nL
        if is_open(nU):
            return nU
        if is_open(nD):
            return nD
        if is_open(nR):
            return nR
        return None

    # === NEW: đóng node hiện tại và tạo link node khi rẽ trái (theo C*) ===

    def _node_state(self, nid: int) -> str:
        self._ensure_node_state(nid)
        return self.rcg.nodes[nid].state

    def _same_lap_neighbors_UD(self, nid: int, lap_dir: str, w: int):
        nL, nU, nD, nR = self._neighbors_LUDR(nid, lap_dir, w)
        return nU, nD

    def _update_state_alg2(self, cur_id: int, nxt_id: Optional[int]):
        lap_dir, w = self.lap_dir, self.w
        # 1) quyết định Op/Cl cho n̂ᵢ
        nL, nU, nD, nR = self._neighbors_LUDR(cur_id, lap_dir, w)

        def st(nid):
            if nid is None:
                return None
            self._ensure_node_state(nid)
            return self.rcg.nodes[nid].state
        u_is_cl = (st(nU) == "Cl")
        d_is_cl = (st(nD) == "Cl")
        self.rcg.nodes[cur_id].state = "Cl" if (u_is_cl or d_is_cl) else "Op"

        # 2) nếu rẽ trái (n̂_{i+1}=n̂^L_i) và n̂ᵢ=Cl: xét tạo link node (chỉ khi U/D là Op và “xa hơn w”)
        if nxt_id is None:
            return
        r, c = self.rcg.nodes[cur_id].position
        r2, c2 = self.rcg.nodes[nxt_id].position
        cur_lap = self.rcg._lap_index((r, c), lap_dir=lap_dir, w=w)
        next_lap = self.rcg._lap_index((r2, c2), lap_dir=lap_dir, w=w)
        turning_left = (next_lap == cur_lap - 1)

        if turning_left and self.rcg.nodes[cur_id].state == "Cl":
            if nU is not None and st(nU) == "Op":
                if (lap_dir == 'vertical' and abs(self.rcg.nodes[nU].position[0] - r) > w) or \
                        (lap_dir == 'horizontal' and abs(self.rcg.nodes[nU].position[1] - c) > w):
                    self.rcg.add_node((r - w, c) if lap_dir == 'vertical' else (r, c - w),
                                      sampling_step=w, lap_dir=lap_dir, obstacles=self.obstacles)
                    self.total_nodes_generated += 1
            if nD is not None and st(nD) == "Op":
                if (lap_dir == 'vertical' and abs(self.rcg.nodes[nD].position[0] - r) > w) or \
                        (lap_dir == 'horizontal' and abs(self.rcg.nodes[nD].position[1] - c) > w):
                    self.rcg.add_node((r + w, c) if lap_dir == 'vertical' else (r, c + w),
                                      sampling_step=w, lap_dir=lap_dir, obstacles=self.obstacles)
                    self.total_nodes_generated += 1
            self.rcg.prune(self.covered_cells, self.obstacles,
                           lap_dir=lap_dir, w=w)
            if self.on_rcg_update_callback:
                self.on_rcg_update_callback(self.rcg)

    def alg1_avoid_pingpong(self, cur_id: int, nxt_id: int):
        # nhớ lịch sử 4 node đích gần nhất
        if not hasattr(self, "_recent_rcg_seq"):
            self._recent_rcg_seq = deque(maxlen=4)
            self._cov_mark = len(self.covered_cells)

        self._recent_rcg_seq.append(nxt_id)

        # phát hiện mẫu ABAB trên đích (… A, B, A, B)
        if len(self._recent_rcg_seq) == 4:
            a, b, c, d = self._recent_rcg_seq
            if a == c and b == d:
                gained = len(self.covered_cells) - self._cov_mark
                # nếu gần như không tăng cover, cấm cạnh hiện tại để buộc rẽ khác
                if gained < max(1, self.w // 2):
                    self._ban_edge(cur_id, nxt_id)
                    # tùy chọn: đóng tạm nxt để vòng sau không chọn lại
                    self._ensure_node_state(nxt_id)
                    self.rcg.nodes[nxt_id].state = "Cl"
        self._cov_mark = len(self.covered_cells)

    def alg1_collect_retreat_nodes(self):
        if not hasattr(self, "_retreat_nodes"):
            self._retreat_nodes: Set[int] = set()
        lap_dir, w = self.lap_dir, self.w
        px, py = self.current_pos
        for nid, n in self.rcg.nodes.items():
            self._ensure_node_state(nid)
            if n.state == "Op":
                r, c = n.position
                if (r - px)**2 + (c - py)**2 <= 2*(w**2):  # ~ (√2 w)²
                    self._retreat_nodes.add(nid)
        # loại bỏ nút đã Closed
        self._retreat_nodes = {
            nid for nid in self._retreat_nodes if self.rcg.nodes[nid].state == "Op"}

    def alg1_escape_dead_end(self, cur_id: int) -> Optional[int]:
        if not getattr(self, "_retreat_nodes", None):
            return None
        # chọn retreat node gần nhất theo A* cost từ current_pos
        best, best_cost = None, float('inf')
        for nid in self._retreat_nodes:
            path = self.a_star_path(self.current_pos, self.rcg.nodes[nid].position,
                                    allow_diagonals=False, prefer_axis=self.lap_dir, prefer_sign=self.sweep_sign)
            if path:
                cost = len(path)
                if cost < best_cost:
                    best, best_cost = nid, cost
        return best

    def _bm_crawl_to_cp(self, iter_tag: int) -> int:
        """
        Boustrophedon vi mô:
        - Chỉ bước vào ô FREE_UNCOVERED theo thứ tự ưu tiên phù hợp hướng quét.
        - Mỗi bước đều gọi _cover_step() => update_observed() => _sampling_front_step()
        ==> RCG được mở rộng "just-in-time" đúng tinh thần C*.
        - Khi không còn ô FREE_UNCOVERED lân cận (critical point) thì dừng.
        Trước khi trả về, thử expand RCG 1 nhịp để tìm thêm seed (nếu có).
        Trả về: số bước đã đi trong pha BM này.
        """
        steps = 0
        r, c = self.current_pos

        # Thứ tự ưu tiên hướng theo quét
        if self.lap_dir == 'vertical':
            dirs = [(self.sweep_sign, 0),
                    (-self.sweep_sign, 0), (0, 1), (0, -1)]
        else:
            dirs = [(0, self.sweep_sign),
                    (0, -self.sweep_sign), (1, 0), (-1, 0)]

        while True:
            moved = False
            for dr, dc in dirs:
                nr, nc = r + dr, c + dc
                # "Hết đường đi" nghĩa là không còn ô FREE_UNCOVERED hợp lệ để tiến
                if self.is_valid_position((nr, nc)) and self.grid[nr][nc] == FREE_UNCOVERED:
                    # sẽ gọi update_observed + sampling_front
                    self._cover_step((nr, nc), iter_tag=iter_tag)
                    r, c = nr, nc
                    steps += 1
                    moved = True
                    break  # tiếp tục giữ thứ tự ưu tiên cũ
            if not moved:
                # Critical point: không còn FREE_UNCOVERED lân cận → thử mở RCG 1 nhịp
                # (để tạo seed frontier mới nếu vừa quan sát ra “mép” chưa có node)
                try_new = self.alg1_expand_rcg(
                    max_iterations=40, lap_dir=self.lap_dir)
                if try_new is not None and self.on_rcg_update_callback:
                    self.on_rcg_update_callback(self.rcg)
                break

        return steps

    def final_boustrophedon_sweep(self):
        """
        Quét nốt theo boustrophedon, CHỈ dọn những ô free-chưa-cover còn reachable.
        Tránh mở rộng RCG; nếu kẹt thì nhảy A* tới ô reachable gần nhất và tiếp tục.
        """
        print("[FinalSweep] Start final boustrophedon sweep.")
        w = self.w
        last_cov = len(self.covered_cells)
        no_progress = 0
        safety = 4000  # nắp an toàn

        while safety > 0:
            safety -= 1

            # Không còn ô tới được → xong
            target_probe = self._nearest_reachable_uncovered()
            if target_probe is None:
                break

            moved = self._advance_along_lap_until_block(
                max_steps=10*w, iter_tag=-4)
            if moved == 0:
                choice = self._choose_shift_side()
                did_shift = False
                if choice:
                    cand, is_left = choice
                    did_shift = self._try_shift_to_adjacent_lap(
                        is_left, iter_tag=-4)
                else:
                    did_shift = self._try_shift_to_adjacent_lap(True, iter_tag=-4) or \
                        self._try_shift_to_adjacent_lap(False, iter_tag=-4)

                if not did_shift:
                    target = self._nearest_reachable_uncovered()
                    if not target:
                        break
                    path = self.a_star_path(self.current_pos, target, allow_diagonals=False,
                                            prefer_axis=self.lap_dir, prefer_sign=self.sweep_sign)
                    if not path or len(path) <= 1:
                        break
                    for p in path[1:]:
                        self._cover_step(p, iter_tag=-4)

            # chống “lặp vô tiến độ”
            if len(self.covered_cells) == last_cov:
                no_progress += 1
                if no_progress >= 5:
                    print(
                        "[FinalSweep] Stalled with no reachable progress; stopping.")
                    break
            else:
                last_cov = len(self.covered_cells)
                no_progress = 0

        print("[FinalSweep] Done.")

    def run(self) -> Tuple[List[Tuple[int, int]], Dict]:
        print("--- Starting C* Algorithm (RCG-driven) ---")
        iteration = 0
        max_iterations = 1000
        lap_dir, w = self.lap_dir, self.w

        # Seed ban đầu
        self.cover_cells(self.current_pos)
        self.update_observed(self.current_pos)
        self._sampling_front_step(lap_dir=lap_dir)

        # Đảm bảo có node RCG tại vị trí start
        if not self.rcg.nodes:
            cur_id = self.rcg.add_node(
                self.current_pos, sampling_step=w, lap_dir=lap_dir, obstacles=self.obstacles)
            self.total_nodes_generated += 1
        else:
            cur_id = self.find_nearest_node(self.current_pos)
            if cur_id is None:
                cur_id = self.rcg.add_node(
                    self.current_pos, sampling_step=w, lap_dir=lap_dir, obstacles=self.obstacles)
                self.total_nodes_generated += 1
        self._ensure_node_state(cur_id)

        # nhớ id gần nhất để phụ trợ chống chọn-ngược nếu cần
        if not hasattr(self, "_last_rcg_id"):
            self._last_rcg_id = None

        while not self.is_coverage_complete() and iteration < max_iterations:
            iteration += 1

            # cập nhật UI mỗi vòng
            if self.on_step_callback:
                self.on_step_callback(
                    self.grid, self.current_pos, self.path, iteration)

            # mở rộng RCG vừa đủ mỗi vòng
            new_id = self.alg1_expand_rcg(max_iterations=40, lap_dir=lap_dir)
            if new_id is not None and self.on_rcg_update_callback:
                self.on_rcg_update_callback(self.rcg)

            # mục tiêu kế theo L→U→D→R (chỉ chọn Open, đã lọc cạnh bị ban trong _neighbors_LUDR)
            nxt_id = self.alg1_pick_next_node_LUDR(cur_id)

            # Nếu có mục tiêu, chạy detector ping-pong ABAB; nếu vừa bị ban cạnh thì chọn lại
            if nxt_id is not None:
                self.alg1_avoid_pingpong(cur_id, nxt_id)
                # Reselect nếu cạnh vừa bị ban hoặc nxt bị đóng
                re_nxt = self.alg1_pick_next_node_LUDR(cur_id)
                if re_nxt is None:
                    nxt_id = None
                else:
                    nxt_id = re_nxt

            if nxt_id is None:
                # dead-end → retreat node gần nhất
                self.alg1_collect_retreat_nodes()
                ret_id = self.alg1_escape_dead_end(cur_id)
                if ret_id is None:
                    # không còn retreat và không còn reachable → kết thúc
                    if not self._reachable_uncovered_exists():
                        print("No reachable uncovered cells; stopping.")
                        break
                    # fallback: tới ô free-chưa-cover gần nhất
                    target = self._nearest_reachable_uncovered()
                    if not target:
                        break
                    path = self.a_star_path(
                        self.current_pos, target,
                        allow_diagonals=False,
                        prefer_axis=lap_dir, prefer_sign=self.sweep_sign
                    )
                    for p in path[1:]:
                        self._cover_step(p, iter_tag=iteration)
                    # set lại cur_id về node RCG gần nhất theo vị trí thực
                    cur_id = self.find_nearest_node(self.current_pos) or cur_id
                    self._last_rcg_id = cur_id
                    continue
                nxt_id = ret_id

            # Cập nhật trạng thái theo Algorithm 2 (đóng n_i nếu U/D Closed, tạo link khi rẽ trái)
            self._update_state_alg2(cur_id, nxt_id)

            # Di chuyển tới goal node bằng A* (có bias theo hướng quét)
            goal_pos = self.rcg.nodes[nxt_id].position
            path = self.a_star_path(
                self.current_pos, goal_pos,
                allow_diagonals=False,
                prefer_axis=lap_dir, prefer_sign=self.sweep_sign
            )
            if not path or len(path) <= 1:
                # fallback: vẫn 4-hướng nhưng bỏ bias
                path = self.a_star_path(
                    self.current_pos, goal_pos, allow_diagonals=False)

            if not path or len(path) <= 1:
                # không tới được → đóng tạm để tránh kẹt, vòng sau chọn khác
                self._ensure_node_state(nxt_id)
                self.rcg.nodes[nxt_id].state = "Cl"
                # và ban cạnh cur-nxt để khỏi quay lại ngay
                self._ban_edge(cur_id, nxt_id)
                continue

            # Thực thi đường A*
            for p in path[1:]:
                self._cover_step(p, iter_tag=iteration)
                self.alg1_collect_retreat_nodes()

            # cập nhật chiều sweep nếu đã đổi lap
            cur_lap = self.rcg._lap_index(
                self.rcg.nodes[cur_id].position, lap_dir=lap_dir, w=w)
            new_lap = self.rcg._lap_index(
                self.rcg.nodes[nxt_id].position, lap_dir=lap_dir, w=w)
            if new_lap != cur_lap:
                self.sweep_sign *= -1  # giữ BM “zig-zag”

            # prune định kỳ cho gọn
            if (self.total_path_length % 20) == 0:
                self.rcg.prune(self.covered_cells, self.obstacles,
                               lap_dir=lap_dir, w=w, k_edge_samples=1)
                if self.on_rcg_update_callback:
                    self.on_rcg_update_callback(self.rcg)

            # TIỀN QUYẾT: xác lập node hiện tại theo vị trí robot (tránh gán cứng = nxt_id gây ping-pong)
            cur_id = self.find_nearest_node(self.current_pos) or nxt_id
            self._last_rcg_id = cur_id

        # Quét vét cuối nếu còn reachable
        if self._nearest_reachable_uncovered() is not None:
            print("\n--- Starting final boustrophedon sweep ---")
            self.final_boustrophedon_sweep()

        # Tổng kết
        final_coverage = len(self.covered_cells)
        total_free_cells = sum(
            1 for r in range(self.rows) for c in range(self.cols) if (r, c) not in self.obstacles
        )
        coverage_percentage = (
            final_coverage / total_free_cells) * 100 if total_free_cells else 100.0

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
