import heapq
import math

# --- 1. Cấu hình và Hằng số ---

# Định nghĩa các hướng di chuyển (Ưu tiên: Bắc, Nam, Đông, Tây cho BM)
# (dy, dx)
DIRECTIONS_BM = [
    (-1, 0),  # Bắc (N)
    (1, 0),   # Nam (S)
    (0, 1),   # Đông (E)
    (0, -1)   # Tây (W)
]

# Định nghĩa tất cả các hướng (Bao gồm chéo) cho A*
DIRECTIONS_ASTAR = [
    (-1, 0), (1, 0), (0, 1), (0, -1),
    (-1, -1), (-1, 1), (1, -1), (1, 1)
]

# Trạng thái ô (tile) trong Mô hình M
FREE_UNCOVERED = 0
OBSTACLE = 1
COVERED = 2

# Visualization constants
BACKTRACKING_POINT = 3
COVERAGE_PATH = 4
ASTAR_PATH = 5


# --- 2. Hỗ trợ cho A* và A*SPT ---

class PriorityQueue:
    """Hàng đợi ưu tiên cho thuật toán A*."""

    def __init__(self):
        self.elements = []

    def empty(self):
        return not self.elements

    def put(self, priority, item):
        heapq.heappush(self.elements, (priority, item))

    def get(self):
        return heapq.heappop(self.elements)[1]


def heuristic(a, b):
    """Ước tính khoảng cách Manhattan (cho heuristic A*)."""
    (x1, y1) = a
    (x2, y2) = b
    return abs(x1 - x2) + abs(y1 - y2)


def is_valid(grid, r, c):
    """Kiểm tra xem vị trí (r, c) có hợp lệ trong lưới hay không."""
    return 0 <= r < len(grid) and 0 <= c < len(grid[0])


def line_of_sight(grid, start, end):
    """
    Kiểm tra Đường ngắm trực tiếp (Line-of-Sight - LOS) giữa hai ô.

    Trong mô phỏng lưới đơn giản, LOS được đảm bảo nếu đoạn thẳng nối
    hai ô không đi qua bất kỳ ô chướng ngại vật nào (OBSTACLE=1).
    Chúng ta sử dụng một thuật toán giống như Bresenham để kiểm tra từng ô.
    """
    x0, y0 = start
    x1, y1 = end

    # Sử dụng thuật toán Bresenham's line algorithm
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy

    while (x0 != x1 or y0 != y1):
        # Kiểm tra ô hiện tại. Nếu là chướng ngại vật, LOS bị chặn.
        if grid[x0][y0] == OBSTACLE:
            return False

        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy

    # Kiểm tra ô đích
    if grid[x1][y1] == OBSTACLE:
        return False

    return True


# --- 3. Thuật toán A* (AStar) ---

def a_star_search(grid, start, goal):
    """
    Thuật toán tìm kiếm A* để lập kế hoạch đường dẫn không va chạm.

    Trả về đường dẫn dưới dạng danh sách các ô (r, c) hoặc None nếu không tìm thấy.
    """
    rows, cols = len(grid), len(grid[0])
    frontier = PriorityQueue()
    frontier.put(0, start)
    came_from = {start: None}
    cost_so_far = {start: 0}

    while not frontier.empty():
        current = frontier.get()

        if current == goal:
            break

        for dr, dc in DIRECTIONS_ASTAR:
            next_cell = (current[0] + dr, current[1] + dc)

            if not is_valid(grid, *next_cell):
                continue

            cell_type = grid[next_cell[0]][next_cell[1]]

            # A* chỉ đi qua các ô KHÔNG PHẢI chướng ngại vật (OBSTACLE)
            if cell_type == OBSTACLE:
                continue

            # Chi phí di chuyển: 1 cho ngang/dọc, sqrt(2) cho chéo (nếu được phép)
            # Convert to avoid float precision issues
            move_cost = int(math.sqrt(dr*dr + dc*dc) * 10) / 10
            new_cost = cost_so_far[current] + move_cost

            if next_cell not in cost_so_far or new_cost < cost_so_far[next_cell]:
                cost_so_far[next_cell] = new_cost  # type: ignore
                priority = new_cost + heuristic(goal, next_cell)
                frontier.put(priority, next_cell)
                came_from[next_cell] = current

    # Tái tạo đường dẫn
    path = []
    current = goal
    while current != start:
        if current not in came_from:
            return None  # Không tìm thấy đường dẫn
        path.append(current)
        current = came_from[current]
    path.append(start)
    path.reverse()
    return path


# --- 4. Thuật toán A*SPT (Algorithm 4) ---

def a_star_spt(grid, path):
    """
    Thuật toán A*SPT: Tìm kiếm A* với đường dẫn được làm mịn trên mô hình ô.

    Đầu vào: Đường dẫn P tìm thấy bằng A* và mô hình M.
    Đầu ra: Đường dẫn được làm mịn P_hat.
    """
    if not path or len(path) <= 1:
        return path or []
    # B1: Khởi tạo
    path_smoothed = [path[0]]
    k = 0
    n = len(path) - 1  # Chỉ số cuối cùng

    while True:
        s_k = path_smoothed[-1]  # Ô hiện tại (s_k)

        # B2: Tìm ô s_i xa nhất có đường ngắm trực tiếp từ s_k
        best_i = k + 1  # Mặc định là ô tiếp theo (ngắn nhất)

        # Lặp lại từ cuối (n) về (k+1)
        for i in range(n, k, -1):
            s_i = path[i]
            if line_of_sight(grid, s_k, s_i):
                best_i = i
                break

        # B3: Thêm ô s_i xa nhất vào P_hat
        s_best = path[best_i]
        path_smoothed.append(s_best)

        # B4: Tăng k (k được đại diện bởi chỉ số của s_best trong đường dẫn ban đầu P)
        k = best_i

        # B5: Kiểm tra ô s_k có phải là điểm tới hạn (s_n) hay không
        if s_best == path[n]:
            break

    return path_smoothed


# --- 5. Thuật toán Chuyển động Boustrophedon (BM) (Algorithm 3) ---

def boustrophedon_motion(
    grid,
    start_pos,
    start_dir_index=0,
    callback=None,
    coverage_id=1,
    sensor_radius=0
):
    """
    BM + cảm biến r + luật đổi trục:
    - Cày luống theo trục chính (NS/EW). Đến biên: dịch ngang 1 ô → quay đầu.
    - Ở biên:
        * Nếu có 2 hướng rẽ khả dụng: BỎ QUA kiểm tra, rẽ theo side_dir.
        * Nếu chỉ có 1 hướng rẽ: chỉ cân nhắc ĐỔI TRỤC khi (cur_lap < prev_lap) và có sensor.
          Đổi trục theo quy tắc: (new_long, new_side) = map(long_dir, side_dir) (xem bảng bên dưới).
    - So sánh cur_lap/prev_lap CHỈ diễn ra khi đang đi dọc và đứng ở biên.

    Trả về: (s_cp, grid, coverage_path)
    """
    rows, cols = len(grid), len(grid[0])
    r, c = start_pos
    coverage_path = []

    # ---------- helpers ----------
    def is_free(rr, cc):
        return 0 <= rr < rows and 0 <= cc < cols and grid[rr][cc] == FREE_UNCOVERED

    def opposite(didx):  # 0:N 1:S 2:E 3:W
        return {0: 1, 1: 0, 2: 3, 3: 2}[didx]

    def apply_axis_switch(long_dir, side_dir):
        """
        Bảng quy tắc đổi trục (đúng theo yêu cầu):
        (lên, phải)  -> (phải, xuống)
        (xuống, phải)-> (phải, lên)
        (phải, lên)  -> (lên, trái)
        (trái, lên)  -> (lên, phải)
        (phải, xuống)-> (xuống, trái)
        (trái, xuống)-> (xuống, phải)
        (lên, trái)  -> (trái, xuống)
        (xuống, trái)-> (trái, lên)
        """
        table = {
            (0, 2): (2, 1),  # N,E -> E,S
            (1, 2): (2, 0),  # S,E -> E,N
            (2, 0): (0, 3),  # E,N -> N,W
            (3, 0): (0, 2),  # W,N -> N,E
            (2, 1): (1, 3),  # E,S -> S,W
            (3, 1): (1, 2),  # W,S -> S,E
            (0, 3): (3, 1),  # N,W -> W,S
            (1, 3): (3, 0),  # S,W -> W,N
        }
        # fallback tổng quát nếu không khớp (vẫn đúng về nguyên tắc)
        return table.get((long_dir, side_dir), (side_dir, opposite(long_dir)))

    # --- sensing (chỉ dùng khi cần đổi trục) ---
    def sense_one_dir(rr, cc, dr, dc, R):
        if R <= 0:
            return 0, False
        steps, x, y, seen = 0, rr, cc, False
        for _ in range(R):
            x += dr
            y += dc
            if not (0 <= x < rows and 0 <= y < cols):
                seen = True
                break
            if grid[x][y] == OBSTACLE:
                seen = True
                break
            steps += 1
        return steps, seen

    def pick_dir_by_rule(rr, cc, main_axis, R):
        # Trả về (dir_idx, new_axis) theo luật cảm biến
        dN, sN = sense_one_dir(rr, cc, -1, 0, R)
        dS, sS = sense_one_dir(rr, cc,  1, 0, R)
        dE, sE = sense_one_dir(rr, cc,  0, 1, R)
        dW, sW = sense_one_dir(rr, cc,  0, -1, R)
        axis_NS_known = (sN and sS)
        axis_EW_known = (sE and sW)

        axes = []
        if axis_NS_known:
            axes.append(("NS", dN + dS))
        if axis_EW_known:
            axes.append(("EW", dE + dW))
        if axes:
            axes.sort(key=lambda t: t[1], reverse=True)
            if axes[0][0] == "NS":
                return (0 if dN >= dS else 1), "NS"
            else:
                return (2 if dE >= dW else 3), "EW"

        if axis_NS_known ^ axis_EW_known:
            if axis_NS_known:  # đi trục EW
                if (not sE) and (not sW):
                    return (2 if dE >= dW else 3), "EW"
                if not sE:
                    return 2, "EW"
                if not sW:
                    return 3, "EW"
                return (2 if dE >= dW else 3), "EW"
            else:              # đi trục NS
                if (not sN) and (not sS):
                    return (0 if dN >= dS else 1), "NS"
                if not sN:
                    return 0, "NS"
                if not sS:
                    return 1, "NS"
                return (0 if dN >= dS else 1), "NS"

        # Fallback theo trục hiện tại
        pref = ([(-1, 0, 0), (1, 0, 1), (0, 1, 2), (0, -1, 3)] if main_axis == "NS"
                else [(0, 1, 2), (0, -1, 3), (-1, 0, 0), (1, 0, 1)])
        for dr, dc, idx in pref:
            if is_free(rr+dr, cc+dc):
                return idx, main_axis
        return None, main_axis

    # ---------- init pattern ----------
    main_axis = "NS" if start_dir_index in (0, 1) else "EW"
    prev_lap = 0
    cur_lap = 0

    # chọn long_dir đầu: cố đi thẳng theo trục chính
    if main_axis == "NS":
        if is_free(r-1, c):
            long_dir = 0  # lên
        elif is_free(r+1, c):
            long_dir = 1  # xuống
        else:
            long_dir = 0
        side_dir = 2  # E
    else:
        if is_free(r, c+1):
            long_dir = 2  # phải
        elif is_free(r, c-1):
            long_dir = 3  # trái
        else:
            long_dir = 2
        side_dir = 0  # N

    going_longitudinal = True
    switch_lock = 0  # >0 nghĩa là vừa đổi trục, bắt buộc đi dọc >=1 bước trước khi xét rẽ

    # mark start
    if grid[r][c] == FREE_UNCOVERED:
        grid[r][c] = COVERED
        coverage_path.append((r, c))
        if callback:
            callback(grid, (r, c), coverage_path, coverage_id)

    # ---------- main loop ----------
    while True:
        moved = False

        if going_longitudinal:
            dr, dc = DIRECTIONS_BM[long_dir]
            nr, nc = r + dr, c + dc

            # Còn đi thẳng được → cứ cày
            if is_free(nr, nc):
                r, c = nr, nc
                grid[r][c] = COVERED
                coverage_path.append((r, c))
                cur_lap += 1
                if callback:
                    callback(grid, (r, c), coverage_path, coverage_id)
                if switch_lock > 0:
                    switch_lock -= 1
                moved = True
                continue

            # === ĐANG Ở BIÊN CỦA LUỐNG ===
            # Nếu vừa đổi trục và chưa đi được 1 bước dọc → chặn mọi rẽ để tránh lật side_dir
            if switch_lock > 0:
                return (r, c), grid, coverage_path

            # Đếm số hướng rẽ có thể đi 1 ô
            lateral = []
            for sd in (side_dir, opposite(side_dir)):
                sdr, sdc = DIRECTIONS_BM[sd]
                if is_free(r + sdr, c + sdc):
                    lateral.append(sd)

            if len(lateral) == 2:
                # Rẽ theo side_dir
                sd = side_dir
                sdr, sdc = DIRECTIONS_BM[sd]
                r, c = r + sdr, c + sdc
                grid[r][c] = COVERED
                coverage_path.append((r, c))
                if callback:
                    callback(grid, (r, c), coverage_path, coverage_id)

                # Hoàn tất luống cũ
                prev_lap = cur_lap
                cur_lap = 0

                # Quay đầu cho luống mới + giữ mẫu snake
                long_dir = opposite(long_dir)
                side_dir = opposite(sd)

                # --- ROLL-IN: trượt ngang thêm cho đến khi ô dọc phía trước trống ---
                # GIỮ hướng ngang vừa rẽ
                slide_dr, slide_dc = DIRECTIONS_BM[sd]
                ldr, ldc = DIRECTIONS_BM[long_dir]
                while not is_free(r + ldr, c + ldc) and is_free(r + slide_dr, c + slide_dc):
                    r += slide_dr
                    c += slide_dc
                    grid[r][c] = COVERED
                    coverage_path.append((r, c))
                    if callback:
                        callback(grid, (r, c), coverage_path, coverage_id)

                # Ép 1 bước dọc nếu có thể
                if is_free(r + ldr, c + ldc):
                    r += ldr
                    c += ldc
                    grid[r][c] = COVERED
                    coverage_path.append((r, c))
                    if callback:
                        callback(grid, (r, c), coverage_path, coverage_id)
                    cur_lap = 1

                going_longitudinal = True
                moved = True
                continue

            if len(lateral) == 1:
                # Có 1 hướng rẽ → có thể cân nhắc đổi trục (cur<prev & có sensor)
                if (cur_lap < prev_lap) and (sensor_radius > 0):
                    dir_idx, new_axis = pick_dir_by_rule(
                        r, c, main_axis, sensor_radius)
                    if new_axis != main_axis:
                        main_axis = new_axis
                        prev_lap = cur_lap
                        cur_lap = 0
                        long_dir, side_dir = apply_axis_switch(
                            long_dir, side_dir)
                        # Khoá: phải đi dọc ít nhất 1 bước trước khi cho rẽ
                        switch_lock = 1

                        # cố gắng đi ngay 1 bước dọc
                        ldr, ldc = DIRECTIONS_BM[long_dir]
                        if is_free(r + ldr, c + ldc):
                            r, c = r + ldr, c + ldc
                            grid[r][c] = COVERED
                            coverage_path.append((r, c))
                            if callback:
                                callback(grid, (r, c),
                                         coverage_path, coverage_id)
                            cur_lap = 1
                            moved = True
                            continue
                        # nếu không đi được bước dọc → rẽ theo hướng duy nhất ở dưới

                # Không đổi trục → rẽ theo hướng duy nhất rồi quay đầu
                sd = lateral[0]

                sdr, sdc = DIRECTIONS_BM[sd]
                r, c = r + sdr, c + sdc
                grid[r][c] = COVERED
                coverage_path.append((r, c))
                if callback:
                    callback(grid, (r, c), coverage_path, coverage_id)

                # Hoàn tất luống cũ
                prev_lap = cur_lap
                cur_lap = 0

                # Quay đầu cho luống mới + giữ mẫu snake
                long_dir = opposite(long_dir)
                side_dir = opposite(sd)

                # --- ROLL-IN tương tự ---
                # GIỮ hướng ngang vừa rẽ
                slide_dr, slide_dc = DIRECTIONS_BM[sd]
                ldr, ldc = DIRECTIONS_BM[long_dir]
                while not is_free(r + ldr, c + ldc) and is_free(r + slide_dr, c + slide_dc):
                    r += slide_dr
                    c += slide_dc
                    grid[r][c] = COVERED
                    coverage_path.append((r, c))
                    if callback:
                        callback(grid, (r, c), coverage_path, coverage_id)

                # Ép 1 bước dọc nếu có thể
                if is_free(r + ldr, c + ldc):
                    r += ldr
                    c += ldc
                    grid[r][c] = COVERED
                    coverage_path.append((r, c))
                    if callback:
                        callback(grid, (r, c), coverage_path, coverage_id)
                    cur_lap = 1

                going_longitudinal = True
                moved = True
                continue

        # Phòng hờ: nếu vì lý do nào đó thoát trạng thái dọc, thử 4 hướng ưu tiên
        for i, (dr, dc) in enumerate(DIRECTIONS_BM):
            nr, nc = r + dr, c + dc
            if is_free(nr, nc):
                r, c = nr, nc
                grid[r][c] = COVERED
                coverage_path.append((r, c))
                if callback:
                    callback(grid, (r, c), coverage_path, coverage_id)
                if (main_axis == "NS" and i in (0, 1)) or (main_axis == "EW" and i in (2, 3)):
                    long_dir = i
                going_longitudinal = True
                moved = True
                break

        if not moved:
            return (r, c), grid, coverage_path


# --- 6. Thuật toán BA* (Algorithm 5) ---


class BAStar:
    def __init__(self, initial_grid, start_pos, sensor_radius=10):
        # B1: Khởi tạo M rỗng (hoặc bản đồ chướng ngại vật ban đầu)
        self.grid = [row[:] for row in initial_grid]
        self.rows = len(initial_grid)
        self.cols = len(initial_grid[0])
        self.current_pos = start_pos
        self.current_cp = start_pos
        self.current_dir_index = 0  # Hướng ban đầu cho BM
        self.total_path = [start_pos]
        self.step_count = 0
        self.coverage_paths = []  # Lưu các đường coverage
        self.astar_paths = []  # Lưu các đường A*
        self.coverage_count = 0  # Đếm số lần coverage
        self.sensor_radius = sensor_radius
        # Callback functions for visualization
        self.on_step_callback = None
        self.on_backtrack_callback = None
        self.on_astar_callback = None

        self.used_backtracks = set()

    def set_callbacks(self, step_callback=None, backtrack_callback=None, astar_callback=None):
        """Set callback functions for visualization"""
        self.on_step_callback = step_callback
        self.on_backtrack_callback = backtrack_callback
        self.on_astar_callback = astar_callback

    def find_backtracking_list(self):
        """
        B3: Phát hiện danh sách điểm quay lui L (theo Công thức 8).

        Sử dụng thuật toán từ nghiên cứu BA*:
        - Kiểm tra 8 ô lân cận theo thứ tự: east, north-east, north, north-west, west, south-west, south, south-east
        - Sử dụng hàm μ(s) để xác định các điểm góc của vùng boustrophedon
        - Chỉ chọn các điểm tại góc để giảm số lượng backtracking points và boustrophedon regions
        """
        backtracking_list = []

        # Định nghĩa 8 hướng theo thứ tự: east, north-east, north, north-west, west, south-west, south, south-east
        # N(s) = {s1, s2, s3, s4, s5, s6, s7, s8}
        eight_directions = [
            (0, 1),   # s1: east
            (-1, 1),  # s2: north-east
            (-1, 0),  # s3: north
            (-1, -1),  # s4: north-west
            (0, -1),  # s5: west
            (1, -1),  # s6: south-west
            (1, 0),   # s7: south
            (1, 1)    # s8: south-east
        ]

        for r in range(self.rows):
            for c in range(self.cols):
                if self.grid[r][c] == COVERED:
                    # Lấy thông tin 8 ô lân cận
                    neighbors = []
                    for dr, dc in eight_directions:
                        nr, nc = r + dr, c + dc
                        if is_valid(self.grid, nr, nc):
                            neighbors.append(self.grid[nr][nc])
                        else:
                            # Biên của lưới coi như obstacle
                            neighbors.append(OBSTACLE)

                    # Tính μ(s) theo công thức (7)
                    mu_s = self.calculate_mu_function(neighbors)

                    # Điều kiện là backtracking point: μ(s) ≥ 1 (theo công thức 8)
                    if mu_s >= 1:
                        backtracking_list.append((r, c))

        # Loại bỏ các điểm trùng lặp và quá gần nhau
        filtered_list = self.filter_redundant_backtracking_points(
            backtracking_list)

        return filtered_list

    def calculate_mu_function(self, neighbors):
        """
        Tính hàm μ(s) theo công thức (7):
        μ(s) = b(s1,s8) + b(s1,s2) + b(s5,s6) + b(s5,s4) + b(s7,s6) + b(s7,s8)

        Trong đó b(si,sj) = 1 nếu (si is free) và (sj is blocked), ngược lại = 0
        """
        def b_function(si_status, sj_status):
            """
            b(si,sj) = 1, if (si is free) and (sj is blocked); 0, otherwise
            """
            return 1 if (si_status == FREE_UNCOVERED and sj_status == OBSTACLE) else 0

        # neighbors = [s1, s2, s3, s4, s5, s6, s7, s8]
        # Chỉ số: s1=0, s2=1, s3=2, s4=3, s5=4, s6=5, s7=6, s8=7

        if len(neighbors) != 8:
            return 0

        s1, s2, s3, s4, s5, s6, s7, s8 = neighbors

        # Tính các thành phần của μ(s)
        b_s1_s8 = b_function(s1, s8)
        b_s1_s2 = b_function(s1, s2)
        b_s5_s6 = b_function(s5, s6)
        b_s5_s4 = b_function(s5, s4)
        b_s7_s6 = b_function(s7, s6)
        b_s7_s8 = b_function(s7, s8)

        mu_s = b_s1_s8 + b_s1_s2 + b_s5_s6 + b_s5_s4 + b_s7_s6 + b_s7_s8

        return mu_s

    def estimate_reachable_uncovered_area(self, start_r, start_c):
        """
        Ước tính số ô chưa bao phủ có thể tiếp cận từ điểm cho trước.
        Sử dụng BFS để đếm các ô FREE_UNCOVERED liên thông.
        """
        visited = set()
        queue = []
        directions = [(-1, 0), (1, 0), (0, 1), (0, -1)]

        # Bắt đầu từ các ô FREE_UNCOVERED lân cận
        for dr, dc in directions:
            nr, nc = start_r + dr, start_c + dc
            if (is_valid(self.grid, nr, nc) and
                self.grid[nr][nc] == FREE_UNCOVERED and
                    (nr, nc) not in visited):
                queue.append((nr, nc))
                visited.add((nr, nc))

        # BFS để tìm vùng liên thông
        count = 0
        max_search = 50  # Giới hạn tìm kiếm để tránh vòng lặp vô hạn

        while queue and count < max_search:
            r, c = queue.pop(0)
            count += 1

            # Thêm các ô lân cận FREE_UNCOVERED
            for dr, dc in directions:
                nr, nc = r + dr, c + dc
                if (is_valid(self.grid, nr, nc) and
                    self.grid[nr][nc] == FREE_UNCOVERED and
                        (nr, nc) not in visited):
                    queue.append((nr, nc))
                    visited.add((nr, nc))

        return min(count, max_search)

    def filter_redundant_backtracking_points(self, backtracking_list):
        """
        Loại bỏ các điểm backtracking trùng lặp hoặc quá gần nhau.
        Giữ lại các điểm quan trọng nhất.
        """
        if len(backtracking_list) <= 1:
            return backtracking_list

        filtered = []
        min_distance = 3  # Khoảng cách tối thiểu giữa các điểm

        for point in backtracking_list:
            is_too_close = False
            for existing_point in filtered:
                distance = abs(point[0] - existing_point[0]) + \
                    abs(point[1] - existing_point[1])
                if distance < min_distance:
                    is_too_close = True
                    break

            if not is_too_close:
                filtered.append(point)

        return filtered

    def select_best_start_point(self, backtracking_list):
        """
        B5: Xác định điểm quay lui tốt nhất s_sp (theo Công thức 9).

        Sử dụng chiến lược greedy:
        s_sp = argmin_{s∈L} f(s, s_cp)

        Trong đó f(s, s_cp) là hàm chi phí dựa trên khoảng cách giữa điểm s trong 
        danh sách backtracking L và điểm tới hạn s_cp hiện tại.
        """
        if not backtracking_list:
            return None, 0

        best_sp = None
        min_cost = float('inf')

        cp_r, cp_c = self.current_cp

        for r, c in backtracking_list:
            # Tính hàm chi phí f(s, s_cp)
            # Có thể sử dụng các loại khoảng cách khác nhau:
            # 1. Euclidean distance
            euclidean_dist = math.sqrt((r - cp_r)**2 + (c - cp_c)**2)

            # 2. Manhattan distance
            manhattan_dist = abs(r - cp_r) + abs(c - cp_c)

            # 3. Weighted combination (ưu tiên Manhattan vì robot di chuyển theo lưới)
            cost = 0.7 * manhattan_dist + 0.3 * euclidean_dist

            # Thêm yếu tố ưu tiên cho các điểm có nhiều vùng uncovered lân cận
            uncovered_potential = self.estimate_reachable_uncovered_area(r, c)
            cost = cost - 0.1 * uncovered_potential  # Giảm cost cho điểm có potential cao

            if cost < min_cost:
                min_cost = cost
                best_sp = (r, c)

        if best_sp is None:
            return None, 0

        # Xác định hướng tiếp theo cho BM từ best starting point
        # Chọn hướng dẫn đến vùng FREE_UNCOVERED gần nhất theo thứ tự ưu tiên
        best_dir_index = 0

        for i, (dr, dc) in enumerate(DIRECTIONS_BM):
            nr, nc = best_sp[0] + dr, best_sp[1] + dc
            if is_valid(self.grid, nr, nc) and self.grid[nr][nc] == FREE_UNCOVERED:
                best_dir_index = i
                break

        return best_sp, best_dir_index

    def run(self):
        """
        Thực thi thuật toán BA*:
        - BM để bao phủ từng vùng.
        - Phát hiện danh sách backtrack.
        - Lọc ứng viên (loại s_cp hiện tại và những điểm đã dùng).
        - Chọn s_sp tốt nhất -> A* -> A*SPT -> theo dõi đường.
        - Cập nhật hướng ưu tiên cho vòng lặp kế tiếp.
        Trả về:
            total_path, grid
        """
        print("--- Bắt đầu Thuật toán BA* ---")
        step = 1

        while True:
            print(f"\n--- Chu trình Bao phủ #{step} ---")
            print(
                f"Vị trí hiện tại: {self.current_pos}, Hướng: {self.current_dir_index}")
            self.coverage_count += 1

            # B2: Bao phủ bằng BM (có truyền bán kính cảm biến)
            print("1. Thực hiện Chuyển động Boustrophedon (BM)...")
            s_cp, self.grid, coverage_path = boustrophedon_motion(
                self.grid,
                self.current_pos,
                self.current_dir_index,
                self.on_step_callback,
                self.coverage_count,
                sensor_radius=self.sensor_radius,  # <<< quan trọng
            )
            self.current_cp = s_cp
            self.coverage_paths.append(coverage_path)

            # B3: Tìm danh sách backtrack
            backtracking_list = self.find_backtracking_list()
            print(
                f"2. Đã phát hiện {len(backtracking_list)} điểm quay lui: {backtracking_list}")

            # Lọc bỏ chính s_cp và các điểm đã dùng
            candidates = [
                p for p in backtracking_list
                if p != self.current_cp and p not in self.used_backtracks
            ]
            print(f"   → Ứng viên sau lọc: {candidates}")

            # Nếu không còn ứng viên
            if not candidates:
                # Nếu không còn ô FREE nào -> hoàn tất
                if not any(FREE_UNCOVERED in row for row in self.grid):
                    print("3. Không còn ô trống. Nhiệm vụ bao phủ hoàn tất.")
                    break

                # Nếu còn ô trống nhưng không có ứng viên hợp lệ -> dừng (hoặc fallback tuỳ bạn)
                print("3. Không còn ứng viên backtrack hợp lệ. Dừng.")
                break

            # B5: Chọn điểm bắt đầu tốt nhất từ candidates (đÃ lọc)
            s_sp, next_dir_index = self.select_best_start_point(candidates)
            if s_sp is None:
                print(" !! Lỗi: Không thể tìm thấy điểm bắt đầu hợp lệ. Dừng.")
                break

            # Phòng thủ: nếu vẫn trùng s_cp thì thử ứng viên khác
            if s_sp == self.current_cp:
                others = [p for p in candidates if p != self.current_cp]
                if others:
                    s_sp, next_dir_index = self.select_best_start_point(others)
                else:
                    print(
                        " !! Sau lọc vẫn ra s_sp == s_cp, không còn ứng viên khác. Dừng.")
                    break

            print(f"4. Điểm bắt đầu tiếp theo (s_sp) được chọn: {s_sp}")
            manhattan = abs(s_cp[0] - s_sp[0]) + abs(s_cp[1] - s_sp[1])
            print(f"   Khoảng cách từ {s_cp} đến {s_sp}: {manhattan}")

            # Ghi nhận đã dùng s_sp để lần sau không chọn lại
            self.used_backtracks.add(s_sp)

            # Hiển thị backtrack point đã chọn
            if self.on_backtrack_callback:
                self.on_backtrack_callback(s_sp)

            # B6: Lập kế hoạch A*
            print(f"5. Lập kế hoạch đường dẫn A* từ {s_cp} đến {s_sp}...")
            if s_cp == s_sp:
                print("   s_cp == s_sp → bỏ qua A* (không cần di chuyển).")
                path_astar = [s_cp]
            else:
                path_astar = a_star_search(self.grid, s_cp, s_sp)
                if not path_astar:
                    print(" !! Lỗi: Không thể tìm thấy đường dẫn A* đến s_sp. Dừng.")
                    break
                print(f"   Đường dẫn A* thô: {len(path_astar)} bước.")

            # B7: Làm mịn
            path_smoothed = a_star_spt(self.grid, path_astar)
            print(
                f"6. Đường dẫn được làm mịn (A*SPT): {len(path_smoothed)} bước.")

            # Lưu cho visualize (khi có di chuyển)
            if len(path_smoothed) >= 2:
                self.astar_paths.append(path_smoothed)

            # B8: Theo dõi đường dẫn (Công thức 11)
            print("7. Theo dõi đường dẫn (Công thức 11)...")
            if len(path_smoothed) >= 2:
                if self.on_astar_callback:
                    self.on_astar_callback(path_smoothed)

                for pos in path_smoothed[1:]:
                    self.total_path.append(pos)
                    self.current_pos = pos
            else:
                # Không di chuyển (đã ở s_sp)
                self.current_pos = s_cp
                print(f"   Robot đã ở s_sp: {self.current_pos}")

            # B9: Điều chỉnh hướng cho BM tiếp theo
            self.current_dir_index = next_dir_index
            print("8. Điều chỉnh hướng ưu tiên cho BM tiếp theo.")

            step += 1

        return self.total_path, self.grid


# --- 7. Ví dụ Mô phỏng Console ---

def print_grid(grid):
    """Hiển thị mô hình M."""
    symbol_map = {
        FREE_UNCOVERED: '⬜',  # Ô trống chưa được bao phủ
        OBSTACLE: '⬛',        # Chướng ngại vật
        COVERED: '🟩',         # Ô đã được bao phủ
        'ROBOT': '🤖'          # Vị trí robot (chỉ để minh họa cuối cùng)
    }

    print("\nMô hình làm việc (M):")
    for r in range(len(grid)):
        row_str = ""
        for c in range(len(grid[0])):
            row_str += symbol_map.get(grid[r][c], ' ')
        print(row_str)
    print("-" * 20)


def run_console_demo():
    """Run console-based demo for testing"""
    MAP_SIZE = 10

    # Một bản đồ thử nghiệm với chướng ngại vật ở giữa và một số góc
    initial_map = [
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 1, 1, 0, 0, 0, 0, 0, 0, 0],
        [0, 1, 1, 0, 1, 1, 1, 1, 0, 0],
        [0, 0, 0, 0, 1, 0, 0, 1, 0, 0],
        [0, 0, 0, 0, 1, 0, 0, 1, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
        [0, 1, 1, 1, 1, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 0, 1, 1, 1, 0],
        [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    ]

    start_point = (0, 0)

    print("--- Khởi tạo Mô phỏng ---")
    print(f"Kích thước bản đồ: {MAP_SIZE}x{MAP_SIZE}")
    print(f"Vị trí bắt đầu: {start_point}")
    print_grid(initial_map)

    # Khởi tạo và chạy BA*
    ba_star_robot = BAStar(initial_map, start_point)
    final_path, final_grid = ba_star_robot.run()

    # Kết quả
    print("\n--- Kết quả Cuối cùng ---")
    print(f"Tổng số bước di chuyển (BM + A*SPT): {len(final_path) - 1}")

    # Tạo bản đồ cuối cùng với robot ở vị trí cuối cùng
    display_grid = [row[:] for row in final_grid]
    if final_path:
        # Đánh dấu vị trí cuối cùng của robot
        r, c = final_path[-1]
        display_grid[r][c] = 'ROBOT'

    print_grid(display_grid)
    print("Ghi chú: '🟩' là ô đã được bao phủ, '⬛' là chướng ngại vật, '⬜' là ô trống còn lại.")


if __name__ == '__main__':
    # Run console demo
    run_console_demo()
