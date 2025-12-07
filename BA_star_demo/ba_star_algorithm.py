from collections import deque
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

def find_adjacent_hole_rep(grid, r, c, max_bfs=500, sensor_radius=None, ignore_dir_index=None):
    """
    Tìm candidate.
    Logic cập nhật: Ưu tiên candidate có khoảng cách (dist) DÀI NHẤT (Longest).
    """
    rows, cols = len(grid), len(grid[0])
    FREE = FREE_UNCOVERED

    # --- Helper: BFS kiểm tra xem có thoát ra biên được không ---
    def is_reachable_to_border(sr, sc, bfs_limit=max_bfs, R=sensor_radius):
        q = deque()
        visited = set()
        q.append((sr, sc, 0))
        visited.add((sr, sc))
        steps = 0
        reached_r_boundary = False
        dirs4 = [(-1, 0), (1, 0), (0, -1), (0, 1)]

        while q:
            rr, cc, dist = q.popleft()

            if rr == 0 or rr == rows - 1 or cc == 0 or cc == cols - 1:
                return True  # Thoát ra biên -> Không phải Hole

            if R is not None and dist >= R:
                for dr, dc in dirs4:
                    nr, nc = rr + dr, cc + dc
                    if 0 <= nr < rows and 0 <= nc < cols and grid[nr][nc] == FREE and (nr, nc) not in visited:
                        reached_r_boundary = True
                steps += 1
                if bfs_limit is not None and steps >= bfs_limit:
                    return None
                continue

            for dr, dc in dirs4:
                nr, nc = rr + dr, cc + dc
                if 0 <= nr < rows and 0 <= nc < cols and (nr, nc) not in visited and grid[nr][nc] == FREE:
                    visited.add((nr, nc))
                    q.append((nr, nc, dist + 1))

            steps += 1
            if bfs_limit is not None and steps >= bfs_limit:
                return None

        if reached_r_boundary:
            return None
        return False

    # --- Helper: Tính khoảng cách ---
    def get_dist_to_obstacle(start_r, start_c, dr, dc):
        dist = 0
        curr_r, curr_c = start_r + dr, start_c + dc
        while 0 <= curr_r < rows and 0 <= curr_c < cols:
            if grid[curr_r][curr_c] in (OBSTACLE, COVERED):
                break
            dist += 1
            curr_r += dr
            curr_c += dc
        return dist

    # --- Main Logic ---

    # 1. Kiểm tra ô hiện tại
    if 0 <= r < rows and 0 <= c < cols and grid[r][c] == FREE:
        res = is_reachable_to_border(r, c, bfs_limit=max_bfs, R=sensor_radius)
        if res is False:
            return (r, c)

    # 2. Thu thập candidates
    candidates = []  # List tuple: ((nr, nc), distance)

    ignore_delta = None
    if ignore_dir_index is not None and 0 <= ignore_dir_index < 4:
        ignore_delta = DIRECTIONS_BM[ignore_dir_index]

    neighbor_dirs = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    for dr, dc in neighbor_dirs:
        if ignore_delta is not None and (dr, dc) == ignore_delta:
            continue

        nr, nc = r + dr, c + dc

        if 0 <= nr < rows and 0 <= nc < cols and grid[nr][nc] == FREE:
            res = is_reachable_to_border(
                nr, nc, bfs_limit=max_bfs, R=sensor_radius)

            if res is False:  # Là Hole
                dist = get_dist_to_obstacle(r, c, dr, dc)
                candidates.append(((nr, nc), dist))
                # print(f"DEBUG: Found HOLE at {nr},{nc}, Dist={dist}")

    # 3. Chọn candidate tốt nhất
    if candidates:
        # --- LOGIC MỚI Ở ĐÂY ---
        # Ưu tiên đoạn DÀI HƠN -> Sắp xếp giảm dần (Descending)
        # Cách 1: dùng reverse=True
        candidates.sort(key=lambda x: x[1], reverse=True)

        # Cách 2 (nếu muốn dùng min/max): sort key = -x[1]
        # candidates.sort(key=lambda x: -x[1])

        best_hole = candidates[0][0]
        max_dist = candidates[0][1]  # Đây là khoảng cách dài nhất

        print(
            f"DEBUG: >>> Chọn HOLE tại {best_hole} với khoảng cách DÀI NHẤT {max_dist}")
        return best_hole

    return None

def boustrophedon_motion(
    grid,
    start_pos,
    start_dir_index=0,
    callback=None,
    coverage_id=1,
    sensor_radius=0,
    stop_on_hole=False,
    allow_hole_detection=True,
    is_hole_scanning=False
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

    print("BM START POINT:", r, c, "start_dir=", start_dir_index)

    coverage_path = []

    # ---------- helpers ----------
    def is_free(rr, cc):
        return 0 <= rr < rows and 0 <= cc < cols and grid[rr][cc] == FREE_UNCOVERED

    def opposite(didx):  # 0:N 1:S 2:E 3:W
        return {0: 1, 1: 0, 2: 3, 3: 2}[didx]

    def apply_axis_switch(long_dir, side_dir):
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
            if grid[x][y] == OBSTACLE or grid[x][y] == COVERED:
                seen = True
                break
            steps += 1
        return steps, seen

    def pick_dir_by_rule(rr, cc, main_axis, R, delta_penalty=0.6):
        """
        Trả về (dir_idx, new_axis) theo luật cảm biến có xét 'một bên quá xa'.
        - Nếu hai phía của một trục đều 'seen' (gặp biên/chướng ngại trong R) → dùng tổng d làm score như cũ.
        - Nếu chỉ thấy một phía → dùng d_phía_thấy + (R * (1 - delta_penalty)) làm score mềm cho phía kia (bị phạt).
        """

        def sense_all(r, c, R):
            dN, sN = sense_one_dir(r, c, -1, 0, R)
            dS, sS = sense_one_dir(r, c,  1, 0, R)
            dE, sE = sense_one_dir(r, c,  0, 1, R)
            dW, sW = sense_one_dir(r, c,  0, -1, R)
            return (dN, sN, dS, sS, dE, sE, dW, sW)

        dN, sN, dS, sS, dE, sE, dW, sW = sense_all(rr, cc, R)

        # --- chấm điểm trục NS ---
        if sN and sS:
            score_NS = dN + dS
            NS_known = True
        else:
            # soft-score: phía không thấy coi như R, nhưng bị phạt
            dN_soft = dN if sN else R / (1 - delta_penalty)
            dS_soft = dS if sS else R / (1 - delta_penalty)
            score_NS = dN_soft + dS_soft
            NS_known = False

        # --- chấm điểm trục EW ---
        if sE and sW:
            score_EW = dE + dW
            EW_known = True
        else:
            dE_soft = dE if sE else R / (1 - delta_penalty)
            dW_soft = dW if sW else R / (1 - delta_penalty)
            score_EW = dE_soft + dW_soft
            EW_known = False
        print(f"SCORE: score_EW = {score_EW} and score_NS = {score_NS}")
        # --- chọn trục: ưu tiên trục có score cao hơn; nếu bằng, ưu tiên khác trục hiện tại ---
        if score_NS > score_EW:
            chosen_axis = "NS"
            print(f"Chọn hướng NS")
        elif score_EW > score_NS:
            chosen_axis = "EW"
            print(f"Chọn hướng EW")
        else:
            chosen_axis = ("EW" if main_axis == "EW" else "NS")
            print(f"Chọn hướng {main_axis}")

        # --- trên trục đã chọn, chọn HƯỚNG 'gần hơn' để giảm roll-in ---
        if chosen_axis == "NS":
            # Nếu cả N và S đều free: chọn hướng có d nhỏ hơn
            cand = []
            if is_free(rr-1, cc):  # N
                cand.append((0, dN))
            if is_free(rr+1, cc):  # S
                cand.append((1, dS))
            if cand:
                cand.sort(key=lambda x: x[1])
                return cand[0][0], "NS"
            return (0 if dN >= dS else 1), "NS"

        else:  # "EW"
            cand = []
            if is_free(rr, cc+1):  # E
                cand.append((2, dE))
            if is_free(rr, cc-1):  # W
                cand.append((3, dW))
            if cand:
                cand.sort(key=lambda x: x[1])
                return cand[0][0], "EW"
            return (2 if dE >= dW else 3), "EW"

    def pick_dir_by_rule_when_run(rr, cc, cur_lap, main_axis, R, delta_penalty=0.6):
        """
        Phiên bản đơn giản của pick_dir_by_rule cho chạy:
        - Chỉ xét trục tương phản (nếu main_axis == "NS" => tính score_EW, ngược lại tính score_NS).
        - So sánh cur_lap với score_trục_khac: nếu cur_lap > score_other -> GIỮ trục hiện tại,
        ngược lại -> ĐỔI trục.
        - Trả về (dir_idx, resulting_axis). dir_idx: 0=N,1=S,2=E,3=W.
        - delta_penalty: dùng để tính soft value cho phía "không seen": soft_val = R * (1 - delta_penalty).
        """
        # bảo đảm delta_penalty hợp lệ
        if delta_penalty < 0:
            delta_penalty = 0.0
        if delta_penalty >= 1.0:
            delta_penalty = 0.9999

        # cảm biến bốn hướng
        dN, sN = sense_one_dir(rr, cc, -1, 0, R)
        dS, sS = sense_one_dir(rr, cc,  1, 0, R)
        dE, sE = sense_one_dir(rr, cc,  0, 1, R)
        dW, sW = sense_one_dir(rr, cc,  0, -1, R)

        soft_val = R / (1 - delta_penalty)

        # helper: chọn hướng trên trục NS theo nguyên tắc: ưu tiên ô free, rồi d nhỏ hơn
        def choose_NS():
            cand = []
            if is_free(rr-1, cc):
                cand.append((0, dN))
            if is_free(rr+1, cc):
                cand.append((1, dS))
            if cand:
                cand.sort(key=lambda x: x[1])  # d nhỏ hơn ưu tiên
                return cand[0][0]
            # fallback: chọn hướng có d nhỏ hơn (gần hơn)
            return 0 if dN <= dS else 1

        # helper: chọn hướng trên trục EW
        def choose_EW():
            cand = []
            if is_free(rr, cc+1):
                cand.append((2, dE))
            if is_free(rr, cc-1):
                cand.append((3, dW))
            if cand:
                cand.sort(key=lambda x: x[1])
                return cand[0][0]
            return 2 if dE <= dW else 3

        # Tính score trục đối diện (soft nếu một phía unknown)
        if main_axis == "NS":
            # ta so sánh cur_lap với score_EW
            score_E = dE if sE else soft_val
            score_W = dW if sW else soft_val
            score_EW = score_E + score_W
            # Quy tắc: nếu cur_lap > score_EW -> giữ NS, else -> đổi sang EW
            print(f"SCORE: score_EW = {score_EW} and score_NS = {cur_lap}")
            if cur_lap > score_EW:
                # giữ trục NS
                chosen_dir = choose_NS()
                return chosen_dir, "NS"
            else:
                # đổi sang EW
                chosen_dir = choose_EW()
                return chosen_dir, "EW"

        else:  # main_axis == "EW"
            score_N = dN if sN else soft_val
            score_S = dS if sS else soft_val
            score_NS = score_N + score_S
            print(f"SCORE: score_EW = {cur_lap} and score_NS = {score_NS}")
            if cur_lap > score_NS:
                # giữ trục EW
                chosen_dir = choose_EW()
                return chosen_dir, "EW"
            else:
                # đổi sang NS
                chosen_dir = choose_NS()
                return chosen_dir, "NS"

    def immediate_free(d):
        nr, nc = r + DIRECTIONS_BM[d][0], c + DIRECTIONS_BM[d][1]
        return 0 <= nr < rows and 0 <= nc < cols and grid[nr][nc] == FREE_UNCOVERED

    # --- Helper: Kiểm tra tường tại hướng side_dir so với vị trí (r, c) ---

    def is_wall_at_side(grid, r, c, side_dir):
        rows, cols = len(grid), len(grid[0])
        sr = r + DIRECTIONS_BM[side_dir][0]
        sc = c + DIRECTIONS_BM[side_dir][1]

        # Ra ngoài biên -> Coi là tường
        if not (0 <= sr < rows and 0 <= sc < cols):
            return True
        # Gặp OBSTACLE hoặc COVERED -> Coi là tường
        if grid[sr][sc] in (OBSTACLE, COVERED):
            return True
        return False

    # --- Hàm 1: Xác định trạng thái bám tường ban đầu (Init) ---


    def determine_initial_wall_state(grid, r, c, long_dir):
        """
        Kiểm tra xem tại vị trí xuất phát, robot có đang sát tường không.
        Trả về: (walking_along_wall, wall_side)
        """
        # Định nghĩa hướng tương đối
        _LEFT_OF_INIT = {0: 3, 1: 2, 2: 0, 3: 1}
        _RIGHT_OF_INIT = {0: 2, 1: 3, 2: 1, 3: 0}

        init_left_sd = _LEFT_OF_INIT[long_dir]
        init_right_sd = _RIGHT_OF_INIT[long_dir]

        # Kiểm tra 2 bên
        is_left_wall = is_wall_at_side(grid, r, c, init_left_sd)
        is_right_wall = is_wall_at_side(grid, r, c, init_right_sd)

        walking_along_wall = False
        wall_side = None

        if is_left_wall:
            walking_along_wall = True
            wall_side = init_left_sd
            print(f"INIT: Bắt đầu sát tường TRÁI (Hướng {wall_side})")
        elif is_right_wall:
            walking_along_wall = True
            wall_side = init_right_sd
            print(f"INIT: Bắt đầu sát tường PHẢI (Hướng {wall_side})")
        else:
            print("INIT: Bắt đầu ở vùng trống (Không sát tường)")

        return walking_along_wall, wall_side

    def check_wall_look_ahead(grid, r, c, nr, nc, long_dir):
        """
        So sánh trạng thái tường ở ô hiện tại (r,c) và ô sắp đến (nr,nc).
        Phát hiện: Vào hẻm (Case C) hoặc Chạm tường (Case D).
        Trả về: (should_check_hole, reason_message)
        """
        LEFT_OF = {0: 3, 1: 2, 2: 0, 3: 1}
        RIGHT_OF = {0: 2, 1: 3, 2: 1, 3: 0}
        left_sd = LEFT_OF[long_dir]
        right_sd = RIGHT_OF[long_dir]

        # Trạng thái tại ô ĐANG ĐỨNG
        cur_left = is_wall_at_side(grid, r, c, left_sd)
        cur_right = is_wall_at_side(grid, r, c, right_sd)

        # Trạng thái tại ô SẮP ĐẾN
        next_left = is_wall_at_side(grid, nr, nc, left_sd)
        next_right = is_wall_at_side(grid, nr, nc, right_sd)

        should_check = False
        reason = ""

        # CASE C: VÀO HẺM (1 vách -> 2 vách)
        if (cur_left != cur_right) and (next_left and next_right):
            should_check = True
            reason = "⚠️ Sắp VÀO hẻm (1->2 vách)"

        # CASE D: BẮT ĐẦU BÁM TƯỜNG (0 vách -> 1 hoặc 2 vách)
        elif not (cur_left or cur_right) and (next_left or next_right):
            should_check = True
            reason = "🧱 Sắp CHẠM tường (0->1 vách)"

        return should_check, reason

    # --- Hàm 3: Cập nhật trạng thái sau khi di chuyển (Post-Move) ---

    def update_wall_state_post_move(grid, r, c, pr, pc, long_dir, walking_along_wall, wall_side):
        """
        So sánh trạng thái tường ở ô cũ (pr,pc) và ô mới (r,c).
        Phát hiện: Hết tường (Case A) hoặc Thoát hẻm (Case B).
        Cập nhật lại biến walking_along_wall và wall_side.
        Trả về: (new_walking_state, new_wall_side, should_check_hole, reason_message)
        """
        LEFT_OF = {0: 3, 1: 2, 2: 0, 3: 1}
        RIGHT_OF = {0: 2, 1: 3, 2: 1, 3: 0}
        left_sd = LEFT_OF[long_dir]
        right_sd = RIGHT_OF[long_dir]

        # Trạng thái tại ô MỚI (Vừa bước vào)
        now_left = is_wall_at_side(grid, r, c, left_sd)
        now_right = is_wall_at_side(grid, r, c, right_sd)

        # Trạng thái tại ô CŨ (Vừa đi qua)
        old_left = is_wall_at_side(grid, pr, pc, left_sd)
        old_right = is_wall_at_side(grid, pr, pc, right_sd)

        should_check = False
        reason = ""

        # Logic kiểm tra sự kiện
        if walking_along_wall:
            # CASE A: HẾT TƯỜNG (Ra vùng trống)
            if not (now_left or now_right):
                should_check = True
                reason = "❌ Hết tường (Ra vùng trống)"
                walking_along_wall = False  # Tắt bám tường
                wall_side = None

            # CASE B: THOÁT HẺM (2 vách -> 1 vách)
            elif (old_left and old_right) and (now_left != now_right):
                should_check = True
                reason = "⚠️ Thoát hẻm (2->1 vách)"
                # Vẫn giữ walking_along_wall = True

        # Cập nhật wall_side nếu đang có tường (hoặc vừa bắt được tường từ Case D ở bước Pre)
        if now_left or now_right:
            if not walking_along_wall:
                walking_along_wall = True  # Bật lại trạng thái nếu vô tình tắt hoặc mới vào

            if now_left:
                wall_side = left_sd
            elif now_right:
                wall_side = right_sd

        return walking_along_wall, wall_side, should_check, reason
    
    # ---------- init pattern ----------
    main_axis = "NS"
    prev_lap = 0
    cur_lap = 1

    dir_idx, new_axis = pick_dir_by_rule(r, c, main_axis, sensor_radius)
    # dùng đề xuất trục từ pick_dir_by_rule
    main_axis = new_axis

    # Nếu dir_idx là hướng khả dụng (ô kế tiếp free) -> dùng luôn


    if dir_idx is not None and immediate_free(dir_idx):
        long_dir = dir_idx
    else:
        # nếu đề xuất không đi được, chọn 1 hướng trên cùng trục (ưu tiên theo sensor nếu bật)
        chosen = None
        if main_axis == "NS":
            candidates = [0, 1]  # N, S
        else:
            candidates = [2, 3]  # E, W

        # try pick candidate with max forward free steps if sensor available
        best_score = -1
        for d in candidates:
            if immediate_free(d):
                if sensor_radius > 0:
                    dr, dc = DIRECTIONS_BM[d]
                    steps, seen = sense_one_dir(r, c, dr, dc, sensor_radius)
                    score = steps
                else:
                    score = 1
                if score > best_score:
                    best_score = score
                    chosen = d

        # fallback: nếu không có candidate trên cùng trục free, tìm bất kỳ hướng free
        if chosen is None:
            for d in (0, 1, 2, 3):
                if immediate_free(d):
                    chosen = d
                    break

        # cuối cùng nếu vẫn None (surrounded), fallback 0
        long_dir = chosen if chosen is not None else 0
    # set side_dir sensibly (opposite axis)
    if main_axis == "NS":
        # prefer E if free else W
        side_dir = 2 if (
            0 <= c+1 < cols and grid[r][c+1] == FREE_UNCOVERED) else 3
    else:
        # prefer N if free else S
        side_dir = 0 if (
            0 <= r-1 < rows and grid[r-1][c] == FREE_UNCOVERED) else 1

    going_longitudinal = True
    switch_lock = 0  # >0 nghĩa là vừa đổi trục, bắt buộc đi dọc >=1 bước trước khi xét rẽ

   # ==============================================================================
    walking_along_wall, wall_side = determine_initial_wall_state(
        grid, r, c, long_dir)

    # ==============================================================================

    # mark start
    # ==============================================================================
    if grid[r][c] != OBSTACLE:
        # Chỉ tô màu nếu nó chưa được tô, nhưng LUÔN thêm vào path
        if grid[r][c] == FREE_UNCOVERED:
            grid[r][c] = COVERED

        coverage_path.append((r, c))

        if callback:
            callback(grid, (r, c), coverage_path, coverage_id)
    # if stop_on_hole and allow_hole_detection:
    #     hole_rep = find_adjacent_hole_rep(
    #         grid, r, c, max_bfs=2000, sensor_radius=sensor_radius, ignore_dir_index=long_dir)
    #     if hole_rep:
    #         return (r, c), grid, coverage_path,hole_rep, long_dir

    # ---------- main loop ----------
    while True:
        moved = False

        if going_longitudinal:
            # Tính toán tọa độ tiếp theo
            dr, dc = DIRECTIONS_BM[long_dir]
            nr, nc = r + dr, c + dc

            # ==============================================================================
            # 2. CHECK LOOK-AHEAD (PRE-MOVE) (Dùng hàm con)
            # ==============================================================================
            if is_free(nr, nc):
                should_check_pre, reason_pre = check_wall_look_ahead(
                    grid, r, c, nr, nc, long_dir)

                if should_check_pre:
                    print(
                        f"   [PRE-CHECK] {reason_pre}. Ô tiếp theo {nr},{nc} bị chặn. Check Hole tại {r},{c}!")
                    if stop_on_hole and allow_hole_detection:
                        hole_rep = find_adjacent_hole_rep(
                            grid, r, c, max_bfs=2000, sensor_radius=sensor_radius, ignore_dir_index=long_dir)
                        if hole_rep:
                            return (r, c), grid, coverage_path, hole_rep, long_dir

            # ==============================================================================
            # THỰC HIỆN DI CHUYỂN
            # ==============================================================================
            if is_free(nr, nc):
                # Lưu tọa độ cũ để dùng cho Post-check
                pr, pc = r, c

                # Cập nhật tọa độ mới
                r, c = nr, nc
                grid[r][c] = COVERED
                coverage_path.append((r, c))
                cur_lap += 1
                if callback:
                    callback(grid, (r, c), coverage_path, coverage_id)
                if switch_lock > 0:
                    switch_lock -= 1
                moved = True

                # ==============================================================================
                # 3. CHECK POST-MOVE & UPDATE STATE (Dùng hàm con)
                # ==============================================================================
                walking_along_wall, wall_side, should_check_post, reason_post = update_wall_state_post_move(
                    grid, r, c, pr, pc, long_dir, walking_along_wall, wall_side
                )
                if is_hole_scanning:
                    should_check_post = False
                # (Optional) Nếu muốn check hole tại post-event thì dùng biến should_check_post
                if should_check_post:
                    print(
                        f"   [POST-CHECK] {reason_post} tại {r},{c}. Check Hole!")
                    if stop_on_hole and allow_hole_detection:
                        hole_rep = find_adjacent_hole_rep(
                            grid, r, c, max_bfs=2000, sensor_radius=sensor_radius, ignore_dir_index=long_dir)
                        if hole_rep:
                            return (r, c), grid, coverage_path, hole_rep, long_dir

                continue  # Hết vòng lặp longitudinal


            # Đếm số hướng rẽ có thể đi 1 ô (hai bên lateral)
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
                cur_lap = 1
                long_dir = opposite(long_dir)
                side_dir = opposite(sd)

                # --- ROLL-IN: trượt ngang thêm cho đến khi ô dọc phía trước trống ---
                slide_dr, slide_dc = DIRECTIONS_BM[sd]
                ldr, ldc = DIRECTIONS_BM[long_dir]
                if not is_free(r + ldr, c + ldc) and is_free(r + slide_dr, c + slide_dc):
                    r += slide_dr
                    c += slide_dc
                    grid[r][c] = COVERED
                    coverage_path.append((r, c))
                    print("ROLL-IN: thực hiện 1 bước trượt ngang (limit=1)")
                    if callback:
                        callback(grid, (r, c), coverage_path, coverage_id)

                # -----------------------------------------------------------
                # [FIX] Ép 1 bước dọc CÓ KIỂM TRA (Look-Ahead)
                # -----------------------------------------------------------
                ldr, ldc = DIRECTIONS_BM[long_dir]
                nr, nc = r + ldr, c + ldc  # Tính thử tọa độ bước ép

                if is_free(nr, nc):
                    # 1. CHECK LOOK-AHEAD TRƯỚC KHI BƯỚC
                    # Vì đây là bước đầu tiên của luống mới, ta cần check ngay xem có chui vào hẻm không
                    should_check_pre, reason_pre = check_wall_look_ahead(
                        grid, r, c, nr, nc, long_dir)

                    if should_check_pre:
                        print(
                            f"   [FORCE-STEP CHECK] {reason_pre} tại {nr},{nc}. Check Hole!")
                        if stop_on_hole and allow_hole_detection:
                            hole_rep = find_adjacent_hole_rep(
                                grid, r, c, max_bfs=2000, sensor_radius=sensor_radius, ignore_dir_index=long_dir)
                            if hole_rep:
                                return (r, c), grid, coverage_path, hole_rep, long_dir

                    if stop_on_hole and allow_hole_detection:
                        hole_rep = find_adjacent_hole_rep(
                            grid, r, c, max_bfs=2000, sensor_radius=sensor_radius, ignore_dir_index=long_dir)
                        if hole_rep:
                            return (r, c), grid, coverage_path, hole_rep, long_dir

                    # 2. Nếu an toàn (hoặc không phát hiện hole), thực hiện bước đi
                    r, c = nr, nc
                    grid[r][c] = COVERED
                    coverage_path.append((r, c))
                    if callback:
                        callback(grid, (r, c), coverage_path, coverage_id)
                    cur_lap = 2
                else:
                    cur_lap = 1

                # -----------------------------------------------------------

                # [NEW] CHECK TƯỜNG CHO LUỐNG MỚI (Cập nhật trạng thái sau khi đã ép bước - hoặc đứng yên)
                walking_along_wall, wall_side = determine_initial_wall_state(
                    grid, r, c, long_dir)

                going_longitudinal = True
                moved = True
                continue
            if len(lateral) == 1:
                # Có 1 hướng rẽ → có thể cân nhắc đổi trục (cur<prev & có sensor)
                if (cur_lap < prev_lap) and (sensor_radius > 0):
                    print(
                        f"-COMPARE:  cur_lap: {cur_lap} and prev_lap: {prev_lap}")
                    dir_idx, new_axis = pick_dir_by_rule_when_run(
                        r, c, cur_lap, main_axis, sensor_radius)
                    if new_axis != main_axis:
                        main_axis = new_axis
                        prev_lap = 0
                        cur_lap = 1
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

                            # [NEW] Check tường ngay sau khi đổi trục và bước 1 bước
                            walking_along_wall, wall_side = determine_initial_wall_state(
                                grid, r, c, long_dir)

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

                prev_lap = cur_lap
                cur_lap = 1

                # Quay đầu cho luống mới + giữ mẫu snake
                long_dir = opposite(long_dir)
                side_dir = opposite(sd)

                slide_dr, slide_dc = DIRECTIONS_BM[sd]
                ldr, ldc = DIRECTIONS_BM[long_dir]

                if not is_free(r + ldr, c + ldc) and is_free(r + slide_dr, c + slide_dc):
                    r += slide_dr
                    c += slide_dc
                    grid[r][c] = COVERED
                    coverage_path.append((r, c))
                    print("ROLL-IN: thực hiện 1 bước trượt ngang (limit=1)")
                    if callback:
                        callback(grid, (r, c), coverage_path, coverage_id)

                # Ép 1 bước dọc nếu có thể
                # -----------------------------------------------------------
                # [FIX] Ép 1 bước dọc CÓ KIỂM TRA (Look-Ahead)
                # -----------------------------------------------------------
                ldr, ldc = DIRECTIONS_BM[long_dir]
                nr, nc = r + ldr, c + ldc  # Tính thử tọa độ bước ép

                if is_free(nr, nc):
                    # 1. CHECK LOOK-AHEAD TRƯỚC KHI BƯỚC
                    # Vì đây là bước đầu tiên của luống mới, ta cần check ngay xem có chui vào hẻm không
                    should_check_pre, reason_pre = check_wall_look_ahead(
                        grid, r, c, nr, nc, long_dir)

                    if should_check_pre:
                        print(
                            f"   [FORCE-STEP CHECK] {reason_pre} tại {nr},{nc}. Check Hole!")
                        if stop_on_hole and allow_hole_detection:
                            hole_rep = find_adjacent_hole_rep(
                                grid, r, c, max_bfs=2000, sensor_radius=sensor_radius, ignore_dir_index=long_dir)
                            if hole_rep:
                                return (r, c), grid, coverage_path, hole_rep, long_dir

                    if stop_on_hole and allow_hole_detection:
                        hole_rep = find_adjacent_hole_rep(
                            grid, r, c, max_bfs=2000, sensor_radius=sensor_radius, ignore_dir_index=long_dir)
                        if hole_rep:
                            return (r, c), grid, coverage_path, hole_rep, long_dir

                    # 2. Nếu an toàn (hoặc không phát hiện hole), thực hiện bước đi
                    r, c = nr, nc
                    grid[r][c] = COVERED
                    coverage_path.append((r, c))
                    if callback:
                        callback(grid, (r, c), coverage_path, coverage_id)
                    
                    cur_lap = 2
                else:
                    cur_lap = 1

                # -----------------------------------------------------------

                # [NEW] CHECK TƯỜNG CHO LUỐNG MỚI (Cập nhật trạng thái sau khi đã ép bước - hoặc đứng yên)
                walking_along_wall, wall_side = determine_initial_wall_state(
                    grid, r, c, long_dir)

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
                #! 2-12
                break

        if not moved:
            return (r, c), grid, coverage_path, None, long_dir


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

        # NEW: hole support and resume stack
        # hole_map: rep_coord -> component_list (list of (r,c))
        self.hole_map = dict()
        # resume stack: push s_cp before going into hole; pop when hole done -> return
        self.resume_stack = []
        self.in_hole_scan = False
        # hole scan depth control (to avoid unbounded recursion / nested scanning)
        self.hole_scan_depth = 0
        self.max_hole_scan_depth = 5

    def set_callbacks(self, step_callback=None, backtrack_callback=None, astar_callback=None):
        """Set callback functions for visualization"""
        self.on_step_callback = step_callback
        self.on_backtrack_callback = backtrack_callback
        self.on_astar_callback = astar_callback

    # ------------------ Hole detection helpers ------------------
    def detect_holes(self):
        """
        Find all connected components of FREE_UNCOVERED that are NOT reachable
        from the grid border (i.e. enclosed holes). Return list of components.
        """
        FREE = FREE_UNCOVERED
        rows, cols = self.rows, self.cols
        visited = [[False]*cols for _ in range(rows)]
        q = deque()

        # enqueue all FREE on outer boundary
        for r in range(rows):
            for c in (0, cols-1):
                if self.grid[r][c] == FREE and not visited[r][c]:
                    visited[r][c] = True
                    q.append((r, c))
        for c in range(cols):
            for r in (0, rows-1):
                if self.grid[r][c] == FREE and not visited[r][c]:
                    visited[r][c] = True
                    q.append((r, c))

        dirs = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        while q:
            r, c = q.popleft()
            for dr, dc in dirs:
                nr, nc = r+dr, c+dc
                if 0 <= nr < rows and 0 <= nc < cols and not visited[nr][nc] and self.grid[nr][nc] == FREE:
                    visited[nr][nc] = True
                    q.append((nr, nc))

        # collect components that are FREE but not visited => holes
        hole_visited = [[False]*cols for _ in range(rows)]
        holes = []
        for r in range(rows):
            for c in range(cols):
                if self.grid[r][c] == FREE and not visited[r][c] and not hole_visited[r][c]:
                    comp = []
                    q = deque()
                    q.append((r, c))
                    hole_visited[r][c] = True
                    comp.append((r, c))
                    while q:
                        rr, cc = q.popleft()
                        for dr, dc in dirs:
                            nr, nc = rr+dr, cc+dc
                            if 0 <= nr < rows and 0 <= nc < cols:
                                if (self.grid[nr][nc] == FREE and not visited[nr][nc] and not hole_visited[nr][nc]):
                                    hole_visited[nr][nc] = True
                                    q.append((nr, nc))
                                    comp.append((nr, nc))
                    holes.append(comp)
        return holes

    def get_hole_representative(self, comp, prefer='entry', robot_pos=None):
        """
        Choose representative coordinate for a hole component.
        prefer: 'entry' (cell adjacent to COVERED/OBSTACLE), 'nearest' (to robot), 'centroid'
        """
        if not comp:
            return None
        if prefer == 'centroid':
            sr = sum(p[0] for p in comp)/len(comp)
            sc = sum(p[1] for p in comp)/len(comp)
            best = min(comp, key=lambda p: (p[0]-sr)**2 + (p[1]-sc)**2)
            return best
        if prefer == 'entry':
            # return first free cell that neighbors COVERED or OBSTACLE (easy entrance)
            for (r, c) in comp:
                for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    nr, nc = r+dr, c+dc
                    if not (0 <= nr < self.rows and 0 <= nc < self.cols) or self.grid[nr][nc] in (COVERED, OBSTACLE):
                        return (r, c)
            return comp[0]
        # default nearest
        if robot_pos is None:
            return comp[0]
        rx, ry = robot_pos
        return min(comp, key=lambda p: abs(p[0]-rx)+abs(p[1]-ry))

    # ------------------ existing methods (unchanged) ------------------
    def find_backtracking_list(self):
        """
        B3: Phát hiện danh sách điểm quay lui L (theo Công thức 8).
        (unchanged) returns list of (r,c) for covered-cells with mu >=1
        """
        backtracking_list = []

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
                    neighbors = []
                    for dr, dc in eight_directions:
                        nr, nc = r + dr, c + dc
                        if is_valid(self.grid, nr, nc):
                            neighbors.append(self.grid[nr][nc])
                        else:
                            neighbors.append(OBSTACLE)
                    mu_s = self.calculate_mu_function(neighbors)
                    if mu_s >= 1:
                        backtracking_list.append((r, c))

        filtered_list = self.filter_redundant_backtracking_points(
            backtracking_list)
        return filtered_list

    def calculate_mu_function(self, neighbors):
        def b_function(si_status, sj_status):
            return 1 if (si_status == FREE_UNCOVERED and sj_status == OBSTACLE) else 0

        if len(neighbors) != 8:
            return 0

        s1, s2, s3, s4, s5, s6, s7, s8 = neighbors
        b_s1_s8 = b_function(s1, s8)
        b_s1_s2 = b_function(s1, s2)
        b_s3_s2 = b_function(s3, s2)
        b_s3_s4 = b_function(s3, s4)
        b_s5_s6 = b_function(s5, s6)
        b_s5_s4 = b_function(s5, s4)
        b_s7_s6 = b_function(s7, s6)
        b_s7_s8 = b_function(s7, s8)
        mu_s = b_s1_s8 + b_s1_s2 + b_s3_s2 + b_s3_s4 + b_s5_s6 + b_s5_s4 + b_s7_s6 + b_s7_s8
        return mu_s

    def estimate_reachable_uncovered_area(self, start_r, start_c):
        visited = set()
        queue = []
        directions = [(-1, 0), (1, 0), (0, 1), (0, -1)]
        for dr, dc in directions:
            nr, nc = start_r + dr, start_c + dc
            if (is_valid(self.grid, nr, nc) and
                self.grid[nr][nc] == FREE_UNCOVERED and
                    (nr, nc) not in visited):
                queue.append((nr, nc))
                visited.add((nr, nc))
        count = 0
        max_search = 50
        while queue and count < max_search:
            r, c = queue.pop(0)
            count += 1
            for dr, dc in directions:
                nr, nc = r + dr, c + dc
                if (is_valid(self.grid, nr, nc) and
                    self.grid[nr][nc] == FREE_UNCOVERED and
                        (nr, nc) not in visited):
                    queue.append((nr, nc))
                    visited.add((nr, nc))
        return min(count, max_search)

    def filter_redundant_backtracking_points(self, backtracking_list):
        if len(backtracking_list) <= 1:
            return backtracking_list
        filtered = []
        min_distance = 3
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

    # ------------------ select_best_start_point (modified to prefer holes) ------------------
    def select_best_start_point(self, backtracking_list):
        """
        backtracking_list: list of (r,c)
        If any (r,c) matches a hole rep (self.hole_map), bias cost to prefer it.
        """
        if not backtracking_list:
            return None, 0

        best_sp = None
        min_cost = float('inf')
        cp_r, cp_c = self.current_cp

        for r, c in backtracking_list:
            euclidean_dist = math.sqrt((r - cp_r)**2 + (c - cp_c)**2)
            manhattan_dist = abs(r - cp_r) + abs(c - cp_c)
            cost = 0.7 * manhattan_dist + 0.3 * euclidean_dist
            uncovered_potential = self.estimate_reachable_uncovered_area(r, c)
            cost = cost - 0.1 * uncovered_potential

            # Bias: if this coordinate is a hole representative, strongly prefer it (reduce cost)
            if (r, c) in self.hole_map:
                comp = self.hole_map[(r, c)]
                # big negative bias scaled by component size (prefer bigger holes)
                cost = cost - (1000 + len(comp))

            if cost < min_cost:
                min_cost = cost
                best_sp = (r, c)

        if best_sp is None:
            return None, 0

        best_dir_index = 0
        for i, (dr, dc) in enumerate(DIRECTIONS_BM):
            nr, nc = best_sp[0] + dr, best_sp[1] + dc
            if is_valid(self.grid, nr, nc) and self.grid[nr][nc] == FREE_UNCOVERED:
                best_dir_index = i
                break

        return best_sp, best_dir_index

    def point_after_backtracking(self, rr, rc):
        rows, cols = self.rows, self.cols

        def safe_get(r, c):
            if 0 <= r < rows and 0 <= c < cols:
                return self.grid[r][c]
            return OBSTACLE
        nbrs = [
            (rr-1, rc),  # up
            (rr+1, rc),  # down
            (rr, rc-1),  # left
            (rr, rc+1),  # right
        ]
        free_candidates = []
        for (nr, nc) in nbrs:
            if safe_get(nr, nc) == FREE_UNCOVERED:
                free_candidates.append((nr, nc))
        if free_candidates:
            best = None
            best_pot = -1
            for (nr, nc) in free_candidates:
                pot = self.estimate_reachable_uncovered_area(nr, nc)
                if pot > best_pot:
                    best_pot = pot
                    best = (nr, nc)
            return best
        covered_candidates = []
        for (nr, nc) in nbrs:
            if safe_get(nr, nc) == COVERED:
                covered_candidates.append((nr, nc))
        if covered_candidates:
            best = None
            best_pot = -1
            for (nr, nc) in covered_candidates:
                pot = self.estimate_reachable_uncovered_area(nr, nc)
                if pot > best_pot:
                    best_pot = pot
                    best = (nr, nc)
            return best
        return rr, rc

    def scan_hole(self, hole_rep, resume_dir_idx=None):
        """
        Xử lý quét 1 hole representative (hole_rep) với cơ chế Đệ quy + Resume.
        """
        # --- 1. Guard & Init ---
        if not hasattr(self, 'hole_scan_depth'):
            self.hole_scan_depth = 0
        if self.hole_scan_depth >= getattr(self, 'max_hole_scan_depth', 5):
            print(
                f"!! Skip hole {hole_rep}: reached max depth ({self.hole_scan_depth}).")
            return False

        # Đảm bảo hole_rep tồn tại trong map
        if hole_rep not in self.hole_map:
            holes = self.detect_holes()
            found = False
            for comp in holes:
                if hole_rep in comp:
                    self.hole_map[hole_rep] = comp
                    for cell in comp:
                        self.used_backtracks.add(cell)
                    found = True
                    break
            if not found:
                return False

        # --- 2. Đánh dấu Resume Point (RP) ---
        self.resume_stack.append(self.current_cp)

        self.hole_scan_depth += 1
        self.in_hole_scan = True
        print(
            f"-> [Depth {self.hole_scan_depth}] START scanning hole {hole_rep}")

        try:
            # --- 3. Di chuyển đến cửa hố (Approach) ---
            path_astar = a_star_search(self.grid, self.current_cp, hole_rep)
            if path_astar:
                path_smoothed = a_star_spt(self.grid, path_astar)
                if self.on_astar_callback:
                    self.on_astar_callback(path_smoothed)

                for pos in path_smoothed[1:]:
                    if not self.total_path or self.total_path[-1] != pos:
                        self.total_path.append(pos)
                    self.current_pos = pos

            # Cập nhật vị trí bắt đầu quét bên trong hố
            br, bc = self.current_pos
            next_start = br, bc
            if next_start != (br, bc):
                self.current_pos = next_start

            # --- 4. VÒNG LẶP QUÉT & ĐỆ QUY (Core Logic) ---
            while True:
                try:
                    # [QUAN TRỌNG] Unpack đủ 5 giá trị từ BM
                    s_cp, self.grid, hole_cov_path, nested_rep, last_long_dir = boustrophedon_motion(
                        self.grid,
                        self.current_pos,
                        start_dir_index=0,
                        callback=self.on_step_callback,
                        coverage_id=self.coverage_count + 1,
                        sensor_radius=self.sensor_radius,
                        stop_on_hole=True,
                        allow_hole_detection=True,
                        is_hole_scanning=True
                    )
                except Exception as e:
                    print(" !! Lỗi BM trong scan_hole:", e)
                    hole_cov_path = []
                    nested_rep = None

                # Cập nhật đường đi (Tô màu Grid)
                if hole_cov_path:
                    self.coverage_count += 1
                    cov_id = self.coverage_count

                    # [FIX VISUAL] Duyệt hết path, không bỏ phần tử đầu tiên
                    for pos in hole_cov_path:
                        pr, pc = pos
                        if 0 <= pr < self.rows and 0 <= pc < self.cols:
                            self.grid[pr][pc] = COVERED

                        if not self.total_path or self.total_path[-1] != pos:
                            self.total_path.append(pos)

                        self.current_pos = pos

                        # if self.on_step_callback:
                        #     try:
                        #         self.on_step_callback(
                        #             self.grid, pos, hole_cov_path, cov_id)
                        #     except:
                        #         pass

                    self.current_cp = hole_cov_path[-1]
                    self.current_pos = hole_cov_path[-1]
                else:
                    if nested_rep is None:
                        break

                # --- XỬ LÝ ĐỆ QUY (NESTED HOLE) ---
                if nested_rep:
                    print(
                        f" >>> [Depth {self.hole_scan_depth}] Phát hiện NESTED HOLE tại {nested_rep}.")
                    self.scan_hole(nested_rep)
                    print(
                        f" <<< [Depth {self.hole_scan_depth}] Resume lại hố hiện tại từ {self.current_pos}")
                else:
                    break

            # --- 5. Clean up Map ---
            finished_reps = []
            for rep, comp in list(self.hole_map.items()):
                if not any(self.grid[r][c] == FREE_UNCOVERED for (r, c) in comp):
                    finished_reps.append(rep)
            for rep in finished_reps:
                self.hole_map.pop(rep, None)

            # --- 6. Quay về Resume Point & RESUME + 1 ---
            if self.resume_stack:
                resume_point = self.resume_stack.pop()

                # A* quay về Resume Point
                if self.current_pos != resume_point:
                    print(
                        f"--> [Depth {self.hole_scan_depth}] Xong hố. A* quay ra resume point {resume_point}")
                    path_astar = a_star_search(
                        self.grid, self.current_pos, resume_point)
                    if path_astar:
                        path_smoothed = a_star_spt(self.grid, path_astar)
                        if self.on_astar_callback:
                            self.on_astar_callback(path_smoothed)
                        for pos in path_smoothed[1:]:
                            if not self.total_path or self.total_path[-1] != pos:
                                self.total_path.append(pos)
                            self.current_pos = pos
                    else:
                        print("   !! Không tìm thấy đường ra resume point.")

                # Đứng tại resume point
                self.current_pos = resume_point
                self.current_cp = resume_point  # Tạm gán để nếu không +1 được thì vẫn đúng logic

                if self.on_resume_callback:
                    try:
                        self.on_resume_callback(resume_point)
                    except:
                        pass

                # ====================================================
                # [LOGIC RESUME + 1] Tiến thêm 1 bước (FIXED: Xử lý cuối luống)
                # ====================================================
                if resume_dir_idx is not None:
                    # Tạo danh sách các hướng ứng viên:
                    # 1. Ưu tiên hướng cũ (resume_dir_idx)
                    # 2. Nếu bị chặn, thử 2 hướng vuông góc (Lateral) để rẽ
                    candidate_dirs = [resume_dir_idx]

                    # Giả sử DIRECTIONS_BM: 0:N, 1:S, 2:E, 3:W
                    # Đang đi dọc (N/S) -> Thêm ngang (E/W)
                    if resume_dir_idx in [0, 1]:
                        candidate_dirs.extend([2, 3])
                    # Đang đi ngang (E/W) -> Thêm dọc (N/S)
                    elif resume_dir_idx in [2, 3]:
                        candidate_dirs.extend([0, 1])

                    move_success = False

                    for try_dir in candidate_dirs:
                        dr, dc = DIRECTIONS_BM[try_dir]
                        start_resume_pos = self.current_pos
                        r_curr, c_curr = start_resume_pos
                        next_step = (r_curr + dr, c_curr + dc)

                        # Kiểm tra hợp lệ: Trong map VÀ Không phải vật cản VÀ (Quan trọng) Chưa được phủ
                        # Lưu ý: Nếu ô đó đã phủ rồi thì không nên đi vào lại, trừ khi không còn đường nào khác
                        if (0 <= next_step[0] < self.rows and
                            0 <= next_step[1] < self.cols and
                                # Chỉ đi vào ô FREE
                                self.grid[next_step[0]][next_step[1]] == FREE_UNCOVERED):

                            # --- CHẤP NHẬN BƯỚC ĐI NÀY ---
                            self.coverage_count += 1  # Tăng ID đường vẽ

                            # 1. Update Grid & Path
                            self.grid[next_step[0]][next_step[1]] = COVERED

                            if not self.total_path or self.total_path[-1] != next_step:
                                self.total_path.append(next_step)

                            # 2. GỌI CALLBACK
                            if self.on_step_callback:
                                try:
                                    self.on_step_callback(
                                        self.grid,
                                        next_step,
                                        [start_resume_pos, next_step],
                                        self.coverage_count
                                    )
                                except:
                                    pass

                            # 3. Update State
                            self.current_pos = next_step
                            self.current_cp = next_step

                            # [QUAN TRỌNG] Nếu rẽ ngang, cần cập nhật lại hướng chính cho robot
                            # Tuy nhiên trong code này self.current_dir_index thường được reset bởi BM vòng sau
                            # nên chỉ cần update vị trí là đủ.

                            print(
                                f"--> [Resume + 1] Thành công hướng {try_dir} tới {next_step}")
                            move_success = True
                            break  # Đã đi được 1 bước thì thoát vòng lặp candidates

                    if not move_success:
                        print(
                            f"--> [Resume + 1] Bế tắc: Không thể tiến thẳng hay rẽ ngang từ {self.current_pos}")

        finally:
            self.hole_scan_depth -= 1
            if self.hole_scan_depth == 0:
                self.in_hole_scan = False
            print(f"<- [Depth {self.hole_scan_depth + 1}] Thoát scan_hole.")

        return True
    # ------------------ Main run (integrated hole handling) ------------------
    def run(self):
        print("--- Bắt đầu Thuật toán BA* ---")
        step = 1

        while True:
            print(f"\n--- Chu trình Bao phủ #{step} ---")
            print(
                f"Vị trí hiện tại: {self.current_pos}, Hướng: {self.current_dir_index}")
            self.coverage_count += 1

            # B2: Bao phủ bằng BM
            print("1. Thực hiện Chuyển động Boustrophedon (BM)...")
            s_cp, self.grid, coverage_path, hole_rep, last_long_dir = boustrophedon_motion(
                self.grid,
                self.current_pos,
                self.current_dir_index,
                self.on_step_callback,
                self.coverage_count,
                sensor_radius=self.sensor_radius,
                stop_on_hole=True,
                is_hole_scanning=False
            )
            self.current_cp = s_cp
            self.coverage_paths.append(coverage_path)

            # Nếu phát hiện hole kề robot -> xử lý hole
            if hole_rep is not None:
                self.scan_hole(hole_rep, resume_dir_idx=last_long_dir)
                step += 1
                continue

            # ----------------------------------------------------------------------------------------------

            # B3: Tìm danh sách backtrack (corner-based)
            backtracking_list = self.find_backtracking_list()
            print(f"2. Phát hiện corner backtracks: {backtracking_list}")

            # --- NEW: detect holes online and add reps vào backtracking_list (nếu không đang quét) ---

            holes = self.detect_holes()
            for comp in holes:
                # chỉ thêm nếu component còn ô FREE_UNCOVERED
                if not any(self.grid[r][c] == FREE_UNCOVERED for (r, c) in comp):
                    continue
                rep = self.get_hole_representative(
                    comp, prefer='entry', robot_pos=self.current_cp)
                if rep is None:
                    continue
                if rep not in backtracking_list and rep not in self.used_backtracks:
                    # add vào map và mark toàn component vào used_backtracks
                    self.hole_map[rep] = comp
                    for cell in comp:
                        self.used_backtracks.add(cell)
                    backtracking_list.append(rep)
                    print(
                        f"   => Phát hiện hole, thêm rep {rep} với size {len(comp)}")
            # else:
            #     # đang quét hole -> skip detect để tránh phát hiện lồng nhau
            #     pass

            print(
                f"   → Tổng backtracking candidates (incl. holes): {backtracking_list}")

            # Lọc bỏ chính s_cp và các điểm đã dùng
            candidates = [p for p in backtracking_list if p !=
                          self.current_cp and p not in self.used_backtracks]
            print(f"   → Ứng viên sau lọc: {candidates}")

            # Nếu không còn ứng viên
            if not candidates:
                if not any(FREE_UNCOVERED in row for row in self.grid):
                    print("3. Không còn ô trống. Nhiệm vụ bao phủ hoàn tất.")
                    break
                print("3. Không còn ứng viên backtrack hợp lệ. Dừng.")
                break

            # B5: Chọn điểm bắt đầu tốt nhất
            s_sp, next_dir_index = self.select_best_start_point(candidates)
            if s_sp is None:
                print(" !! Lỗi: Không thể tìm thấy điểm bắt đầu hợp lệ. Dừng.")
                break

            if s_sp == self.current_cp:
                others = [p for p in candidates if p != self.current_cp]
                if others:
                    s_sp, next_dir_index = self.select_best_start_point(others)
                else:
                    print(
                        " !! Sau lọc vẫn ra s_sp == s_cp, không còn ứng viên khác. Dừng.")
                    break

            print(f"4. Điểm bắt đầu tiếp theo (s_sp) được chọn: {s_sp}")
            manhattan = abs(self.current_cp[0] - s_sp[0]) + \
                abs(self.current_cp[1] - s_sp[1])
            print(
                f"   Khoảng cách từ {self.current_cp} đến {s_sp}: {manhattan}")

            # Ghi nhận đã dùng s_sp để lần sau không chọn lại
            # (đã có marking toàn component khi thêm hole vào used_backtracks; đảm bảo không chọn lại)
            self.used_backtracks.add(s_sp)
            backtracking_list = [p for p in backtracking_list if p != s_sp]
            candidates = [p for p in candidates if p != s_sp]

            # If s_sp is a hole representative, push resume point and mark intention
            is_hole_target = s_sp in self.hole_map
            if is_hole_target:
                print(
                    f"   -> s_sp {s_sp} is a hole representative; pushing resume point {self.current_cp}")
                self.resume_stack.append(self.current_cp)

            # Hiển thị backtrack point đã chọn
            if self.on_backtrack_callback:
                self.on_backtrack_callback(s_sp)

            # B6: Lập kế hoạch A*
            print(
                f"5. Lập kế hoạch đường dẫn A* từ {self.current_cp} đến {s_sp}...")
            if self.current_cp == s_sp:
                print("   s_cp == s_sp → bỏ qua A* (không cần di chuyển).")
                path_astar = [self.current_cp]
            else:
                path_astar = a_star_search(self.grid, self.current_cp, s_sp)
                if not path_astar:
                    print(" !! Lỗi: Không thể tìm thấy đường dẫn A* đến s_sp. Dừng.")
                    break
                print(f" Đường dẫn A* thô: {len(path_astar)} bước.")

            # B7: Làm mịn
            path_smoothed = a_star_spt(self.grid, path_astar)
            print(
                f"6. Đường dẫn được làm mịn (A*SPT): {len(path_smoothed)} bước.")

            if len(path_smoothed) >= 2:
                self.astar_paths.append(path_smoothed)

            # B8: Theo dõi đường dẫn (di chuyển bằng A*; KHÔNG mark COVERED khi di chuyển)
            print("7. Theo dõi đường dẫn (Công thức 11)...")
            if len(path_smoothed) >= 2:
                if self.on_astar_callback:
                    self.on_astar_callback(path_smoothed)
                for pos in path_smoothed[1:]:
                    if not self.total_path or self.total_path[-1] != pos:
                        self.total_path.append(pos)
                    self.current_pos = pos
            else:
                self.current_pos = self.current_cp
                print(f"   Robot đã ở s_sp: {self.current_pos}")

            # Sau khi đến s_sp, xác định ô bắt đầu BM bên trong vùng mục tiêu
            print("🔍 Xác định điểm khởi động BM sau backtracking...")
            br, bc = self.current_pos
            next_start = self.point_after_backtracking(br, bc)
            if next_start != (br, bc):
                print(f"   → Robot dịch sang ô vùng mới: {next_start}")
                self.current_pos = next_start
            else:
                print("   → Robot đã ở vị trí hợp lệ để bắt đầu BM.")

            # nếu s_sp là hole rep, thì khi BM xong vòng tiếp theo chúng ta sẽ detect hole finished (logic trên)
            # B9: Điều chỉnh hướng cho BM tiếp theo
            self.current_dir_index = next_dir_index
            print("8. Điều chỉnh hướng ưu tiên cho BM tiếp theo.")

            step += 1

        return self.total_path, self.grid

