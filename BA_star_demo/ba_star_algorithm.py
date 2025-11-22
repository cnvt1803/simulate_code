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
def find_adjacent_hole_rep(grid, r, c, max_bfs=500):
    """
    Kiểm tra nhanh: có ô FREE_UNCOVERED kề robot mà ô đó thuộc một hole (không reach biên)?
    Trả về:
      - (nr,nc) representative nếu chắc chắn là hole,
      - None nếu không tìm thấy hole,
      - 'inconclusive' (None với cách dùng khác) nếu BFS timeout -> KHÔNG coi là hole.
    """
    rows, cols = len(grid), len(grid[0])
    FREE = FREE_UNCOVERED

    def is_reachable_to_border(sr, sc, bfs_limit=max_bfs):
        q = deque()
        visited = set()
        q.append((sr, sc))
        visited.add((sr, sc))
        steps = 0
        dirs = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        while q:
            rr, cc = q.popleft()
            # nếu nằm trên biên và là FREE thì reachable
            if rr == 0 or rr == rows-1 or cc == 0 or cc == cols-1:
                return True
            for dr, dc in dirs:
                nr, nc = rr+dr, cc+dc
                if 0 <= nr < rows and 0 <= nc < cols and (nr, nc) not in visited and grid[nr][nc] == FREE:
                    visited.add((nr, nc))
                    q.append((nr, nc))
            steps += 1
            if steps >= bfs_limit:
                # Không đủ thông tin -> inconclusive (đừng vội phán là hole)
                return None
        # BFS hoàn tất mà không chạm biên -> thực sự là hole (not reachable)
        return False

    # kiểm các ô 4 hướng kề robot
    if 0 <= r < rows and 0 <= c < cols and grid[r][c] == FREE:
        res = is_reachable_to_border(r, c, bfs_limit=max_bfs)
        if res is False:
            return (r, c)   # trả luôn chính ô đầu vào

    for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
        nr, nc = r+dr, c+dc
        if 0 <= nr < rows and 0 <= nc < cols and grid[nr][nc] == FREE:
            reachable = is_reachable_to_border(nr, nc)
            # Nếu chắc chắn không reachable -> đây là hole rep
            if reachable is False:
                return (nr, nc)
            # Nếu inconclusive (None) -> skip, không coi là hole
            # Nếu reachable is True -> không phải hole, skip
    return None


def boustrophedon_motion(
    grid,
    start_pos,
    start_dir_index=0,
    callback=None,
    coverage_id=1,
    sensor_radius=0,
    stop_on_hole=False,
    allow_hole_detection=True
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

    # ---------- init pattern ----------
    main_axis = "NS"
    prev_lap = 0
    cur_lap = 1

    dir_idx, new_axis = pick_dir_by_rule(r, c, main_axis, sensor_radius)
    # dùng đề xuất trục từ pick_dir_by_rule
    main_axis = new_axis

    # Nếu dir_idx là hướng khả dụng (ô kế tiếp free) -> dùng luôn

    def immediate_free(d):
        nr, nc = r + DIRECTIONS_BM[d][0], c + DIRECTIONS_BM[d][1]
        return 0 <= nr < rows and 0 <= nc < cols and grid[nr][nc] == FREE_UNCOVERED

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

    # --- NEW: trạng thái đi sát tường ---
    walking_along_wall = False
    wall_side = None   # hướng chứa tường (0=N,1=S,2=E,3=W)

    # mark start
    if grid[r][c] == FREE_UNCOVERED:
        grid[r][c] = COVERED
        coverage_path.append((r, c))
        if callback:
            callback(grid, (r, c), coverage_path, coverage_id)
        #! Có thể k ảnh hưởng
        # if stop_on_hole and allow_hole_detection:
        #     print(
        #         f'dã kiểm tra. Stop on hole: {stop_on_hole} allow_hole_detection{allow_hole_detection}')
        #     hole_rep = find_adjacent_hole_rep(grid, r, c)
        #     if hole_rep:
        #         return (r, c), grid, coverage_path, hole_rep

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
                # print(f"{cur_lap}-", end="")
                if callback:
                    callback(grid, (r, c), coverage_path, coverage_id)
                if switch_lock > 0:
                    switch_lock -= 1
                moved = True

                # --- NEW: detect START of walking along wall ---
                # if not walking_along_wall:
                #     # --- use mapping instead of (±1)%4 because DIRECTIONS_BM is [N,S,E,W] ---
                #     LEFT_OF = {0: 3, 1: 2, 2: 0, 3: 1}
                #     RIGHT_OF = {0: 2, 1: 3, 2: 1, 3: 0}

                #     left_sd = LEFT_OF[long_dir]
                #     right_sd = RIGHT_OF[long_dir]

                #     def _is_block(dir_idx):
                #         rr = r + DIRECTIONS_BM[dir_idx][0]
                #         cc = c + DIRECTIONS_BM[dir_idx][1]
                #         return not (0 <= rr < rows and 0 <= cc < cols) or grid[rr][cc] == OBSTACLE

                #     left_block = _is_block(left_sd)
                #     right_block = _is_block(right_sd)

                #     left_free = is_free(
                #         r + DIRECTIONS_BM[left_sd][0],  c + DIRECTIONS_BM[left_sd][1])
                #     right_free = is_free(
                #         r + DIRECTIONS_BM[right_sd][0], c + DIRECTIONS_BM[right_sd][1])

                #     # forward cell
                #     fdr, fdc = DIRECTIONS_BM[long_dir]
                #     fr, fc = r + fdr, c + fdc

                #     def _is_free_cell(rr, cc):
                #         return 0 <= rr < rows and 0 <= cc < cols and grid[rr][cc] == FREE_UNCOVERED

                #     def _is_blocking_cell(rr, cc):
                #         # OBSTACLE, COVERED, OOB all count as blocking for side-check
                #         if not (0 <= rr < rows and 0 <= cc < cols):
                #             return True
                #         return grid[rr][cc] in (OBSTACLE, COVERED)

                #     # lateral neighbours of the forward cell (relative left/right of forward)
                #     left_of_forward_r = fr + DIRECTIONS_BM[left_sd][0]
                #     left_of_forward_c = fc + DIRECTIONS_BM[left_sd][1]
                #     right_of_forward_r = fr + DIRECTIONS_BM[right_sd][0]
                #     right_of_forward_c = fc + DIRECTIONS_BM[right_sd][1]

                #     forward_is_free = _is_free_cell(fr, fc)
                #     forward_sides_blocked = _is_blocking_cell(
                #         left_of_forward_r, left_of_forward_c) and _is_blocking_cell(right_of_forward_r, right_of_forward_c)

                #     # TRIGGER: forward is free but BOTH sides of forward are blocked
                #     if forward_is_free and forward_sides_blocked:
                #         # decide start wall-following using original left/right of robot
                #         if left_block and right_free:
                #             walking_along_wall = True
                #             wall_side = left_sd
                #         elif right_block and left_free:
                #             walking_along_wall = True
                #             wall_side = right_sd

                #         # Attempt hole detection: try forward cell first, then diagonal into pocket, then fallback current pos
                #         if stop_on_hole and allow_hole_detection:
                #             hole_rep = find_adjacent_hole_rep(grid, fr, fc)
                #             if hole_rep:
                #                 return (r, c), grid, coverage_path, hole_rep

                #             # safe compute away_side only if wall_side set
                #             if wall_side is not None:
                #                 away_side = (wall_side + 2) % 4
                #                 d_fr = fr + DIRECTIONS_BM[away_side][0]
                #                 d_fc = fc + DIRECTIONS_BM[away_side][1]
                #                 if 0 <= d_fr < rows and 0 <= d_fc < cols and grid[d_fr][d_fc] == FREE_UNCOVERED:
                #                     hole_rep = find_adjacent_hole_rep(
                #                         grid, d_fr, d_fc)
                #                     if hole_rep:
                #                         return (r, c), grid, coverage_path, hole_rep

                #             # fallback: current robot pos
                #             hole_rep = find_adjacent_hole_rep(grid, r, c)
                #             if hole_rep:
                #                 return (r, c), grid, coverage_path, hole_rep

                #     # else: forward not match -> keep original behavior (do nothing here)

                # elif walking_along_wall:
                #     # giữ nguyên logic kết thúc corridor
                #     away_side = (wall_side + 2) % 4
                #     ar = r + DIRECTIONS_BM[away_side][0]
                #     ac = c + DIRECTIONS_BM[away_side][1]
                #     if not (0 <= ar < rows and 0 <= ac < cols and grid[ar][ac] == FREE_UNCOVERED):
                #         if stop_on_hole and allow_hole_detection:
                #             hole_rep = find_adjacent_hole_rep(grid, r, c)
                #             if hole_rep:
                #                 return (r, c), grid, coverage_path, hole_rep
                #         walking_along_wall = False
                #         wall_side = None

                # continue
                if not walking_along_wall:
                    # mapping tương thích với DIRECTIONS_BM = [N, S, E, W]
                    LEFT_OF = {0: 3, 1: 2, 2: 0, 3: 1}
                    RIGHT_OF = {0: 2, 1: 3, 2: 1, 3: 0}

                    left_sd = LEFT_OF[long_dir]
                    right_sd = RIGHT_OF[long_dir]

                    def _is_block(dir_idx):
                        rr = r + DIRECTIONS_BM[dir_idx][0]
                        cc = c + DIRECTIONS_BM[dir_idx][1]
                        return not (0 <= rr < rows and 0 <= cc < cols) or grid[rr][cc] == OBSTACLE

                    left_block = _is_block(left_sd)
                    right_block = _is_block(right_sd)

                    left_free = is_free(
                        r + DIRECTIONS_BM[left_sd][0],  c + DIRECTIONS_BM[left_sd][1])
                    right_free = is_free(
                        r + DIRECTIONS_BM[right_sd][0], c + DIRECTIONS_BM[right_sd][1])

                    # forward cell
                    fdr, fdc = DIRECTIONS_BM[long_dir]
                    fr, fc = r + fdr, c + fdc

                    def _is_free_cell(rr, cc):
                        return 0 <= rr < rows and 0 <= cc < cols and grid[rr][cc] == FREE_UNCOVERED

                    def _is_adjacent_wall_or_oob(rr, cc):
                        # returns True if any 4-neighbour is OBSTACLE or OOB
                        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                            nr, nc = rr + dr, cc + dc
                            if not (0 <= nr < rows and 0 <= nc < cols):
                                return True
                            if grid[nr][nc] == OBSTACLE:
                                return True
                        return False

                    forward_is_free = _is_free_cell(fr, fc)
                    # NEW: trigger when forward is free AND forward cell is adjacent to wall/out-of-bounds
                    forward_adjacent_to_wall = False
                    if 0 <= fr < rows and 0 <= fc < cols:
                        forward_adjacent_to_wall = _is_adjacent_wall_or_oob(
                            fr, fc)

                    # TRIGGER: forward is free and forward cell is next-to-wall
                    if forward_is_free and forward_adjacent_to_wall:
                        # decide start wall-following using original left/right of robot (unchanged)
                        if left_block and right_free:
                            walking_along_wall = True
                            wall_side = left_sd
                        elif right_block and left_free:
                            walking_along_wall = True
                            wall_side = right_sd

                        # Attempt hole detection: prefer forward cell, then diagonal into pocket, then fallback robot pos
                        # DEBUG: paste này ngay trước `if forward_is_free and forward_adjacent_to_wall:`
                        print("DBG_CHECK: robot:", (r, c),
                              "long_dir=", long_dir)
                        print(" forward cell:", (fr, fc), "value:", (None if not (
                            0 <= fr < rows and 0 <= fc < cols) else grid[fr][fc]))
                        neighbors = []
                        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                            nr, nc = fr+dr, fc+dc
                            val = "OOB" if not (
                                0 <= nr < rows and 0 <= nc < cols) else grid[nr][nc]
                            neighbors.append(((nr, nc), val))
                        print(" forward neighbors:", neighbors)
                        print(" forward_is_free:", forward_is_free,
                              "forward_adjacent_to_wall:", forward_adjacent_to_wall)
                        # Also quick check find_adjacent_hole_rep returns:
                        try:
                            res = find_adjacent_hole_rep(
                                grid, fr, fc, max_bfs=2000)
                        except TypeError:
                            res = find_adjacent_hole_rep(grid, fr, fc)
                        print(" find_adjacent_hole_rep(fr,fc) =>", res)

                        if stop_on_hole and allow_hole_detection:
                            print("đã bật (forward adjacent to wall)")
                            hole_rep = find_adjacent_hole_rep(grid, fr, fc)
                            if hole_rep:
                                return (r, c), grid, coverage_path, hole_rep

                            # check diagonal forward+away if wall_side available
                            if wall_side is not None:
                                away_side = (wall_side + 2) % 4
                                d_fr = fr + DIRECTIONS_BM[away_side][0]
                                d_fc = fc + DIRECTIONS_BM[away_side][1]
                                if 0 <= d_fr < rows and 0 <= d_fc < cols and grid[d_fr][d_fc] == FREE_UNCOVERED:
                                    hole_rep = find_adjacent_hole_rep(
                                        grid, d_fr, d_fc)
                                    if hole_rep:
                                        return (r, c), grid, coverage_path, hole_rep

                            # fallback: current robot pos
                            hole_rep = find_adjacent_hole_rep(grid, r, c)
                            if hole_rep:
                                return (r, c), grid, coverage_path, hole_rep

                # else keep original behavior

                elif walking_along_wall:
                    # giữ nguyên logic kết thúc corridor
                    away_side = (wall_side + 2) % 4
                    ar = r + DIRECTIONS_BM[away_side][0]
                    ac = c + DIRECTIONS_BM[away_side][1]
                    if not (0 <= ar < rows and 0 <= ac < cols and grid[ar][ac] == FREE_UNCOVERED):
                        if stop_on_hole and allow_hole_detection:
                            hole_rep = find_adjacent_hole_rep(grid, r, c)
                            if hole_rep:
                                return (r, c), grid, coverage_path, hole_rep
                        walking_along_wall = False
                        wall_side = None

                continue

            # === ĐANG Ở BIÊN CỦA LUỐNG ===
            # Nếu vừa đổi trục và chưa đi được 1 bước dọc → chặn mọi rẽ để tránh lật side_dir
            # print(f"-----DEBUG cur_lap: {cur_lap} and prev_lap: {prev_lap}")
            if switch_lock > 0:
                switch_lock -= 1
                continue
            # Đếm số hướng rẽ có thể đi 1 ô
            lateral = []
            for sd in (side_dir, opposite(side_dir)):
                sdr, sdc = DIRECTIONS_BM[sd]
                if is_free(r + sdr, c + sdc):
                    lateral.append(sd)

            if stop_on_hole and allow_hole_detection and lateral:
                # check current and lateral candidate cells
                hole_candidates = [(r, c)]
                for sd in lateral:
                    sdr, sdc = DIRECTIONS_BM[sd]
                    hole_candidates.append((r + sdr, c + sdc))
                for (hr, hc) in hole_candidates:
                    if 0 <= hr < rows and 0 <= hc < cols and grid[hr][hc] == FREE_UNCOVERED:
                        hole_rep = find_adjacent_hole_rep(grid, hr, hc)
                        if hole_rep:
                            return (r, c), grid, coverage_path, hole_rep
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
                # print(f"----------------------------------------------------------")
                prev_lap = cur_lap
                cur_lap = 1
                long_dir = opposite(long_dir)
                side_dir = opposite(sd)
                #! mới comment hồi 22:00 19-11
                # if stop_on_hole and allow_hole_detection:
                #     hole_rep = find_adjacent_hole_rep(grid, r, c)
                #     if hole_rep:
                #         return (r, c), grid, coverage_path, hole_rep

                # --- ROLL-IN: trượt ngang thêm cho đến khi ô dọc phía trước trống ---
                # GIỮ hướng ngang vừa rẽ
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
                if is_free(r + ldr, c + ldc):
                    r += ldr
                    c += ldc
                    grid[r][c] = COVERED

                    coverage_path.append((r, c))
                    if callback:
                        callback(grid, (r, c), coverage_path, coverage_id)
                    cur_lap = 2
                    #! mới comment hồi 22:30 ngày 19-11
                    # check hole after forced vertical step
                    # if stop_on_hole and allow_hole_detection:
                    #     hole_rep = find_adjacent_hole_rep(grid, r, c)
                    #     if hole_rep:
                    #         return (r, c), grid, coverage_path, hole_rep
                else:
                    cur_lap = 1

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
                # print(f"----------------------------------------------------------")
                prev_lap = cur_lap
                cur_lap = 1
                # --- NEW: check hole when finishing a full strip ---
                #! mới comment hồi 22:30 ngày 19-11
                if stop_on_hole and allow_hole_detection:
                    hole_rep = find_adjacent_hole_rep(grid, r, c)
                    if hole_rep:
                        return (r, c), grid, coverage_path, hole_rep

                # Quay đầu cho luống mới + giữ mẫu snake
                long_dir = opposite(long_dir)
                side_dir = opposite(sd)

                # --- ROLL-IN tương tự ---
                # GIỮ hướng ngang vừa rẽ
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
                if is_free(r + ldr, c + ldc):
                    r += ldr
                    c += ldc
                    grid[r][c] = COVERED
                    # print(f"TRUONG")
                    # print(
                    #     f" cur_lap: {cur_lap} prev_lap: {prev_lap}")
                    coverage_path.append((r, c))
                    if callback:
                        callback(grid, (r, c), coverage_path, coverage_id)
                    cur_lap = 2

                    # print(
                    #     f" cur_lap: {cur_lap} prev_lap: {prev_lap}")
                else:
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
                if stop_on_hole and allow_hole_detection:
                    hole_rep = find_adjacent_hole_rep(grid, r, c)
                    if hole_rep:
                        # return current pos and the found hole representative
                        return (r, c), grid, coverage_path, hole_rep
                break

        if not moved:
            return (r, c), grid, coverage_path, None


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
        b_s5_s6 = b_function(s5, s6)
        b_s5_s4 = b_function(s5, s4)
        b_s7_s6 = b_function(s7, s6)
        b_s7_s8 = b_function(s7, s8)
        mu_s = b_s1_s8 + b_s1_s2 + b_s5_s6 + b_s5_s4 + b_s7_s6 + b_s7_s8
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
            s_cp, self.grid, coverage_path, hole_rep = boustrophedon_motion(
                self.grid,
                self.current_pos,
                self.current_dir_index,
                self.on_step_callback,
                self.coverage_count,
                sensor_radius=self.sensor_radius,
                stop_on_hole=True,
            )
            self.current_cp = s_cp
            self.coverage_paths.append(coverage_path)

            # Nếu phát hiện hole kề robot -> xử lý hole
            if hole_rep is not None:
                print(
                    f"--> Phát hiện hole kề robot tại {hole_rep}. Chuyển sang quét hole ngay.")

                # nếu hole_rep chưa lưu trong map, detect holes 1 lần và lưu component; đánh dấu toàn component vào used_backtracks
                if hole_rep not in self.hole_map:
                    holes = self.detect_holes()
                    for comp in holes:
                        if hole_rep in comp:
                            self.hole_map[hole_rep] = comp
                            for cell in comp:
                                self.used_backtracks.add(cell)
                            break

                # push resume point trước khi vào hole
                self.resume_stack.append(self.current_cp)

                # BẬT CỜ: đang quét hole -> skip detect_holes() ở vòng chính
                self.in_hole_scan = True
                try:
                    # 1) A* tới hole_rep (chỉ di chuyển, KHÔNG mark COVERED ở bước di chuyển)
                    path_astar = a_star_search(
                        self.grid, self.current_cp, hole_rep)
                    if path_astar:
                        path_smoothed = a_star_spt(self.grid, path_astar)
                        if self.on_astar_callback:
                            self.on_astar_callback(path_smoothed)
                        for pos in path_smoothed[1:]:
                            # di chuyển theo A* (KHÔNG mark COVERED ở đây)
                            if not self.total_path or self.total_path[-1] != pos:
                                self.total_path.append(pos)
                            self.current_pos = pos
                    else:
                        print(
                            " !! Không thể lập đường A* tới hole_rep; bỏ qua vào hole (tiếp tục).")

                    # 2) Bước vào ô bắt đầu BM trong hole (point_after_backtracking)
                    br, bc = self.current_pos
                    next_start = (br, bc)
                    # next_start = self.point_after_backtracking(br, bc)
                    if next_start != (br, bc):
                        print(
                            f"   → Robot dịch sang ô vùng hole mới: {next_start}")
                        self.current_pos = next_start
                    else:
                        print(
                            "   → Robot đã ở vị trí hợp lệ để bắt đầu BM trong hole.")
                    print("=== START BM IN HOLE at", self.current_pos)
                    # 3) CHẠY BM TRONG HOLE (stop_on_hole=False)
                    try:
                        hole_s_cp, self.grid, hole_cov_path, _ = boustrophedon_motion(
                            self.grid,
                            self.current_pos,
                            start_dir_index=0,
                            callback=self.on_step_callback,
                            coverage_id=self.coverage_count + 1,
                            sensor_radius=self.sensor_radius,
                            stop_on_hole=False,
                            allow_hole_detection=False
                        )
                    except Exception as e:
                        print(" !! Lỗi khi chạy BM trong hole:", e)
                        hole_cov_path = []
                        hole_s_cp = self.current_pos

                    # 4) Áp dụng hole_cov_path: skip nếu đầu trùng current_pos; mark COVERED trước khi append; dedupe
                    if hole_cov_path:
                        self.coverage_count += 1
                        cov_id = self.coverage_count
                        start_idx = 1 if hole_cov_path and hole_cov_path[0] == self.current_pos else 0
                        for pos in hole_cov_path[start_idx:]:
                            pr, pc = pos
                            if 0 <= pr < self.rows and 0 <= pc < self.cols:
                                # BM thực tế quét -> mark COVERED
                                self.grid[pr][pc] = COVERED
                            if not self.total_path or self.total_path[-1] != pos:
                                self.total_path.append(pos)
                            self.current_pos = pos
                            if self.on_step_callback:
                                try:
                                    self.on_step_callback(
                                        self.grid, pos, hole_cov_path, cov_id)
                                except Exception:
                                    pass
                        self.current_cp = hole_cov_path[-1]
                        self.current_pos = hole_cov_path[-1]
                    else:
                        # nếu không có path trả về (hiếm) -> đặt cp về hole_s_cp
                        self.current_cp = hole_s_cp
                        self.current_pos = hole_s_cp

                    # 5) Xóa hole_map entry nếu component đã sạch
                    finished_reps = []
                    for rep, comp in list(self.hole_map.items()):
                        if not any(self.grid[r][c] == FREE_UNCOVERED for (r, c) in comp):
                            finished_reps.append(rep)
                    for rep in finished_reps:
                        self.hole_map.pop(rep, None)
                    # 6) A* quay về resume_point
                    if self.resume_stack:
                        resume_point = self.resume_stack.pop()
                        print(
                            f"--> Quay về resume point {resume_point} sau khi quét hole.")

                        # notify visualizer
                        if hasattr(self, "on_resume_callback") and self.on_resume_callback:
                            try:
                                self.on_resume_callback(resume_point)
                            except Exception:
                                pass

                        # Nếu current_cp == resume_point → không cần A*
                        if self.current_cp == resume_point:
                            print("   current_cp == resume_point → bỏ qua A*.")
                            self.current_pos = self.current_cp
                        else:
                            # A*
                            path_astar = a_star_search(
                                self.grid, self.current_cp, resume_point)
                            if not path_astar:
                                print(
                                    "   !! Không thể lập đường A* quay về resume_point.")
                            else:
                                # làm mịn
                                path_smoothed = a_star_spt(
                                    self.grid, path_astar)

                                # callback A*
                                if self.on_astar_callback:
                                    try:
                                        self.on_astar_callback(path_smoothed)
                                    except Exception:
                                        pass

                                # follow path (KHÔNG mark COVERED)
                                for pos in path_smoothed[1:]:
                                    if not self.total_path or self.total_path[-1] != pos:
                                        self.total_path.append(pos)
                                    self.current_pos = pos

                        # Set lại current_cp đúng resume_point (không chỉnh gì thêm)
                        self.current_cp = resume_point

                        # --- TÍNH next_start nhưng KHÔNG tự động dịch ---
                        try:
                            br, bc = self.current_pos
                            next_start = self.point_after_backtracking(br, bc)
                            if next_start and next_start != (br, bc):
                                print(
                                    f"   → next_start (gợi ý bắt đầu BM) được tính là: {next_start} (không tự động dịch).")
                            else:
                                print(
                                    "   → next_start trùng vị trí hiện tại hoặc không cần dịch.")
                        except Exception as e:
                            print("!! Lỗi khi tính next_start sau resume:", e)

                        print("   Returned from hole, resuming BM from resume_point.")

                finally:
                    # tắt cờ in_hole_scan dù có exception
                    self.in_hole_scan = False

                step += 1
                continue  # trở lại vòng while chính

            # ----------------------------------------------------------------------------------------------

            # B3: Tìm danh sách backtrack (corner-based)
            backtracking_list = self.find_backtracking_list()
            print(f"2. Phát hiện corner backtracks: {backtracking_list}")

            # --- NEW: detect holes online and add reps vào backtracking_list (nếu không đang quét) ---
            if not self.in_hole_scan:
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
            else:
                # đang quét hole -> skip detect để tránh phát hiện lồng nhau
                pass

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
