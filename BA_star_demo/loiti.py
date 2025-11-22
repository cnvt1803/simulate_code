from collections import deque
import heapq
import math

# --- 1. Cấu hình và Hằng số ---

DIRECTIONS_BM = [
    (-1, 0),  # Bắc (N)
    (1, 0),   # Nam (S)
    (0, 1),   # Đông (E)
    (0, -1)   # Tây (W)
]

DIRECTIONS_ASTAR = [
    (-1, 0), (1, 0), (0, 1), (0, -1),
    (-1, -1), (-1, 1), (1, -1), (1, 1)
]

FREE_UNCOVERED = 0
OBSTACLE = 1
COVERED = 2

BACKTRACKING_POINT = 3
COVERAGE_PATH = 4
ASTAR_PATH = 5

# --- Priority queue, heuristics, helpers (unchanged) ---


class PriorityQueue:
    def __init__(self):
        self.elements = []

    def empty(self):
        return not self.elements

    def put(self, priority, item):
        heapq.heappush(self.elements, (priority, item))

    def get(self):
        return heapq.heappop(self.elements)[1]


def heuristic(a, b):
    (x1, y1) = a
    (x2, y2) = b
    return abs(x1 - x2) + abs(y1 - y2)


def is_valid(grid, r, c):
    return 0 <= r < len(grid) and 0 <= c < len(grid[0])


def line_of_sight(grid, start, end):
    x0, y0 = start
    x1, y1 = end
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    while (x0 != x1 or y0 != y1):
        if grid[x0][y0] == OBSTACLE:
            return False
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy
    if grid[x1][y1] == OBSTACLE:
        return False
    return True

# --- A* and A*SPT (unchanged) ---


def a_star_search(grid, start, goal):
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
            if cell_type == OBSTACLE:
                continue
            move_cost = int(math.sqrt(dr*dr + dc*dc) * 10) / 10
            new_cost = cost_so_far[current] + move_cost
            if next_cell not in cost_so_far or new_cost < cost_so_far[next_cell]:
                cost_so_far[next_cell] = new_cost
                priority = new_cost + heuristic(goal, next_cell)
                frontier.put(priority, next_cell)
                came_from[next_cell] = current
    path = []
    current = goal
    while current != start:
        if current not in came_from:
            return None
        path.append(current)
        current = came_from[current]
    path.append(start)
    path.reverse()
    return path


def a_star_spt(grid, path):
    if not path or len(path) <= 1:
        return path or []
    path_smoothed = [path[0]]
    k = 0
    n = len(path) - 1
    while True:
        s_k = path_smoothed[-1]
        best_i = k + 1
        for i in range(n, k, -1):
            s_i = path[i]
            if line_of_sight(grid, s_k, s_i):
                best_i = i
                break
        s_best = path[best_i]
        path_smoothed.append(s_best)
        k = best_i
        if s_best == path[n]:
            break
    return path_smoothed

# --- Modified boustrophedon_motion: supports hole_detector callback ---


def boustrophedon_motion(
    grid,
    start_pos,
    start_dir_index=0,
    callback=None,
    coverage_id=1,
    sensor_radius=0,
    hole_detector=None,   # NEW: callable(grid, last_pos) -> rep or None
):
    """
    Same BM logic but calls hole_detector(grid, (r,c)) right after marking COVERED.
    If detector returns rep -> return (s_cp, grid, coverage_path, rep).
    Otherwise return (s_cp, grid, coverage_path).
    """
    rows, cols = len(grid), len(grid[0])
    r, c = start_pos
    coverage_path = []

    def is_free(rr, cc):
        return 0 <= rr < rows and 0 <= cc < cols and grid[rr][cc] == FREE_UNCOVERED

    def opposite(didx):
        return {0: 1, 1: 0, 2: 3, 3: 2}[didx]

    def apply_axis_switch(long_dir, side_dir):
        table = {
            (0, 2): (2, 1), (1, 2): (2, 0), (2, 0): (0, 3), (3, 0): (0, 2),
            (2, 1): (1, 3), (3, 1): (1, 2), (0, 3): (3, 1), (1, 3): (3, 0),
        }
        return table.get((long_dir, side_dir), (side_dir, opposite(long_dir)))

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

    # pick_dir_by_rule and pick_dir_by_rule_when_run unchanged (kept inline)
    def pick_dir_by_rule(rr, cc, main_axis, R, delta_penalty=0.6):
        def sense_all(r, c, R):
            dN, sN = sense_one_dir(r, c, -1, 0, R)
            dS, sS = sense_one_dir(r, c,  1, 0, R)
            dE, sE = sense_one_dir(r, c,  0, 1, R)
            dW, sW = sense_one_dir(r, c,  0, -1, R)
            return (dN, sN, dS, sS, dE, sE, dW, sW)
        dN, sN, dS, sS, dE, sE, dW, sW = sense_all(rr, cc, R)
        if sN and sS:
            score_NS = dN + dS
        else:
            dN_soft = dN if sN else R / (1 - delta_penalty)
            dS_soft = dS if sS else R / (1 - delta_penalty)
            score_NS = dN_soft + dS_soft
        if sE and sW:
            score_EW = dE + dW
        else:
            dE_soft = dE if sE else R / (1 - delta_penalty)
            dW_soft = dW if sW else R / (1 - delta_penalty)
            score_EW = dE_soft + dW_soft
        if score_NS > score_EW:
            chosen_axis = "NS"
        elif score_EW > score_NS:
            chosen_axis = "EW"
        else:
            chosen_axis = ("EW" if main_axis == "EW" else "NS")
        if chosen_axis == "NS":
            cand = []
            if is_free(rr-1, cc):
                cand.append((0, dN))
            if is_free(rr+1, cc):
                cand.append((1, dS))
            if cand:
                cand.sort(key=lambda x: x[1])
                return cand[0][0], "NS"
            return (0 if dN >= dS else 1), "NS"
        else:
            cand = []
            if is_free(rr, cc+1):
                cand.append((2, dE))
            if is_free(rr, cc-1):
                cand.append((3, dW))
            if cand:
                cand.sort(key=lambda x: x[1])
                return cand[0][0], "EW"
            return (2 if dE >= dW else 3), "EW"

    def pick_dir_by_rule_when_run(rr, cc, cur_lap, main_axis, R, delta_penalty=0.6):
        if delta_penalty < 0:
            delta_penalty = 0.0
        if delta_penalty >= 1.0:
            delta_penalty = 0.9999
        dN, sN = sense_one_dir(rr, cc, -1, 0, R)
        dS, sS = sense_one_dir(rr, cc,  1, 0, R)
        dE, sE = sense_one_dir(rr, cc,  0, 1, R)
        dW, sW = sense_one_dir(rr, cc,  0, -1, R)
        soft_val = R / (1 - delta_penalty)

        def choose_NS():
            cand = []
            if is_free(rr-1, cc):
                cand.append((0, dN))
            if is_free(rr+1, cc):
                cand.append((1, dS))
            if cand:
                cand.sort(key=lambda x: x[1])
                return cand[0][0]
            return 0 if dN <= dS else 1

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
        if main_axis == "NS":
            score_E = dE if sE else soft_val
            score_W = dW if sW else soft_val
            score_EW = score_E + score_W
            if cur_lap > score_EW:
                chosen_dir = choose_NS()
                return chosen_dir, "NS"
            else:
                chosen_dir = choose_EW()
                return chosen_dir, "EW"
        else:
            score_N = dN if sN else soft_val
            score_S = dS if sS else soft_val
            score_NS = score_N + score_S
            if cur_lap > score_NS:
                chosen_dir = choose_EW()
                return chosen_dir, "EW"
            else:
                chosen_dir = choose_NS()
                return chosen_dir, "NS"

    # ---------- init ----------
    main_axis = "NS"
    prev_lap = 0
    cur_lap = 1

    dir_idx, new_axis = pick_dir_by_rule(r, c, main_axis, sensor_radius)
    main_axis = new_axis
    if main_axis == "NS":
        if is_free(r-1, c):
            long_dir = 0
        elif is_free(r+1, c):
            long_dir = 1
        else:
            long_dir = 0
        side_dir = 2
    else:
        if is_free(r, c+1):
            long_dir = 2
        elif is_free(r, c-1):
            long_dir = 3
        else:
            long_dir = 2
        side_dir = 0

    going_longitudinal = True
    switch_lock = 0

    # mark start -> call detector
    if grid[r][c] == FREE_UNCOVERED:
        grid[r][c] = COVERED
        coverage_path.append((r, c))
        if callback:
            callback(grid, (r, c), coverage_path, coverage_id)
        if hole_detector:
            rep = hole_detector(grid, (r, c))
            if rep is not None:
                return (r, c), grid, coverage_path, rep

    # ---------- main loop ----------
    while True:
        moved = False
        if going_longitudinal:
            dr, dc = DIRECTIONS_BM[long_dir]
            nr, nc = r + dr, c + dc

            if is_free(nr, nc):
                r, c = nr, nc
                grid[r][c] = COVERED
                coverage_path.append((r, c))
                cur_lap += 1
                if callback:
                    callback(grid, (r, c), coverage_path, coverage_id)
                # detect hole
                if hole_detector:
                    rep = hole_detector(grid, (r, c))
                    if rep is not None:
                        return (r, c), grid, coverage_path, rep
                if switch_lock > 0:
                    switch_lock -= 1
                moved = True
                continue

            # at lateral boundary
            print(f"-----DEBUG cur_lap: {cur_lap} and prev_lap: {prev_lap}")
            if switch_lock > 0:
                switch_lock -= 1
                continue

            lateral = []
            for sd in (side_dir, opposite(side_dir)):
                sdr, sdc = DIRECTIONS_BM[sd]
                if is_free(r + sdr, c + sdc):
                    lateral.append(sd)

            if len(lateral) == 2:
                sd = side_dir
                sdr, sdc = DIRECTIONS_BM[sd]
                r, c = r + sdr, c + sdc
                grid[r][c] = COVERED
                coverage_path.append((r, c))
                if callback:
                    callback(grid, (r, c), coverage_path, coverage_id)
                if hole_detector:
                    rep = hole_detector(grid, (r, c))
                    if rep is not None:
                        return (r, c), grid, coverage_path, rep

                prev_lap = cur_lap
                cur_lap = 1
                long_dir = opposite(long_dir)
                side_dir = opposite(sd)

                # ROLL-IN (1 step)
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
                    if hole_detector:
                        rep = hole_detector(grid, (r, c))
                        if rep is not None:
                            return (r, c), grid, coverage_path, rep

                if is_free(r + ldr, c + ldc):
                    r += ldr
                    c += ldc
                    grid[r][c] = COVERED
                    coverage_path.append((r, c))
                    if callback:
                        callback(grid, (r, c), coverage_path, coverage_id)
                    if hole_detector:
                        rep = hole_detector(grid, (r, c))
                        if rep is not None:
                            return (r, c), grid, coverage_path, rep
                    cur_lap = 2
                else:
                    cur_lap = 1

                going_longitudinal = True
                moved = True
                continue

            if len(lateral) == 1:
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
                        switch_lock = 1
                        ldr, ldc = DIRECTIONS_BM[long_dir]
                        if is_free(r + ldr, c + ldc):
                            r, c = r + ldr, c + ldc
                            grid[r][c] = COVERED
                            coverage_path.append((r, c))
                            if callback:
                                callback(grid, (r, c),
                                         coverage_path, coverage_id)
                            if hole_detector:
                                rep = hole_detector(grid, (r, c))
                                if rep is not None:
                                    return (r, c), grid, coverage_path, rep
                            cur_lap = 1
                            moved = True
                        continue

                sd = lateral[0]
                sdr, sdc = DIRECTIONS_BM[sd]
                r, c = r + sdr, c + sdc
                grid[r][c] = COVERED
                coverage_path.append((r, c))
                if callback:
                    callback(grid, (r, c), coverage_path, coverage_id)
                if hole_detector:
                    rep = hole_detector(grid, (r, c))
                    if rep is not None:
                        return (r, c), grid, coverage_path, rep

                prev_lap = cur_lap
                cur_lap = 1
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
                    if hole_detector:
                        rep = hole_detector(grid, (r, c))
                        if rep is not None:
                            return (r, c), grid, coverage_path, rep

                if is_free(r + ldr, c + ldc):
                    r += ldr
                    c += ldc
                    grid[r][c] = COVERED
                    coverage_path.append((r, c))
                    if callback:
                        callback(grid, (r, c), coverage_path, coverage_id)
                    if hole_detector:
                        rep = hole_detector(grid, (r, c))
                        if rep is not None:
                            return (r, c), grid, coverage_path, rep
                    cur_lap = 2
                else:
                    cur_lap = 1

                going_longitudinal = True
                moved = True
                continue

        # fallback try 4 directions
        for i, (dr, dc) in enumerate(DIRECTIONS_BM):
            nr, nc = r + dr, c + dc
            if is_free(nr, nc):
                r, c = nr, nc
                grid[r][c] = COVERED
                coverage_path.append((r, c))
                if callback:
                    callback(grid, (r, c), coverage_path, coverage_id)
                if hole_detector:
                    rep = hole_detector(grid, (r, c))
                    if rep is not None:
                        return (r, c), grid, coverage_path, rep
                if (main_axis == "NS" and i in (0, 1)) or (main_axis == "EW" and i in (2, 3)):
                    long_dir = i
                going_longitudinal = True
                moved = True
                break

        if not moved:
            return (r, c), grid, coverage_path

# --- BAStar class with integrated immediate hole handling ---


class BAStar:
    def __init__(self, initial_grid, start_pos, sensor_radius=10):
        self.grid = [row[:] for row in initial_grid]
        self.rows = len(initial_grid)
        self.cols = len(initial_grid[0])
        self.current_pos = start_pos
        self.current_cp = start_pos
        self.current_dir_index = 0
        self.total_path = [start_pos]
        self.step_count = 0
        self.coverage_paths = []
        self.astar_paths = []
        self.coverage_count = 0
        self.sensor_radius = sensor_radius
        self.on_step_callback = None
        self.on_backtrack_callback = None
        self.on_astar_callback = None
        self.used_backtracks = set()
        self.hole_map = dict()   # rep -> comp
        self.resume_stack = []

    def set_callbacks(self, step_callback=None, backtrack_callback=None, astar_callback=None):
        self.on_step_callback = step_callback
        self.on_backtrack_callback = backtrack_callback
        self.on_astar_callback = astar_callback

    # detect_holes / get_hole_representative unchanged from your code
    def detect_holes(self):
        FREE = FREE_UNCOVERED
        rows, cols = self.rows, self.cols
        visited = [[False]*cols for _ in range(rows)]
        q = deque()
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
        if not comp:
            return None
        if prefer == 'centroid':
            sr = sum(p[0] for p in comp)/len(comp)
            sc = sum(p[1] for p in comp)/len(comp)
            best = min(comp, key=lambda p: (p[0]-sr)**2 + (p[1]-sc)**2)
            return best
        if prefer == 'entry':
            for (r, c) in comp:
                for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    nr, nc = r+dr, c+dc
                    if not (0 <= nr < self.rows and 0 <= nc < self.cols) or self.grid[nr][nc] in (COVERED, OBSTACLE):
                        return (r, c)
            return comp[0]
        if robot_pos is None:
            return comp[0]
        rx, ry = robot_pos
        return min(comp, key=lambda p: abs(p[0]-rx)+abs(p[1]-ry))

    # existing helper methods unchanged (find_backtracking_list, calculate_mu_function, estimate_reachable_uncovered_area, filter... etc.)
    def find_backtracking_list(self):
        backtracking_list = []
        eight_directions = [(0, 1), (-1, 1), (-1, 0),
                            (-1, -1), (0, -1), (1, -1), (1, 0), (1, 1)]
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
        mu_s = (b_function(s1, s8) + b_function(s1, s2) + b_function(s5, s6) +
                b_function(s5, s4) + b_function(s7, s6) + b_function(s7, s8))
        return mu_s

    def estimate_reachable_uncovered_area(self, start_r, start_c):
        visited = set()
        queue = []
        directions = [(-1, 0), (1, 0), (0, 1), (0, -1)]
        for dr, dc in directions:
            nr, nc = start_r+dr, start_c+dc
            if (is_valid(self.grid, nr, nc) and self.grid[nr][nc] == FREE_UNCOVERED and (nr, nc) not in visited):
                queue.append((nr, nc))
                visited.add((nr, nc))
        count = 0
        max_search = 50
        while queue and count < max_search:
            r, c = queue.pop(0)
            count += 1
            for dr, dc in directions:
                nr, nc = r+dr, c+dc
                if (is_valid(self.grid, nr, nc) and self.grid[nr][nc] == FREE_UNCOVERED and (nr, nc) not in visited):
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
                distance = abs(point[0]-existing_point[0]) + \
                    abs(point[1]-existing_point[1])
                if distance < min_distance:
                    is_too_close = True
                    break
            if not is_too_close:
                filtered.append(point)
        return filtered

    def select_best_start_point(self, backtracking_list):
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
            if (r, c) in self.hole_map:
                comp = self.hole_map[(r, c)]
                cost = cost - (1000 + len(comp))
            if cost < min_cost:
                min_cost = cost
                best_sp = (r, c)
        if best_sp is None:
            return None, 0
        best_dir_index = 0
        for i, (dr, dc) in enumerate(DIRECTIONS_BM):
            nr, nc = best_sp[0]+dr, best_sp[1]+dc
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
        nbrs = [(rr-1, rc), (rr+1, rc), (rr, rc-1), (rr, rc+1)]
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

    # ------------------ Main run (immediate hole handling) ------------------
    def run(self):
        print("--- Bắt đầu Thuật toán BA* ---")
        step = 1

        # wrapper detector uses local detection and existing hole_map to avoid duplicates
        MIN_HOLE_SIZE = 2  # tune: min cells to consider a hole

        def hole_detector_wrapper(grid_snapshot, last_pos):
            # detect holes (could be optimized to bbox around last_pos)
            holes = self.detect_holes()
            for comp in holes:
                if len(comp) < MIN_HOLE_SIZE:
                    continue
                rep = self.get_hole_representative(
                    comp, prefer='entry', robot_pos=self.current_cp)
                if rep is None:
                    continue
                if rep not in self.hole_map and rep not in self.used_backtracks:
                    # register hole (so next tick it will appear in backtracking_list if desired)
                    self.hole_map[rep] = comp
                    print(
                        f"    [hole_detector] Detected hole rep {rep} (size {len(comp)})")
                    return rep
            return None

        while True:
            print(f"\n--- Chu trình Bao phủ #{step} ---")
            print(
                f"Vị trí hiện tại: {self.current_pos}, Hướng: {self.current_dir_index}")
            self.coverage_count += 1

            # call BM with hole_detector => BM can return immediately if hole found
            print("1. Thực hiện Chuyển động Boustrophedon (BM)...")
            bm_res = boustrophedon_motion(
                self.grid,
                self.current_pos,
                self.current_dir_index,
                self.on_step_callback,
                self.coverage_count,
                sensor_radius=self.sensor_radius,
                hole_detector=hole_detector_wrapper
            )

            # BM may return (s_cp, grid, coverage_path) or (s_cp, grid, coverage_path, hole_rep)
            if len(bm_res) == 3:
                s_cp, self.grid, coverage_path = bm_res
                hole_rep = None
            else:
                s_cp, self.grid, coverage_path, hole_rep = bm_res

            self.current_cp = s_cp
            self.coverage_paths.append(coverage_path)

            # If BM returned due to hole detection -> process immediately
            if hole_rep is not None:
                print(
                    f"-> Immediate hole detected at {hole_rep}. Handling now.")
                # push resume
                self.resume_stack.append(self.current_cp)
                # plan A* to hole_rep from current_pos
                start = self.current_pos
                path_to_hole = a_star_search(self.grid, start, hole_rep)
                if not path_to_hole:
                    print(
                        "   !! Cannot reach detected hole rep, marking used and continue.")
                    self.used_backtracks.add(hole_rep)
                    # remove hole_map entry to avoid re-detection
                    self.hole_map.pop(hole_rep, None)
                    # continue main loop
                    step += 1
                    continue
                path_to_hole_sm = a_star_spt(self.grid, path_to_hole)
                if self.on_astar_callback:
                    self.on_astar_callback(path_to_hole_sm)
                for pos in path_to_hole_sm[1:]:
                    self.total_path.append(pos)
                    self.current_pos = pos
                # Now inside or next to hole_rep: run BM inside hole, disable detector to avoid nested interrupts
                print("   -> Entering hole BM (detector disabled while inside hole).")
                s_cp2, self.grid, path_in_hole = boustrophedon_motion(
                    self.grid,
                    self.current_pos,
                    self.current_dir_index,
                    self.on_step_callback,
                    self.coverage_count,
                    sensor_radius=self.sensor_radius,
                    hole_detector=None  # disable nested immediate interrupts
                )
                # append hole coverage
                self.coverage_paths.append(path_in_hole)
                for pos in path_in_hole:
                    self.total_path.append(pos)
                self.current_pos = s_cp2
                # mark hole as finished (remove from hole_map) and mark used
                if hole_rep in self.hole_map:
                    del self.hole_map[hole_rep]
                self.used_backtracks.add(hole_rep)
                # after finishing hole: plan return to resume (if exists)
                if self.resume_stack:
                    resume_point = self.resume_stack.pop()
                    print(
                        f"   -> Planning return to resume point {resume_point}")
                    path_back = a_star_search(
                        self.grid, self.current_pos, resume_point)
                    if path_back:
                        path_smoothed_back = a_star_spt(self.grid, path_back)
                        if self.on_astar_callback:
                            self.on_astar_callback(path_smoothed_back)
                        for pos in path_smoothed_back[1:]:
                            self.total_path.append(pos)
                            self.current_pos = pos
                        self.current_cp = resume_point
                        print("   -> Returned from hole, resuming BM.")
                        step += 1
                        continue
                    else:
                        print(
                            "   !! Cannot plan return path to resume point; continuing.")
                        step += 1
                        continue
                else:
                    step += 1
                    continue

            # --- If no immediate hole handling, continue original flow ---
            # after BM, check if any hole finished and return if needed (existing logic)
            finished_holes = []
            for rep, comp in list(self.hole_map.items()):
                still_free = any(self.grid[r][c] ==
                                 FREE_UNCOVERED for (r, c) in comp)
                if not still_free:
                    finished_holes.append(rep)
            if finished_holes and self.resume_stack:
                resume_point = self.resume_stack.pop()
                print(
                    f"--> Hole(s) {finished_holes} finished. Returning to resume point {resume_point}.")
                for rep in finished_holes:
                    if rep in self.hole_map:
                        del self.hole_map[rep]
                for rep in finished_holes:
                    self.used_backtracks.add(rep)
                path_back = a_star_search(
                    self.grid, self.current_cp, resume_point)
                if path_back:
                    path_smoothed_back = a_star_spt(self.grid, path_back)
                    if self.on_astar_callback:
                        self.on_astar_callback(path_smoothed_back)
                    for pos in path_smoothed_back[1:]:
                        self.total_path.append(pos)
                        self.current_pos = pos
                    self.current_cp = resume_point
                    print("   Returned from hole, resuming BM from resume point.")
                    step += 1
                    continue
                else:
                    print("   !! Cannot plan return path to resume point; continuing.")
                    for rep in finished_holes:
                        self.hole_map.pop(rep, None)

            # B3: Find backtracking list (corner-based)
            backtracking_list = self.find_backtracking_list()
            print(f"2. Phát hiện corner backtracks: {backtracking_list}")

            # add holes to backtracking candidates (if not used)
            holes = self.detect_holes()
            for comp in holes:
                rep = self.get_hole_representative(
                    comp, prefer='entry', robot_pos=self.current_cp)
                if rep is None:
                    continue
                if rep not in backtracking_list and rep not in self.used_backtracks:
                    self.hole_map[rep] = comp
                    backtracking_list.append(rep)
                    print(
                        f"   => Phát hiện hole, thêm rep {rep} với size {len(comp)}")

            print(
                f"   → Tổng backtracking candidates (incl. holes): {backtracking_list}")

            candidates = [p for p in backtracking_list if p !=
                          self.current_cp and p not in self.used_backtracks]
            print(f"   → Ứng viên sau lọc: {candidates}")

            if not candidates:
                if not any(FREE_UNCOVERED in row for row in self.grid):
                    print("3. Không còn ô trống. Nhiệm vụ bao phủ hoàn tất.")
                    break
                print("3. Không còn ứng viên backtrack hợp lệ. Dừng.")
                break

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
            manhattan = abs(
                self.current_cp[0] - s_sp[0]) + abs(self.current_cp[1] - s_sp[1])
            print(
                f"   Khoảng cách từ {self.current_cp} đến {s_sp}: {manhattan}")

            # register used and remove copies locally (important to avoid re-show)
            self.used_backtracks.add(s_sp)
            backtracking_list = [p for p in backtracking_list if p != s_sp]
            candidates = [p for p in candidates if p != s_sp]

            is_hole_target = s_sp in self.hole_map
            if is_hole_target:
                print(
                    f"   -> s_sp {s_sp} is a hole representative; pushing resume point {self.current_cp}")
                self.resume_stack.append(self.current_cp)

            if self.on_backtrack_callback:
                self.on_backtrack_callback(s_sp)

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
                print(f"   Đường dẫn A* thô: {len(path_astar)} bước.")

            path_smoothed = a_star_spt(self.grid, path_astar)
            print(
                f"6. Đường dẫn được làm mịn (A*SPT): {len(path_smoothed)} bước.")
            if len(path_smoothed) >= 2:
                self.astar_paths.append(path_smoothed)

            print("7. Theo dõi đường dẫn (Công thức 11)...")
            if len(path_smoothed) >= 2:
                if self.on_astar_callback:
                    self.on_astar_callback(path_smoothed)
                for pos in path_smoothed[1:]:
                    self.total_path.append(pos)
                    self.current_pos = pos
            else:
                self.current_pos = self.current_cp
                print(f"   Robot đã ở s_sp: {self.current_pos}")

            print("🔍 Xác định điểm khởi động BM sau backtracking...")
            br, bc = self.current_pos
            next_start = self.point_after_backtracking(br, bc)
            if next_start != (br, bc):
                print(f"   → Robot dịch sang ô vùng mới: {next_start}")
                self.current_pos = next_start
            else:
                print("   → Robot đã ở vị trí hợp lệ để bắt đầu BM.")

            self.current_dir_index = next_dir_index
            print("8. Điều chỉnh hướng ưu tiên cho BM tiếp theo.")

            step += 1

        return self.total_path, self.grid

# --- printing, demo (unchanged) ---


def print_grid(grid):
    symbol_map = {
        FREE_UNCOVERED: '⬜',
        OBSTACLE: '⬛',
        COVERED: '🟩',
        'ROBOT': '🤖'
    }
    print("\nMô hình làm việc (M):")
    for r in range(len(grid)):
        row_str = ""
        for c in range(len(grid[0])):
            row_str += symbol_map.get(grid[r][c], ' ')
        print(row_str)
    print("-" * 20)


def run_console_demo():
    MAP_SIZE = 10
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
    print_grid(initial_map)
    ba_star_robot = BAStar(initial_map, start_point)
    final_path, final_grid = ba_star_robot.run()
    print("\n--- Kết quả Cuối cùng ---")
    print(f"Tổng số bước di chuyển (BM + A*SPT): {len(final_path) - 1}")
    display_grid = [row[:] for row in final_grid]
    if final_path:
        r, c = final_path[-1]
        display_grid[r][c] = 'ROBOT'
    print_grid(display_grid)


if __name__ == '__main__':
    run_console_demo()
