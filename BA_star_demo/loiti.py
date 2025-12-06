# ==============================================================================
# KHỞI TẠO TRẠNG THÁI BÁM TƯỜNG (DYNAMIC INIT)
# Thay vì gán False, kiểm tra ngay vị trí xuất phát xem có sát tường không
# ==============================================================================

# 1. Định nghĩa hướng tương đối (cho bước init)
_LEFT_OF_INIT = {0: 3, 1: 2, 2: 0, 3: 1}
_RIGHT_OF_INIT = {0: 2, 1: 3, 2: 1, 3: 0}

 _init_left_sd = _LEFT_OF_INIT[long_dir]
  _init_right_sd = _RIGHT_OF_INIT[long_dir]

   # 2. Hàm check tường tại chỗ
   def _is_wall_init(rr, cc, side_dir):
        sr = rr + DIRECTIONS_BM[side_dir][0]
        sc = cc + DIRECTIONS_BM[side_dir][1]
        # Ra ngoài biên hoặc gặp OBSTACLE/COVERED -> Coi là tường
        if not (0 <= sr < rows and 0 <= sc < cols):
            return True
        if grid[sr][sc] in (OBSTACLE, COVERED):
            return True
        return False

    # 3. Kiểm tra 2 bên
    _is_start_left_wall = _is_wall_init(r, c, _init_left_sd)
    _is_start_right_wall = _is_wall_init(r, c, _init_right_sd)

    # 4. Gán trạng thái ban đầu
    walking_along_wall = False
    wall_side = None

    if _is_start_left_wall:
        walking_along_wall = True
        wall_side = _init_left_sd
        print(f"INIT: Bắt đầu sát tường TRÁI (Hướng {wall_side})")
    elif _is_start_right_wall:
        walking_along_wall = True
        wall_side = _init_right_sd
        print(f"INIT: Bắt đầu sát tường PHẢI (Hướng {wall_side})")
    else:
        print("INIT: Bắt đầu ở vùng trống (Không sát tường)")

    # ==============================================================================




def find_adjacent_hole_rep(grid, r, c, max_bfs=500, sensor_radius=None, ignore_dir_index=None):
    """
    Kiểm trong phạm vi cảm biến xem có ô FREE_UNCOVERED kề robot thuộc hole không.
    Trả về:
      - (nr,nc) nếu chắc chắn là hole representative (within sensor radius / BFS),
      - None nếu không tìm thấy hoặc inconclusive.
    Tham số:
      - max_bfs: giới hạn số bước BFS để tránh tốn thời gian (None hoặc int).
      - sensor_radius: None => tìm đến biên (hành vi cũ).
                       >=0  => không mở rộng BFS ngoài Manhattan distance > sensor_radius.
      - ignore_dir_index: nếu != None (0:N,1:S,2:E,3:W) sẽ bỏ qua neighbour tương ứng.
    """
    rows, cols = len(grid), len(grid[0])
    FREE = FREE_UNCOVERED

    # helper BFS trả về:
    #   True  -> reachable to border
    #   False -> fully explored within R (no border reached) => confirmed hole
    #   None  -> inconclusive (timeout hoặc gặp rìa sensor có neighbour free)
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

            # nếu chạm biên lưới thì reachable
            if rr == 0 or rr == rows - 1 or cc == 0 or cc == cols - 1:
                return True

            # nếu có sensor_radius và đã đạt R: KHÔNG expand tiếp
            if R is not None and dist >= R:
                # nếu có neighbor FREE mà chưa visited -> inconclusive (có thể nối ra ngoài phạm vi)
                for dr, dc in dirs4:
                    nr, nc = rr + dr, cc + dc
                    if 0 <= nr < rows and 0 <= nc < cols and grid[nr][nc] == FREE and (nr, nc) not in visited:
                        reached_r_boundary = True
                steps += 1
                if bfs_limit is not None and steps >= bfs_limit:
                    return None
                continue

            # expand bình thường
            for dr, dc in dirs4:
                nr, nc = rr + dr, cc + dc
                if 0 <= nr < rows and 0 <= nc < cols and (nr, nc) not in visited and grid[nr][nc] == FREE:
                    visited.add((nr, nc))
                    q.append((nr, nc, dist + 1))

            steps += 1
            if bfs_limit is not None and steps >= bfs_limit:
                return None

        # BFS kết thúc mà không chạm biên
        if reached_r_boundary:
            return None
        return False

    # prepare ignore delta if provided
    ignore_delta = None
    if ignore_dir_index is not None and 0 <= ignore_dir_index < 4:
        ignore_delta = DIRECTIONS_BM[ignore_dir_index]

    # 1) kiểm ô hiện tại trước
    if 0 <= r < rows and 0 <= c < cols and grid[r][c] == FREE:
        res = is_reachable_to_border(r, c, bfs_limit=max_bfs, R=sensor_radius)
        if res is False:
            return (r, c)
        # res True -> không phải hole; res None -> inconclusive -> skip

    # 2) kiểm 4 neighbour nhưng bỏ hướng forward (ignore_delta) nếu có
    neighbor_dirs = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    for dr, dc in neighbor_dirs:
        if ignore_delta is not None and (dr, dc) == ignore_delta:
            # bỏ qua ô phía trước
            continue
        nr, nc = r + dr, c + dc
        if 0 <= nr < rows and 0 <= nc < cols and grid[nr][nc] == FREE:
            res = is_reachable_to_border(
                nr, nc, bfs_limit=max_bfs, R=sensor_radius)
            if res is False:
                # In ra để xem nó bắt được ô nào
                print(
                    f"DEBUG: Phát hiện HOLE tại {nr}, {nc}. Robot đang ở {r}, {c}")
                return (nr, nc)
            # if res True or None -> tiếp tục kiểm các neighbour khác

    return None