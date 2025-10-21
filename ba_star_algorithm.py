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
            move_cost = int(math.sqrt(dr*dr + dc*dc) * 10) / 10  # Convert to avoid float precision issues
            new_cost = cost_so_far[current] + move_cost

            if next_cell not in cost_so_far or new_cost < cost_so_far[next_cell]:
                cost_so_far[next_cell] = new_cost #type: ignore
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
    if not path:
        return []

    # B1: Khởi tạo
    path_smoothed = [path[0]]
    k = 0
    n = len(path) - 1 # Chỉ số cuối cùng

    while True:
        s_k = path_smoothed[-1] # Ô hiện tại (s_k)
        
        # B2: Tìm ô s_i xa nhất có đường ngắm trực tiếp từ s_k
        best_i = k + 1 # Mặc định là ô tiếp theo (ngắn nhất)
        
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

def boustrophedon_motion(grid, start_pos, start_dir_index=0, callback=None, coverage_id=1):
    """
    Thực hiện chuyển động boustrophedon (BM) theo Algorithm 3.
    
    Robot sẽ kiểm tra lần lượt các ô trống theo hướng ưu tiên: north-south-east-west
    
    Đầu vào: Mô hình M, vị trí bắt đầu, hướng ưu tiên bắt đầu, callback để visualization, coverage_id để phân biệt màu.
    Đầu ra: Vị trí cuối (s_cp), mô hình M đã cập nhật, và đường dẫn coverage.
    """
    rows, cols = len(grid), len(grid[0])
    r, c = start_pos
    coverage_path = []
    
    # Step 4: Thêm ô bắt đầu vào mô hình M (đánh dấu là đã bao phủ)
    if grid[r][c] == FREE_UNCOVERED:
        grid[r][c] = COVERED
        coverage_path.append((r, c))
        if callback:
            callback(grid, (r, c), coverage_path, coverage_id)
    
    print(f"  -> BM bắt đầu tại: {start_pos}")
    
    # Boustrophedon Motion theo Algorithm 3
    while True:
        moved = False
        
        # Step 1: Kiểm tra hướng đầu tiên khả dụng theo thứ tự ưu tiên: North-South-East-West
        # Step 2: Di chuyển một bước theo hướng này
        for i, (dr, dc) in enumerate(DIRECTIONS_BM):
            nr, nc = r + dr, c + dc
            
            # Kiểm tra xem ô tiếp theo có hợp lệ và chưa được bao phủ không
            if is_valid(grid, nr, nc) and grid[nr][nc] == FREE_UNCOVERED:
                # Step 2: Di chuyển một bước theo hướng này
                r, c = nr, nc
                
                # Step 3: Tạo tile s = (x, y, 2r) - đánh dấu ô như đường kính robot
                # Step 4: Thêm tile s vào mô hình M
                grid[r][c] = COVERED
                coverage_path.append((r, c))
                moved = True
                
                if callback:
                    callback(grid, (r, c), coverage_path, coverage_id)
                
                print(f"  -> Di chuyển từ {(r-dr, c-dc)} đến {(r, c)} theo hướng {['Bắc', 'Nam', 'Đông', 'Tây'][i]}")
                break  # Tìm thấy hướng hợp lệ, thoát khỏi vòng lặp kiểm tra hướng
        
        # Step 1: Nếu tất cả các hướng đều bị chặn, critical point đã đạt được
        if not moved:
            print(f"  -> BM kết thúc tại Critical Point (s_cp): {(r, c)}")
            return (r, c), grid, coverage_path


# --- 6. Thuật toán BA* (Algorithm 5) ---

class BAStar:
    def __init__(self, initial_grid, start_pos):
        self.grid = [row[:] for row in initial_grid]  # B1: Khởi tạo M rỗng (hoặc bản đồ chướng ngại vật ban đầu)
        self.rows = len(initial_grid)
        self.cols = len(initial_grid[0])
        self.current_pos = start_pos
        self.current_cp = start_pos
        self.current_dir_index = 0 # Hướng ban đầu cho BM
        self.total_path = [start_pos]
        self.step_count = 0
        self.coverage_paths = []  # Lưu các đường coverage
        self.astar_paths = []  # Lưu các đường A*
        self.coverage_count = 0  # Đếm số lần coverage
        
        # Callback functions for visualization
        self.on_step_callback = None
        self.on_backtrack_callback = None
        self.on_astar_callback = None
        
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
            (-1, -1), # s4: north-west
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
        filtered_list = self.filter_redundant_backtracking_points(backtracking_list)
        
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
                distance = abs(point[0] - existing_point[0]) + abs(point[1] - existing_point[1])
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
        """Thực thi thuật toán BA*."""
        print("--- Bắt đầu Thuật toán BA* ---")
        step = 1
        
        while True:
            print(f"\n--- Chu trình Bao phủ #{step} ---")
            print(f"Vị trí hiện tại: {self.current_pos}, Hướng: {self.current_dir_index}")
            self.coverage_count += 1
            
            # B2: Bao phủ không gian làm việc dựa trên BM
            print("1. Thực hiện Chuyển động Boustrophedon (BM)...")
            s_cp, self.grid, coverage_path = boustrophedon_motion(
                self.grid, self.current_pos, self.current_dir_index, 
                self.on_step_callback, self.coverage_count
            )
            self.current_cp = s_cp
            self.coverage_paths.append(coverage_path)
            
            # B3: Phát hiện danh sách điểm quay lui L
            backtracking_list = self.find_backtracking_list()
            print(f"2. Đã phát hiện {len(backtracking_list)} điểm quay lui: {backtracking_list}")
            
            # B4: Kiểm tra L
            if not backtracking_list:
                print("3. Danh sách quay lui rỗng. Nhiệm vụ bao phủ hoàn tất.")
                break
                
            # B5: Xác định điểm bắt đầu s_sp tốt nhất
            result = self.select_best_start_point(backtracking_list)
            if result[0] is None:
                print("   !! Lỗi: Không thể tìm thấy điểm bắt đầu hợp lệ. Dừng.")
                break
            s_sp, next_dir_index = result
            print(f"4. Điểm bắt đầu tiếp theo (s_sp) được chọn: {s_sp}")
            print(f"   Khoảng cách từ {s_cp} đến {s_sp}: {abs(s_cp[0]-s_sp[0]) + abs(s_cp[1]-s_sp[1])}")
            
            # Hiển thị điểm backtracking được chọn
            if self.on_backtrack_callback:
                self.on_backtrack_callback(s_sp)
            
            # B6: Lập kế hoạch đường dẫn không va chạm (A*) từ s_cp đến s_sp
            print(f"5. Lập kế hoạch đường dẫn A* từ {s_cp} đến {s_sp}...")
            path_astar = a_star_search(self.grid, s_cp, s_sp)
            
            if not path_astar:
                print("   !! Lỗi: Không thể tìm thấy đường dẫn A* đến s_sp. Dừng.")
                break
            
            print(f"   Đường dẫn A* thô: {len(path_astar)} bước.")
            
            # B7: Rút ngắn đường dẫn bằng A*SPT
            path_smoothed = a_star_spt(self.grid, path_astar)
            print(f"6. Đường dẫn được làm mịn (A*SPT): {len(path_smoothed)} bước.")
            
            # Lưu đường A* để visualization
            self.astar_paths.append(path_smoothed)
            
            # B8: Theo dõi đường dẫn thu được (Cập nhật vị trí và tổng đường đi)
            print("7. Theo dõi đường dẫn (Công thức 11)...")
            
            # Hiển thị đường A* với màu đen
            if self.on_astar_callback:
                self.on_astar_callback(path_smoothed)
            
            # Chỉ thêm phần đường dẫn di chuyển vào tổng đường đi
            for pos in path_smoothed[1:]: 
                self.total_path.append(pos)
                self.current_pos = pos
                
            print(f"   Robot đã di chuyển đến s_sp: {self.current_pos}")
            
            # B9: Điều chỉnh góc hướng tại s_sp (Công thức 12)
            self.current_dir_index = next_dir_index
            print(f"8. Điều chỉnh hướng ưu tiên cho BM tiếp theo.")
            
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