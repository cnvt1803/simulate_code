import heapq
import math
import pygame
import sys
import time
import threading
import random

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
AStar_PATH = 5


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

def boustrophedon_motion(grid, start_pos, start_dir_index=0, visualizer=None, step_counter=None, coverage_id=1):
    """
    Thực hiện chuyển động boustrophedon (BM) thực sự - chuyển động zigzag như cày ruộng.
    
    Đầu vào: Mô hình M, vị trí bắt đầu, hướng ưu tiên bắt đầu, coverage_id để phân biệt màu.
    Đầu ra: Vị trí cuối (s_cp), mô hình M đã cập nhật, và đường dẫn coverage.
    """
    rows, cols = len(grid), len(grid[0])
    r, c = start_pos
    coverage_path = []
    
    # Xác định hướng chính cho boustrophedon (0: ngang, 1: dọc)
    if start_dir_index in [0, 1]:  # Bắc/Nam -> di chuyển dọc trước
        primary_direction = 1  # Dọc
        secondary_direction = 0  # Ngang
    else:  # Đông/Tây -> di chuyển ngang trước
        primary_direction = 0  # Ngang
        secondary_direction = 1  # Dọc
    
    # B3, B4: Thêm ô bắt đầu vào M
    if grid[r][c] == FREE_UNCOVERED:
        grid[r][c] = COVERED
        coverage_path.append((r, c))
        if visualizer and step_counter:
            step_counter[0] += 1
            visualizer.update_display(grid, (r, c), step_counter[0], coverage_path, coverage_id)
    
    print(f"  -> BM bắt đầu tại: {start_pos}, pattern: {'dọc' if primary_direction == 1 else 'ngang'}")
    
    # Boustrophedon Motion chính
    current_direction = start_dir_index
    sweep_direction = 1 if current_direction in [0, 2] else -1  # 1: tiến, -1: lùi
    
    while True:
        moved = False
        
        # Bước 1: Tiếp tục di chuyển theo hướng hiện tại (sweep)
        if primary_direction == 1:  # Di chuyển dọc
            dr = sweep_direction if current_direction in [0, 1] else -sweep_direction
            dc = 0
        else:  # Di chuyển ngang
            dr = 0
            dc = sweep_direction if current_direction in [2, 3] else -sweep_direction
        
        nr, nc = r + dr, c + dc
        
        if is_valid(grid, nr, nc) and grid[nr][nc] == FREE_UNCOVERED:
            # Tiếp tục sweep theo hướng hiện tại
            r, c = nr, nc
            grid[r][c] = COVERED
            coverage_path.append((r, c))
            moved = True
            
            if visualizer and step_counter:
                step_counter[0] += 1
                visualizer.update_display(grid, (r, c), step_counter[0], coverage_path, coverage_id)
        else:
            # Bước 2: Không thể tiếp tục sweep, thử chuyển sang hàng/cột tiếp theo
            if primary_direction == 1:  # Đang sweep dọc, chuyển sang cột tiếp theo
                dc = 1 if sweep_direction > 0 else -1
                dr = 0
            else:  # Đang sweep ngang, chuyển sang hàng tiếp theo
                dr = 1 if sweep_direction > 0 else -1
                dc = 0
            
            nr, nc = r + dr, c + dc
            
            if is_valid(grid, nr, nc) and grid[nr][nc] == FREE_UNCOVERED:
                # Chuyển sang hàng/cột mới và đảo chiều sweep
                r, c = nr, nc
                grid[r][c] = COVERED
                coverage_path.append((r, c))
                sweep_direction *= -1  # Đảo chiều cho boustrophedon zigzag
                moved = True
                
                if visualizer and step_counter:
                    step_counter[0] += 1
                    visualizer.update_display(grid, (r, c), step_counter[0], coverage_path, coverage_id)
            else:
                # Bước 3: Thử tất cả hướng khác (fallback)
                for i, (test_dr, test_dc) in enumerate(DIRECTIONS_BM):
                    nr, nc = r + test_dr, c + test_dc
                    if is_valid(grid, nr, nc) and grid[nr][nc] == FREE_UNCOVERED:
                        r, c = nr, nc
                        grid[r][c] = COVERED
                        coverage_path.append((r, c))
                        current_direction = i
                        moved = True
                        
                        if visualizer and step_counter:
                            step_counter[0] += 1
                            visualizer.update_display(grid, (r, c), step_counter[0], coverage_path, coverage_id)
                        break
        
        if not moved:
            # Điểm tới hạn đã đạt được
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
        self.visualizer = None
        self.step_count = 0
        self.coverage_paths = []  # Lưu các đường coverage
        self.astar_paths = []  # Lưu các đường A*
        self.coverage_count = 0  # Đếm số lần coverage
        
    def set_visualizer(self, visualizer):
        """Set the visualizer for real-time display"""
        self.visualizer = visualizer
        
    def find_backtracking_list(self):
        """
        B3: Phát hiện danh sách điểm quay lui L (theo Công thức 8).
        
        Điểm quay lui là điểm đã được bao phủ (COVERED) và:
        1. Có ít nhất một ô lân cận chưa được bao phủ (FREE_UNCOVERED)
        2. Là điểm biên của vùng đã bao phủ (không nằm giữa các ô đã bao phủ)
        3. Có thể tiếp cận được vùng chưa bao phủ mới
        """
        backtracking_list = []
        
        # Kiểm tra 4 hướng chính (vì BM chỉ di chuyển ngang/dọc)
        directions = [(-1, 0), (1, 0), (0, 1), (0, -1)]

        for r in range(self.rows):
            for c in range(self.cols):
                if self.grid[r][c] == COVERED:
                    # Kiểm tra xem đây có phải là điểm biên không
                    free_neighbors = 0
                    covered_neighbors = 0
                    obstacle_neighbors = 0
                    
                    # Đếm các loại lân cận
                    for dr, dc in directions:
                        nr, nc = r + dr, c + dc
                        if is_valid(self.grid, nr, nc):
                            if self.grid[nr][nc] == FREE_UNCOVERED:
                                free_neighbors += 1
                            elif self.grid[nr][nc] == COVERED:
                                covered_neighbors += 1
                            elif self.grid[nr][nc] == OBSTACLE:
                                obstacle_neighbors += 1
                        else:
                            # Biên của lưới coi như obstacle
                            obstacle_neighbors += 1
                    
                    # Điều kiện là backtracking point:
                    # 1. Có ít nhất 1 ô FREE_UNCOVERED lân cận
                    # 2. Không bị bao vây hoàn toàn bởi các ô COVERED
                    # 3. Có thể dẫn đến vùng coverage mới
                    is_critical = False
                    
                    if free_neighbors > 0:
                        # Có ô tự do lân cận
                        # Kiểm tra xem có thể dẫn đến vùng coverage rộng không
                        reachable_area = self.estimate_reachable_uncovered_area(r, c)
                        if reachable_area >= 3:  # Ít nhất 3 ô có thể bao phủ thêm
                            is_critical = True
                    
                    if is_critical:
                        backtracking_list.append((r, c))
                        
        # Loại bỏ các điểm trùng lặp và quá gần nhau
        filtered_list = self.filter_redundant_backtracking_points(backtracking_list)
        
        return filtered_list
    
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
        
        Để đơn giản, chúng ta chọn điểm quay lui gần nhất với điểm tới hạn (s_cp) hiện tại.
        """
        if not backtracking_list:
            return None, 0
        
        best_sp = None
        min_dist = float('inf')
        
        cp_r, cp_c = self.current_cp
        
        for r, c in backtracking_list:
            dist = math.hypot(r - cp_r, c - cp_c)
            if dist < min_dist:
                min_dist = dist
                best_sp = (r, c)
                
        if best_sp is None:
            return None, 0
                
        # Công thức 9 cũng cần xác định hướng tiếp theo (chỉ số hướng)
        # Hướng ưu tiên sẽ là hướng dẫn đến vùng FREE_UNCOVERED gần nhất
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
        step_counter = [self.step_count]  # Use list for mutable reference
        
        while True:
            print(f"\n--- Chu trình Bao phủ #{step} ---")
            print(f"Vị trí hiện tại: {self.current_pos}, Hướng: {self.current_dir_index}")
            self.coverage_count += 1
            
            # B2: Bao phủ không gian làm việc dựa trên BM
            print("1. Thực hiện Chuyển động Boustrophedon (BM)...")
            s_cp, self.grid, coverage_path = boustrophedon_motion(
                self.grid, self.current_pos, self.current_dir_index, 
                self.visualizer, step_counter, self.coverage_count
            )
            self.current_cp = s_cp
            self.step_count = step_counter[0]
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
            if self.visualizer:
                self.visualizer.show_selected_backtracking_point(s_sp)
            
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
            if self.visualizer:
                self.visualizer.show_astar_path(path_smoothed)
            
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

# --- 7. Visualization Class ---

class GridVisualizer:
    def __init__(self, grid_size=50, cell_size=10, step_size=0.001):
        self.grid_size = grid_size
        self.cell_size = cell_size
        self.step_size = step_size
        self.window_size = grid_size * cell_size
        self.panel_width = 300
        self.total_width = self.window_size + self.panel_width + 20
        self.total_height = max(self.window_size, 700)
        
        # Game state
        self.is_paused = False
        self.is_running = False
        self.algorithm_running = False
        self.step_count = 0
        
        # Path tracking for visualization
        self.coverage_paths = []  # List of coverage paths with different colors
        self.selected_backtracking_points = []  # Only show selected backtracking points
        self.current_astar_path = []
        self.path_colors = [(0, 0, 255), (128, 0, 128), (255, 165, 0), (139, 69, 19), (255, 20, 147), (0, 255, 255)]
        
        # Colors
        self.colors = {
            'white': (255, 255, 255),
            'black': (0, 0, 0),
            'gray': (128, 128, 128),
            'light_gray': (211, 211, 211),
            'dark_gray': (64, 64, 64),
            'green': (0, 255, 0),
            'dark_green': (0, 128, 0),
            'red': (255, 0, 0),
            'dark_red': (128, 0, 0),
            'covered': (144, 238, 144),
            'panel_bg': (240, 240, 240),
            'button_normal': (200, 200, 200),
            'button_hover': (180, 180, 180),
        }
        
        # Initialize pygame
        pygame.init()
        self.screen = pygame.display.set_mode((self.total_width, self.total_height))
        pygame.display.set_caption("BA* Algorithm Visualization - 50x50 Grid")
        self.clock = pygame.time.Clock()
        
        # Fonts
        self.font_large = pygame.font.Font(None, 28)
        self.font_medium = pygame.font.Font(None, 22)
        self.font_small = pygame.font.Font(None, 18)
        
        # UI elements
        self.buttons = self.setup_buttons()
        
        # Initialize grid
        self.grid = [[0 for _ in range(grid_size)] for _ in range(grid_size)]
        self.robot_pos = (0, 0)
        self.covered_count = 0
        self.prev_robot_pos = (0, 0)  # For optimized drawing
        
        # Add some obstacles for demonstration
        self.add_sample_obstacles()
    
    def setup_buttons(self):
        """Setup UI buttons"""
        button_width = 120
        button_height = 35
        start_x = self.window_size + 40
        start_y = 220  # Moved down to avoid covering parameters
        
        return {
            'start': {
                'rect': pygame.Rect(start_x, start_y, button_width, button_height),
                'text': 'Start BA*',
                'enabled': True,
                'color': self.colors['button_normal']
            },
            'pause': {
                'rect': pygame.Rect(start_x, start_y + 45, button_width, button_height),
                'text': 'Pause',
                'enabled': False,
                'color': self.colors['button_normal']
            },
            'reset': {
                'rect': pygame.Rect(start_x, start_y + 90, button_width, button_height),
                'text': 'Reset',
                'enabled': True,
                'color': self.colors['button_normal']
            }
        }
    
    def add_sample_obstacles(self):
        """Add randomly generated obstacles to demonstrate the algorithm"""
        # Generate random obstacles
        num_obstacles = random.randint(5, 8)  # Random number of obstacles (5-8)
        
        for _ in range(num_obstacles):
            # Random obstacle size (2x2 to 5x5)
            obstacle_width = random.randint(2, 5)
            obstacle_height = random.randint(2, 5)
            
            # Random position, ensuring obstacle fits within grid and leaves border space
            max_x = self.grid_size - obstacle_width - 5  # Leave 5 cell border
            max_y = self.grid_size - obstacle_height - 5
            
            if max_x > 5 and max_y > 5:  # Ensure valid placement area
                start_r = random.randint(5, max_y)
                start_c = random.randint(5, max_x)
                
                # Add the obstacle
                for r in range(start_r, start_r + obstacle_height):
                    for c in range(start_c, start_c + obstacle_width):
                        if 0 <= r < self.grid_size and 0 <= c < self.grid_size:
                            self.grid[r][c] = OBSTACLE
        
        # Add some scattered single-cell obstacles for variety
        num_scattered = random.randint(3, 7)
        for _ in range(num_scattered):
            r = random.randint(10, self.grid_size - 10)
            c = random.randint(10, self.grid_size - 10)
            if self.grid[r][c] == FREE_UNCOVERED:
                self.grid[r][c] = OBSTACLE
    
    def handle_events(self):
        """Handle pygame events"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.is_running = False
                return False
            
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:  # Left click
                    self.handle_mouse_click(event.pos)
            
            elif event.type == pygame.MOUSEMOTION:
                self.handle_mouse_hover(event.pos)
            
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    self.toggle_pause()
                elif event.key == pygame.K_r:
                    self.reset_grid()
                elif event.key == pygame.K_s:
                    self.start_algorithm()
        
        return True
    
    def handle_mouse_click(self, pos):
        """Handle mouse clicks on UI elements"""
        for btn_id, btn_data in self.buttons.items():
            if btn_data['rect'].collidepoint(pos) and btn_data['enabled']:
                if btn_id == 'start':
                    self.start_algorithm()
                elif btn_id == 'pause':
                    self.toggle_pause()
                elif btn_id == 'reset':
                    self.reset_grid()
    
    def handle_mouse_hover(self, pos):
        """Handle mouse hover effects"""
        for btn_data in self.buttons.values():
            if btn_data['rect'].collidepoint(pos) and btn_data['enabled']:
                btn_data['color'] = self.colors['button_hover']
            else:
                btn_data['color'] = self.colors['button_normal']
    
    def show_selected_backtracking_point(self, selected_point):
        """Show only the selected backtracking point when it's chosen for navigation"""
        if selected_point not in self.selected_backtracking_points:
            self.selected_backtracking_points.append(selected_point)
    
    def show_astar_path(self, astar_path):
        """Show A* path in black"""
        if astar_path not in self.current_astar_path:
            self.current_astar_path.append(astar_path.copy())
            time.sleep(0.5)  # Show A* path for a moment
    
    def draw_grid(self):
        """Draw the main grid"""
        grid_surface = pygame.Surface((self.window_size, self.window_size))
        grid_surface.fill(self.colors['white'])
        
        # Base color mapping
        colors = {
            FREE_UNCOVERED: self.colors['white'],
            OBSTACLE: self.colors['black'],
            COVERED: self.colors['white']  # Keep white to avoid eye strain
        }
        
        # Draw base grid cells
        for r in range(self.grid_size):
            for c in range(self.grid_size):
                x = c * self.cell_size
                y = r * self.cell_size
                rect = pygame.Rect(x, y, self.cell_size, self.cell_size)
                
                color = colors.get(self.grid[r][c], self.colors['white'])
                pygame.draw.rect(grid_surface, color, rect)
                pygame.draw.rect(grid_surface, self.colors['light_gray'], rect, 1)
        
        # Draw coverage paths with different colors
        for path_idx, coverage_path in enumerate(self.coverage_paths):
            if coverage_path and len(coverage_path) > 1:
                path_color = self.path_colors[path_idx % len(self.path_colors)]
                points = [(c * self.cell_size + self.cell_size // 2,
                          r * self.cell_size + self.cell_size // 2) for r, c in coverage_path]
                pygame.draw.lines(grid_surface, path_color, False, points, 3)
        
        # Draw A* paths in black
        for astar_path in self.current_astar_path:
            if len(astar_path) > 1:
                points = [(c * self.cell_size + self.cell_size // 2,
                          r * self.cell_size + self.cell_size // 2) for r, c in astar_path]
                pygame.draw.lines(grid_surface, self.colors['black'], False, points, 4)
        
        # Only draw selected backtracking points as red diamonds
        for r, c in self.selected_backtracking_points:
            cx = c * self.cell_size + self.cell_size // 2
            cy = r * self.cell_size + self.cell_size // 2
            size = self.cell_size // 3
            
            # Create diamond shape
            points = [
                (cx, cy - size),  # top
                (cx + size, cy),  # right
                (cx, cy + size),  # bottom
                (cx - size, cy)   # left
            ]
            pygame.draw.polygon(grid_surface, self.colors['red'], points)
            pygame.draw.polygon(grid_surface, self.colors['dark_red'], points, 2)
        
        # Draw robot position as a small car
        robot_r, robot_c = self.robot_pos
        robot_x = robot_c * self.cell_size + self.cell_size // 2
        robot_y = robot_r * self.cell_size + self.cell_size // 2
        
        # Draw robot as a small car
        self.draw_robot_car(grid_surface, robot_x, robot_y)
        
        return grid_surface
    
    def draw_robot_car(self, surface, x, y):
        """Draw robot as a small car"""
        car_width = max(6, self.cell_size // 2)
        car_height = max(4, self.cell_size // 3)
        wheel_size = max(1, self.cell_size // 8)
        
        # Car body (main rectangle)
        car_rect = pygame.Rect(x - car_width//2, y - car_height//2, car_width, car_height)
        pygame.draw.rect(surface, self.colors['green'], car_rect)
        pygame.draw.rect(surface, self.colors['dark_green'], car_rect, 1)
        
        # Car windshield (smaller rectangle at front)
        windshield_width = car_width // 3
        windshield_height = car_height // 2
        windshield_rect = pygame.Rect(x + car_width//2 - windshield_width, 
                                    y - windshield_height//2, 
                                    windshield_width, windshield_height)
        pygame.draw.rect(surface, (200, 255, 200), windshield_rect)  # Light green windshield
        pygame.draw.rect(surface, self.colors['dark_green'], windshield_rect, 1)
        
        # Wheels (4 small circles)
        wheel_offset_x = car_width // 3
        wheel_offset_y = car_height // 2
        
        # Front wheels
        pygame.draw.circle(surface, (64, 64, 64), 
                         (x + wheel_offset_x, y - wheel_offset_y), wheel_size)
        pygame.draw.circle(surface, (64, 64, 64), 
                         (x + wheel_offset_x, y + wheel_offset_y), wheel_size)
        
        # Rear wheels  
        pygame.draw.circle(surface, (64, 64, 64), 
                         (x - wheel_offset_x, y - wheel_offset_y), wheel_size)
        pygame.draw.circle(surface, (64, 64, 64), 
                         (x - wheel_offset_x, y + wheel_offset_y), wheel_size)
        
        # Direction indicator (small arrow pointing forward)
        arrow_size = max(2, self.cell_size // 6)
        arrow_points = [
            (x + car_width//2 + 2, y),  # Arrow tip
            (x + car_width//2 - arrow_size, y - arrow_size//2),  # Top
            (x + car_width//2 - arrow_size, y + arrow_size//2)   # Bottom
        ]
        pygame.draw.polygon(surface, self.colors['red'], arrow_points)
    
    def draw_info_panel(self):
        """Draw the information panel"""
        panel_surface = pygame.Surface((self.panel_width, self.total_height))
        panel_surface.fill(self.colors['panel_bg'])
        
        y_offset = 20
        
        # Title
        title_text = self.font_large.render("BA* Algorithm", True, self.colors['black'])
        panel_surface.blit(title_text, (20, y_offset))
        y_offset += 40
        
        # Algorithm info
        info_texts = [
            f"Step: {self.step_count}",
            f"Position: {self.robot_pos}",
            f"Covered: {self.covered_count}",
            f"Coverage Paths: {len(self.coverage_paths)}",
            f"A* Paths: {len(self.current_astar_path)}",
            f"Status: {'Running' if self.algorithm_running else 'Ready'}"
        ]
        
        for text in info_texts:
            text_surface = self.font_small.render(text, True, self.colors['dark_gray'])
            panel_surface.blit(text_surface, (20, y_offset))
            y_offset += 25
        
        y_offset += 80
        
        # Draw buttons
        for btn_data in self.buttons.values():
            btn_color = btn_data['color'] if btn_data['enabled'] else self.colors['gray']
            btn_rect_local = pygame.Rect(btn_data['rect'].x - (self.window_size + 20), btn_data['rect'].y,
                                       btn_data['rect'].width, btn_data['rect'].height)
            
            pygame.draw.rect(panel_surface, btn_color, btn_rect_local)
            pygame.draw.rect(panel_surface, self.colors['dark_gray'], btn_rect_local, 2)
            
            # Button text
            text_color = self.colors['black'] if btn_data['enabled'] else self.colors['gray']
            btn_text = self.font_small.render(btn_data['text'], True, text_color)
            text_rect = btn_text.get_rect(center=btn_rect_local.center)
            panel_surface.blit(btn_text, text_rect)
        
        # Legend
        y_offset += 50
        legend_text = self.font_medium.render("Legend:", True, self.colors['black'])
        panel_surface.blit(legend_text, (20, y_offset))
        y_offset += 30
        
        legend_items = [
            ("Coverage Paths: Colored lines", self.path_colors[0]),
            ("A* Paths: Black lines", self.colors['black']),
            ("Backtrack: Red diamonds", self.colors['red']),
            ("Robot: Green circle", self.colors['green'])
        ]
        
        for text, color in legend_items:
            legend_surface = self.font_small.render(text, True, color)
            panel_surface.blit(legend_surface, (20, y_offset))
            y_offset += 20
        
        return panel_surface
    
    def update_display(self, new_grid, robot_pos, step_count, coverage_path=None, coverage_id=1):
        """Update the display with new grid state and coverage path"""
        # Quick check if we should continue
        if not self.is_running:
            return
            
        # Wait while paused
        while self.is_paused and self.is_running:
            pygame.event.pump()
            time.sleep(0.05)
        
        if not self.is_running:
            return
            
        self.grid = [row[:] for row in new_grid]
        self.robot_pos = robot_pos
        self.step_count = step_count
        
        # Update coverage path if provided
        if coverage_path is not None:
            # Ensure we have enough coverage paths stored
            while len(self.coverage_paths) < coverage_id:
                self.coverage_paths.append([])
            
            # Update the specific coverage path
            if coverage_id <= len(self.coverage_paths):
                self.coverage_paths[coverage_id - 1] = coverage_path.copy()
        
        # Count covered cells (only when needed)
        if self.step_count % 10 == 0:  # Update count every 10 steps
            self.covered_count = sum(row.count(COVERED) for row in self.grid)
        
        # Apply speed delay only if not paused
        if not self.is_paused and self.step_size > 0:
            time.sleep(max(0.001, self.step_size))  # Minimum 1ms delay
    
    def toggle_pause(self):
        """Toggle pause/resume state"""
        if self.algorithm_running:
            self.is_paused = not self.is_paused
            self.buttons['pause']['text'] = 'Resume' if self.is_paused else 'Pause'
    
    def start_algorithm(self):
        """Start the BA* algorithm in a separate thread"""
        if not self.algorithm_running:
            self.algorithm_running = True
            self.update_button_states()
            
            def run_ba_star():
                ba_star_robot = BAStar(self.grid, self.robot_pos)
                ba_star_robot.set_visualizer(self)
                final_path, final_grid = ba_star_robot.run()
                self.algorithm_running = False
                self.update_button_states()
            
            thread = threading.Thread(target=run_ba_star)
            thread.daemon = True
            thread.start()
    
    def update_button_states(self):
        """Update button enabled states"""
        running = self.algorithm_running
        self.buttons['start']['enabled'] = not running
        self.buttons['pause']['enabled'] = running
        self.buttons['reset']['enabled'] = not running
        self.is_paused = False
        self.buttons['pause']['text'] = 'Pause'
    
    def reset_grid(self):
        """Reset the grid to initial state"""
        if not self.algorithm_running:
            self.is_running = False
            self.is_paused = False
            self.grid = [[0 for _ in range(self.grid_size)] for _ in range(self.grid_size)]
            self.robot_pos = (0, 0)
            self.step_count = 0
            self.covered_count = 0
            self.coverage_paths = []
            self.selected_backtracking_points = []
            self.current_astar_path = []
            self.add_sample_obstacles()
            self.update_button_states()
    
    def run(self):
        """Start the GUI main loop"""
        self.is_running = True
        
        print("--- BA* Algorithm Visualization ---")
        print("Controls:")
        print("  S: Start Algorithm")
        print("  SPACE: Pause/Resume")
        print("  R: Reset")
        print("Starting Pygame GUI...")
        
        while self.is_running:
            if not self.handle_events():
                break
            
            # Draw everything
            self.screen.fill(self.colors['white'])
            
            # Draw grid
            grid_surface = self.draw_grid()
            self.screen.blit(grid_surface, (10, 10))
            
            # Draw info panel
            panel_surface = self.draw_info_panel()
            self.screen.blit(panel_surface, (self.window_size + 20, 0))
            
            # Update display
            pygame.display.flip()
            self.clock.tick(60)
        
        pygame.quit()
        sys.exit()

# --- 8. Ví dụ Mô phỏng ---

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
    # Launch the visualization window with 20x20 grid and bigger cells
    visualizer = GridVisualizer(grid_size=20, cell_size=25, step_size=0.01)
    visualizer.run()
