"""
C* Algorithm Visualization using Pygame
Mô phỏng trực quan thuật toán C* (Coverage Path Planning Algorithm) bằng Pygame
"""

import pygame
import sys
import threading
import time
import math
from c_star_algorithm import CStar, FREE_UNCOVERED, OBSTACLE, COVERED, FRONTIER


class CStarPygameVisualizer:
    def __init__(self, grid_size=50, cell_size=12, step_size=0.01):
        # ======== Cấu hình ban đầu ========
        self.grid_size = grid_size              # Số ô theo mỗi chiều (50x50)
        self.cell_size = cell_size              # Kích thước mỗi ô (pixel)
        self.step_size = step_size              # Tốc độ cập nhật (s)
        self.window_size = grid_size * cell_size
        self.panel_width = 300                  # Bảng điều khiển bên phải
        self.total_width = self.window_size + self.panel_width
        self.total_height = max(self.window_size, 700)
        self.sensing_radius = 15  # số ô (cells)
        self.sensing_radius_min = 0
        self.sensing_radius_max = 10

        
        # ======== Trạng thái trò chơi / thuật toán ========
        self.is_paused = False                  # Tạm dừng hay không
        self.is_running = False                 # Cửa sổ đang chạy
        self.algorithm_running = False          # Thuật toán đang hoạt động
        
        # ======== Dữ liệu hiển thị ========
        self.current_path = []                  # Đường đi hiện tại của robot
        self.rcg_nodes = {}                     # Các node của RCG (Region Connection Graph)
        self.rcg_edges = []                     # Các cạnh trong RCG
        self.frontier_nodes = []                # Các node frontier (biên chưa thăm)
        self.covered_areas = []                 # Các vùng đã bao phủ
        
        # ======== Bảng màu ========
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
            'blue': (0, 0, 255),
            'light_blue': (173, 216, 230),
            'yellow': (255, 255, 0),
            'orange': (255, 165, 0),
            'purple': (128, 0, 128),
            'cyan': (0, 255, 255),
            'panel_bg': (240, 240, 240),
            'button_normal': (200, 200, 200),
            'button_hover': (180, 180, 180),
            'button_pressed': (160, 160, 160),
            'covered': (144, 238, 144),  # vùng đã bao phủ
            'path': (255, 20, 147),      # đường đi (màu hồng đậm)
            'rcg_node': (30, 144, 255),  # node của RCG
            'rcg_edge': (100, 149, 237), # cạnh RCG
            'frontier': (255, 69, 0),    # frontier nodes
            'sensing_area': (255, 255, 0, 50)  # vùng cảm biến bán trong suốt
        }
        
        # ======== Khởi tạo pygame ========
        pygame.init()
        self.screen = pygame.display.set_mode((self.total_width, self.total_height))
        pygame.display.set_caption("C* Algorithm Visualization - Pygame")
        self.clock = pygame.time.Clock()
        
        # ======== Font chữ ========
        self.font_large = pygame.font.Font(None, 24)
        self.font_medium = pygame.font.Font(None, 20)
        self.font_small = pygame.font.Font(None, 16)
        
        # ======== Khởi tạo lưới bản đồ ========
        self.grid = [[0 for _ in range(grid_size)] for _ in range(grid_size)]
        self.robot_pos = (0, 0)     # Vị trí robot ban đầu
        self.step_count = 0         # Số bước đã đi
        self.covered_count = 0      # Số ô đã bao phủ
        
        # ======== Thành phần giao diện ========
        self.buttons = {}
        self.slider_value = 10      # Giá trị thanh trượt tốc độ
        self.slider_rect = pygame.Rect(self.window_size + 20, 200, 200, 20)
        self.slider_handle = pygame.Rect(self.window_size + 20, 195, 10, 30)
        self.dragging_slider = False
        
        # ======== Kết quả thuật toán ========
        self.results = {}
        
        # Gọi hàm khởi tạo UI & chướng ngại vật mẫu
        self.setup_ui_elements()
        self.add_sample_obstacles()
    
    # -----------------------------------------------------
    def setup_ui_elements(self):
        """Tạo các nút UI: Start, Pause, Reset"""
        button_width = 120
        button_height = 30
        start_x = self.window_size + 20
        start_y = 220  # Vị trí trên panel
        
        # Danh sách nút
        button_configs = [
            ('start', 'Start C*', start_y),
            ('pause', 'Pause', start_y + 40),
            ('reset', 'Reset', start_y + 80)
        ]
        
        # Tạo từng nút với hình chữ nhật và trạng thái
        for btn_id, text, y_pos in button_configs:
            self.buttons[btn_id] = {
                'rect': pygame.Rect(start_x, y_pos, button_width, button_height),
                'text': text,
                'enabled': True if btn_id != 'pause' else False,
                'color': self.colors['button_normal']
            }

    # -----------------------------------------------------
    def add_sample_obstacles(self):
        """Thêm các chướng ngại vật mẫu vào bản đồ"""
        obstacles = [
            (8, 8, 12, 12),
            (20, 25, 25, 30),
            (35, 15, 40, 20),
            (15, 35, 20, 40),
            (30, 30, 35, 35)
        ]
        # Vẽ ô đen trong phạm vi từng obstacle
        for start_r, start_c, end_r, end_c in obstacles:
            for r in range(start_r, end_r + 1):
                for c in range(start_c, end_c + 1):
                    if 0 <= r < self.grid_size and 0 <= c < self.grid_size:
                        self.grid[r][c] = OBSTACLE
    
    # -----------------------------------------------------
    def handle_events(self):
        """Xử lý các sự kiện (click, phím, thoát, di chuột...)"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.is_running = False
                return False
            
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:
                    self.handle_mouse_click(event.pos)
            
            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1:
                    self.dragging_slider = False
            
            elif event.type == pygame.MOUSEMOTION:
                if self.dragging_slider:
                    self.update_slider(event.pos[0])
                else:
                    self.handle_mouse_hover(event.pos)
            
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    self.toggle_pause()
                elif event.key == pygame.K_r:
                    self.reset_grid()
                elif event.key == pygame.K_s and not self.algorithm_running:
                    self.start_algorithm()
        
        return True
    
    # -----------------------------------------------------
    def handle_mouse_click(self, pos):
        """Xử lý nhấn chuột vào các nút hoặc thanh trượt"""
        for btn_id, btn_data in self.buttons.items():
            if btn_data['rect'].collidepoint(pos) and btn_data['enabled']:
                if btn_id == 'start':
                    self.start_algorithm()
                elif btn_id == 'pause':
                    self.toggle_pause()
                elif btn_id == 'reset':
                    self.reset_grid()
        
        if self.slider_rect.collidepoint(pos):
            self.dragging_slider = True
            self.update_slider(pos[0])
    
    def handle_mouse_hover(self, pos):
        """Đổi màu khi hover chuột lên nút"""
        for btn_data in self.buttons.values():
            if btn_data['rect'].collidepoint(pos) and btn_data['enabled']:
                btn_data['color'] = self.colors['button_hover']
            else:
                btn_data['color'] = self.colors['button_normal']
    
    # -----------------------------------------------------
    def update_slider(self, mouse_x):
        """Cập nhật giá trị thanh trượt tốc độ"""
        relative_x = mouse_x - self.slider_rect.x
        relative_x = max(0, min(relative_x, self.slider_rect.width))
        self.slider_handle.x = self.slider_rect.x + relative_x - 5
        self.slider_value = int((relative_x / self.slider_rect.width) * 200)
        self.step_size = self.slider_value / 1000.0  # tốc độ càng cao, delay càng thấp
    
    # -----------------------------------------------------
    def toggle_pause(self):
        """Tạm dừng hoặc tiếp tục thuật toán"""
        if self.algorithm_running:
            self.is_paused = not self.is_paused
            self.buttons['pause']['text'] = 'Resume' if self.is_paused else 'Pause'
    
    # -----------------------------------------------------
    def start_algorithm(self):
        """Khởi chạy thuật toán C* trong thread riêng"""
        if not self.algorithm_running:
            self.algorithm_running = True
            self.buttons['start']['enabled'] = False
            self.buttons['pause']['enabled'] = True
            self.is_paused = False
            self.buttons['pause']['text'] = 'Pause'
            
            def run_c_star():
                # Tạo instance thuật toán C*
                c_star = CStar(self.grid, self.robot_pos, sensing_radius=self.sensing_radius)

                # Đăng ký callback để cập nhật GUI mỗi bước
                c_star.set_callbacks(
                    step_callback=self.update_display,
                    rcg_update_callback=self.update_rcg_display,
                    path_update_callback=self.update_path_display
                )
                
                # Chạy thuật toán
                final_path, results = c_star.run()
                self.results = results
                
                # Sau khi xong
                self.algorithm_running = False
                self.buttons['start']['enabled'] = True
                self.buttons['pause']['enabled'] = False
                self.buttons['pause']['text'] = 'Pause'
            
            # Chạy trong thread riêng để GUI không bị treo
            thread = threading.Thread(target=run_c_star)
            thread.daemon = True
            thread.start()
    
    # -----------------------------------------------------
    # -----------------------------------------------------
    def update_display(self, new_grid, robot_pos, path, iteration):
        """Callback được gọi sau mỗi bước của thuật toán C* để cập nhật GUI"""
        # Nếu người dùng tắt chương trình hoặc thuật toán ngừng → dừng update
        if not self.is_running or not self.algorithm_running:
            return
            
        # Nếu đang tạm dừng thì chờ (vòng lặp nhỏ giữ GUI vẫn responsive)
        while self.is_paused and self.is_running and self.algorithm_running:
            time.sleep(0.05)
            pygame.event.pump()  # xử lý event tạm thời
        
        if not self.is_running or not self.algorithm_running:
            return
        
        # Cập nhật trạng thái bản đồ và robot
        self.grid = [row[:] for row in new_grid]
        self.robot_pos = robot_pos
        self.current_path = path.copy()
        self.step_count = iteration
        
        # Đếm số ô đã bao phủ (mỗi 5 bước để giảm tần suất tính toán)
        if self.step_count % 5 == 0:
            self.covered_count = sum(row.count(COVERED) for row in self.grid)
        
        # Điều chỉnh tốc độ hiển thị (delay tùy theo step_size)
        if not self.is_paused and self.step_size > 0:
            time.sleep(max(0.001, self.step_size))
    
    # -----------------------------------------------------
    def update_rcg_display(self, rcg):
        """Callback để cập nhật dữ liệu hiển thị RCG (Region Connection Graph)"""
        self.rcg_nodes = {}
        self.rcg_edges = []
        self.frontier_nodes = []
        
        # Duyệt tất cả node trong đồ thị RCG
        for node_id, node in rcg.nodes.items():
            self.rcg_nodes[node_id] = {
                'position': node.position,
                'is_frontier': node.is_frontier
            }
            # Nếu node là frontier (vùng biên chưa được bao phủ) → lưu riêng
            if node.is_frontier:
                self.frontier_nodes.append(node.position)
        
        # Duyệt các cạnh giữa node (tránh trùng cạnh 2 chiều)
        for node_id, node in rcg.nodes.items():
            for neighbor_id in node.neighbors:
                if neighbor_id > node_id:
                    self.rcg_edges.append(
                        (node.position, rcg.nodes[neighbor_id].position)
                    )
    
    # -----------------------------------------------------
    def update_path_display(self, path):
        """Callback cập nhật đường đi (path) của robot"""
        self.current_path = path.copy()
    
    # -----------------------------------------------------
    def draw_grid(self):
        """Vẽ toàn bộ lưới bản đồ và các phần tử trên đó"""
        grid_surface = pygame.Surface((self.window_size, self.window_size))
        grid_surface.fill(self.colors['white'])
        
        # Bảng ánh xạ ô → màu
        cell_colors = {
            FREE_UNCOVERED: self.colors['white'],
            OBSTACLE: self.colors['black'],
            COVERED: self.colors['covered']
        }
        
        # Vẽ từng ô trong grid
        for r in range(self.grid_size):
            for c in range(self.grid_size):
                x = c * self.cell_size
                y = r * self.cell_size
                rect = pygame.Rect(x, y, self.cell_size, self.cell_size)
                
                color = cell_colors.get(self.grid[r][c], self.colors['white'])
                pygame.draw.rect(grid_surface, color, rect)
                pygame.draw.rect(grid_surface, self.colors['light_gray'], rect, 1)
        
        # Vẽ các cạnh (edges) của RCG
        for start_pos, end_pos in self.rcg_edges:
            start_x = start_pos[1] * self.cell_size + self.cell_size // 2
            start_y = start_pos[0] * self.cell_size + self.cell_size // 2
            end_x = end_pos[1] * self.cell_size + self.cell_size // 2
            end_y = end_pos[0] * self.cell_size + self.cell_size // 2
            pygame.draw.line(grid_surface, self.colors['rcg_edge'], 
                             (start_x, start_y), (end_x, end_y), 2)
        
        # Vẽ đường đi (path) của robot
        if len(self.current_path) > 1:
            points = []
            for r, c in self.current_path:
                x = c * self.cell_size + self.cell_size // 2
                y = r * self.cell_size + self.cell_size // 2
                points.append((x, y))
            pygame.draw.lines(grid_surface, self.colors['path'], False, points, 3)
        
        # Vẽ các node RCG
        for node_id, node_data in self.rcg_nodes.items():
            r, c = node_data['position']
            x = c * self.cell_size + self.cell_size // 2
            y = r * self.cell_size + self.cell_size // 2
            if node_data['is_frontier']:
                # Node frontier → chấm đỏ
                pygame.draw.circle(grid_surface, self.colors['frontier'], (x, y), 4)
                pygame.draw.circle(grid_surface, self.colors['dark_red'], (x, y), 4, 2)
            else:
                # Node thường → chấm xanh
                pygame.draw.circle(grid_surface, self.colors['rcg_node'], (x, y), 3)
        
        # Vẽ robot + vùng cảm biến
        robot_r, robot_c = self.robot_pos
        robot_x = robot_c * self.cell_size + self.cell_size // 2
        robot_y = robot_r * self.cell_size + self.cell_size // 2
        
        # Vẽ vùng cảm biến (bán kính 3 ô)
        sensing_radius_pixels = self.sensing_radius * self.cell_size

        sensing_surface = pygame.Surface(
            (sensing_radius_pixels * 2, sensing_radius_pixels * 2), pygame.SRCALPHA
        )
        pygame.draw.circle(
            sensing_surface, self.colors['sensing_area'],
            (sensing_radius_pixels, sensing_radius_pixels), sensing_radius_pixels
        )
        grid_surface.blit(
            sensing_surface, (robot_x - sensing_radius_pixels, robot_y - sensing_radius_pixels)
        )
        
        # Vẽ robot (hình tròn màu xanh)
        robot_radius = max(3, self.cell_size // 4)
        pygame.draw.circle(grid_surface, self.colors['green'], (robot_x, robot_y), robot_radius)
        pygame.draw.circle(grid_surface, self.colors['dark_green'], (robot_x, robot_y), robot_radius, 2)
        
        return grid_surface
    
    # -----------------------------------------------------
    def draw_info_panel(self):
        """Vẽ bảng thông tin bên phải (panel)"""
        panel_surface = pygame.Surface((self.panel_width, self.total_height))
        panel_surface.fill(self.colors['panel_bg'])
        
        y_offset = 20
        # --- Tiêu đề ---
        title_text = self.font_large.render("C* Algorithm", True, self.colors['black'])
        panel_surface.blit(title_text, (20, y_offset))
        y_offset += 40
        
        # --- Hiển thị thông tin ---
        info_texts = [
            f"Iteration: {self.step_count}",
            f"Position: {self.robot_pos}",
            f"Covered: {self.covered_count}",
            f"Path Length: {len(self.current_path)}",
            f"RCG Nodes: {len(self.rcg_nodes)}",
            f"Frontiers: {len(self.frontier_nodes)}",
            "",
            "Speed Control:",
        ]
        for text in info_texts:
            if text:
                t = self.font_medium.render(text, True, self.colors['black'])
                panel_surface.blit(t, (20, y_offset))
            y_offset += 25
        
        # --- Thanh trượt tốc độ ---
        pygame.draw.rect(panel_surface, self.colors['light_gray'],
                         (self.slider_rect.x - self.window_size, self.slider_rect.y,
                          self.slider_rect.width, self.slider_rect.height))
        pygame.draw.rect(panel_surface, self.colors['blue'],
                         (self.slider_handle.x - self.window_size, self.slider_handle.y,
                          self.slider_handle.width, self.slider_handle.height))
        # Giá trị tốc độ
        speed_text = f"{self.step_size:.3f}s"
        speed_surface = self.font_small.render(speed_text, True, self.colors['black'])
        panel_surface.blit(speed_surface, (20, y_offset + 35))
        y_offset += 70
        
        # --- Các nút Start/Pause/Reset ---
        for btn_data in self.buttons.values():
            btn_color = btn_data['color'] if btn_data['enabled'] else self.colors['gray']
            btn_rect_local = pygame.Rect(
                btn_data['rect'].x - self.window_size, btn_data['rect'].y,
                btn_data['rect'].width, btn_data['rect'].height
            )
            pygame.draw.rect(panel_surface, btn_color, btn_rect_local)
            pygame.draw.rect(panel_surface, self.colors['dark_gray'], btn_rect_local, 2)
            btn_text = self.font_medium.render(btn_data['text'], True,
                                               self.colors['black'] if btn_data['enabled'] else self.colors['gray'])
            text_rect = btn_text.get_rect(center=btn_rect_local.center)
            panel_surface.blit(btn_text, text_rect)
        
        # --- Hiển thị kết quả cuối ---
        if self.results:
            y_offset += 50
            results_title = self.font_medium.render("Results:", True, self.colors['black'])
            panel_surface.blit(results_title, (20, y_offset))
            y_offset += 30
            for text in [
                f"Coverage: {self.results.get('coverage_percentage', 0):.1f}%",
                f"Total Nodes: {self.results.get('nodes_generated', 0)}",
                f"Path Length: {self.results.get('total_path_length', 0)}",
                f"Iterations: {self.results.get('iterations', 0)}"
            ]:
                r_text = self.font_small.render(text, True, self.colors['dark_gray'])
                panel_surface.blit(r_text, (20, y_offset))
                y_offset += 20
        
        # --- Chú giải màu sắc ---
        legend_y = max(y_offset + 20, 450)
        panel_surface.blit(self.font_medium.render("Legend:", True, self.colors['black']), (20, legend_y))
        legend_y += 30
        legend_items = [
            ("• Covered: Light green", self.colors['covered']),
            ("• Path: Pink line", self.colors['path']),
            ("• RCG node: Blue dot", self.colors['rcg_node']),
            ("• Frontier: Red dot", self.colors['frontier']),
            ("• Robot: Green circle", self.colors['green']),
            ("• Sensing area: Yellow circle", self.colors['yellow'])
        ]
        for text, color in legend_items:
            t = self.font_small.render(text, True, color)
            panel_surface.blit(t, (20, legend_y))
            legend_y += 18
        
        # --- Hướng dẫn điều khiển ---
        controls_y = legend_y + 20
        panel_surface.blit(self.font_medium.render("Controls:", True, self.colors['black']), (20, controls_y))
        controls_y += 25
        for text in ["SPACE: Pause/Resume", "S: Start Algorithm", "R: Reset Grid"]:
            c_text = self.font_small.render(text, True, self.colors['dark_gray'])
            panel_surface.blit(c_text, (20, controls_y))
            controls_y += 18
        
        return panel_surface
    
    # -----------------------------------------------------
    def run(self):
        """Vòng lặp chính của chương trình Pygame"""
        self.is_running = True
        while self.is_running:
            # Xử lý các sự kiện
            if not self.handle_events():
                break
            
            # Làm sạch màn hình
            self.screen.fill(self.colors['white'])
            
            # Vẽ lưới (map)
            grid_surface = self.draw_grid()
            self.screen.blit(grid_surface, (0, 0))
            
            # Vẽ bảng thông tin
            panel_surface = self.draw_info_panel()
            self.screen.blit(panel_surface, (self.window_size, 0))
            
            # Cập nhật giao diện
            pygame.display.flip()
            self.clock.tick(60)  # 60 FPS
        
        # Khi thoát khỏi vòng lặp → đóng pygame
        pygame.quit()
        sys.exit()


if __name__ == '__main__':
    # Create and run the pygame visualization
    print("--- C* Algorithm Visualization (Pygame) ---")
    print("Grid Size: 50x50")
    print("Starting Pygame GUI...")
    
    visualizer = CStarPygameVisualizer(grid_size=50, cell_size=12, step_size=0.01)
    visualizer.run()