"""
App chính: vòng lặp, xử lý event, gọi renderer & callbacks
"""

import pygame
import sys
import threading
from ..cstar_algorithm.cstar import CStar
from ..cstar_algorithm.constants import FREE_UNCOVERED, OBSTACLE, COVERED, FRONTIER


from .visualizer_ui import UIState, setup_ui_elements, handle_mouse_hover, update_slider
from .visualizer_render import draw_grid, draw_info_panel
from .visualizer_callbacks import update_display, update_rcg_display, update_path_display

class CStarPygameVisualizer:
    def __init__(self, grid_size=50, cell_size=12, step_size=0.01):
        # ---------- cấu hình ----------
        self.grid_size = grid_size
        self.cell_size = cell_size
        self.step_size = step_size
        self.window_size = grid_size * cell_size
        self.panel_width = 300
        self.total_width = self.window_size + self.panel_width
        self.total_height = max(self.window_size, 700)

        # sensing radius (sửa cho hợp lý)
        self.sensing_radius = 6
        self.sensing_radius_min = 1
        self.sensing_radius_max = 15

        # ---------- trạng thái thuật toán ----------
        self.is_paused = False
        self.is_running = False
        self.algorithm_running = False

        # ---------- dữ liệu hiển thị ----------
        self.current_path = []
        self.rcg_nodes = {}
        self.rcg_edges = []
        self.frontier_nodes = []
        self.covered_areas = []

        # ---------- màu ----------
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
            'covered': (144, 238, 144),
            'path': (255, 20, 147),
            'rcg_node': (30, 144, 255),
            'rcg_edge': (100, 149, 237),
            'frontier': (255, 69, 0),
            'sensing_area': (255, 255, 0, 50),
        }

        # ---------- pygame ----------
        pygame.init()
        self.screen = pygame.display.set_mode((self.total_width, self.total_height))
        pygame.display.set_caption("C* Algorithm Visualization - Pygame")
        self.clock = pygame.time.Clock()

        # fonts
        self.font_large = pygame.font.Font(None, 24)
        self.font_medium = pygame.font.Font(None, 20)
        self.font_small = pygame.font.Font(None, 16)

        # grid & robot
        self.grid = [[0 for _ in range(grid_size)] for _ in range(grid_size)]
        self.robot_pos = (0, 0)
        self.step_count = 0
        self.covered_count = 0

        # UI state
        self.ui = UIState(self.window_size, self.panel_width)
        setup_ui_elements(self.ui, self.colors)

        # results
        self.results = {}

        # obstacles mẫu
        self.add_sample_obstacles()

    # ---------- obstacles ----------
    def add_sample_obstacles(self):
        obstacles = [
            (8, 8, 12, 12),
            (20, 25, 25, 30),
            (35, 15, 40, 20),
            (15, 35, 20, 40),
            (30, 30, 35, 35),
        ]
        for start_r, start_c, end_r, end_c in obstacles:
            for r in range(start_r, end_r + 1):
                for c in range(start_c, end_c + 1):
                    if 0 <= r < self.grid_size and 0 <= c < self.grid_size:
                        self.grid[r][c] = OBSTACLE

    # ---------- events ----------
    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.is_running = False
                return False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:
                    self.handle_mouse_click(event.pos)
            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1:
                    self.ui.dragging_slider = False
            elif event.type == pygame.MOUSEMOTION:
                if self.ui.dragging_slider:
                    self.step_size = update_slider(self.ui, event.pos[0])
                else:
                    handle_mouse_hover(self.ui, self.colors, event.pos)
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    self.toggle_pause()
                elif event.key == pygame.K_r:
                    self.reset_grid()
                elif event.key == pygame.K_s and not self.algorithm_running:
                    self.start_algorithm()
        return True

    def handle_mouse_click(self, pos):
        for btn_id, btn_data in self.ui.buttons.items():
            if btn_data['rect'].collidepoint(pos) and btn_data['enabled']:
                if btn_id == 'start':
                    self.start_algorithm()
                elif btn_id == 'pause':
                    self.toggle_pause()
                elif btn_id == 'reset':
                    self.reset_grid()
        if self.ui.slider_rect.collidepoint(pos):
            self.ui.dragging_slider = True
            self.step_size = update_slider(self.ui, pos[0])

    def toggle_pause(self):
        if self.algorithm_running:
            self.is_paused = not self.is_paused
            self.ui.buttons['pause']['text'] = 'Resume' if self.is_paused else 'Pause'

    def reset_grid(self):
        n = self.grid_size
        self.grid = [[0 for _ in range(n)] for _ in range(n)]
        self.robot_pos = (0, 0)
        self.current_path.clear()
        self.rcg_nodes.clear()
        self.rcg_edges.clear()
        self.frontier_nodes.clear()
        self.step_count = 0
        self.covered_count = 0
        self.results.clear()
        self.algorithm_running = False
        self.ui.buttons['start']['enabled'] = True
        self.ui.buttons['pause']['enabled'] = False
        self.ui.buttons['pause']['text'] = 'Pause'

    # ---------- algorithm ----------
    def start_algorithm(self):
        if self.algorithm_running:
            return
        self.algorithm_running = True
        self.ui.buttons['start']['enabled'] = False
        self.ui.buttons['pause']['enabled'] = True
        self.is_paused = False
        self.ui.buttons['pause']['text'] = 'Pause'

        def run_c_star():
            c_star = CStar(self.grid, self.robot_pos, sensing_radius=self.sensing_radius)
            c_star.set_callbacks(
                step_callback=lambda *args, **kwargs: update_display(self, *args, **kwargs),
                rcg_update_callback=lambda *args, **kwargs: update_rcg_display(self, *args, **kwargs),
                path_update_callback=lambda *args, **kwargs: update_path_display(self, *args, **kwargs),
            )
            _, results = c_star.run()
            self.results = results
            self.algorithm_running = False
            self.ui.buttons['start']['enabled'] = True
            self.ui.buttons['pause']['enabled'] = False
            self.ui.buttons['pause']['text'] = 'Pause'

        thread = threading.Thread(target=run_c_star, daemon=True)
        thread.start()

    # ---------- draw ----------
    def draw_grid(self):
        return draw_grid(
            self.window_size, self.cell_size, self.grid, self.colors,
            self.current_path, self.rcg_edges, self.rcg_nodes,
            self.robot_pos, self.sensing_radius
        )

    def draw_info_panel(self):
        return draw_info_panel(
            self.panel_width, self.total_height,
            (self.font_large, self.font_medium, self.font_small), self.colors,
            self.step_count, self.robot_pos, self.covered_count,
            len(self.current_path), len(self.rcg_nodes), len(self.frontier_nodes),
            self.ui.slider_rect, self.ui.slider_handle, self.step_size,
            self.ui.buttons, self.results, self.window_size
        )

    # ---------- main loop ----------
    def run(self):
        self.is_running = True
        while self.is_running:
            if not self.handle_events():
                break
            self.screen.fill(self.colors['white'])

            grid_surface = self.draw_grid()
            self.screen.blit(grid_surface, (0, 0))

            panel_surface = self.draw_info_panel()
            self.screen.blit(panel_surface, (self.window_size, 0))

            pygame.display.flip()
            self.clock.tick(60)

        pygame.quit()
        sys.exit()

if __name__ == '__main__':
    print("--- C* Algorithm Visualization (Pygame) ---")
    print("Grid Size: 50x50")
    print("Starting Pygame GUI...")
    visualizer = CStarPygameVisualizer(grid_size=50, cell_size=12, step_size=0.01)
    visualizer.run()
