"""
Comparison Visualization for BA* and C* Algorithms
Shows both algorithms running side by side for research comparison
"""

import pygame
import sys
import threading
import time
import math
from ba_star_algorithm import BAStar, FREE_UNCOVERED as BA_FREE, OBSTACLE as BA_OBSTACLE, COVERED as BA_COVERED
from c_star_algorithm import CStar, FREE_UNCOVERED as C_FREE, OBSTACLE as C_OBSTACLE, COVERED as C_COVERED

class AlgorithmComparisonVisualizer:
    def __init__(self, grid_size=40, cell_size=10, step_size=0.02):
        self.grid_size = grid_size
        self.cell_size = cell_size
        self.step_size = step_size
        self.window_size = grid_size * cell_size
        self.panel_width = 300
        self.total_width = self.window_size * 2 + self.panel_width + 40  # Two grids + panel + spacing
        self.total_height = max(self.window_size, 800)
        
        # Game state
        self.is_paused = False
        self.is_running = False
        self.algorithms_running = False
        
        # Algorithm states
        self.ba_star_running = False
        self.c_star_running = False
        self.ba_star_finished = False
        self.c_star_finished = False
        
        # Visualization data for BA*
        self.ba_grid = []
        self.ba_robot_pos = (0, 0)
        self.ba_coverage_paths = []
        self.ba_astar_paths = []
        self.ba_backtrack_points = []
        self.ba_step_count = 0
        self.ba_covered_count = 0
        
        # Visualization data for C*
        self.c_grid = []
        self.c_robot_pos = (0, 0)
        self.c_path = []
        self.c_rcg_nodes = {}
        self.c_rcg_edges = []
        self.c_frontier_nodes = []
        self.c_step_count = 0
        self.c_covered_count = 0
        
        # Results
        self.ba_results = {}
        self.c_results = {}
        
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
            'blue': (0, 0, 255),
            'light_blue': (173, 216, 230),
            'yellow': (255, 255, 0),
            'orange': (255, 165, 0),
            'purple': (128, 0, 128),
            'cyan': (0, 255, 255),
            'pink': (255, 20, 147),
            'panel_bg': (240, 240, 240),
            'button_normal': (200, 200, 200),
            'button_hover': (180, 180, 180),
            'covered': (144, 238, 144),
            'ba_path': (255, 0, 255),      # Magenta for BA* paths
            'c_path': (255, 20, 147),      # Deep pink for C* path
            'rcg_node': (30, 144, 255),    # Dodger blue for RCG nodes
            'rcg_edge': (100, 149, 237),   # Cornflower blue for RCG edges
            'frontier': (255, 69, 0),      # Red orange for frontier nodes
            'backtrack': (255, 0, 0),      # Red for backtracking points
        }
        
        # Initialize pygame
        pygame.init()
        self.screen = pygame.display.set_mode((self.total_width, self.total_height))
        pygame.display.set_caption("BA* vs C* Algorithm Comparison")
        self.clock = pygame.time.Clock()
        
        # Fonts
        self.font_large = pygame.font.Font(None, 28)
        self.font_medium = pygame.font.Font(None, 22)
        self.font_small = pygame.font.Font(None, 18)
        
        # UI elements
        self.buttons = {}
        self.slider_value = 20  # 0-200 range for speed control
        self.slider_rect = pygame.Rect(self.window_size * 2 + 40, 200, 200, 20)
        self.slider_handle = pygame.Rect(self.window_size * 2 + 40, 195, 10, 30)
        self.dragging_slider = False
        
        # Initialize grids
        self.original_grid = self.create_test_grid()
        self.reset_grids()
        
        self.setup_ui_elements()
        
    def create_test_grid(self):
        """Create a test grid with obstacles"""
        grid = [[0 for _ in range(self.grid_size)] for _ in range(self.grid_size)]
        
        # Add obstacles
        obstacles = [
            (5, 5, 8, 8),
            (15, 20, 18, 25),
            (25, 10, 28, 15),
            (10, 25, 15, 28),
            (30, 30, 35, 35)
        ]
        
        for start_r, start_c, end_r, end_c in obstacles:
            for r in range(start_r, min(end_r + 1, self.grid_size)):
                for c in range(start_c, min(end_c + 1, self.grid_size)):
                    if 0 <= r < self.grid_size and 0 <= c < self.grid_size:
                        grid[r][c] = 1  # Obstacle
        
        return grid
    
    def reset_grids(self):
        """Reset both grids to initial state"""
        self.ba_grid = [row[:] for row in self.original_grid]
        self.c_grid = [row[:] for row in self.original_grid]
        self.ba_robot_pos = (0, 0)
        self.c_robot_pos = (0, 0)
        self.ba_coverage_paths = []
        self.ba_astar_paths = []
        self.ba_backtrack_points = []
        self.c_path = []
        self.c_rcg_nodes = {}
        self.c_rcg_edges = []
        self.c_frontier_nodes = []
        self.ba_step_count = 0
        self.c_step_count = 0
        self.ba_covered_count = 0
        self.c_covered_count = 0
        self.ba_results = {}
        self.c_results = {}
        self.ba_star_finished = False
        self.c_star_finished = False
    
    def setup_ui_elements(self):
        """Setup UI buttons and elements"""
        button_width = 140
        button_height = 35
        start_x = self.window_size * 2 + 60
        start_y = 280  # Moved down to avoid covering parameters
        
        # Define buttons
        button_configs = [
            ('start_both', 'Start Both', start_y),
            ('start_ba', 'Start BA* Only', start_y + 45),
            ('start_c', 'Start C* Only', start_y + 90),
            ('pause', 'Pause', start_y + 135),
            ('reset', 'Reset', start_y + 180)
        ]
        
        for btn_id, text, y_pos in button_configs:
            self.buttons[btn_id] = {
                'rect': pygame.Rect(start_x, y_pos, button_width, button_height),
                'text': text,
                'enabled': True if btn_id not in ['pause'] else False,
                'color': self.colors['button_normal']
            }
    
    def handle_events(self):
        """Handle pygame events"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.is_running = False
                return False
            
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:  # Left click
                    self.handle_mouse_click(event.pos)
            
            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1:  # Left click release
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
                    self.reset_comparison()
                elif event.key == pygame.K_1:
                    self.start_both_algorithms()
                elif event.key == pygame.K_2:
                    self.start_ba_star()
                elif event.key == pygame.K_3:
                    self.start_c_star()
        
        return True
    
    def handle_mouse_click(self, pos):
        """Handle mouse clicks on UI elements"""
        # Check button clicks
        for btn_id, btn_data in self.buttons.items():
            if btn_data['rect'].collidepoint(pos) and btn_data['enabled']:
                if btn_id == 'start_both':
                    self.start_both_algorithms()
                elif btn_id == 'start_ba':
                    self.start_ba_star()
                elif btn_id == 'start_c':
                    self.start_c_star()
                elif btn_id == 'pause':
                    self.toggle_pause()
                elif btn_id == 'reset':
                    self.reset_comparison()
        
        # Check slider click
        if self.slider_rect.collidepoint(pos):
            self.dragging_slider = True
            self.update_slider(pos[0])
    
    def handle_mouse_hover(self, pos):
        """Handle mouse hover effects"""
        for btn_data in self.buttons.values():
            if btn_data['rect'].collidepoint(pos) and btn_data['enabled']:
                btn_data['color'] = self.colors['button_hover']
            else:
                btn_data['color'] = self.colors['button_normal']
    
    def update_slider(self, mouse_x):
        """Update slider position and speed value"""
        relative_x = mouse_x - self.slider_rect.x
        relative_x = max(0, min(relative_x, self.slider_rect.width))
        
        self.slider_handle.x = self.slider_rect.x + relative_x - 5
        self.slider_value = int((relative_x / self.slider_rect.width) * 200)
        self.step_size = self.slider_value / 1000.0
    
    def toggle_pause(self):
        """Toggle pause/resume state"""
        if self.algorithms_running:
            self.is_paused = not self.is_paused
            self.buttons['pause']['text'] = 'Resume' if self.is_paused else 'Pause'
    
    def start_both_algorithms(self):
        """Start both algorithms simultaneously"""
        if not self.algorithms_running:
            self.algorithms_running = True
            self.ba_star_running = True
            self.c_star_running = True
            self.update_button_states()
            
            # Start BA* in thread
            ba_thread = threading.Thread(target=self.run_ba_star)
            ba_thread.daemon = True
            ba_thread.start()
            
            # Start C* in thread
            c_thread = threading.Thread(target=self.run_c_star)
            c_thread.daemon = True
            c_thread.start()
    
    def start_ba_star(self):
        """Start only BA* algorithm"""
        if not self.algorithms_running:
            self.algorithms_running = True
            self.ba_star_running = True
            self.update_button_states()
            
            ba_thread = threading.Thread(target=self.run_ba_star)
            ba_thread.daemon = True
            ba_thread.start()
    
    def start_c_star(self):
        """Start only C* algorithm"""
        if not self.algorithms_running:
            self.algorithms_running = True
            self.c_star_running = True
            self.update_button_states()
            
            c_thread = threading.Thread(target=self.run_c_star)
            c_thread.daemon = True
            c_thread.start()
    
    def update_button_states(self):
        """Update button enabled states"""
        running = self.algorithms_running
        self.buttons['start_both']['enabled'] = not running
        self.buttons['start_ba']['enabled'] = not running
        self.buttons['start_c']['enabled'] = not running
        self.buttons['pause']['enabled'] = running
        self.is_paused = False
        self.buttons['pause']['text'] = 'Pause'
    
    def reset_comparison(self):
        """Reset the comparison"""
        if not self.algorithms_running:
            self.reset_grids()
            self.update_button_states()
    
    def run_ba_star(self):
        """Run BA* algorithm"""
        try:
            ba_star = BAStar(self.ba_grid, (0, 0))
            ba_star.set_callbacks(
                step_callback=self.update_ba_display,
                backtrack_callback=self.update_ba_backtrack,
                astar_callback=self.update_ba_astar
            )
            
            final_path, final_grid = ba_star.run()
            
            # Calculate results
            covered_count = sum(row.count(BA_COVERED) for row in final_grid)
            total_free = sum(1 for r in range(self.grid_size) for c in range(self.grid_size) 
                           if self.original_grid[r][c] != 1)
            
            self.ba_results = {
                'path_length': len(final_path),
                'coverage_percentage': (covered_count / total_free) * 100,
                'total_steps': len(final_path) - 1,
                'coverage_paths': len(ba_star.coverage_paths),
                'astar_paths': len(ba_star.astar_paths)
            }
            
        except Exception as e:
            print(f"BA* Error: {e}")
        finally:
            self.ba_star_running = False
            self.ba_star_finished = True
            self.check_completion()
    
    def run_c_star(self):
        """Run C* algorithm"""
        try:
            c_star = CStar(self.c_grid, (0, 0), sensing_radius=3)
            c_star.set_callbacks(
                step_callback=self.update_c_display,
                rcg_update_callback=self.update_c_rcg,
                path_update_callback=self.update_c_path
            )
            
            final_path, results = c_star.run()
            self.c_results = results
            
        except Exception as e:
            print(f"C* Error: {e}")
        finally:
            self.c_star_running = False
            self.c_star_finished = True
            self.check_completion()
    
    def check_completion(self):
        """Check if both algorithms are completed"""
        if not self.ba_star_running and not self.c_star_running:
            self.algorithms_running = False
            self.update_button_states()
    
    def update_ba_display(self, new_grid, robot_pos, coverage_path=None, coverage_id=1):
        """Update BA* display"""
        if not self.is_running or not self.ba_star_running:
            return
        
        while self.is_paused and self.is_running and self.ba_star_running:
            time.sleep(0.05)
            pygame.event.pump()
        
        if not self.is_running or not self.ba_star_running:
            return
        
        self.ba_grid = [row[:] for row in new_grid]
        self.ba_robot_pos = robot_pos
        self.ba_step_count += 1
        
        if coverage_path and coverage_id:
            while len(self.ba_coverage_paths) < coverage_id:
                self.ba_coverage_paths.append([])
            if coverage_id <= len(self.ba_coverage_paths):
                self.ba_coverage_paths[coverage_id - 1] = coverage_path.copy()
        
        if self.ba_step_count % 10 == 0:
            self.ba_covered_count = sum(row.count(BA_COVERED) for row in self.ba_grid)
        
        if not self.is_paused and self.step_size > 0:
            time.sleep(max(0.001, self.step_size))
    
    def update_ba_backtrack(self, point):
        """Update BA* backtracking points"""
        if point not in self.ba_backtrack_points:
            self.ba_backtrack_points.append(point)
    
    def update_ba_astar(self, path):
        """Update BA* A* paths"""
        if path not in self.ba_astar_paths:
            self.ba_astar_paths.append(path.copy())
    
    def update_c_display(self, new_grid, robot_pos, path, iteration):
        """Update C* display"""
        if not self.is_running or not self.c_star_running:
            return
        
        while self.is_paused and self.is_running and self.c_star_running:
            time.sleep(0.05)
            pygame.event.pump()
        
        if not self.is_running or not self.c_star_running:
            return
        
        self.c_grid = [row[:] for row in new_grid]
        self.c_robot_pos = robot_pos
        self.c_path = path.copy()
        self.c_step_count = iteration
        
        if self.c_step_count % 5 == 0:
            self.c_covered_count = sum(row.count(C_COVERED) for row in self.c_grid)
        
        if not self.is_paused and self.step_size > 0:
            time.sleep(max(0.001, self.step_size))
    
    def update_c_rcg(self, rcg):
        """Update C* RCG display"""
        self.c_rcg_nodes = {}
        self.c_rcg_edges = []
        self.c_frontier_nodes = []
        
        for node_id, node in rcg.nodes.items():
            self.c_rcg_nodes[node_id] = {
                'position': node.position,
                'is_frontier': node.is_frontier
            }
            
            if node.is_frontier:
                self.c_frontier_nodes.append(node.position)
        
        for node_id, node in rcg.nodes.items():
            for neighbor_id in node.neighbors:
                if neighbor_id > node_id:
                    self.c_rcg_edges.append((node.position, rcg.nodes[neighbor_id].position))
    
    def update_c_path(self, path):
        """Update C* path display"""
        self.c_path = path.copy()
    
    def draw_ba_grid(self):
        """Draw the BA* grid"""
        grid_surface = pygame.Surface((self.window_size, self.window_size))
        grid_surface.fill(self.colors['white'])
        
        # Draw base grid
        cell_colors = {0: self.colors['white'], 1: self.colors['black'], 2: self.colors['covered']}
        
        for r in range(self.grid_size):
            for c in range(self.grid_size):
                x = c * self.cell_size
                y = r * self.cell_size
                rect = pygame.Rect(x, y, self.cell_size, self.cell_size)
                color = cell_colors.get(self.ba_grid[r][c], self.colors['white'])
                pygame.draw.rect(grid_surface, color, rect)
                pygame.draw.rect(grid_surface, self.colors['light_gray'], rect, 1)
        
        # Draw coverage paths
        path_colors = [self.colors['blue'], self.colors['purple'], self.colors['orange'], 
                      self.colors['cyan'], self.colors['pink']]
        
        for i, coverage_path in enumerate(self.ba_coverage_paths):
            if coverage_path and len(coverage_path) > 1:
                color = path_colors[i % len(path_colors)]
                points = [(c * self.cell_size + self.cell_size // 2,
                          r * self.cell_size + self.cell_size // 2) for r, c in coverage_path]
                if len(points) > 1:
                    pygame.draw.lines(grid_surface, color, False, points, 2)
        
        # Draw A* paths
        for astar_path in self.ba_astar_paths:
            if len(astar_path) > 1:
                points = [(c * self.cell_size + self.cell_size // 2,
                          r * self.cell_size + self.cell_size // 2) for r, c in astar_path]
                if len(points) > 1:
                    pygame.draw.lines(grid_surface, self.colors['black'], False, points, 3)
        
        # Draw backtracking points
        for r, c in self.ba_backtrack_points:
            x = c * self.cell_size + self.cell_size // 2
            y = r * self.cell_size + self.cell_size // 2
            pygame.draw.circle(grid_surface, self.colors['backtrack'], (x, y), 4)
        
        # Draw robot
        robot_r, robot_c = self.ba_robot_pos
        robot_x = robot_c * self.cell_size + self.cell_size // 2
        robot_y = robot_r * self.cell_size + self.cell_size // 2
        pygame.draw.circle(grid_surface, self.colors['green'], (robot_x, robot_y), 4)
        pygame.draw.circle(grid_surface, self.colors['dark_green'], (robot_x, robot_y), 4, 2)
        
        return grid_surface
    
    def draw_c_grid(self):
        """Draw the C* grid"""
        grid_surface = pygame.Surface((self.window_size, self.window_size))
        grid_surface.fill(self.colors['white'])
        
        # Draw base grid
        cell_colors = {0: self.colors['white'], 1: self.colors['black'], 2: self.colors['covered']}
        
        for r in range(self.grid_size):
            for c in range(self.grid_size):
                x = c * self.cell_size
                y = r * self.cell_size
                rect = pygame.Rect(x, y, self.cell_size, self.cell_size)
                color = cell_colors.get(self.c_grid[r][c], self.colors['white'])
                pygame.draw.rect(grid_surface, color, rect)
                pygame.draw.rect(grid_surface, self.colors['light_gray'], rect, 1)
        
        # Draw RCG edges
        for start_pos, end_pos in self.c_rcg_edges:
            start_x = start_pos[1] * self.cell_size + self.cell_size // 2
            start_y = start_pos[0] * self.cell_size + self.cell_size // 2
            end_x = end_pos[1] * self.cell_size + self.cell_size // 2
            end_y = end_pos[0] * self.cell_size + self.cell_size // 2
            pygame.draw.line(grid_surface, self.colors['rcg_edge'], 
                           (start_x, start_y), (end_x, end_y), 1)
        
        # Draw path
        if len(self.c_path) > 1:
            points = [(c * self.cell_size + self.cell_size // 2,
                      r * self.cell_size + self.cell_size // 2) for r, c in self.c_path]
            if len(points) > 1:
                pygame.draw.lines(grid_surface, self.colors['c_path'], False, points, 2)
        
        # Draw RCG nodes
        for node_id, node_data in self.c_rcg_nodes.items():
            r, c = node_data['position']
            x = c * self.cell_size + self.cell_size // 2
            y = r * self.cell_size + self.cell_size // 2
            
            if node_data['is_frontier']:
                pygame.draw.circle(grid_surface, self.colors['frontier'], (x, y), 3)
            else:
                pygame.draw.circle(grid_surface, self.colors['rcg_node'], (x, y), 2)
        
        # Draw robot
        robot_r, robot_c = self.c_robot_pos
        robot_x = robot_c * self.cell_size + self.cell_size // 2
        robot_y = robot_r * self.cell_size + self.cell_size // 2
        pygame.draw.circle(grid_surface, self.colors['green'], (robot_x, robot_y), 4)
        pygame.draw.circle(grid_surface, self.colors['dark_green'], (robot_x, robot_y), 4, 2)
        
        return grid_surface
    
    def draw_info_panel(self):
        """Draw the information panel"""
        panel_surface = pygame.Surface((self.panel_width, self.total_height))
        panel_surface.fill(self.colors['panel_bg'])
        
        y_offset = 20
        
        # Title
        title_text = self.font_large.render("Algorithm Comparison", True, self.colors['black'])
        panel_surface.blit(title_text, (20, y_offset))
        y_offset += 40
        
        # BA* Info
        ba_title = self.font_medium.render("BA* Algorithm:", True, self.colors['blue'])
        panel_surface.blit(ba_title, (20, y_offset))
        y_offset += 25
        
        ba_info = [
            f"Steps: {self.ba_step_count}",
            f"Covered: {self.ba_covered_count}",
            f"Paths: {len(self.ba_coverage_paths)}",
            f"A* Calls: {len(self.ba_astar_paths)}",
            f"Status: {'Running' if self.ba_star_running else 'Finished' if self.ba_star_finished else 'Ready'}"
        ]
        
        for text in ba_info:
            text_surface = self.font_small.render(text, True, self.colors['dark_gray'])
            panel_surface.blit(text_surface, (30, y_offset))
            y_offset += 18
        
        y_offset += 10
        
        # C* Info
        c_title = self.font_medium.render("C* Algorithm:", True, self.colors['red'])
        panel_surface.blit(c_title, (20, y_offset))
        y_offset += 25
        
        c_info = [
            f"Iterations: {self.c_step_count}",
            f"Covered: {self.c_covered_count}",
            f"Path Length: {len(self.c_path)}",
            f"RCG Nodes: {len(self.c_rcg_nodes)}",
            f"Frontiers: {len(self.c_frontier_nodes)}",
            f"Status: {'Running' if self.c_star_running else 'Finished' if self.c_star_finished else 'Ready'}"
        ]
        
        for text in c_info:
            text_surface = self.font_small.render(text, True, self.colors['dark_gray'])
            panel_surface.blit(text_surface, (30, y_offset))
            y_offset += 18
        
        y_offset += 20
        
        # Speed control
        speed_text = self.font_medium.render("Speed Control:", True, self.colors['black'])
        panel_surface.blit(speed_text, (20, y_offset))
        y_offset += 30
        
        # Draw slider
        slider_x = self.slider_rect.x - (self.window_size * 2 + 40)
        pygame.draw.rect(panel_surface, self.colors['light_gray'], 
                        (slider_x, self.slider_rect.y, self.slider_rect.width, self.slider_rect.height))
        handle_x = self.slider_handle.x - (self.window_size * 2 + 40)
        pygame.draw.rect(panel_surface, self.colors['blue'], 
                        (handle_x, self.slider_handle.y, self.slider_handle.width, self.slider_handle.height))
        
        speed_value = f"{self.step_size:.3f}s"
        speed_surface = self.font_small.render(speed_value, True, self.colors['black'])
        panel_surface.blit(speed_surface, (20, y_offset + 35))
        y_offset += 70
        
        # Draw buttons
        for btn_data in self.buttons.values():
            btn_color = btn_data['color'] if btn_data['enabled'] else self.colors['gray']
            btn_rect_local = pygame.Rect(btn_data['rect'].x - (self.window_size * 2 + 40), btn_data['rect'].y,
                                       btn_data['rect'].width, btn_data['rect'].height)
            
            pygame.draw.rect(panel_surface, btn_color, btn_rect_local)
            pygame.draw.rect(panel_surface, self.colors['dark_gray'], btn_rect_local, 2)
            
            # Button text
            text_color = self.colors['black'] if btn_data['enabled'] else self.colors['gray']
            btn_text = self.font_small.render(btn_data['text'], True, text_color)
            text_rect = btn_text.get_rect(center=btn_rect_local.center)
            panel_surface.blit(btn_text, text_rect)
        
        # Results comparison (if both finished)
        if self.ba_star_finished and self.c_star_finished:
            y_offset += 50
            results_title = self.font_medium.render("Final Comparison:", True, self.colors['black'])
            panel_surface.blit(results_title, (20, y_offset))
            y_offset += 30
            
            if self.ba_results and self.c_results:
                comparison_items = [
                    f"BA* Coverage: {self.ba_results.get('coverage_percentage', 0):.1f}%",
                    f"C* Coverage: {self.c_results.get('coverage_percentage', 0):.1f}%",
                    "",
                    f"BA* Path: {self.ba_results.get('path_length', 0)}",
                    f"C* Path: {self.c_results.get('total_path_length', 0)}",
                    "",
                    f"BA* Coverage Paths: {self.ba_results.get('coverage_paths', 0)}",
                    f"C* RCG Nodes: {self.c_results.get('nodes_generated', 0)}"
                ]
                
                for text in comparison_items:
                    if text:
                        comp_text = self.font_small.render(text, True, self.colors['dark_gray'])
                        panel_surface.blit(comp_text, (20, y_offset))
                    y_offset += 16
        
        return panel_surface
    
    def run(self):
        """Main game loop"""
        self.is_running = True
        
        while self.is_running:
            # Handle events
            if not self.handle_events():
                break
            
            # Draw everything
            self.screen.fill(self.colors['white'])
            
            # Draw BA* grid (left side)
            ba_surface = self.draw_ba_grid()
            self.screen.blit(ba_surface, (0, 0))
            
            # Draw C* grid (right side)
            c_surface = self.draw_c_grid()
            self.screen.blit(c_surface, (self.window_size + 20, 0))
            
            # Draw labels
            ba_label = self.font_large.render("BA* Algorithm", True, self.colors['blue'])
            self.screen.blit(ba_label, (10, self.window_size + 10))
            
            c_label = self.font_large.render("C* Algorithm", True, self.colors['red'])
            self.screen.blit(c_label, (self.window_size + 30, self.window_size + 10))
            
            # Draw info panel
            panel_surface = self.draw_info_panel()
            self.screen.blit(panel_surface, (self.window_size * 2 + 40, 0))
            
            # Update display
            pygame.display.flip()
            self.clock.tick(60)  # 60 FPS
        
        pygame.quit()
        sys.exit()

if __name__ == '__main__':
    print("--- BA* vs C* Algorithm Comparison ---")
    print("Controls:")
    print("  1: Start both algorithms")
    print("  2: Start BA* only")
    print("  3: Start C* only")
    print("  SPACE: Pause/Resume")
    print("  R: Reset")
    
    visualizer = AlgorithmComparisonVisualizer(grid_size=40, cell_size=15, step_size=0.02)
    visualizer.run()