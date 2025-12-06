"""
BA* Algorithm Visualization using Pygame
Provides better performance and smoother rendering compared to tkinter
"""

import pygame
import sys
import threading
import time
import random
from ba_star_algorithm import BAStar, FREE_UNCOVERED, OBSTACLE, COVERED


class PygameVisualizer:
    def __init__(self, grid_size=50, cell_size=12, step_size=0.001):
        self.grid_size = grid_size
        self.cell_size = cell_size
        self.step_size = step_size
        self.window_size = grid_size * cell_size
        self.panel_width = 250
        self.total_width = self.window_size + self.panel_width
        self.total_height = max(self.window_size, 600)

        # Game state
        self.is_paused = False
        self.is_running = False
        self.algorithm_running = False

        # Path tracking for visualization
        self.coverage_paths = []  # List of coverage paths with different colors
        # Only show selected backtracking points
        self.selected_backtracking_points = []
        self.current_astar_path = []
        self.path_colors = [
            (0, 100, 255),      # Blue
            (128, 0, 128),      # Purple
            (255, 165, 0),      # Orange
            (139, 69, 19),      # Brown
            (255, 20, 147),     # Pink
            (0, 255, 255),      # Cyan
            (255, 0, 255),      # Magenta
            (255, 255, 0)       # Yellow
        ]

        # Initialize pygame
        pygame.init()
        self.screen = pygame.display.set_mode(
            (self.total_width, self.total_height))
        pygame.display.set_caption("BA* Algorithm Visualization - Pygame")
        self.clock = pygame.time.Clock()

        # Fonts
        self.font_large = pygame.font.Font(None, 24)
        self.font_medium = pygame.font.Font(None, 20)
        self.font_small = pygame.font.Font(None, 16)

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
            'panel_bg': (240, 240, 240),
            'button_normal': (200, 200, 200),
            'button_hover': (180, 180, 180),
            'button_pressed': (160, 160, 160)
        }

        # Initialize grid
        self.grid = [[0 for _ in range(grid_size)] for _ in range(grid_size)]
        self.robot_pos = (0, 0)
        self.step_count = 0
        self.covered_count = 0

        # UI elements
        self.buttons = {}
        self.slider_value = 1  # 0-200 range for speed control
        self.slider_rect = pygame.Rect(self.window_size + 20, 200, 200, 20)
        self.slider_handle = pygame.Rect(self.window_size + 20, 195, 10, 30)
        self.dragging_slider = False

        self.setup_ui_elements()
        # self.add_sample_obstacles()
        self.set_fixed_map_layout()

    def setup_ui_elements(self):
        """Setup UI buttons and elements"""
        button_width = 120
        button_height = 30
        start_x = self.window_size + 20
        start_y = 220  # Moved down to avoid covering parameters

        # Define buttons
        button_configs = [
            ('start', 'Start BA*', start_y),
            ('pause', 'Pause', start_y + 40),
            ('reset', 'Reset', start_y + 80)
        ]

        for btn_id, text, y_pos in button_configs:
            self.buttons[btn_id] = {
                'rect': pygame.Rect(start_x, y_pos, button_width, button_height),
                'text': text,
                'enabled': True if btn_id != 'pause' else False,
                'color': self.colors['button_normal']
            }

    def add_sample_obstacles(self):
        """Add randomly generated obstacles to demonstrate the algorithm"""
        # Generate random obstacles
        # Random number of obstacles (5-8)
        num_obstacles = random.randint(5, 8)

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

    def set_fixed_map_layout(self):
        MAP_20 = [
            "....................",
            "....................",
            "....................",
            "....................",
            "....................",
            ".........####.......",
            ".........####.......",
            ".........#####......",
            ".......#######......",
            ".......##...........",
            ".....####.#.........",
            ".....#######........",
            ".......########.....",
            ".........######.....",
            ".........######.....",
            "....................",
            "....................",
            "....................",
            "....................",
            "....................",
        ]
        for r, row in enumerate(MAP_20):
            for c, ch in enumerate(row):
                self.grid[r][c] = OBSTACLE if ch == "#" else FREE_UNCOVERED

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
                    self.reset_grid()
                elif event.key == pygame.K_s and not self.algorithm_running:
                    self.start_algorithm()

        return True

    def handle_mouse_click(self, pos):
        """Handle mouse clicks on UI elements"""
        # Check button clicks
        for btn_id, btn_data in self.buttons.items():
            if btn_data['rect'].collidepoint(pos) and btn_data['enabled']:
                if btn_id == 'start':
                    self.start_algorithm()
                elif btn_id == 'pause':
                    self.toggle_pause()
                elif btn_id == 'reset':
                    self.reset_grid()

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
        # Constrain to slider bounds
        relative_x = mouse_x - self.slider_rect.x
        relative_x = max(0, min(relative_x, self.slider_rect.width))

        # Update slider handle position
        self.slider_handle.x = self.slider_rect.x + relative_x - 5

        # Calculate slider value (0-200)
        self.slider_value = int((relative_x / self.slider_rect.width) * 200)
        self.step_size = self.slider_value / 1000.0

    def toggle_pause(self):
        """Toggle pause/resume state"""
        if self.algorithm_running:
            self.is_paused = not self.is_paused
            self.buttons['pause']['text'] = 'Resume' if self.is_paused else 'Pause'

    def start_algorithm(self):
        """Start the BA* algorithm in a separate thread"""
        if not self.algorithm_running:
            self.algorithm_running = True
            self.buttons['start']['enabled'] = False
            self.buttons['pause']['enabled'] = True
            self.is_paused = False
            self.buttons['pause']['text'] = 'Pause'

            def run_ba_star():
                ba_star_robot = BAStar(self.grid, self.robot_pos)

                # Set up callbacks for visualization
                ba_star_robot.set_callbacks(
                    step_callback=self.update_display,
                    backtrack_callback=self.show_selected_backtracking_point,
                    astar_callback=self.show_astar_path
                )

                final_path, final_grid = ba_star_robot.run()

                # Algorithm completed
                self.algorithm_running = False
                self.buttons['start']['enabled'] = True
                self.buttons['pause']['enabled'] = False
                self.buttons['pause']['text'] = 'Pause'

            thread = threading.Thread(target=run_ba_star)
            thread.daemon = True
            thread.start()

    def reset_grid(self):
        """Reset the grid to initial state"""
        if not self.algorithm_running:
            self.grid = [[0 for _ in range(self.grid_size)]
                         for _ in range(self.grid_size)]
            self.robot_pos = (0, 0)
            self.step_count = 0
            self.covered_count = 0
            self.coverage_paths = []
            self.selected_backtracking_points = []
            self.current_astar_path = []
            self.is_paused = False
            # self.add_sample_obstacles()
            self.set_fixed_map_layout()
            # Reset buttons
            self.buttons['start']['enabled'] = True
            self.buttons['pause']['enabled'] = False
            self.buttons['pause']['text'] = 'Pause'

    def update_display(self, new_grid, robot_pos, coverage_path=None, coverage_id=1):
        """Update the display with new grid state and coverage path"""
        # Quick check if we should continue
        if not self.is_running or not self.algorithm_running:
            return

        # Wait while paused
        while self.is_paused and self.is_running and self.algorithm_running:
            time.sleep(0.05)
            # Process events even while paused to maintain responsiveness
            pygame.event.pump()

        if not self.is_running or not self.algorithm_running:
            return

        self.grid = [row[:] for row in new_grid]
        self.robot_pos = robot_pos
        self.step_count += 1

        # Update coverage path if provided
        if coverage_path is not None:
            # Ensure we have enough coverage paths stored
            while len(self.coverage_paths) < coverage_id:
                self.coverage_paths.append([])

            # Update the specific coverage path
            if coverage_id <= len(self.coverage_paths):
                self.coverage_paths[coverage_id - 1] = coverage_path.copy()

        # Count covered cells (only when needed)
        if self.step_count % 10 == 0:
            self.covered_count = sum(row.count(COVERED) for row in self.grid)

        # Apply speed delay only if not paused
        if not self.is_paused and self.step_size > 0:
            time.sleep(max(0.001, self.step_size))

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
        cell_colors = {
            FREE_UNCOVERED: self.colors['white'],
            OBSTACLE: self.colors['black'],
            COVERED: self.colors['white']
        }

        # Draw base grid cells
        for r in range(self.grid_size):
            for c in range(self.grid_size):
                x = c * self.cell_size
                y = r * self.cell_size
                rect = pygame.Rect(x, y, self.cell_size, self.cell_size)

                color = cell_colors.get(self.grid[r][c], self.colors['white'])
                pygame.draw.rect(grid_surface, color, rect)
                pygame.draw.rect(
                    grid_surface, self.colors['light_gray'], rect, 1)

        # Draw coverage paths with different colors
        for path_idx, coverage_path in enumerate(self.coverage_paths):
            if coverage_path and len(coverage_path) > 1:
                path_color = self.path_colors[path_idx % len(self.path_colors)]
                points = []
                for r, c in coverage_path:
                    x = c * self.cell_size + self.cell_size // 2
                    y = r * self.cell_size + self.cell_size // 2
                    points.append((x, y))

                if len(points) > 1:
                    pygame.draw.lines(
                        grid_surface, path_color, False, points, 3)

        # Draw A* paths in black
        for astar_path in self.current_astar_path:
            if len(astar_path) > 1:
                points = []
                for r, c in astar_path:
                    x = c * self.cell_size + self.cell_size // 2
                    y = r * self.cell_size + self.cell_size // 2
                    points.append((x, y))

                if len(points) > 1:
                    pygame.draw.lines(
                        grid_surface, self.colors['black'], False, points, 4)

        # Draw selected backtracking points as red diamonds
        for r, c in self.selected_backtracking_points:
            cx = c * self.cell_size + self.cell_size // 2
            cy = r * self.cell_size + self.cell_size // 2
            size = self.cell_size // 3

            # Create diamond shape points
            points = [
                (cx, cy - size),      # top
                (cx + size, cy),      # right
                (cx, cy + size),      # bottom
                (cx - size, cy)       # left
            ]
            pygame.draw.polygon(grid_surface, self.colors['red'], points)
            pygame.draw.polygon(
                grid_surface, self.colors['dark_red'], points, 2)

        # Draw robot position as green circle
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
        car_rect = pygame.Rect(x - car_width//2, y -
                               car_height//2, car_width, car_height)
        pygame.draw.rect(surface, self.colors['green'], car_rect)
        pygame.draw.rect(surface, self.colors['dark_green'], car_rect, 1)

        # Car windshield (smaller rectangle at front)
        windshield_width = car_width // 3
        windshield_height = car_height // 2
        windshield_rect = pygame.Rect(x + car_width//2 - windshield_width,
                                      y - windshield_height//2,
                                      windshield_width, windshield_height)
        # Light green windshield
        pygame.draw.rect(surface, (200, 255, 200), windshield_rect)
        pygame.draw.rect(
            surface, self.colors['dark_green'], windshield_rect, 1)

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
        title_text = self.font_large.render(
            "BA* Algorithm", True, self.colors['black'])
        panel_surface.blit(title_text, (20, y_offset))
        y_offset += 40

        # Information display
        info_texts = [
            f"Step: {self.step_count}",
            f"Position: {self.robot_pos}",
            f"Covered: {self.covered_count}",
            "",
            "Speed Control:",
        ]

        for text in info_texts:
            if text:
                text_surface = self.font_medium.render(
                    text, True, self.colors['black'])
                panel_surface.blit(text_surface, (20, y_offset))
            y_offset += 25

        # Speed slider
        pygame.draw.rect(panel_surface, self.colors['light_gray'],
                         (self.slider_rect.x - self.window_size, self.slider_rect.y,
                         self.slider_rect.width, self.slider_rect.height))
        pygame.draw.rect(panel_surface, self.colors['blue'],
                         (self.slider_handle.x - self.window_size, self.slider_handle.y,
                         self.slider_handle.width, self.slider_handle.height))

        # Speed value
        speed_text = f"{self.step_size:.3f}s"
        speed_surface = self.font_small.render(
            speed_text, True, self.colors['black'])
        panel_surface.blit(speed_surface, (20, y_offset + 35))

        # Draw buttons
        for btn_data in self.buttons.values():
            btn_color = btn_data['color'] if btn_data['enabled'] else self.colors['gray']
            btn_rect_local = pygame.Rect(btn_data['rect'].x - self.window_size, btn_data['rect'].y,
                                         btn_data['rect'].width, btn_data['rect'].height)

            pygame.draw.rect(panel_surface, btn_color, btn_rect_local)
            pygame.draw.rect(
                panel_surface, self.colors['dark_gray'], btn_rect_local, 2)

            # Button text
            text_color = self.colors['black'] if btn_data['enabled'] else self.colors['gray']
            btn_text = self.font_medium.render(
                btn_data['text'], True, text_color)
            text_rect = btn_text.get_rect(center=btn_rect_local.center)
            panel_surface.blit(btn_text, text_rect)

        # Legend
        legend_y = 350
        legend_title = self.font_medium.render(
            "Legend:", True, self.colors['black'])
        panel_surface.blit(legend_title, (20, legend_y))
        legend_y += 30

        legend_items = [
            ("• Coverage paths: Different colors", self.colors['blue']),
            ("• A* paths: Black lines", self.colors['black']),
            ("• Backtrack points: Red diamonds", self.colors['red']),
            ("• Robot: Green circle", self.colors['green']),
        ]

        for text, color in legend_items:
            legend_text = self.font_small.render(text, True, color)
            panel_surface.blit(legend_text, (20, legend_y))
            legend_y += 20

        # Controls
        controls_y = legend_y + 20
        controls_title = self.font_medium.render(
            "Controls:", True, self.colors['black'])
        panel_surface.blit(controls_title, (20, controls_y))
        controls_y += 30

        control_items = [
            "SPACE: Pause/Resume",
            "S: Start Algorithm",
            "R: Reset Grid",
        ]

        for text in control_items:
            control_text = self.font_small.render(
                text, True, self.colors['dark_gray'])
            panel_surface.blit(control_text, (20, controls_y))
            controls_y += 20

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

            # Draw grid
            grid_surface = self.draw_grid()
            self.screen.blit(grid_surface, (0, 0))

            # Draw info panel
            panel_surface = self.draw_info_panel()
            self.screen.blit(panel_surface, (self.window_size, 0))

            # Update display
            pygame.display.flip()
            self.clock.tick(60)  # 60 FPS

        pygame.quit()
        sys.exit()


# Backward compatibility alias
GridVisualizer = PygameVisualizer

if __name__ == '__main__':
    # Create and run the pygame visualization
    print("--- BA* Algorithm Visualization (Pygame) ---")
    print("Grid Size: 20x20")
    print("Starting Pygame GUI...")

    visualizer = PygameVisualizer(grid_size=20, cell_size=25, step_size=0.01)
    visualizer.run()
