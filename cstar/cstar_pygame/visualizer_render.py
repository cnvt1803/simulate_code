import pygame

from ..cstar_algorithm.constants import FREE_UNCOVERED, OBSTACLE, COVERED  # sửa nếu khác tên module

def draw_grid(surface_size, cell_size, grid, colors,
              current_path, rcg_edges, rcg_nodes,
              robot_pos, sensing_radius):
    grid_surface = pygame.Surface((surface_size, surface_size))
    grid_surface.fill(colors['white'])

    # cell colors
    cell_colors = {
        FREE_UNCOVERED: colors['white'],
        OBSTACLE: colors['black'],
        COVERED: colors['covered'],
    }

    rows = cols = surface_size // cell_size

    # cells
    for r in range(rows):
        for c in range(cols):
            x = c * cell_size
            y = r * cell_size
            rect = pygame.Rect(x, y, cell_size, cell_size)
            color = cell_colors.get(grid[r][c], colors['white'])
            pygame.draw.rect(grid_surface, color, rect)
            pygame.draw.rect(grid_surface, colors['light_gray'], rect, 1)

    # rcg edges
    for start_pos, end_pos in rcg_edges:
        start_x = start_pos[1] * cell_size + cell_size // 2
        start_y = start_pos[0] * cell_size + cell_size // 2
        end_x = end_pos[1] * cell_size + cell_size // 2
        end_y = end_pos[0] * cell_size + cell_size // 2
        pygame.draw.line(grid_surface, colors['rcg_edge'],
                         (start_x, start_y), (end_x, end_y), 2)

    # path
    if len(current_path) > 1:
        points = []
        for r, c in current_path:
            x = c * cell_size + cell_size // 2
            y = r * cell_size + cell_size // 2
            points.append((x, y))
        pygame.draw.lines(grid_surface, colors['path'], False, points, 3)

    # rcg nodes
    for node_id, node_data in rcg_nodes.items():
        r, c = node_data['position']
        x = c * cell_size + cell_size // 2
        y = r * cell_size + cell_size // 2
        if node_data['is_frontier']:
            pygame.draw.circle(grid_surface, colors['frontier'], (x, y), 4)
            pygame.draw.circle(grid_surface, colors['dark_red'], (x, y), 4, 2)
        else:
            pygame.draw.circle(grid_surface, colors['rcg_node'], (x, y), 3)

    # robot + sensing
    robot_r, robot_c = robot_pos
    robot_x = robot_c * cell_size + cell_size // 2
    robot_y = robot_r * cell_size + cell_size // 2

    sensing_radius_pixels = sensing_radius * cell_size
    sensing_surface = pygame.Surface(
        (sensing_radius_pixels * 2, sensing_radius_pixels * 2), pygame.SRCALPHA
    )
    pygame.draw.circle(
        sensing_surface, colors['sensing_area'],
        (sensing_radius_pixels, sensing_radius_pixels), sensing_radius_pixels
    )
    grid_surface.blit(
        sensing_surface, (robot_x - sensing_radius_pixels, robot_y - sensing_radius_pixels)
    )

    robot_radius = max(3, cell_size // 4)
    pygame.draw.circle(grid_surface, colors['green'], (robot_x, robot_y), robot_radius)
    pygame.draw.circle(grid_surface, colors['dark_green'], (robot_x, robot_y), robot_radius, 2)

    return grid_surface

def draw_info_panel(panel_width, total_height, fonts, colors,
                    step_count, robot_pos, covered_count,
                    current_path_len, rcg_nodes_len, frontier_len,
                    slider_rect, slider_handle, step_size,
                    buttons, results, window_size):
    font_large, font_medium, font_small = fonts
    panel_surface = pygame.Surface((panel_width, total_height))
    panel_surface.fill(colors['panel_bg'])

    y_offset = 20
    # title
    title_text = font_large.render("C* Algorithm", True, colors['black'])
    panel_surface.blit(title_text, (20, y_offset))
    y_offset += 40

    info_texts = [
        f"Iteration: {step_count}",
        f"Position: {robot_pos}",
        f"Covered: {covered_count}",
        f"Path Length: {current_path_len}",
        f"RCG Nodes: {rcg_nodes_len}",
        f"Frontiers: {frontier_len}",
        "",
        "Speed Control:",
    ]
    for text in info_texts:
        if text:
            t = font_medium.render(text, True, colors['black'])
            panel_surface.blit(t, (20, y_offset))
        y_offset += 25

    # slider
    pygame.draw.rect(panel_surface, colors['light_gray'],
                     (slider_rect.x - window_size, slider_rect.y,
                      slider_rect.width, slider_rect.height))
    pygame.draw.rect(panel_surface, colors['blue'],
                     (slider_handle.x - window_size, slider_handle.y,
                      slider_handle.width, slider_handle.height))
    speed_text = f"{step_size:.3f}s"
    speed_surface = font_small.render(speed_text, True, colors['black'])
    panel_surface.blit(speed_surface, (20, y_offset + 35))
    y_offset += 70

    # buttons
    for btn_data in buttons.values():
        btn_color = btn_data['color'] if btn_data['enabled'] else colors['gray']
        btn_rect_local = pygame.Rect(
            btn_data['rect'].x - window_size, btn_data['rect'].y,
            btn_data['rect'].width, btn_data['rect'].height
        )
        pygame.draw.rect(panel_surface, btn_color, btn_rect_local)
        pygame.draw.rect(panel_surface, colors['dark_gray'], btn_rect_local, 2)
        btn_text = font_medium.render(btn_data['text'], True,
                                      colors['black'] if btn_data['enabled'] else colors['gray'])
        text_rect = btn_text.get_rect(center=btn_rect_local.center)
        panel_surface.blit(btn_text, text_rect)

    # results
    if results:
        y_offset += 50
        results_title = font_medium.render("Results:", True, colors['black'])
        panel_surface.blit(results_title, (20, y_offset))
        y_offset += 30
        for text in [
            f"Coverage: {results.get('coverage_percentage', 0):.1f}%",
            f"Total Nodes: {results.get('nodes_generated', 0)}",
            f"Path Length: {results.get('total_path_length', 0)}",
            f"Iterations: {results.get('iterations', 0)}",
        ]:
            r_text = font_small.render(text, True, colors['dark_gray'])
            panel_surface.blit(r_text, (20, y_offset))
            y_offset += 20

    # legend
    legend_y = max(y_offset + 20, 450)
    panel_surface.blit(font_medium.render("Legend:", True, colors['black']), (20, legend_y))
    legend_y += 30
    legend_items = [
        ("• Covered: Light green", colors['covered']),
        ("• Path: Pink line", colors['path']),
        ("• RCG node: Blue dot", colors['rcg_node']),
        ("• Frontier: Red dot", colors['frontier']),
        ("• Robot: Green circle", colors['green']),
        ("• Sensing area: Yellow circle", colors['yellow']),
    ]
    for text, color in legend_items:
        t = font_small.render(text, True, color)
        panel_surface.blit(t, (20, legend_y))
        legend_y += 18

    # controls
    controls_y = legend_y + 20
    panel_surface.blit(font_medium.render("Controls:", True, colors['black']), (20, controls_y))
    controls_y += 25
    for text in ["SPACE: Pause/Resume", "S: Start Algorithm", "R: Reset Grid"]:
        c_text = font_small.render(text, True, colors['dark_gray'])
        panel_surface.blit(c_text, (20, controls_y))
        controls_y += 18

    return panel_surface
