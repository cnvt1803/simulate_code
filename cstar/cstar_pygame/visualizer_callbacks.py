import time
import pygame
from ..cstar_algorithm.constants import COVERED  # sửa nếu khác tên module

def update_display(app, new_grid, robot_pos, path, iteration):
    # app: instance CStarPygameVisualizer (ở file app)
    if not app.is_running or not app.algorithm_running:
        return
    while app.is_paused and app.is_running and app.algorithm_running:
        time.sleep(0.05)
        pygame.event.pump()
    if not app.is_running or not app.algorithm_running:
        return

    app.grid = [row[:] for row in new_grid]
    app.robot_pos = robot_pos
    app.current_path = path.copy()
    app.step_count = iteration

    if app.step_count % 5 == 0:
        app.covered_count = sum(row.count(COVERED) for row in app.grid)

    if not app.is_paused and app.step_size > 0:
        time.sleep(max(0.001, app.step_size))

def update_rcg_display(app, rcg):
    app.rcg_nodes = {}
    app.rcg_edges = []
    app.frontier_nodes = []
    for node_id, node in rcg.nodes.items():
        app.rcg_nodes[node_id] = {
            'position': node.position,
            'is_frontier': node.is_frontier
        }
        if node.is_frontier:
            app.frontier_nodes.append(node.position)
    for node_id, node in rcg.nodes.items():
        for neighbor_id in node.neighbors:
            if neighbor_id > node_id:
                app.rcg_edges.append((node.position, rcg.nodes[neighbor_id].position))

def update_path_display(app, path):
    app.current_path = path.copy()
