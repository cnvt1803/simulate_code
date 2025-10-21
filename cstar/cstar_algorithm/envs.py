from typing import List
from .constants import FREE_UNCOVERED, OBSTACLE

def create_test_environment(size: int = 30) -> List[List[int]]:
    """Tạo môi trường kiểm thử đơn giản với vài obstacle hình chữ nhật"""
    grid = [[FREE_UNCOVERED for _ in range(size)] for _ in range(size)]
    
    obstacles = [
        (5, 5, 8, 8),
        (15, 20, 18, 25),
        (25, 10, 28, 15),
        (10, 25, 15, 28)
    ]
    
    for start_r, start_c, end_r, end_c in obstacles:
        for r in range(start_r, min(end_r + 1, size)):
            for c in range(start_c, min(end_c + 1, size)):
                if 0 <= r < size and 0 <= c < size:
                    grid[r][c] = OBSTACLE
    return grid
