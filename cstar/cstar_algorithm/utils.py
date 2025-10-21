from typing import Tuple

def snap_to_grid(p: Tuple[int, int], grid_step: int) -> Tuple[int, int]:
    """Làm tròn vị trí về lưới sampling step"""
    r, c = p
    return (max(0, r // grid_step * grid_step),
            max(0, c // grid_step * grid_step))
