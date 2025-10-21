# BA* Algorithm - Pygame Version Entry Point
# This file imports and runs the pygame visualization

from ba_star_pygame_visualization import PygameVisualizer

if __name__ == '__main__':
    # Create and run the 50x50 grid visualization with pygame
    print("--- BA* Algorithm Visualization (Pygame) ---")
    print("Grid Size: 50x50")
    print("Enhanced performance with Pygame")
    print("Starting GUI...")
    
    # Launch the pygame visualization window
    visualizer = PygameVisualizer(grid_size=50, cell_size=12, step_size=0.001)
    visualizer.run()