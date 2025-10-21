# BA* Algorithm - Main Entry Point
# This file imports and runs the visualization

from ba_star_visualization import GridVisualizer

if __name__ == '__main__':
    # Create and run the 50x50 grid visualization
    print("--- BA* Algorithm Visualization ---")
    print("Grid Size: 50x50")
    print("Starting GUI...")
    
    # Launch the visualization window
    visualizer = GridVisualizer(grid_size=50, cell_size=12, step_size=0.001)
    visualizer.run()