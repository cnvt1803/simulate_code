"""
Main Runner for Coverage Path Planning Algorithm Visualizations
Provides menu-driven access to BA*, C*, and comparison visualizations
"""

import sys
import os

def print_banner():
    """Print application banner"""
    print("=" * 70)
    print("    COVERAGE PATH PLANNING ALGORITHMS - RESEARCH VISUALIZATION")
    print("=" * 70)
    print()

def print_menu():
    """Print the main menu"""
    print("Available Visualizations:")
    print()
    print("1. BA* Algorithm (Pygame1)")
    print("   - Boustrophedon motion with A* pathfinding")
    print("   - Full coverage path planning")
    print("   - Path smoothing with A*SPT")
    print()
    print("2. BA* Algorithm (Pygame2)")
    print("   - Alternative visualization with step-by-step execution")
    print("   - Detailed algorithm inspection")
    print()
    print("3. C* Algorithm (Pygame)")
    print("   - Rapidly Covering Graph construction")
    print("   - Multi-objective frontier selection")
    print("   - RRT-like sampling approach")
    print()
    print("4. Algorithm Comparison (Side-by-side)")
    print("   - BA* vs C* running simultaneously")
    print("   - Performance metrics comparison")
    print("   - Interactive speed control")
    print()
    print("5. Research Demos")
    print("   - Pre-configured scenarios for research")
    print("   - Different grid configurations")
    print()
    print("0. Exit")
    print()

def run_ba_star_pygame_1():
    """Run BA* with Pygame visualization"""
    print("Starting BA* Algorithm with Pygame visualization...")
    try:
        from ba_star_pygame_visualization import PygameVisualizer
        visualizer = PygameVisualizer()
        visualizer.run()
    except ImportError as e:
        print(f"Error importing BA* Pygame visualization: {e}")
        print("Make sure ba_star_pygame_visualization.py is available")
    except Exception as e:
        print(f"Error running BA* Pygame: {e}")

def run_ba_star_pygame_2():
    """Run BA* with Pygame visualization"""
    print("Starting BA* Algorithm with Pygame visualization...")
    try:
        from ba_star_visualization import GridVisualizer
        visualizer = GridVisualizer(grid_size=50, cell_size=12, step_size=0.001)
        visualizer.run()
    except ImportError as e:
        print(f"Error importing BA* Tkinter visualization: {e}")
        print("Make sure ba_star_visualization.py is available")
    except Exception as e:
        print(f"Error running BA* Tkinter: {e}")

def run_c_star_pygame():
    """Run C* with Pygame visualization"""
    print("Starting C* Algorithm with Pygame visualization...")
    try:
        from c_star_pygame_visualization import CStarPygameVisualizer
        visualizer = CStarPygameVisualizer()
        visualizer.run()
    except ImportError as e:
        print(f"Error importing C* Pygame visualization: {e}")
        print("Make sure c_star_pygame_visualization.py is available")
    except Exception as e:
        print(f"Error running C* Pygame: {e}")

def run_algorithm_comparison():
    """Run side-by-side algorithm comparison"""
    print("Starting Algorithm Comparison visualization...")
    try:
        from algorithm_comparison import AlgorithmComparisonVisualizer
        visualizer = AlgorithmComparisonVisualizer()
        visualizer.run()
    except ImportError as e:
        print(f"Error importing comparison visualization: {e}")
        print("Make sure algorithm_comparison.py is available")
    except Exception as e:
        print(f"Error running comparison: {e}")

def run_research_demos():
    """Run research demonstration scenarios"""
    print()
    print("Research Demo Scenarios:")
    print("1. Small Grid (20x20) - Quick demonstration")
    print("2. Medium Grid (40x40) - Standard research size")
    print("3. Large Grid (60x60) - Performance comparison")
    print("4. Obstacle-heavy environment")
    print("5. Sparse obstacle environment")
    print("6. Custom grid configuration")
    print("7. Back to main menu")
    print()
    
    while True:
        try:
            demo_choice = input("Select demo scenario (1-7): ").strip()
            
            if demo_choice == '1':
                run_demo_small_grid()
                break
            elif demo_choice == '2':
                run_demo_medium_grid()
                break
            elif demo_choice == '3':
                run_demo_large_grid()
                break
            elif demo_choice == '4':
                run_demo_obstacle_heavy()
                break
            elif demo_choice == '5':
                run_demo_sparse_obstacles()
                break
            elif demo_choice == '6':
                run_demo_custom()
                break
            elif demo_choice == '7':
                return
            else:
                print("Invalid choice. Please enter a number between 1-7.")
        except KeyboardInterrupt:
            print("\nReturning to main menu...")
            return

def run_demo_small_grid():
    """Run demo with small grid"""
    print("Running small grid demo (20x20)...")
    try:
        from algorithm_comparison import AlgorithmComparisonVisualizer
        visualizer = AlgorithmComparisonVisualizer(grid_size=20, cell_size=20, step_size=0.05)
        visualizer.run()
    except Exception as e:
        print(f"Error running small grid demo: {e}")

def run_demo_medium_grid():
    """Run demo with medium grid"""
    print("Running medium grid demo (40x40)...")
    try:
        from algorithm_comparison import AlgorithmComparisonVisualizer
        visualizer = AlgorithmComparisonVisualizer(grid_size=40, cell_size=15, step_size=0.02)
        visualizer.run()
    except Exception as e:
        print(f"Error running medium grid demo: {e}")

def run_demo_large_grid():
    """Run demo with large grid"""
    print("Running large grid demo (60x60)...")
    try:
        from algorithm_comparison import AlgorithmComparisonVisualizer
        visualizer = AlgorithmComparisonVisualizer(grid_size=60, cell_size=10, step_size=0.01)
        visualizer.run()
    except Exception as e:
        print(f"Error running large grid demo: {e}")

def run_demo_obstacle_heavy():
    """Run demo with obstacle-heavy environment"""
    print("Running obstacle-heavy environment demo...")
    try:
        from algorithm_comparison import AlgorithmComparisonVisualizer
        
        # Create custom visualizer with obstacle-heavy grid
        visualizer = AlgorithmComparisonVisualizer(grid_size=40, cell_size=15, step_size=0.02)
        
        # Override grid creation with more obstacles
        def create_obstacle_heavy_grid():
            grid = [[0 for _ in range(40)] for _ in range(40)]
            
            # Add many obstacle blocks
            obstacles = [
                (5, 5, 10, 10),    # Large central block
                (15, 2, 18, 8),    # Vertical barrier
                (25, 15, 35, 18),  # Horizontal barrier
                (8, 20, 12, 25),   # Small block
                (20, 25, 25, 30),  # Medium block
                (30, 5, 35, 12),   # Side block
                (2, 30, 8, 35),    # Corner block
                (18, 35, 22, 38),  # Bottom block
            ]
            
            for start_r, start_c, end_r, end_c in obstacles:
                for r in range(start_r, min(end_r + 1, 40)):
                    for c in range(start_c, min(end_c + 1, 40)):
                        if 0 <= r < 40 and 0 <= c < 40:
                            grid[r][c] = 1
            
            return grid
        
        visualizer.original_grid = create_obstacle_heavy_grid()
        visualizer.reset_grids()
        visualizer.run()
        
    except Exception as e:
        print(f"Error running obstacle-heavy demo: {e}")

def run_demo_sparse_obstacles():
    """Run demo with sparse obstacles"""
    print("Running sparse obstacles environment demo...")
    try:
        from algorithm_comparison import AlgorithmComparisonVisualizer
        
        # Create custom visualizer with sparse obstacles
        visualizer = AlgorithmComparisonVisualizer(grid_size=40, cell_size=15, step_size=0.02)
        
        # Override grid creation with fewer obstacles
        def create_sparse_grid():
            grid = [[0 for _ in range(40)] for _ in range(40)]
            
            # Add few scattered obstacles
            obstacles = [
                (10, 10, 12, 12),  # Small block 1
                (25, 8, 27, 10),   # Small block 2
                (15, 25, 17, 27),  # Small block 3
            ]
            
            for start_r, start_c, end_r, end_c in obstacles:
                for r in range(start_r, min(end_r + 1, 40)):
                    for c in range(start_c, min(end_c + 1, 40)):
                        if 0 <= r < 40 and 0 <= c < 40:
                            grid[r][c] = 1
            
            return grid
        
        visualizer.original_grid = create_sparse_grid()
        visualizer.reset_grids()
        visualizer.run()
        
    except Exception as e:
        print(f"Error running sparse obstacles demo: {e}")

def run_demo_custom():
    """Run demo with custom configuration"""
    print()
    print("Custom Configuration:")
    print("Enter your preferred settings (press Enter for defaults)")
    
    try:
        # Get grid size
        grid_input = input("Grid size (default 40): ").strip()
        grid_size = int(grid_input) if grid_input and grid_input.isdigit() else 40
        grid_size = max(10, min(grid_size, 100))  # Clamp between 10-100
        
        # Calculate appropriate cell size
        if grid_size <= 20:
            cell_size = 20
        elif grid_size <= 40:
            cell_size = 15
        elif grid_size <= 60:
            cell_size = 10
        else:
            cell_size = 8
        
        # Get step size
        step_input = input("Step delay in seconds (default 0.02): ").strip()
        try:
            step_size = float(step_input) if step_input else 0.02
            step_size = max(0.001, min(step_size, 1.0))  # Clamp between 0.001-1.0
        except ValueError:
            step_size = 0.02
        
        print(f"Running custom demo with grid {grid_size}x{grid_size}, cell size {cell_size}, step delay {step_size}s")
        
        from algorithm_comparison import AlgorithmComparisonVisualizer
        visualizer = AlgorithmComparisonVisualizer(grid_size=grid_size, cell_size=cell_size, step_size=step_size)
        visualizer.run()
        
    except ValueError:
        print("Invalid input. Using default settings.")
        run_demo_medium_grid()
    except Exception as e:
        print(f"Error running custom demo: {e}")

def show_help():
    """Show help information"""
    print()
    print("Help Information:")
    print("================")
    print()
    print("BA* Algorithm:")
    print("- Uses boustrophedon (back-and-forth) motion pattern")
    print("- Employs A* for pathfinding between coverage areas")
    print("- Guarantees complete coverage when possible")
    print("- Uses A*SPT for path smoothing")
    print()
    print("C* Algorithm:")
    print("- Uses Rapidly Covering Graphs (RCG) approach")
    print("- RRT-like sampling for node generation")
    print("- Multi-objective frontier selection")
    print("- Probabilistically complete coverage")
    print()
    print("Visualization Controls:")
    print("- SPACE: Pause/Resume algorithm execution")
    print("- R: Reset to initial state")
    print("- Number keys: Quick start options")
    print("- Mouse: Interactive speed control slider")
    print()
    print("Research Applications:")
    print("- Algorithm performance comparison")
    print("- Coverage efficiency analysis")
    print("- Path planning strategy evaluation")
    print("- Obstacle avoidance behavior study")
    print()

def main():
    """Main application entry point"""
    print_banner()
    
    while True:
        print_menu()
        
        try:
            choice = input("Select option (0-5, h for help): ").strip().lower()
            print()
            
            if choice == '0':
                print("Exiting application. Goodbye!")
                sys.exit(0)
            
            elif choice == '1':
                run_ba_star_pygame_1()
            
            elif choice == '2':
                run_ba_star_pygame_2()
            
            elif choice == '3':
                run_c_star_pygame()
            
            elif choice == '4':
                run_algorithm_comparison()
            
            elif choice == '5':
                run_research_demos()
            
            elif choice == 'h' or choice == 'help':
                show_help()
            
            else:
                print("Invalid choice. Please select a number from 0-5 or 'h' for help.")
            
            print()
            
        except KeyboardInterrupt:
            print("\n\nExiting application. Goodbye!")
            sys.exit(0)
        except Exception as e:
            print(f"An error occurred: {e}")
            print("Please try again or contact support if the problem persists.")
            print()

if __name__ == '__main__':
    main()