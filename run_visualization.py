# BA* Algorithm Visualization - Comparison Runner
# This file allows you to choose between tkinter and pygame versions

import sys

def main():
    print("=== BA* Algorithm Visualization ===")
    print("Choose your preferred visualization:")
    print("1. Tkinter Version (traditional GUI)")
    print("2. Pygame Version (enhanced performance)")
    print("3. Exit")
    
    while True:
        try:
            choice = input("\nEnter your choice (1-3): ").strip()
            
            if choice == '1':
                print("\n--- Starting Tkinter Version ---")
                from ba_star_visualization import GridVisualizer
                visualizer = GridVisualizer(grid_size=50, cell_size=12, step_size=0.001)
                visualizer.run()
                break
                
            elif choice == '2':
                print("\n--- Starting Pygame Version ---")
                try:
                    from ba_star_pygame_visualization import PygameVisualizer
                    visualizer = PygameVisualizer(grid_size=50, cell_size=12, step_size=0.001)
                    visualizer.run()
                except ImportError as e:
                    print(f"Error: Could not import pygame. {e}")
                    print("Please install pygame with: pip install pygame")
                    continue
                break
                
            elif choice == '3':
                print("Goodbye!")
                sys.exit(0)
                
            else:
                print("Invalid choice. Please enter 1, 2, or 3.")
                
        except KeyboardInterrupt:
            print("\nGoodbye!")
            sys.exit(0)
        except Exception as e:
            print(f"Error: {e}")

if __name__ == '__main__':
    main()