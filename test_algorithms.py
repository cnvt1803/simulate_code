"""
Test Script for Coverage Path Planning Algorithms
Quick verification that all implementations work correctly
"""

import sys
import traceback

def test_ba_star_algorithm():
    """Test BA* algorithm core functionality"""
    print("Testing BA* Algorithm...")
    try:
        from ba_star_algorithm import BAStar
        
        # Create simple test grid
        test_grid = [
            [0, 0, 0, 1, 0],
            [0, 1, 0, 0, 0],
            [0, 0, 0, 1, 0],
            [1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0]
        ]
        
        ba_star = BAStar(test_grid, (0, 0))
        path, final_grid = ba_star.run()
        
        print(f"  ✓ BA* completed successfully")
        print(f"  ✓ Path length: {len(path)}")
        print(f"  ✓ Coverage paths generated: {len(ba_star.coverage_paths)}")
        return True
        
    except Exception as e:
        print(f"  ✗ BA* test failed: {e}")
        traceback.print_exc()
        return False

def test_c_star_algorithm():
    """Test C* algorithm core functionality"""
    print("Testing C* Algorithm...")
    try:
        from c_star_algorithm import CStar
        
        # Create simple test grid
        test_grid = [
            [0, 0, 0, 1, 0],
            [0, 1, 0, 0, 0],
            [0, 0, 0, 1, 0],
            [1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0]
        ]
        
        c_star = CStar(test_grid, (0, 0), sensing_radius=2)
        path, results = c_star.run()
        
        print(f"  ✓ C* completed successfully")
        print(f"  ✓ Path length: {len(path)}")
        print(f"  ✓ Coverage percentage: {results.get('coverage_percentage', 0):.1f}%")
        print(f"  ✓ RCG nodes generated: {results.get('nodes_generated', 0)}")
        return True
        
    except Exception as e:
        print(f"  ✗ C* test failed: {e}")
        traceback.print_exc()
        return False

def test_pygame_dependencies():
    """Test if pygame is available"""
    print("Testing Pygame dependencies...")
    try:
        import pygame
        print("  ✓ Pygame available")
        return True
    except ImportError:
        print("  ✗ Pygame not available - install with: pip install pygame")
        return False

def test_numpy_dependencies():
    """Test if numpy is available"""
    print("Testing Numpy dependencies...")
    try:
        import numpy
        print("  ✓ Numpy available")
        return True
    except ImportError:
        print("  ✗ Numpy not available - install with: pip install numpy")
        return False

def test_visualizations():
    """Test visualization imports"""
    print("Testing Visualization modules...")
    
    # Test BA* pygame visualization
    try:
        from ba_star_pygame_visualization import PygameVisualizer
        print("  ✓ BA* Pygame visualization available")
        ba_pygame_ok = True
    except Exception as e:
        print(f"  ✗ BA* Pygame visualization failed: {e}")
        ba_pygame_ok = False
    
    # Test C* pygame visualization  
    try:
        from c_star_pygame_visualization import CStarPygameVisualizer
        print("  ✓ C* Pygame visualization available")
        c_pygame_ok = True
    except Exception as e:
        print(f"  ✗ C* Pygame visualization failed: {e}")
        c_pygame_ok = False
    
    # Test comparison visualization
    try:
        from algorithm_comparison import AlgorithmComparisonVisualizer
        print("  ✓ Algorithm comparison visualization available")
        comparison_ok = True
    except Exception as e:
        print(f"  ✗ Algorithm comparison failed: {e}")
        comparison_ok = False
    
    return ba_pygame_ok and c_pygame_ok and comparison_ok

def run_quick_demo():
    """Run a quick demonstration of both algorithms"""
    print("\nRunning Quick Demonstration...")
    print("=" * 50)
    
    # Create a small test grid
    test_grid = [
        [0, 0, 0, 0, 1, 0, 0, 0],
        [0, 1, 1, 0, 0, 0, 1, 0],
        [0, 0, 0, 0, 1, 0, 0, 0],
        [1, 0, 1, 0, 0, 0, 0, 1],
        [0, 0, 0, 0, 1, 1, 0, 0],
        [0, 1, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 1, 0, 0, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 0]
    ]
    
    print("Grid layout (0=free, 1=obstacle):")
    for row in test_grid:
        print("  " + " ".join(str(cell) for cell in row))
    print()
    
    # Test BA* algorithm
    print("BA* Algorithm Results:")
    try:
        from ba_star_algorithm import BAStar
        ba_star = BAStar(test_grid, (0, 0))
        ba_path, ba_grid = ba_star.run()
        
        # Calculate coverage
        from ba_star_algorithm import COVERED
        covered_cells = sum(row.count(COVERED) for row in ba_grid)
        total_free = sum(1 for r in range(len(test_grid)) for c in range(len(test_grid[0])) 
                        if test_grid[r][c] == 0)
        ba_coverage = (covered_cells / total_free) * 100
        
        print(f"  Path length: {len(ba_path)}")
        print(f"  Coverage: {ba_coverage:.1f}% ({covered_cells}/{total_free} cells)")
        print(f"  Coverage paths: {len(ba_star.coverage_paths)}")
        print(f"  A* paths used: {len(ba_star.astar_paths)}")
        
    except Exception as e:
        print(f"  BA* demo failed: {e}")
    
    print()
    
    # Test C* algorithm
    print("C* Algorithm Results:")
    try:
        from c_star_algorithm import CStar
        c_star = CStar(test_grid, (0, 0), sensing_radius=2)
        c_path, c_results = c_star.run()
        
        print(f"  Path length: {len(c_path)}")
        print(f"  Coverage: {c_results.get('coverage_percentage', 0):.1f}%")
        print(f"  RCG nodes: {c_results.get('nodes_generated', 0)}")
        print(f"  Iterations: {c_results.get('iterations', 0)}")
        
    except Exception as e:
        print(f"  C* demo failed: {e}")

def main():
    """Main test function"""
    print("Coverage Path Planning Algorithms - Test Suite")
    print("=" * 60)
    print()
    
    tests_passed = 0
    total_tests = 0
    
    # Test dependencies
    total_tests += 1
    if test_pygame_dependencies():
        tests_passed += 1
    
    total_tests += 1
    if test_numpy_dependencies():
        tests_passed += 1
    
    print()
    
    # Test algorithms
    total_tests += 1
    if test_ba_star_algorithm():
        tests_passed += 1
    
    print()
    
    total_tests += 1
    if test_c_star_algorithm():
        tests_passed += 1
    
    print()
    
    # Test visualizations
    total_tests += 1
    if test_visualizations():
        tests_passed += 1
    
    print()
    print("=" * 60)
    print(f"Test Results: {tests_passed}/{total_tests} tests passed")
    
    if tests_passed == total_tests:
        print("✓ All tests passed! All systems ready for research visualization.")
        
        # Ask if user wants to see a quick demo
        try:
            demo_choice = input("\nRun quick algorithm demonstration? (y/n): ").strip().lower()
            if demo_choice in ['y', 'yes']:
                run_quick_demo()
                print("\nTo start full visualizations, run: python main_runner.py")
        except KeyboardInterrupt:
            print("\nTest complete.")
    else:
        print("✗ Some tests failed. Please check dependencies and implementations.")
        
        if tests_passed >= 3:  # If core algorithms work
            print("\nCore algorithms appear functional. You may still be able to run basic demos.")
    
    print()

if __name__ == '__main__':
    main()