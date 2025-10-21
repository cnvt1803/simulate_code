"""
Troubleshooting Script for Visualization Issues
Helps identify and fix common problems with pygame visualizations
"""

import sys
import os

def check_python_version():
    """Check Python version compatibility"""
    print("=== Python Version Check ===")
    version = sys.version_info
    print(f"Python Version: {version.major}.{version.minor}.{version.micro}")
    
    if version.major < 3 or (version.major == 3 and version.minor < 7):
        print("❌ Python 3.7+ is required")
        return False
    else:
        print("✅ Python version is compatible")
        return True

def check_dependencies():
    """Check if required packages are installed"""
    print("\n=== Dependency Check ===")
    
    # Check pygame
    try:
        import pygame
        print(f"✅ Pygame {pygame.version.ver} is installed")
        pygame_ok = True
    except ImportError as e:
        print(f"❌ Pygame not found: {e}")
        print("   Install with: pip install pygame")
        pygame_ok = False
    
    # Check numpy
    try:
        import numpy
        print(f"✅ Numpy {numpy.__version__} is installed")
        numpy_ok = True
    except ImportError as e:
        print(f"❌ Numpy not found: {e}")
        print("   Install with: pip install numpy")
        numpy_ok = False
    
    return pygame_ok and numpy_ok

def check_file_paths():
    """Check if all required files exist"""
    print("\n=== File Path Check ===")
    
    required_files = [
        'ba_star_algorithm.py',
        'c_star_algorithm.py',
        'ba_star_pygame_visualization.py',
        'c_star_pygame_visualization.py',
        'algorithm_comparison.py'
    ]
    
    current_dir = os.path.dirname(os.path.abspath(__file__))
    print(f"Current directory: {current_dir}")
    
    all_files_exist = True
    for file in required_files:
        file_path = os.path.join(current_dir, file)
        if os.path.exists(file_path):
            print(f"✅ {file} found")
        else:
            print(f"❌ {file} missing")
            all_files_exist = False
    
    return all_files_exist

def test_pygame_display():
    """Test if pygame can initialize display"""
    print("\n=== Pygame Display Test ===")
    
    try:
        import pygame
        pygame.init()
        
        # Try to create a small test window
        screen = pygame.display.set_mode((100, 100))
        pygame.display.set_caption("Test Window")
        print("✅ Pygame display initialization successful")
        
        # Clean up
        pygame.quit()
        return True
        
    except Exception as e:
        print(f"❌ Pygame display failed: {e}")
        print("   This might be a graphics driver or display issue")
        return False

def test_algorithm_imports():
    """Test if algorithm modules can be imported"""
    print("\n=== Algorithm Import Test ===")
    
    try:
        from ba_star_algorithm import BAStar
        print("✅ BA* algorithm imports successfully")
        ba_ok = True
    except Exception as e:
        print(f"❌ BA* algorithm import failed: {e}")
        ba_ok = False
    
    try:
        from c_star_algorithm import CStar
        print("✅ C* algorithm imports successfully")
        c_ok = True
    except Exception as e:
        print(f"❌ C* algorithm import failed: {e}")
        c_ok = False
    
    return ba_ok and c_ok

def test_visualization_imports():
    """Test if visualization modules can be imported"""
    print("\n=== Visualization Import Test ===")
    
    try:
        from ba_star_pygame_visualization import PygameVisualizer
        print("✅ BA* pygame visualization imports successfully")
        ba_viz_ok = True
    except Exception as e:
        print(f"❌ BA* pygame visualization import failed: {e}")
        ba_viz_ok = False
    
    try:
        from c_star_pygame_visualization import CStarPygameVisualizer
        print("✅ C* pygame visualization imports successfully")
        c_viz_ok = True
    except Exception as e:
        print(f"❌ C* pygame visualization import failed: {e}")
        c_viz_ok = False
    
    try:
        from algorithm_comparison import AlgorithmComparisonVisualizer
        print("✅ Algorithm comparison imports successfully")
        comp_ok = True
    except Exception as e:
        print(f"❌ Algorithm comparison import failed: {e}")
        comp_ok = False
    
    return ba_viz_ok and c_viz_ok and comp_ok

def run_simple_test():
    """Run a simple algorithm test"""
    print("\n=== Simple Algorithm Test ===")
    
    try:
        # Test BA*
        from ba_star_algorithm import BAStar
        test_grid = [[0, 1, 0], [0, 0, 0], [1, 0, 0]]
        ba = BAStar(test_grid, (0, 0))
        path, grid = ba.run()
        print(f"✅ BA* test successful - Path length: {len(path)}")
        
        # Test C*
        from c_star_algorithm import CStar
        c = CStar(test_grid, (0, 0), sensing_radius=1)
        path, results = c.run()
        print(f"✅ C* test successful - Coverage: {results.get('coverage_percentage', 0):.1f}%")
        
        return True
        
    except Exception as e:
        print(f"❌ Algorithm test failed: {e}")
        return False

def suggest_solutions():
    """Suggest solutions for common problems"""
    print("\n=== Common Solutions ===")
    print("If you're having issues, try these solutions:")
    print()
    print("1. Install/Update Dependencies:")
    print("   pip install pygame numpy")
    print()
    print("2. Run from Correct Directory:")
    print("   cd \"d:\\Ho Chi Minh University of Technology\\Final Project\\code\"")
    print("   python c_star_pygame_visualization.py")
    print()
    print("3. Use Full Paths:")
    print("   python \"d:\\Ho Chi Minh University of Technology\\Final Project\\code\\c_star_pygame_visualization.py\"")
    print()
    print("4. Use the Batch File:")
    print("   Double-click: run_visualizations.bat")
    print()
    print("5. Graphics Driver Issues:")
    print("   - Update your graphics drivers")
    print("   - Try running as administrator")
    print("   - Check Windows display settings")
    print()
    print("6. Permission Issues:")
    print("   - Run command prompt as administrator")
    print("   - Check file permissions in the project folder")

def main():
    """Main troubleshooting function"""
    print("🔧 VISUALIZATION TROUBLESHOOTING TOOL 🔧")
    print("=" * 50)
    
    all_checks_passed = True
    
    # Run all checks
    if not check_python_version():
        all_checks_passed = False
    
    if not check_dependencies():
        all_checks_passed = False
    
    if not check_file_paths():
        all_checks_passed = False
    
    if not test_pygame_display():
        all_checks_passed = False
    
    if not test_algorithm_imports():
        all_checks_passed = False
    
    if not test_visualization_imports():
        all_checks_passed = False
    
    if not run_simple_test():
        all_checks_passed = False
    
    # Results
    print("\n" + "=" * 50)
    if all_checks_passed:
        print("🎉 ALL CHECKS PASSED!")
        print("Your system should be ready for visualizations.")
        print()
        print("Try running:")
        print("  python c_star_pygame_visualization.py")
        print("  python algorithm_comparison.py")
    else:
        print("❌ SOME ISSUES FOUND")
        print("Please check the failed items above.")
        suggest_solutions()
    
    print("\n" + "=" * 50)

if __name__ == '__main__':
    main()