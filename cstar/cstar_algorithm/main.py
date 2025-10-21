from .cstar import CStar
from .envs import create_test_environment

def run_console_test():
    print("=== C* Algorithm Console Test ===")
    grid = create_test_environment(20)
    start_pos = (0, 0)
    c_star = CStar(grid, start_pos, sensing_radius=2)
    path, results = c_star.run()
    print(f"\nFinal Results:")
    print(f"Path length: {len(path)}")
    print(f"Coverage: {results['coverage_percentage']:.1f}%")
    print(f"RCG nodes: {results['nodes_generated']}")

if __name__ == "__main__":
    run_console_test()
