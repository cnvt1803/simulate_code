# Coverage Path Planning Algorithms - Research Visualization

This repository contains implementations of **BA\*** (Boustrophedon A\*) and **C\*** algorithms for coverage path planning research, complete with interactive pygame visualizations for algorithm comparison and analysis.

## 🎯 Overview

Coverage path planning is crucial for applications like robotic vacuum cleaning, agricultural surveying, search and rescue operations, and area monitoring. This project implements two major approaches:

- **BA\* Algorithm**: Uses boustrophedon (back-and-forth) motion patterns with A\* pathfinding
- **C\* Algorithm**: Employs Rapidly Covering Graphs (RCG) with RRT-like sampling

## 🚀 Features

- **Complete Algorithm Implementations**: Full BA* and C* algorithms with research-grade quality
- **Interactive Visualizations**: Real-time pygame-based visualizations with speed control
- **Side-by-Side Comparison**: Compare both algorithms running simultaneously
- **Multiple Grid Configurations**: Test with different obstacle densities and environments
- **Performance Metrics**: Coverage percentage, path length, execution time analysis
- **Research-Ready**: Configurable parameters for academic research and analysis

## 📋 Requirements

```bash
pip install pygame numpy
```

Optional for tkinter visualization:

```bash
# Usually included with Python installation
pip install tkinter
```

## 🎮 Quick Start

### Option 1: Main Menu Interface

```bash
python main_runner.py
```

This provides a menu-driven interface with all visualization options:

1. BA\* Algorithm (Pygame)
2. BA\* Algorithm (Tkinter)
3. C\* Algorithm (Pygame)
4. Algorithm Comparison (Side-by-side)
5. Research Demos

### Option 2: Direct Algorithm Execution

**Run BA\* Algorithm:**

```bash
python ba_star_pygame_visualization.py
```

**Run C\* Algorithm:**

```bash
python c_star_pygame_visualization.py
```

**Run Side-by-Side Comparison:**

```bash
python algorithm_comparison.py
```

### Option 3: Test Suite

```bash
python test_algorithms.py
```

Verify all implementations and run quick demonstrations.

## 🕹️ Controls

### Visualization Controls

- **SPACE**: Pause/Resume algorithm execution
- **R**: Reset to initial state
- **1**: Start both algorithms (comparison mode)
- **2**: Start BA\* only
- **3**: Start C\* only
- **Mouse**: Interactive speed control slider

### Grid Display

- **White cells**: Uncovered free space
- **Black cells**: Obstacles
- **Light green cells**: Covered areas
- **Green circle**: Robot position
- **Colored lines**: Algorithm paths

## 📊 Algorithm Details

### BA* (Boustrophedon A*) Algorithm

**Core Concepts:**

- **Boustrophedon Motion**: Systematic back-and-forth coverage pattern
- **A\* Integration**: Optimal pathfinding between coverage regions
- **A\*SPT Path Smoothing**: Post-processing for optimized paths
- **Complete Coverage**: Guarantees full area coverage when possible

**Key Features:**

- Deterministic coverage pattern
- Optimal pathfinding between regions
- Handles complex obstacle configurations
- Efficient for structured environments

**Visualization Colors:**

- Blue/Purple lines: Coverage paths
- Black lines: A\* pathfinding routes
- Red dots: Backtracking points

### C\* (Rapidly Covering Graphs) Algorithm

**Core Concepts:**

- **RCG Construction**: Rapidly growing graph of coverage nodes
- **RRT-like Sampling**: Probabilistic node generation and expansion
- **Frontier Detection**: Identify uncovered boundary regions
- **Multi-objective Optimization**: Balance coverage and path efficiency

**Key Features:**

- Probabilistic completeness
- Adaptive to irregular environments
- Multi-objective frontier selection
- Suitable for unknown or dynamic environments

**Visualization Colors:**

- Pink lines: C\* coverage path
- Blue dots: RCG nodes
- Light blue lines: RCG connections
- Orange dots: Frontier nodes

## 📁 File Structure

```
code/
├── main_runner.py                    # Main menu interface
├── test_algorithms.py               # Test suite and verification
├── algorithm_comparison.py          # Side-by-side comparison
├──
├── ba_star_algorithm.py             # BA* core implementation
├── ba_star_pygame_visualization.py  # BA* pygame visualization
├── ba_star_visualization.py         # BA* tkinter visualization
├──
├── c_star_algorithm.py              # C* core implementation
├── c_star_pygame_visualization.py   # C* pygame visualization
├──
└── README.md                        # This file
```

## 🔬 Research Applications

### Performance Comparison

- **Coverage Efficiency**: Compare coverage percentages achieved
- **Path Optimality**: Analyze total path lengths and execution times
- **Obstacle Handling**: Evaluate performance in different environments
- **Scalability**: Test on various grid sizes and complexities

### Configurable Parameters

**BA\* Algorithm:**

- Grid size and resolution
- Step size and visualization speed
- Obstacle configurations
- Starting positions

**C\* Algorithm:**

- Sensing radius for coverage detection
- RCG node generation parameters
- Frontier selection criteria
- Multi-objective weights

### Demo Scenarios

1. **Small Grid (20x20)**: Quick demonstrations
2. **Medium Grid (40x40)**: Standard research size
3. **Large Grid (60x60)**: Performance analysis
4. **Obstacle-heavy**: Dense obstacle environments
5. **Sparse obstacles**: Open area coverage
6. **Custom configurations**: User-defined parameters

## 🛠️ Implementation Details

### BA\* Algorithm Architecture

```python
class BAStar:
    def __init__(self, grid, start_pos)
    def run()                    # Main execution
    def boustrophedon_coverage() # Coverage pattern generation
    def astar_pathfind()         # A* pathfinding
    def smooth_path()            # A*SPT path optimization
```

### C\* Algorithm Architecture

```python
class CStar:
    def __init__(self, grid, start_pos, sensing_radius)
    def run()                    # Main execution
    def expand_rcg()             # RCG construction
    def select_frontier()        # Multi-objective frontier selection
    def sample_and_connect()     # RRT-like node generation
```

### Visualization Features

- Real-time algorithm execution display
- Interactive speed control (0.001s to 1.0s per step)
- Comprehensive information panels
- Coverage progress tracking
- Performance metrics display

## 📈 Research Metrics

The implementations provide detailed metrics for research analysis:

**Coverage Metrics:**

- Total coverage percentage
- Covered cell count
- Uncovered regions identification

**Path Metrics:**

- Total path length
- Number of direction changes
- Path optimality measures

**Execution Metrics:**

- Algorithm runtime
- Iteration count
- Memory usage patterns

**BA\*-Specific Metrics:**

- Number of coverage paths
- A\* pathfinding calls
- Backtracking instances

**C\*-Specific Metrics:**

- RCG nodes generated
- Frontier nodes detected
- Sampling efficiency

## 🎯 Usage Examples

### Basic Algorithm Testing

```python
from ba_star_algorithm import BAStar
from c_star_algorithm import CStar

# Create test grid
grid = [[0, 0, 1, 0],
        [0, 1, 0, 0],
        [0, 0, 0, 1],
        [1, 0, 0, 0]]

# Test BA*
ba = BAStar(grid, (0, 0))
ba_path, ba_grid = ba.run()

# Test C*
c = CStar(grid, (0, 0), sensing_radius=2)
c_path, c_results = c.run()
```

### Research Comparison

```python
from algorithm_comparison import AlgorithmComparisonVisualizer

# Create comparison with custom parameters
visualizer = AlgorithmComparisonVisualizer(
    grid_size=50,
    cell_size=12,
    step_size=0.01
)
visualizer.run()
```

## 🔧 Customization

### Custom Grid Generation

```python
def create_custom_grid(size, obstacle_density=0.2):
    grid = [[0 for _ in range(size)] for _ in range(size)]
    # Add custom obstacle patterns
    return grid
```

### Algorithm Callbacks

```python
def step_callback(grid, position, iteration):
    # Custom processing during execution
    pass

ba_star.set_callbacks(step_callback=step_callback)
c_star.set_callbacks(step_callback=step_callback)
```

## 📚 Academic References

The implementations are based on research in coverage path planning:

- **Boustrophedon Coverage**: Systematic coverage patterns for complete area coverage
- **A\* Pathfinding**: Optimal path planning for navigation between regions
- **Rapidly Covering Graphs**: Probabilistic approaches to coverage planning
- **Multi-objective Optimization**: Balancing coverage efficiency and path optimality

## 🤝 Contributing

This is a research implementation designed for academic use. Key areas for enhancement:

1. **Algorithm Extensions**: Additional coverage strategies
2. **Visualization Features**: 3D environments, enhanced metrics
3. **Performance Optimization**: Large-scale environment handling
4. **Research Tools**: Automated experiment frameworks

## 📄 License

This implementation is provided for research and educational purposes. Please cite appropriately if used in academic work.

## 🚀 Getting Started Checklist

1. **Install Dependencies**:

   ```bash
   pip install pygame numpy
   ```

2. **Verify Installation**:

   ```bash
   python test_algorithms.py
   ```

3. **Run Demonstrations**:

   ```bash
   python main_runner.py
   ```

4. **Start Research**:
   - Use algorithm comparison for side-by-side analysis
   - Modify grid configurations for different scenarios
   - Analyze performance metrics for research insights

## 📞 Support

For research-related questions or implementation details, refer to the code documentation and comments within each algorithm file. The implementations follow academic standards and include comprehensive error handling and validation.

---

**Happy Research! 🔬🤖**
