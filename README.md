# A* Maze Solver

https://github.com/b4r4d41z/A_star_maze_solver/raw/main/a-star-solving.mp4

## Project Overview

This project implements an A* pathfinding algorithm to solve a 16x16 maze in a simulator built with Unreal Engine 5. The robot uses sensors to detect walls and navigate through the maze while finding the optimal path to the goal.

## Algorithm Overview

The A* algorithm used in this project combines:
- **Pathfinding**: Uses f(n) = g(n) + h(n) where:
  - g(n): Cost from start to current node
  - h(n): Estimated cost from current node to goal (Manhattan distance)
  - f(n): Total estimated cost of path through node n

## Key Features

1. **Intelligent Navigation**
   - Uses four directional sensors (front, right, left, back)
   - Maintains a visited cells map
   - Implements backtracking when needed

2. **Maze Exploration**
   - 16x16 grid environment
   - Real-time sensor data processing
   - Multiple goal positions handling
   - Optimal path recording

3. **Robot Control**
   - Four movement directions (forward, backward, left, right)
   - Orientation tracking
   - Automated path execution
   - Multiple maze run attempts

## Technical Details

- Map size: 16x16 grid
- Goal positions: (7,7), (7,8), (8,7), (8,8)
- Sensor threshold: 70 units
- Communication: HTTP API requests
- Position normalization: -1300 to 1300 range

## Project Structure

- `A-star_final.py` - Main algorithm implementation
- `70/run_no_UI.exe` - No UI version of the simulator
- `a-star-solving.mp4` - Algorithm demonstration video

## Getting Started

1. Clone the repository:
```bash
git clone https://github.com/b4r4d41z/A_star_maze_solver.git
```

2. Simulator Installation:
   - You can use the no_UI version of the simulator provided in the repository
   - For the full version with UI (>100MB), please contact me as GitHub doesn't allow files larger than 100MB
   - The full version provides better visualization and debugging capabilities

3. Run the algorithm:
```bash
python A-star_final.py
```

## Requirements

- Python 3.x
- NumPy
- Requests
- Unreal Engine 5 (for simulator)

## License

This project is available under the MIT License.