# mobile-robot-search
implementation of Astar, BFS and DFS in grid base environment for mobile robot


#environment setup

1. go to the 'project' directory
2. python3 -m venv venv
3. source venv/bin/activate
4. pip install -r requirements.txt
5. source venv/bin/activate


## Recent Changes

### GridMap Class Enhancements
- Implemented the `get_neighbors` method in the `GridMap` class to support 4-connected movement.
- This method returns valid neighboring positions based on the current grid state, allowing pathfinding algorithms to efficiently explore the grid.

### Future Considerations
- Consider whether to extend the `get_neighbors` method to support 8-connected movement based on project requirements.
