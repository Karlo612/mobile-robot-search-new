"""
Base Planner - Abstract Interface for Path Planning Algorithms

This module defines the abstract base class that all path planning algorithms
inherit from. It provides a common interface for different search algorithms
(A*, BFS, DFS) to work with the navigation system.
"""

class Planner:
    """
    Abstract base class for all path planning algorithms.
    
    All planners must implement the plan() method which takes a start and goal
    position and returns a path (list of coordinates) or None if no path exists.
    The planner works with a grid map and can optionally use a visualizer for
    real-time visualization of the search process.
    """
    def __init__(self, grid_map, motion_model="8n", visualizer=None):
        """
        Initialize the planner with a grid map and configuration.
        
        Args:
            grid_map: GridMap object representing the search space
            motion_model: "4n" for 4-neighbor movement or "8n" for 8-neighbor (diagonal)
            visualizer: Optional visualizer for real-time search visualization
        """
        self.grid_map = grid_map
        self.visualizer = visualizer
        self.motion_model = motion_model

    def plan(self, start, goal):
        """
        Plan a path from start to goal.
        
        This method must be implemented by all subclasses. It should return
        a list of (x, y) coordinates representing the path, or None if
        no path exists.
        
        Args:
            start: Tuple (x, y) representing the start position
            goal: Tuple (x, y) representing the goal position
            
        Returns:
            List of (x, y) tuples representing the path, or None if no path found
        """
        raise NotImplementedError("plan() must be implemented by subclasses")