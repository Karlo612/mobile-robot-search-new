"""
Mobile Robot - Robot Representation

This module provides the MobileRobot class which represents the robot in the
environment. The robot has a physical size (radius) and maintains positions
in both grid coordinates (for path planning) and world coordinates (for actual
navigation).
"""

from .grid_map import GridMap

class MobileRobot:
    """
    Represents a mobile robot in the environment.
    
    The robot has a physical radius and maintains positions in both grid
    coordinates (for path planning algorithms) and world coordinates (for
    actual navigation and visualization). The robot knows its start and
    goal positions and can convert between coordinate systems.
    """

    def __init__(self, radius, start_grid, goal_grid):
        """
        Initialize the mobile robot.
        
        Args:
            radius: Physical radius of the robot in world units
            start_grid: Start position as (x, y) in grid coordinates
            goal_grid: Goal position as (x, y) in grid coordinates
        """
        self.radius = radius
        self.sx, self.sy = start_grid  # Start position in grid coordinates
        self.gx, self.gy = goal_grid  # Goal position in grid coordinates
        self.x = self.y = None  # Current position in world coordinates
        self.grid_map = None
        self.world_map = None
        
    def attach_maps(self, grid_map, world_map):
        """
        Attach grid and world maps to the robot.
        
        This allows the robot to convert between coordinate systems and
        initializes the robot's world position based on its grid start position.
        
        Args:
            grid_map: GridMap object for grid coordinate operations
            world_map: WorldMap object for coordinate conversions
        """
        self.grid_map = grid_map
        self.world_map = world_map
        # Initialize world position from grid start position
        wx, wy = world_map.grid_to_world(self.sx, self.sy)
        self.x = wx
        self.y = wy
