"""
Grid Map - Representation of the Search Space

This module provides the GridMap class which represents the environment as a
2D grid where each cell can be free (0) or an obstacle (1). The grid also
supports obstacle inflation, where cells near obstacles are marked as blocked
to account for the robot's physical size.
"""

import numpy as np

class GridMap:
    """
    Represents the environment as a 2D grid map.
    
    The grid stores free cells (0) and obstacles (1) in grid coordinates.
    Note that grid indexing uses [row][column] which corresponds to [gy][gx],
    where y is the row and x is the column.
    
    The grid also supports obstacle inflation, where cells within the robot's
    radius of an obstacle are marked as blocked to ensure the robot can fit
    through passages.
    """

    def __init__(self, grid_array, resolution):
        """
        Initialize the grid map with a numpy array.
        
        Args:
            grid_array: 2D numpy array where 0 = free, 1 = obstacle
            resolution: Size of each grid cell in world units (e.g., meters)
        """
        self.height, self.width = grid_array.shape
        self.resolution = resolution
        self.grid = grid_array

        # Inflated grid is created lazily when obstacle inflation is needed
        # This saves memory if inflation is never used
        self.inflated_grid = None

    def init_inflation(self):
        """
        Initialize the inflated grid for obstacle inflation.
        
        Creates a boolean array to track which cells are blocked due to
        obstacle inflation (cells too close to obstacles for the robot to fit).
        """
        self.inflated_grid = np.zeros((self.height, self.width), dtype=bool)

    def set_cell(self, gx, gy, value):
        """
        Set the value of a grid cell.
        
        Args:
            gx: Grid x coordinate (column)
            gy: Grid y coordinate (row)
            value: 0 for free, 1 for obstacle
        """
        self.grid[gy][gx] = value

    def mark_inflated(self, gx, gy):
        """
        Mark a cell as blocked by obstacle inflation.
        
        Args:
            gx: Grid x coordinate
            gy: Grid y coordinate
        """
        self.inflated_grid[gy][gx] = True

    def get_cell(self, gx, gy):
        """
        Get the value of a grid cell.
        
        Args:
            gx: Grid x coordinate
            gy: Grid y coordinate
            
        Returns:
            0 for free, 1 for obstacle
        """
        return self.grid[gy][gx]

    def is_inside(self, gx, gy):
        """
        Check if coordinates are within grid boundaries.
        
        Args:
            gx: Grid x coordinate
            gy: Grid y coordinate
            
        Returns:
            True if coordinates are valid, False otherwise
        """
        return 0 <= gx < self.width and 0 <= gy < self.height
    
    def is_free(self, gx, gy):
        """
        Check if a cell is free (not an obstacle).
        
        Args:
            gx: Grid x coordinate
            gy: Grid y coordinate
            
        Returns:
            True if cell is free, False if it's an obstacle
        """
        return True if self.grid[gy][gx] == 0 else False

    def is_obstacle(self, gx, gy):
        """
        Check if a cell contains an obstacle.
        
        Args:
            gx: Grid x coordinate
            gy: Grid y coordinate
            
        Returns:
            True if cell is an obstacle, False otherwise
        """
        return True if self.grid[gy][gx] == 1 else False
    
    def is_inflated(self, gx, gy):
        """
        Check if a cell is blocked by obstacle inflation.
        
        A cell is inflated if it's within the robot's radius of an obstacle,
        meaning the robot is too large to safely pass through.
        
        Args:
            gx: Grid x coordinate
            gy: Grid y coordinate
            
        Returns:
            True if cell is blocked by inflation, False otherwise
        """
        if self.inflated_grid is None:
            return False
        return self.inflated_grid[gy][gx]
    
    
    