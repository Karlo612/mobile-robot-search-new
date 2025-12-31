"""
World Map - Coordinate System Conversion

This module provides the WorldMap class which handles conversions between
grid coordinates (discrete cell indices) and world coordinates (continuous
physical positions). This is essential for path planning algorithms that work
in grid space but need to output paths in real-world coordinates.
"""

class WorldMap:
    """
    Handles conversion between grid coordinates and world coordinates.
    
    Grid coordinates are discrete cell indices (integers), while world
    coordinates are continuous physical positions (floats). The conversion
    accounts for the grid's origin and resolution, and returns the center
    of each grid cell in world coordinates.
    """

    def __init__(self, origin, resolution):
        """
        Initialize the world map with origin and resolution.
        
        Args:
            origin: Tuple (x, y) representing the world coordinates of grid (0, 0)
            resolution: Size of each grid cell in world units (e.g., meters)
        """
        self.origin_x = origin[0]
        self.origin_y = origin[1]
        self.resolution = resolution

    def grid_to_world(self, gx, gy):
        """
        Convert grid coordinates to world coordinates.
        
        Returns the center point of the grid cell in world coordinates.
        The conversion formula is:
        world = origin + grid * resolution + resolution/2
        
        Example:
            gx = 4, gy = 3
            resolution = 1.0
            origin = (0, 0)
            world position = (4.5, 3.5)  # Center of cell (4, 3)
        
        Args:
            gx: Grid x coordinate (column index)
            gy: Grid y coordinate (row index)
            
        Returns:
            Tuple (wx, wy) representing the world coordinates of the cell center
        """
        wx = self.origin_x + gx * self.resolution + self.resolution / 2
        wy = self.origin_y + gy * self.resolution + self.resolution / 2
        return wx, wy

 