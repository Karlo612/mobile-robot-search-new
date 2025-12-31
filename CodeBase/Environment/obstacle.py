"""
Obstacle - Obstacle Representation

This module provides the Obstacle class which represents a single obstacle
in the environment. Obstacles are stored with their grid coordinates and
optionally their world coordinates.
"""

class Obstacle:
    """
    Represents a single obstacle in the environment.
    
    Obstacles are stored with their grid coordinates (gx, gy) which are
    required. World coordinates (wx, wy) are optional and can be computed
    from grid coordinates when needed.
    """

    def __init__(self, gx, gy, wx=None, wy=None):
        """
        Initialize an obstacle.
        
        Args:
            gx: Grid x coordinate (column)
            gy: Grid y coordinate (row)
            wx: Optional world x coordinate
            wy: Optional world y coordinate
        """
        self.gx = gx  # Grid x coordinate
        self.gy = gy  # Grid y coordinate
        self.wx = wx  # World x coordinate (optional)
        self.wy = wy  # World y coordinate (optional)

    def __repr__(self):
        return f"Obstacle(gx={self.gx}, gy={self.gy})"