class WorldMap:
    """
    WorldMap handles the conversion between:
    - GRID coordinates (cell indices: gx, gy)
    - WORLD coordinates (continuous meters: wx, wy)
    """

    def __init__(self, origin, resolution):
        self.origin_x = origin[0]
        self.origin_y = origin[1]
        self.resolution = resolution

    def grid_to_world(self, gx, gy):
        """
        Convert grid coordinates (gx, gy) → world coordinates (wx, wy).

        A grid cell is a square. Its index gives its position in the grid.
        
        Formula:
            wx = origin_x + gx * resolution + resolution/2
            wy = origin_y + gy * resolution + resolution/2

        Why + resolution/2?
        - Robot stands at the CENTER of the cell in the real world.
        - Without this, the robot would be drawn on the grid line.

        Example:
            gx = 4, gy = 3
            resolution = 1.0
            origin = (0, 0)

            world position = (4.5, 3.5)
        """
        wx = self.origin_x + gx * self.resolution + self.resolution / 2
        wy = self.origin_y + gy * self.resolution + self.resolution / 2
        return wx, wy

 