"""
Obstacle Inflator - Robot Size Compensation

This module provides obstacle inflation functionality. Since the robot has a
physical size (radius), cells that are too close to obstacles must be marked
as blocked to ensure the robot can actually fit through passages. This process
"inflates" obstacles by the robot's radius.
"""

import math
from .obstacle import Obstacle

class ObstacleInflator:
    """
    Inflates obstacles to account for the robot's physical size.
    
    Cells within the robot's radius of an obstacle are marked as blocked,
    ensuring that any path found will be wide enough for the robot to actually
    navigate through. This is essential for path planning with non-point robots.
    """
    def __init__(self, robot_radius):
        """
        Initialize the obstacle inflator.
        
        Args:
            robot_radius: The radius of the robot in world units (e.g., meters)
        """
        self.grid_map = None
        self.world_map = None
        self.robot_radius = robot_radius
        self.obstacles = None

    def distance(self, obs_x, obs_y, wx, wy):
        """
        Calculate Euclidean distance between two points in world coordinates.
        
        Args:
            obs_x, obs_y: First point coordinates
            wx, wy: Second point coordinates
            
        Returns:
            Euclidean distance between the points
        """
        return math.sqrt((obs_x - wx)**2 + (obs_y - wy)**2)

    def inflate(self, grid_map, world_map, obstacles):
        """
        Inflate obstacles by marking nearby cells as blocked.
        
        For each free cell in the grid, check if it's within the robot's radius
        of any obstacle. If so, mark it as inflated (blocked). This ensures
        the robot can safely navigate through the environment.
        
        Args:
            grid_map: GridMap to mark inflated cells in
            world_map: WorldMap for coordinate conversions
            obstacles: List of Obstacle objects to inflate around
        """
        self.grid_map = grid_map
        self.world_map = world_map
        self.obstacles = obstacles
        # Obstacle radius is half the grid cell size (assuming obstacles fill cells)
        obstacle_radius = grid_map.resolution / 2.0

        # Initialize the inflated grid if not already done
        self.grid_map.init_inflation()
        
        # Check every cell in the grid
        for gy in range(grid_map.height):
            for gx in range(grid_map.width):
                # Skip cells that are already obstacles
                if self.grid_map.is_obstacle(gx, gy):
                    continue

                # Get the world coordinates of this cell's center
                wx, wy = self.world_map.grid_to_world(gx, gy)

                # Check distance to all obstacles
                for obstacle in self.obstacles:
                    # Get obstacle center in world coordinates
                    obs_wx, obs_wy = self.world_map.grid_to_world(obstacle.gx, obstacle.gy)
                    # Calculate distance between cell center and obstacle center
                    ds = self.distance(obs_wx, obs_wy, wx, wy)
                    # If too close, mark as inflated (blocked)
                    if ds < self.robot_radius + obstacle_radius:
                        self.grid_map.mark_inflated(gx, gy)
                        break  # No need to check other obstacles for this cell