# CodeBase/Environment/random_grid_loader.py

import numpy as np
import random

from CodeBase.Environment.grid_map import GridMap
from CodeBase.Environment.world_map import WorldMap
from CodeBase.Environment.mobile_robot import MobileRobot
from CodeBase.Environment.inflator import ObstacleInflator
from CodeBase.Environment.obstacle import Obstacle


def generate_random_grid(size: int, obstacle_ratio: float) -> np.ndarray:
    """
    Create random grid with boundary walls.
    0 = free, 1 = obstacle
    """
    grid = np.random.choice(
        [0, 1],
        size=(size, size),
        p=[1 - obstacle_ratio, obstacle_ratio]
    ).astype(int)

    # force boundary walls
    grid[0, :] = 1
    grid[-1, :] = 1
    grid[:, 0] = 1
    grid[:, -1] = 1

    return grid


def grid_to_obstacles(grid: np.ndarray):
    """
    Convert obstacle cells (value=1) to Obstacle objects.
    """
    obstacles = []
    h, w = grid.shape
    for y in range(h):
        for x in range(w):
            if grid[y, x] == 1:
                obstacles.append(Obstacle(x, y))
    return obstacles


def manhattan(a, b) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def pick_start_goal(grid_map: GridMap, min_dist: int, max_tries: int = 10_000):
    """
    Pick start and goal from cells that are:
      - NOT obstacles
      - NOT inflated
    And ensure Manhattan(start, goal) >= min_dist.
    """

    h, w = grid_map.height, grid_map.width

    def pick_free():
        for _ in range(max_tries):
            gx = random.randrange(w)
            gy = random.randrange(h)
            if (not grid_map.is_obstacle(gx, gy) and
                not grid_map.is_inflated(gx, gy)):
                return gx, gy
        raise RuntimeError("Failed to find free (non-obstacle, non-inflated) cell.")

    start = pick_free()

    for _ in range(max_tries):
        goal = pick_free()
        if goal != start and manhattan(start, goal) >= min_dist:
            # print("[PICKED]", start, goal, "dist =", manhattan(start, goal))
            return start, goal

    raise RuntimeError(
        f"Failed to find goal far enough from start (min_dist={min_dist}). "
        f"Try lowering min_dist, robot radius, or obstacle density."
    )


def create_random_grid_environment(
    size: int,
    obstacle_ratio: float,
    robot_radius: float,
    resolution: float = 1.0
):
    """
    Full random environment generator (GUI-ready):
      1) generate random grid with boundary walls
      2) create obstacles list
      3) build GridMap + WorldMap
      4) attach robot
      5) inflate obstacles ONCE
      6) pick start/goal far apart (uses pick_start_goal)

    Returns:
      grid_map, world_map, robot, obstacles
    """
    grid_array = generate_random_grid(size, obstacle_ratio)

    # Create obstacles
    obstacles = grid_to_obstacles(grid_array)
    print("[GRID FACTORY] obstacle count =", len(obstacles))

    # Create maps
    grid_map = GridMap(grid_array, resolution=resolution)
    world_map = WorldMap(origin=(0, 0), resolution=resolution)

    # Create robot (dummy start/goal first)
    robot = MobileRobot(robot_radius, (0, 0), (0, 0))
    robot.attach_maps(grid_map, world_map)

    # Inflate ONCE and KEEP it
    inflator = ObstacleInflator(robot_radius)
    inflator.inflate(grid_map, world_map, obstacles)

    # Debug inflated cells count
    inflated_count = sum(
        1
        for y in range(grid_map.height)
        for x in range(grid_map.width)
        if grid_map.is_inflated(x, y)
    )
    print("[GRID FACTORY] inflated cells =", inflated_count)

    # IMPORTANT FIX:
    # Use pick_start_goal() (your constraint function) instead of random.sample()
    min_dist = max(5, size // 2)  # scales: 10->5, 50->25, 100->50
    start, goal = pick_start_goal(grid_map, min_dist=min_dist)

    robot.sx, robot.sy = start
    robot.gx, robot.gy = goal

    return grid_map, world_map, robot, obstacles
