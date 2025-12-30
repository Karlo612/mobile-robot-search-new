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


import random

def manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def pick_start_goal(grid_map, max_tries=5_000):
    safe_cells = [
        (x, y)
        for y in range(grid_map.height)
        for x in range(grid_map.width)
        if not grid_map.is_obstacle(x, y)
        and not grid_map.is_inflated(x, y)
    ]

    if len(safe_cells) < 2:
        raise RuntimeError("Not enough safe cells for start/goal")

    for _ in range(max_tries):
        start, goal = random.sample(safe_cells, 2)
        if start != goal:
            return start, goal

    raise RuntimeError("Failed to pick distinct start/goal")

def create_random_grid_environment(
    size: int,
    obstacle_ratio: float,
    robot_radius: float,
    resolution: float = 1.0,
    max_retries: int = 5,
):
    """
    Robust random environment generator.

    Tries up to `max_retries` times before failing.
    """

    last_error = None

    for attempt in range(1, max_retries + 1):
        try:
            print(
                f"\n[GRID FACTORY CALL attempt {attempt}/{max_retries}]",
                f"size={size}",
                f"obstacle_ratio={obstacle_ratio}",
                f"robot_radius={robot_radius}",
                f"resolution={resolution}"
            )

            grid_array = generate_random_grid(size, obstacle_ratio)
            obstacles = grid_to_obstacles(grid_array)

            grid_map = GridMap(grid_array, resolution=resolution)
            world_map = WorldMap(origin=(0, 0), resolution=resolution)

            robot = MobileRobot(robot_radius, (0, 0), (0, 0))
            robot.attach_maps(grid_map, world_map)

            # Inflate once
            inflator = ObstacleInflator(robot_radius)
            inflator.inflate(grid_map, world_map, obstacles)

            inflated_count = sum(
                grid_map.is_inflated(x, y)
                for y in range(grid_map.height)
                for x in range(grid_map.width)
            )
            print("[GRID FACTORY] inflated cells =", inflated_count)

            # ---- START / GOAL PICK ----
            start, goal = pick_start_goal(grid_map)

            # HARD GUARANTEE
            if start == goal:
                raise RuntimeError("Start equals goal (invalid)")

            robot.sx, robot.sy = start
            robot.gx, robot.gy = goal

            print(
                "[GRID FACTORY RESULT]",
                "robot.radius =", robot.radius,
                "start =", start,
                "goal =", goal
            )

            return grid_map, world_map, robot, obstacles

        except Exception as e:
            last_error = e
            print(f"[GRID FACTORY] retry failed: {e}")

    # 🔥 After all retries
    raise RuntimeError(
        f"Grid generation failed after {max_retries} retries: {last_error}"
    )

