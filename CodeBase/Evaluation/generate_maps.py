# CodeBase/Evaluation/generate_maps.py

from CodeBase.Util.random_grid_factory import create_random_grid_environment

def generate_maps(size, count, obstacle_ratio, robot_radius, resolution):
    maps = []

    for i in range(count):
        grid_map, world_map, robot, obstacles = create_random_grid_environment(
            size=size,
            obstacle_ratio=obstacle_ratio,
            robot_radius=robot_radius,
            resolution=resolution
        )

        maps.append({
            "grid_map": grid_map,
            "world_map": world_map,
            "robot": robot,
            "obstacles": obstacles,
            "start": (robot.sx, robot.sy),
            "goal": (robot.gx, robot.gy),
        })

    return maps
