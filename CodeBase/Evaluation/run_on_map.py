"""
Run Planner on Map - Headless Path Planning Execution

This module provides functionality to run path planning algorithms headlessly
(without visualization) on a given environment. It measures performance metrics
including runtime, memory usage, path length, and node expansions.
"""

import time
import tracemalloc
from CodeBase.navigation_system import NavigationSystem


def run_planner_on_map(env_data, exec_config):
    """
    Run a planner headlessly on a fixed environment.

    Parameters
    ----------
    env_data : dict
        {
            "grid_map": GridMap,
            "world_map": WorldMap,
            "robot": MobileRobot,
            "obstacles": list[Obstacle]
        }

    exec_config : dict
        {
            "planner": "Astar" | "BFS" | "DFS",
            "motion": "4n" | "8n",
            "use_tree_search": bool,
            "visualize_search": False,
            "navigation_mode": "batch" | "single"
        }
    """

    # Force headless + batch-safe defaults
    exec_config = dict(exec_config)  # shallow copy
    exec_config["visualize_search"] = False
    exec_config.setdefault("navigation_mode", "batch")

    nav = NavigationSystem(
        env_data=env_data,
        exec_config=exec_config,
        visualizer=None
    )

    tracemalloc.start()

    start_time = time.perf_counter()
    path = nav.run()
    end_time = time.perf_counter()
    
    current, peak = tracemalloc.get_traced_memory()
    tracemalloc.stop()

    planner = nav.planner

    # ---- metrics ----
    found = path is not None
    path_len = len(path) if found else 0

    if found and hasattr(planner, "cost"):
        path_cost = sum(
            planner.cost(*p1, *p2)
            for p1, p2 in zip(path[:-1], path[1:])
        )
    else:
        path_cost = float("inf")
    
    return {
        "planner": exec_config["planner"],
        "tree": exec_config["use_tree_search"],
        "motion": exec_config["motion"],

        "found": found,
        "path_len": path_len,
        "path_cost": path_cost,
        "path": path,

        "expanded_nodes": getattr(planner, "expanded_count", 0),
        "runtime_ms": (end_time - start_time) * 1000.0,
        "memory_kb": peak / 1024.0,

        "expansion_map": getattr(planner, "expansion_map", {}),
    }
