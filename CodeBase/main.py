# main.py
from .navigation_system import NavigationSystem
import json
import os

def load_environment_from_config(cfg):
    """Load environment from config file"""
    from .Environment.map_loader import MapLoader
    from .Environment.grid_map import GridMap
    from .Environment.world_map import WorldMap
    from .Environment.mobile_robot import MobileRobot
    from .Environment.inflator import ObstacleInflator
    
    # Get paths relative to project root
    project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    map_file = os.path.join(project_root, cfg["map_file"])
    
    # Load map
    loader = MapLoader()
    grid_array, obstacles = loader.load(map_file)
    
    # Create grid map and world map
    resolution = cfg.get("resolution", 1.0)
    origin = cfg.get("origin", [0, 0])
    
    grid_map = GridMap(grid_array, resolution)
    world_map = WorldMap(origin=origin, resolution=resolution)
    
    # Create robot
    start_grid = cfg.get("start_grid", [0, 0])
    goal_grid = cfg.get("goal_grid", [0, 0])
    robot_radius = cfg.get("robot_radius", 1.0)
    
    robot = MobileRobot(robot_radius, start_grid, goal_grid)
    robot.attach_maps(grid_map, world_map)
    
    # Inflate obstacles
    inflator = ObstacleInflator(robot_radius)
    inflator.inflate(grid_map, world_map, obstacles)
    
    return {
        "grid_map": grid_map,
        "world_map": world_map,
        "robot": robot,
        "obstacles": obstacles,
        "start": tuple(start_grid),
        "goal": tuple(goal_grid)
    }

def main():
    # Get project root for config path
    project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    config_path = os.path.join(project_root, "Data", "configs", "config.json")
    
    with open(config_path) as f:
        cfg = json.load(f)

    mode = cfg.get("mode", "nav").lower()

    if mode == "nav":
        # Load environment from config
        env_data = load_environment_from_config(cfg)
        
        # Create execution config
        exec_config = {
            "planner": cfg.get("planner", "Astar"),
            "motion": cfg.get("motion", "8n"),
            "use_tree_search": cfg.get("use_tree_search", False),
            "visualize_search": cfg.get("visualize_search", True),
            "navigation_mode": "single"
        }
        
        # Create visualizer if needed
        visualizer = None
        root = None
        if exec_config["visualize_search"]:
            try:
                from .Visualization.embedded_visualizer import EmbeddedVisualizer
                import tkinter as tk
                root = tk.Tk()
                root.title("Path Planning Visualization")
                vis_frame = tk.Frame(root)
                vis_frame.pack(fill="both", expand=True)
                visualizer = EmbeddedVisualizer(vis_frame, env_data["grid_map"])
            except Exception as e:
                print(f"Warning: Could not initialize visualizer: {e}")
                print("Running without visualization...")
                exec_config["visualize_search"] = False
        
        # Create and run navigation system
        system = NavigationSystem(env_data, exec_config, visualizer)
        system.run()
        
        # Keep window open if visualizing
        if visualizer and root:
            root.mainloop()

    elif mode == "bench":
        print("\n=== Running Benchmark Mode ===")
        try:
            from .Benchmark.comparator import Comparator
            robot_r = cfg.get("robot_radius", 1.0)
            motion  = cfg.get("motion", "8n")
            cmp = Comparator(robot_radius=robot_r, motion_model=motion)
            cmp.run_and_plot()
        except ImportError:
            print("ERROR: Benchmark module not found. Please ensure CodeBase/Benchmark/comparator.py exists.")
            print("Falling back to navigation mode...")
            # Fallback to nav mode
            env_data = load_environment_from_config(cfg)
            exec_config = {
                "planner": cfg.get("planner", "Astar"),
                "motion": cfg.get("motion", "8n"),
                "use_tree_search": cfg.get("use_tree_search", False),
                "visualize_search": cfg.get("visualize_search", True),
                "navigation_mode": "single"
            }
            system = NavigationSystem(env_data, exec_config, None)
            system.run()

    else:
        raise ValueError(f"Unknown config mode: {mode}")

if __name__ == "__main__":
    main()