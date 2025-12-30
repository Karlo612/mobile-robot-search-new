#!/usr/bin/env python
"""
Command-line comparison tool for pathfinding algorithms
Runs the same comparison functionality as the GUI, but without GUI dependencies
"""
import sys
import os
import json
import csv
import numpy as np
from pathlib import Path

# Add project root to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from CodeBase.Evaluation.run_on_map import run_planner_on_map
from CodeBase.Util.random_grid_factory import create_random_grid_environment
from CodeBase.Util.heatmap_utils import expansion_map_to_array


def generate_maps(config):
    """Generate maps based on configuration"""
    maps = {}
    grids = {}
    starts = {}
    goals = {}
    
    obstacle_ratio = config.get("obstacle_ratio", 0.2)
    robot_radius = config.get("robot_radius", 0.5)
    resolution = config.get("resolution", 1.0)
    base_seed = config.get("seed", None)
    
    map_config = [
        ("small", 10, config.get("small_count", 5), config.get("use_small", True)),
        ("medium", 50, config.get("medium_count", 5), config.get("use_medium", True)),
        ("large", 100, config.get("large_count", 5), config.get("use_large", True)),
    ]
    
    print("\n" + "="*60)
    print("GENERATING MAPS")
    print("="*60)
    
    for name, size, count, enabled in map_config:
        if not enabled:
            continue
            
        maps[name] = []
        success = 0
        
        for i in range(count):
            # Deterministic seed if provided
            if base_seed is not None:
                import random
                s = base_seed + (hash(name) % 10_000) + i * 97
                random.seed(s)
                np.random.seed(s % (2**32 - 1))
            
            try:
                grid_map, world_map, robot, obstacles = create_random_grid_environment(
                    size=size,
                    obstacle_ratio=obstacle_ratio,
                    robot_radius=robot_radius,
                    resolution=resolution
                )
                
                map_id = f"{name}_{i}"
                
                # Store static data
                grids[map_id] = grid_map.grid.copy()
                starts[map_id] = (robot.sx, robot.sy)
                goals[map_id] = (robot.gx, robot.gy)
                
                # Create env data
                env = {
                    "grid_map": grid_map,
                    "world_map": world_map,
                    "robot": robot,
                    "obstacles": obstacles,
                    "start": (robot.sx, robot.sy),
                    "goal": (robot.gx, robot.gy),
                    "meta": {
                        "map_id": map_id,
                        "size_name": name,
                        "size": size,
                        "map_index": i,
                    }
                }
                
                maps[name].append(env)
                success += 1
                
            except Exception as e:
                print(f"  [SKIP] {name} map {i} failed: {e}")
        
        print(f"  [OK] {name} ({size}x{size}): {success}/{count} maps generated")
    
    print("="*60 + "\n")
    
    return maps, grids, starts, goals


def run_comparison(maps, config):
    """Run comparison experiments"""
    planners = config.get("planners", [
        ("BFS", False),
        ("DFS", False),
        ("Astar", False),
    ])
    
    motion = config.get("motion", "8n")
    
    # Separate planners by tree/graph
    tree_planners = [(name, True) for name, tree in planners if tree]
    graph_planners = [(name, False) for name, tree in planners if not tree]
    
    # Reorder tree planners: A* → BFS → DFS
    def sort_key(planner_tuple):
        name, _ = planner_tuple
        if name == "Astar":
            return 0
        elif name == "BFS":
            return 1
        elif name == "DFS":
            return 2
        else:
            return 3
    
    tree_planners_sorted = sorted(tree_planners, key=sort_key)
    
    # Combine: tree-based first (in order), then graph-based
    ordered_planners = tree_planners_sorted + graph_planners
    
    results = []
    heatmaps = {}
    paths = {}
    
    total_jobs = sum(len(envs) for envs in maps.values()) * len(ordered_planners)
    current_job = 0
    
    print("="*60)
    print("RUNNING EXPERIMENTS")
    print("="*60)
    
    for size_name, envs in maps.items():
        for mid, env_data in enumerate(envs):
            map_id = env_data["meta"]["map_id"]
            
            for planner_name, use_tree_search in ordered_planners:
                exec_cfg = {
                    "planner": planner_name,
                    "motion": motion,
                    "use_tree_search": use_tree_search,
                    "visualize_search": False,
                    "navigation_mode": "batch",
                }
                
                # Run planner
                result = run_planner_on_map(env_data, exec_cfg)
                
                # Capture path
                path = result.pop("path", None)
                path_id = f"{map_id}_{planner_name}_{use_tree_search}_{motion}"
                if path is not None:
                    paths[path_id] = path
                
                # Capture heatmap
                grid_map = env_data["grid_map"]
                width, height = grid_map.width, grid_map.height
                heatmap = expansion_map_to_array(
                    result.get("expansion_map", {}),
                    width,
                    height
                )
                heatmap_id = f"{map_id}_{planner_name}_{use_tree_search}_{motion}"
                heatmaps[heatmap_id] = heatmap
                
                # Flatten result
                result.pop("heatmap", None)
                result["heatmap_id"] = heatmap_id
                result["map_size"] = size_name
                result["map_id"] = mid
                result["planner"] = planner_name
                result["tree"] = use_tree_search
                result["motion"] = motion
                
                results.append(result)
                
                current_job += 1
                progress = (current_job / total_jobs) * 100
                
                status = ""
                if result.get('exceeded_expansion_limit', False):
                    status = " [STOPPED: expansion limit exceeded]"
                
                print(f"  [{current_job}/{total_jobs} ({progress:.1f}%)] "
                      f"{size_name} map {mid} | {planner_name} "
                      f"{'Tree' if use_tree_search else 'Graph'} | "
                      f"time={result.get('runtime_ms', -1):.1f}ms "
                      f"exp={result.get('expanded_nodes', 'NA')} "
                      f"path={result.get('path_len', 'NA')}{status}")
    
    print("="*60 + "\n")
    
    return results, heatmaps, paths


def export_results(results, heatmaps, paths, grids, starts, goals, output_dir):
    """Export results to CSV and NPZ"""
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    
    csv_path = output_dir / "comparison_results.csv"
    npz_path = output_dir / "comparison_data.npz"
    
    # Export CSV
    if results:
        cols = sorted({k for r in results for k in r.keys()})
        with open(csv_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=cols)
            writer.writeheader()
            for r in results:
                writer.writerow(r)
        print(f"✓ CSV exported: {csv_path}")
    
    # Export NPZ
    np.savez_compressed(
        npz_path,
        **{f"grid/{k}": v for k, v in grids.items()},
        **{f"start/{k}": v for k, v in starts.items()},
        **{f"goal/{k}": v for k, v in goals.items()},
        **{f"heatmap/{k}": v for k, v in heatmaps.items()},
        **{f"path/{k}": v for k, v in paths.items()},
    )
    print(f"✓ NPZ exported: {npz_path}")
    
    return csv_path, npz_path


def main():
    """Main function"""
    print("\n" + "="*60)
    print("PATHFINDING ALGORITHM COMPARISON")
    print("="*60)
    
    # Default configuration
    config = {
        # Map generation
        "use_small": True,
        "use_medium": True,
        "use_large": True,
        "small_count": 5,
        "medium_count": 5,
        "large_count": 5,
        "obstacle_ratio": 0.2,
        "robot_radius": 0.5,
        "resolution": 1.0,
        "seed": None,  # Set to a number for reproducible results
        
        # Experiment settings
        "motion": "8n",  # "4n" or "8n"
        "planners": [
            ("BFS", False),   # (planner_name, use_tree_search)
            ("DFS", False),
            ("Astar", False),
            # Uncomment to include tree-based versions:
            # ("BFS", True),
            # ("DFS", True),
            # ("Astar", True),
        ],
        
        # Output
        "output_dir": "comparison_results",
    }
    
    # Check for config file
    config_file = Path("comparison_config.json")
    if config_file.exists():
        print(f"Loading configuration from {config_file}")
        with open(config_file) as f:
            user_config = json.load(f)
            config.update(user_config)
    
    # Generate maps
    maps, grids, starts, goals = generate_maps(config)
    
    if not maps:
        print("ERROR: No maps generated. Check your configuration.")
        return
    
    # Run comparison
    results, heatmaps, paths = run_comparison(maps, config)
    
    # Export results
    csv_path, npz_path = export_results(
        results, heatmaps, paths, grids, starts, goals, config["output_dir"]
    )
    
    # Summary
    print("="*60)
    print("COMPARISON COMPLETE")
    print("="*60)
    print(f"Total experiments: {len(results)}")
    print(f"Results saved to: {config['output_dir']}")
    print(f"  - CSV: {csv_path}")
    print(f"  - NPZ: {npz_path}")
    print("\nTo generate plots, use:")
    print(f"  python -m CodeBase.Evaluation.plot_experiment_results")
    print("="*60 + "\n")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nExperiment cancelled by user.")
        sys.exit(1)
    except Exception as e:
        print(f"\n\nERROR: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

