import time
import tracemalloc
import numpy as np
import copy
from dataclasses import dataclass
from typing import List, Dict

from CodeBase.Benchmark.grid_factory import generate_grid
from CodeBase.Search.astar_graph_based import AStarPlanner_graphbased
from CodeBase.Search.astar_tree_based import AStarPlanner_treebased
from CodeBase.Search.bfs import BFSPlanner_graphbased, BFSPlanner_treesearch
from CodeBase.Search.dfs import DFSPlanner_graphbased
from CodeBase.Benchmark.plotter import plot_all_tests


# ---------------------------------------------------------
# Planner registry 
# ---------------------------------------------------------
PLANNERS = {
    "A* Graph": AStarPlanner_graphbased,
    #"A* Tree":  AStarPlanner_treebased,
    "BFS Graph": BFSPlanner_graphbased,
    "DFS Graph": DFSPlanner_graphbased
    #"BFS Tree":  BFSPlanner_treesearch,
    # "DFS":      DFSPlanner,
}

@dataclass
class ResultRow:
    """Flat row just for text table / quick summary."""
    test_title: str
    planner_name: str
    runtime: float
    expansions: int
    path_length: int
    memory_kb: float


class Comparator:
    def __init__(self, robot_radius=1.0, motion_model="8n"):
        self.results: List[ResultRow] = []   # flat table-style results
        self.test_runs: List[Dict] = []      # structured per-test data for plotting
        self.robot_radius = robot_radius
        self.motion_model = motion_model

    # ---------------------------------------------------------
    # Main benchmark runner
    # ---------------------------------------------------------
    def run_all_tests(self):

        tests = [
            ("Small 10x10", 10),
            ("Medium 50x50", 50),
            ("Large 100x100", 100),
        ]

        for title, size in tests:
            print(f"\n=== Running benchmark on {title} ===")

            # Generate grid + inflated obstacles + random start/goal
            grid_map, start, goal, obstacles  = generate_grid(size, self.robot_radius)

            # Per-test structure for plotting
            per_test_data = {
                "title": title,
                "size": size,
                "start": start,
                "goal": goal,
                "radius": self.robot_radius,
                "obstacles": [(o.gx, o.gy) for o in obstacles],
                "resolution": grid_map.resolution,
                "grid_height": grid_map.height,
                "grid_width": grid_map.width,
                "planners": {}   # filled below
            }

            for planner_name, PlannerClass in PLANNERS.items():
                print(f"  -> {planner_name}")


                grid_copy = copy.deepcopy(grid_map)
                # Instantiate planner (no visualizer in benchmark mode)
                planner = PlannerClass(grid_copy, motion_model=self.motion_model, visualizer=None)

                # Measure time + memory
                tracemalloc.start()
                t0 = time.time()
                path = planner.plan(start, goal)
                t1 = time.time()
                current, peak = tracemalloc.get_traced_memory()
                tracemalloc.stop()

                runtime = t1 - t0
                memory_kb = peak / 1024.0
                expansions = getattr(planner, "expanded_count", -1)
                path_len = len(path) if path else 0
                reached_goal = bool(path and path[-1] == goal)

                # Build heatmap from planner.expansion_map if present
                h, w = grid_map.height, grid_map.width
                heat = np.zeros((h, w), dtype=int)
                expansion_map = getattr(planner, "expansion_map", {})

                for (x, y), count in expansion_map.items():
                    if 0 <= x < w and 0 <= y < h:
                        heat[y, x] = count

                # Store flat row for text summary
                self.results.append(
                    ResultRow(
                        test_title=title,
                        planner_name=planner_name,
                        runtime=runtime,
                        expansions=expansions,
                        path_length=path_len,
                        memory_kb=memory_kb,
                    )
                )

                # Store structured stats for plotting
                per_test_data["planners"][planner_name] = {
                    "runtime": runtime,
                    "expansions": expansions,
                    "path_len": path_len,
                    "memory_kb": memory_kb,
                    "heatmap": heat,
                    "reached_goal": reached_goal,
                    "path": path if path else [] 
                }

            # Add this test to list of runs
            self.test_runs.append(per_test_data)

        return self.test_runs

    # ---------------------------------------------------------
    # Console print
    # ---------------------------------------------------------
    def print_results(self):
        print("\n====== Benchmark Results ======")
        for r in self.results:
            print(
                f"{r.test_title} - {r.planner_name}: "
                f"time={r.runtime:.4f}s, "
                f"exp={r.expansions}, "
                f"path_len={r.path_length}, "
                f"mem={r.memory_kb:.1f} KB"
            )

    # ---------------------------------------------------------
    # Orchestrate: run + print + plot
    # ---------------------------------------------------------
    def run_and_plot(self):
        test_runs = self.run_all_tests()
        self.print_results()
        plot_all_tests(test_runs)