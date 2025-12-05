import time
import tracemalloc
from dataclasses import dataclass

from CodeBase.Benchmark.grid_factory import generate_grid
from CodeBase.Search.astar_graph_based import AStarPlanner_graphbased
from CodeBase.Search.astar_tree_based import AStarPlanner_treebased
#from CodeBase.Search.bfs import BFSPlanner
#from CodeBase.Search.dfs import DFSPlanner
from CodeBase.Benchmark.plotter import plot_all


# ---------------------------------------------------------
# Planner registry 
# ---------------------------------------------------------
PLANNERS = {
    "A* Graph": AStarPlanner_graphbased,
    "A* Tree":  AStarPlanner_treebased,
    # "BFS":      BFSPlanner,
    # "DFS":      DFSPlanner,
}


@dataclass
class Result:
    name: str
    runtime: float
    expansions: int
    path_length: int
    memory_kb: float


class Comparator:

    def __init__(self, robot_radius=1.0):
        self.results = []
        self.robot_radius = robot_radius   # read from config.json

    # ---------------------------------------------------------
    # Run A* graph and tree on small, medium, large grids
    # ---------------------------------------------------------
    def run_all_tests(self):

        tests = [
            ("Small 10x10", 10),
            ("Medium 50x50", 50),
            ("Large 100x100", 100),
        ]

        for title, size in tests:

            print(f"\n=== Running benchmark on {title} ===")

            # FIX 1 → pass robot radius!
            grid_map, start, goal = generate_grid(size, self.robot_radius)

            for planner_name, PlannerClass in PLANNERS.items():

                print(f"  -> {planner_name}")

                # Create planner
                planner = PlannerClass(grid_map, motion_model="8n", visualizer=None)

                #start memory measurment
                tracemalloc.start()
                # Start timing
                t0 = time.time()
                path = planner.plan(start, goal)
                t1 = time.time()

                # Get memory stats
                current, peak = tracemalloc.get_traced_memory()
                tracemalloc.stop()

                memory_used_kb = peak / 1024.0

                # FIX 2 – expansions must be counted by planner
                expansions = getattr(planner, "expanded_count", -1)

                self.results.append(
                    Result(
                        name=f"{title} - {planner_name}",
                        runtime=t1 - t0,
                        expansions=expansions,
                        path_length=len(path) if path else 0,
                        memory_kb=memory_used_kb
                    )
                )

    def print_results(self):
        print("\n====== Benchmark Results ======")
        for r in self.results:
            print(f"{r.name}: "
                  f"time={r.runtime:.4f}s, "
                  f"exp={r.expansions}, "
                  f"path_len={r.path_length}, "
                  f"mem={r.memory_kb:.1f} KB")

    def run_and_plot(self):
        self.run_all_tests()
        self.print_results()
        plot_all(self.results)