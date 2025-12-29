from CodeBase.Benchmark.grid_factory import generate_grid
from CodeBase.Search.astar_graph_based import AStarPlanner_graphbased

def debug_one():
    size = 100
    radius = 1.0

    print("\n=== DEBUG: A* on Large 100x100 ===")

    # Generate grid
    grid_map, start, goal, obstacles = generate_grid(size, radius)

    print(f"[DEBUG] grid size     = {grid_map.width} x {grid_map.height}")
    print(f"[DEBUG] start         = {start}")
    print(f"[DEBUG] goal          = {goal}")
    print(f"[DEBUG] #obstacles    = {len(obstacles)}")

    # Check if start/goal are valid
    print(f"[DEBUG] start obstacle?  {grid_map.is_obstacle(*start)}")
    print(f"[DEBUG] start inflated?  {grid_map.is_inflated(*start)}")
    print(f"[DEBUG] goal obstacle?   {grid_map.is_obstacle(*goal)}")
    print(f"[DEBUG] goal inflated?   {grid_map.is_inflated(*goal)}")

    # Instantiate A*
    planner = AStarPlanner_graphbased(grid_map, motion_model="8n", visualizer=None)

    # Add debug before running
    print("[DEBUG] Starting A* search...")

    # Run
    path = planner.plan(start, goal)

    # Print output
    print(f"[DEBUG] expanded_count = {planner.expanded_count}")
    print(f"[DEBUG] path found?    = {path is not None}")
    print(f"[DEBUG] path length    = {len(path) if path else 0}")

if __name__ == "__main__":
    debug_one()