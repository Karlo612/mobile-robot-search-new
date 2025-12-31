"""
Navigation System - Execution Engine for Path Planning

This module provides the NavigationSystem class which orchestrates the path planning
process. It takes a pre-configured environment and execution settings, then runs the
appropriate search algorithm to find a path from start to goal.

The system supports multiple planners (A*, BFS, DFS) with both graph-based and
tree-based search modes, and can work with or without visualization.
"""

from CodeBase.Search.astar_graph_based import AStarPlanner_graphbased
from CodeBase.Search.astar_tree_based import AStarPlanner_treebased
from CodeBase.Search.bfs import BFSPlanner_graphbased, BFSPlanner_treesearch
from CodeBase.Search.dfs_graph_based import DFSPlanner_graphbased
from CodeBase.Search.dfs_tree_based import DFSPlanner_treebased


class NavigationSystem:
    """
    Main execution engine for path planning operations.
    
    This class coordinates the entire path planning workflow: it validates the
    environment, creates the appropriate planner based on configuration, runs
    the search algorithm, and handles visualization. The environment (grid map,
    robot, obstacles) must be fully configured before being passed to this system.
    
    The system uses a factory pattern to instantiate the correct planner based
    on the algorithm type (A*, BFS, DFS) and search mode (graph-based or tree-based).
    """

    def __init__(self, env_data, exec_config, visualizer=None):
        """
        Initialize the navigation system with environment and configuration.
        
        Args:
            env_data: Dictionary containing the environment setup:
                - "grid_map": GridMap object representing the grid world
                - "world_map": WorldMap object for coordinate transformations
                - "robot": MobileRobot object with start/goal positions
                - "obstacles": List of Obstacle objects in the environment
            exec_config: Dictionary with execution settings:
                - "planner": Algorithm name ("Astar", "BFS", or "DFS")
                - "motion": Motion model ("4n" for 4-neighbor, "8n" for 8-neighbor)
                - "use_tree_search": True for tree-based, False for graph-based search
                - "visualize_search": Whether to show visualization during search
                - "navigation_mode": "single" for one run, "comparison" for batch
            visualizer: Optional EmbeddedVisualizer for real-time visualization
        """
        self.planner = None
        self.env = env_data
        self.cfg = exec_config
        self.vis = visualizer

        # Extract and normalize execution settings
        self.planner_name = exec_config.get("planner", "Astar").upper()
        self.motion = exec_config.get("motion", "8n")
        self.use_tree = exec_config.get("use_tree_search", False)
        self.visualize = exec_config.get("visualize_search", True)
        self.mode = exec_config.get("navigation_mode", "single")
        self.max_expansions = exec_config.get("max_expansions", 50000)

    # --------------------------------------------------
    # Validation
    # --------------------------------------------------
    def _validate_start_goal(self, grid_map, robot):
        """
        Check that the robot's start position is valid for path planning.
        
        Validates that the start position is not on an obstacle and not blocked
        by inflated obstacles (obstacles expanded by robot radius).
        
        Args:
            grid_map: The grid map to check against
            robot: The robot object with start position (sx, sy)
            
        Raises:
            RuntimeError: If start position is invalid
        """
        print("[NAV] Validating start:", robot.sx, robot.sy)
        sx, sy = robot.sx, robot.sy

        # Check if start is directly on an obstacle
        if grid_map.is_obstacle(sx, sy):
            raise RuntimeError("Start is on an obstacle")

        # Check if start is blocked by inflated obstacles (robot too large for gap)
        if grid_map.is_inflated(sx, sy):
            raise RuntimeError("Start is blocked after obstacle inflation")

    # --------------------------------------------------
    # Planner factory
    # --------------------------------------------------
    def _create_planner(self, grid_map):
        """
        Create the appropriate planner instance based on configuration.
        
        Uses a factory pattern to instantiate the correct planner class. The planner
        type depends on the algorithm (A*, BFS, DFS) and search mode (graph-based
        or tree-based). Graph-based search tracks visited nodes to avoid revisiting,
        while tree-based search allows revisiting but may be more memory efficient.
        
        Args:
            grid_map: The grid map the planner will search on
            
        Returns:
            A planner instance (AStarPlanner, BFSPlanner, or DFSPlanner)
            
        Raises:
            NotImplementedError: If the requested planner type is not supported
        """
        print(
            "[NAV] Creating planner:",
            self.planner_name,
            "| motion =", self.motion,
            "| tree =", self.use_tree
        )
        
        # A* algorithm - optimal path finding with heuristic
        if self.planner_name == "ASTAR":
            if self.use_tree:
                return AStarPlanner_treebased(
                    grid_map, motion_model=self.motion, visualizer=self.vis
                )
            return AStarPlanner_graphbased(
                grid_map, motion_model=self.motion, visualizer=self.vis
            )

        # BFS algorithm - breadth-first search, finds shortest path in unweighted graphs
        elif self.planner_name == "BFS":
            if self.use_tree:
                return BFSPlanner_treesearch(
                    grid_map, motion_model=self.motion, visualizer=self.vis,
                    max_expansions=self.max_expansions
                )
            return BFSPlanner_graphbased(
                grid_map, motion_model=self.motion, visualizer=self.vis
            )

        # DFS algorithm - depth-first search, explores deeply before backtracking
        elif self.planner_name == "DFS":
            if self.use_tree:
                return DFSPlanner_treebased(
                grid_map, motion_model=self.motion, visualizer=self.vis,
                max_expansions=self.max_expansions
                )
            return DFSPlanner_graphbased(
                grid_map, motion_model=self.motion, visualizer=self.vis    
            )

        else:
            raise NotImplementedError(
                f"Planner '{self.planner_name}' not supported"
            )

    # --------------------------------------------------
    # MAIN EXECUTION
    # --------------------------------------------------
    def run(self):
        """
        Execute the complete path planning workflow.
        
        This method orchestrates the entire path planning process:
        1. Validates the environment and start/goal positions
        2. Sets up visualization if enabled
        3. Creates and runs the appropriate planner
        4. Updates robot position and visualizes the final path
        
        Returns:
            List of (x, y) tuples representing the path from start to goal,
            or None if no path was found
        """
        print("[NAV] Entering NavigationSystem.run()")

        # Extract environment components
        grid_map = self.env["grid_map"]
        world_map = self.env["world_map"]
        robot = self.env["robot"]
        obstacles = self.env["obstacles"]

        # --------------------------------------------------
        # 1. Check obstacle inflation status
        # --------------------------------------------------
        # Obstacles are inflated during environment creation, so we verify
        # the inflation state here rather than re-inflating
        
        inflated_count = sum(
            grid_map.is_inflated(x, y)
            for y in range(grid_map.height)
            for x in range(grid_map.width)
        )

        print("[NAV] Inflated cells:", inflated_count)

        # --------------------------------------------------
        # 2. Validate and extract start/goal positions
        # --------------------------------------------------
        # Check if start/goal are explicitly provided in env_data (for GUI flexibility)
        # Otherwise, use the robot's stored positions
        if "start" in self.env and "goal" in self.env:
            start = self.env["start"]
            goal  = self.env["goal"]

            # Sync robot positions for consistency with GUI
            robot.sx, robot.sy = start
            robot.gx, robot.gy = goal
        else:
            # Validate that robot's start position is valid
            self._validate_start_goal(grid_map, robot)
            start = (robot.sx, robot.sy)
            goal  = (robot.gx, robot.gy)

        print("[NAV] Using start/goal:", start, goal)

        # --------------------------------------------------
        # 3. Initialize visualization if enabled
        # --------------------------------------------------
        if self.vis and self.visualize:
            self.vis.setup()
            self.vis.draw_start_goal(start, goal)
            self.vis.update()

        # --------------------------------------------------
        # 4. Create the appropriate planner instance
        # --------------------------------------------------
        planner = self._create_planner(grid_map)
        self.planner = planner

        # --------------------------------------------------
        # 5. Execute path planning algorithm
        # --------------------------------------------------
        path = planner.plan(start, goal)

        if path:
            print("[NAV] Path length:", len(path)-1)
        else:
            print("[NAV] No path found")

        # Return early if no path was found
        if not path:
            print("[NavigationSystem] No path found")
            return None

        # --------------------------------------------------
        # 6. Update visualization and robot position
        # --------------------------------------------------
        if self.vis and self.visualize:
            self.vis.draw_final_path(path)
            self.vis.update()

        # Convert final grid position to world coordinates and update robot
        gx, gy = path[-1]
        robot.x, robot.y = world_map.grid_to_world(gx, gy)

        print(
            "[NAV] Final world pose:",
            (robot.x, robot.y),
            "| resolution =", world_map.resolution
        )

        return path
