# CodeBase/navigation_system.py

from CodeBase.Environment.inflator import ObstacleInflator

from CodeBase.Search.astar_graph_based import AStarPlanner_graphbased
from CodeBase.Search.astar_tree_based import AStarPlanner_treebased
from CodeBase.Search.bfs import BFSPlanner_graphbased, BFSPlanner_treesearch
from CodeBase.Search.dfs_graph_based import DFSPlanner_graphbased
from CodeBase.Search.dfs_tree_based import DFSPlanner_treebased


class NavigationSystem:
    """
    NavigationSystem is an EXECUTION ENGINE.

    It assumes the environment (grid, robot, obstacles, origin, resolution)
    is already built and provided via env_data.

    It is GUI-only and comparison-ready.
    """

    def __init__(self, env_data, exec_config, visualizer=None):
        """
        env_data:
            {
                "grid_map": GridMap,
                "world_map": WorldMap,
                "robot": MobileRobot,
                "obstacles": List[Obstacle]
            }

        exec_config:
            {
                "planner": "Astar" | "BFS" | "DFS",
                "motion": "4n" | "8n",
                "use_tree_search": bool,
                "visualize_search": bool,
                "navigation_mode": "single" | "comparison"
            }

        visualizer:
            EmbeddedVisualizer or None
        """
        self.planner = None
        self.env = env_data
        self.cfg = exec_config
        self.vis = visualizer

        # execution settings
        self.planner_name = exec_config.get("planner", "Astar").upper()
        self.motion = exec_config.get("motion", "8n")
        self.use_tree = exec_config.get("use_tree_search", False)
        self.visualize = exec_config.get("visualize_search", True)
        self.mode = exec_config.get("navigation_mode", "single")

    # --------------------------------------------------
    # Validation
    # --------------------------------------------------
    def _validate_start_goal(self, grid_map, robot):
        print("[NAV] Validating start:", robot.sx, robot.sy)
        sx, sy = robot.sx, robot.sy

        if grid_map.is_obstacle(sx, sy):
            raise RuntimeError("Start is on an obstacle")

        if grid_map.is_inflated(sx, sy):
            raise RuntimeError("Start is blocked after inflation")

    # --------------------------------------------------
    # Planner factory
    # --------------------------------------------------
    def _create_planner(self, grid_map):
        print(
            "[NAV] Creating planner:",
            self.planner_name,
            "| motion =", self.motion,
            "| tree =", self.use_tree
        )
        if self.planner_name == "ASTAR":
            if self.use_tree:
                return AStarPlanner_treebased(
                    grid_map, motion_model=self.motion, visualizer=self.vis
                )
            return AStarPlanner_graphbased(
                grid_map, motion_model=self.motion, visualizer=self.vis
            )

        elif self.planner_name == "BFS":
            if self.use_tree:
                return BFSPlanner_treesearch(
                    grid_map, motion_model=self.motion, visualizer=self.vis
                )
            return BFSPlanner_graphbased(
                grid_map, motion_model=self.motion, visualizer=self.vis
            )

        elif self.planner_name == "DFS":
            if self.use_tree:
                return DFSPlanner_treebased(
                grid_map, motion_model=self.motion, visualizer=self.vis
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
        print("[NAV] Entering NavigationSystem.run()")

        grid_map = self.env["grid_map"]
        world_map = self.env["world_map"]
        robot = self.env["robot"]
        obstacles = self.env["obstacles"]

        # --------------------------------------------------
        # 1. Inflate obstacles (ONCE per run)
        # --------------------------------------------------
        #inflator = ObstacleInflator(robot.radius)
        #inflator.inflate(grid_map, world_map, obstacles)

        inflated_count = sum(
            grid_map.is_inflated(x, y)
            for y in range(grid_map.height)
            for x in range(grid_map.width)
        )

        print("[NAV] Inflated cells:", inflated_count)

        # --------------------------------------------------
        # 2. Validate start / goal
        # --------------------------------------------------

        if "start" in self.env and "goal" in self.env:
            start = self.env["start"]
            goal  = self.env["goal"]

            # sync robot for GUI consistency
            robot.sx, robot.sy = start
            robot.gx, robot.gy = goal
        else:
            self._validate_start_goal(grid_map, robot)
            start = (robot.sx, robot.sy)
            goal  = (robot.gx, robot.gy)

        print("[NAV] Using start/goal:", start, goal)

        # --------------------------------------------------
        # 3. Visualization setup
        # --------------------------------------------------
        if self.vis and self.visualize:
            self.vis.setup()
            self.vis.draw_start_goal(start, goal)
            self.vis.update()

        # --------------------------------------------------
        # 4. Planner creation
        # --------------------------------------------------
        planner = self._create_planner(grid_map)
        self.planner = planner

        # --------------------------------------------------
        # 5. Run planner
        # --------------------------------------------------
        path = planner.plan(start, goal)

        if path:
            print("[NAV] Path length:", len(path)-1)
        else:
            print("[NAV] No path found")

        if not path:
            print("[NavigationSystem] No path found")
            return None

        # --------------------------------------------------
        # 6. Final visualization + robot pose
        # --------------------------------------------------
        if self.vis and self.visualize:
            self.vis.draw_final_path(path)
            self.vis.update()

        gx, gy = path[-1]
        robot.x, robot.y = world_map.grid_to_world(gx, gy)

 
        print(
            "[NAV] Final world pose:",
            (robot.x, robot.y),
            "| resolution =", world_map.resolution
        )

        return path
