# navigation_system.py
from Environment.map_loader import MapLoader
from Environment.grid_map import GridMap
from Environment.inflator import ObstacleInflator
from Environment.mobile_robot import MobileRobot
from Environment.world_map import WorldMap
from Search.a_star import AStarPlanner
from Visualization.visualizer import Visualizer

class NavigationSystem:

    def __init__(self, config):

        self.config = config

        # reading Map onfig
        self.map_path = config["map_file"]
        self.grid_resolution = config["resolution"]
        self.grid_origin = config["origin"]

        # reading Robot config
        self.robot_radius = config["robot_radius"]

        # reading Start/Goal in GRID coordinates
        self.start_grid = config["start_grid"]
        self.goal_grid = config["goal_grid"]

        # reading Planner & visualization settings
        self.motion_type = config.get("motion", "8n")
        self.search_type = config["planner"]
        self.visualize_search = config["visualize_search"]
                

    def setup_environment(self):

        #Create MapLoder
        map_loader = MapLoader()

        #Load map (GridMap + obstacle list)
        grid_array, obstacles = map_loader.load(self.map_path)

        #Create gridMap using resolution from the grid_array
        grid_map = GridMap(grid_array, self.grid_resolution)     

        #Create worldmap - real coordinate 
        world_map = WorldMap(self.grid_origin, self.grid_resolution)   

        #Create Robot object
        robot= MobileRobot(self.robot_radius,self.start_grid,self.goal_grid)       

        robot.attach_maps(grid_map, world_map)

        return grid_map, world_map, robot , obstacles
      

    def execute_planner(self, planner, robot, vis):
        start = (robot.sx, robot.sy)
        goal = (robot.gx, robot.gy)

        # Run planner
        path = planner.plan(start, goal)

        # Failure case
        if not path:
            print("No path found.")
            if vis:
                vis.show()
            return None

        # Draw final path
        if vis:
            vis.draw_final_path(path)
            vis.update()
            vis.show()

        return path


    def run(self):

        # Setup environment (load map, obstacles, robot)
        grid_map, world_map, robot, obstacles = self.setup_environment()

        #Inflate obstacles
        inflator = ObstacleInflator(robot.radius)
        inflator.inflate(grid_map,world_map,obstacles)

        # Initialize visualization
        vis = None
        if self.visualize_search:
            vis = Visualizer(grid_map,cell_size=self.grid_resolution)
            vis.setup()
            vis.draw_start_goal(
                start=(robot.sx, robot.sy),
                goal=(robot.gx, robot.gy),
            )

        #setup planner
        planner_name = self.search_type.upper()

        if planner_name == "ASTAR":
            planner = AStarPlanner(grid_map,motion_model=self.motion_type, visualizer=vis)
            path = self.execute_planner(planner, robot, vis)
        else:
            raise NotImplementedError(f"Planner '{self.search_type}' not implemented yet.")

        if path:
            gx, gy = path[-1]
            robot.x, robot.y = world_map.grid_to_world(gx, gy)


