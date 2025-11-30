# bfs.py

from collections import deque
from Search.planner import Planner

class BFS_Planner(Planner):
    """
    Breadth-First Search planner operating purely on grid coordinates.
    """

    def get_neighbors(self, gx, gy):
        """
        TODO:
        - Return valid neighbor cells (up, down, left, right)
        - Check grid_map.is_inside()
        - Check grid_map.get_cell() != obstacle
        """
        pass

    def plan(self, start, goal):
        """
        TODO:
        - BFS search:
            * Use queue
            * Track visited cells
            * Track parents
            * Stop when goal reached
            * animate visulaization
            vis.draw_frontier(nx, ny)
            vis.update()

            vis.draw_explored(cx, cy)
            vis.update()

            vis.draw_blocked(nx, ny)
            vis.update()

            vis.draw_path_segment(...)
        -   Return path as list of (gx, gy)
        """


        pass