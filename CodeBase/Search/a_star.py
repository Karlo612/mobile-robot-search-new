# astar.py

from Search.planner import Planner
import heapq
import math

class AStarPlanner(Planner):
    """
    A* search algorithm (grid-based).
    """
    def __init__(self, grid_map, visualizer=None):
        super().__init__(grid_map, visualizer)
        
    def heuristic(self, a, b):
        """
        Heuristic: Manhattan distance (fast, grid-friendly).
        a = (gx1, gy1)
        b = (gx2, gy2)
        """
        x1, y1 = a
        x2, y2 = b
        return abs(x1 - x2) + abs(y1 - y2)

    def get_neighbors(self, gx, gy):
        """
        8-direction moves. Change to 4 if needed.
        """
        moves = [
            (1, 0), (-1, 0),
            (0, 1), (0, -1),
            (1, 1), (1, -1),
            (-1, 1), (-1, -1),
        ]

        for dx, dy in moves:
            nx, ny = gx + dx, gy + dy
            if self.grid_map.is_inside(nx, ny) and not self.grid_map.is_obstacle(nx, ny):
                yield nx, ny

    def plan(self, start, goal):
        """
        Standard A* search with priority queue.
        start = (sx, sy)
        goal  = (gx, gy)
        """

        sx, sy = start
        gx, gy = goal

        # Priority queue (cost + heuristic)
        open_set = []
        heapq.heappush(open_set, (0, (sx, sy)))

        came_from = {}
        g_cost = { (sx, sy): 0 }

        closed = set()

        # Visualization
        vis = self.visualizer
        if vis:
            vis.draw_start_goal(start, goal)
            vis.update()

        # -----------------------------
        # Main A* Loop
        # -----------------------------
        while open_set:
            _, current = heapq.heappop(open_set)
            cx, cy = current

            if current in closed:
                continue

            closed.add(current)

            # Visualize explored cell
            if vis:
                vis.draw_explored(cx, cy)
                vis.update()

            # Goal reached
            if current == goal:
                return self.reconstruct_path(came_from, start, goal)

            # Explore neighbors
            for nx, ny in self.get_neighbors(cx, cy):

                if (nx, ny) in closed:
                    continue

                new_cost = g_cost[current] + self.cost(cx, cy, nx, ny)

                if (nx, ny) not in g_cost or new_cost < g_cost[(nx, ny)]:
                    g_cost[(nx, ny)] = new_cost
                    priority = new_cost + self.heuristic((nx, ny), goal)
                    heapq.heappush(open_set, (priority, (nx, ny)))
                    came_from[(nx, ny)] = (cx, cy)

                    if vis:
                        vis.draw_frontier(nx, ny)

        return None  # No path found

    # Cost function for movement
    def cost(self, x1, y1, x2, y2):
        # Diagonals = sqrt(2), straight = 1
        if x1 != x2 and y1 != y2:
            return math.sqrt(2)
        return 1

    # Reconstruct path from came_from table
    def reconstruct_path(self, came_from, start, goal):
        path = [goal]
        current = goal

        while current != start:
            current = came_from[current]
            path.append(current)

        path.reverse()
        return path