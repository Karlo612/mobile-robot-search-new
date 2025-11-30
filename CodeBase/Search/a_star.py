# astar.py

from Search.planner import Planner
import heapq
import math

class AStarPlanner(Planner):
    """
    A* search algorithm (grid-based).
    """
    def __init__(self, grid_map, motion_model, visualizer=None):
        super().__init__(grid_map, motion_model, visualizer)
        self.res = grid_map.resolution

    def heuristic(self, a, b):
        x1, y1 = a
        x2, y2 = b
        return (abs(x1 - x2) + abs(y1 - y2)) * self.res

    def get_neighbors(self, gx, gy):

        if self.motion_model == "4n":
            moves = [
                (1, 0), (-1, 0),
                (0, 1), (0, -1),
            ]
        else:  # default: "8n"
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
        sx, sy = start
        gx, gy = goal

        # Priority queue (cost + heuristic)
        open_set = []
        heapq.heappush(open_set, (0, (sx, sy)))

        came_from = {}
        g_cost = {(sx, sy): 0}

        closed = set()

        # Visualization
        vis = self.visualizer
        if vis:
            vis.draw_start_goal(start, goal)
            vis.update()

        while open_set:
            _, current = heapq.heappop(open_set)
            cx, cy = current

            if current in closed:
                continue

            closed.add(current)

            # Visualize explored cell + partial path
            if vis:
                vis.draw_explored(cx, cy)
                partial = self.build_partial_path(came_from, start, current)
                for i in range(len(partial) - 1):
                    x1, y1 = partial[i]
                    x2, y2 = partial[i + 1]
                    vis.draw_path_segment(x1, y1, x2, y2)
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
        # Diagonal move occurs in 8n
        if x1 != x2 and y1 != y2:
            return math.sqrt(2) * self.res
        # 4n move
        return 1 * self.res

    def build_partial_path(self, came_from, start, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
            if current == start:
                break
        path.reverse()
        return path

    # Reconstruct path from came_from table
    def reconstruct_path(self, came_from, start, goal):
        path = [goal]
        current = goal

        while current != start:
            current = came_from[current]
            path.append(current)

        path.reverse()
        return path