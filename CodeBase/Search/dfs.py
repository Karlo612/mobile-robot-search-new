from .planner import Planner
import math

class DFSPlanner(Planner):
    """
    Depth First Search Planner that extends the base Planner class.
    """
    def __init__(self, grid_map, motion_model="8n", visualizer=None):
        super().__init__(grid_map, motion_model, visualizer)
        self.res = grid_map.resolution
    
    def cost(self, x1, y1, x2, y2):
        if x1 != x2 and y1 != y2:
            return math.sqrt(2) * self.res
        return 1 * self.res
    
    def get_neighbors(self, gx, gy):

        if self.motion_model == "4n":
            moves = [(1,0), (-1,0), (0,1), (0,-1)]
        else:
            moves = [
                (1,0), (-1,0), (0,1), (0,-1),
                (1,1), (1,-1), (-1,1), (-1,-1)
            ]

        for dx, dy in moves:
            nx, ny = gx + dx, gy + dy

            if not self.grid_map.is_inside(nx, ny):
                continue
            if self.grid_map.is_obstacle(nx, ny):
                continue
            if self.grid_map.is_inflated(nx, ny):
                if self.visualizer:
                    self.visualizer.draw_inflated(nx, ny)
                continue

            yield (nx, ny)
    
    def plan(self, start, goal):
        """
        Perform DFS to find a path from start to goal.
        """
        stack = [(start, [start])]  # stack of (current_position, path)
        visited = set()  # to keep track of visited nodes

        while stack:
            (current_position, path) = stack.pop()
            if current_position in visited:
                continue

            visited.add(current_position)

            # Check if we reached the goal
            if current_position == goal:
                return self.reconstruct_path(parent, start, goal)  # return the path to the goal

            # Get neighbors (assuming grid_map has a method to get valid neighbors)
            for neighbor in self.grid_map.get_neighbors(current_position):
                if neighbor not in visited:
                    stack.append((neighbor, path + [neighbor]))

        return None  # return None if no path is found
    
    def reconstruct_path(self, parent, start, goal):
        path = [goal]
        cur = goal
        while cur != start:
            cur = parent[cur]
            path.append(cur)
        path.reverse()
        return path

    def build_partial_path(self, parent, start, current):
        path = [current]
        while current in parent:
            current = parent[current]
            path.append(current)
            if current == start:
                break
        path.reverse()
        return path