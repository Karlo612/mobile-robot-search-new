from CodeBase.Search.planner import Planner

class DFSPlanner(Planner):
    """
    Depth First Search Planner that extends the base Planner class.
    """

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
                return path  # return the path to the goal

            # Get neighbors (assuming grid_map has a method to get valid neighbors)
            for neighbor in self.grid_map.get_neighbors(current_position):
                if neighbor not in visited:
                    stack.append((neighbor, path + [neighbor]))

        return None  # return None if no path is found