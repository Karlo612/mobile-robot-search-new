"""
Breadth-First Search (BFS) Path Planners

This module implements BFS path planning in both graph-based and tree-based modes.
BFS explores all nodes at the current depth before moving to the next level,
guaranteeing the shortest path in unweighted graphs. It uses a queue (FIFO)
instead of a priority queue.
"""

from .planner import Planner
from collections import deque, defaultdict

class BFSPlanner_graphbased(Planner):
    """
    Breadth-First Search planner using graph-based approach.
    
    Graph-based BFS maintains a CLOSED set to avoid revisiting nodes, ensuring
    we find the shortest path in terms of number of steps. The algorithm explores
    level by level, always expanding the shallowest unexpanded node first.
    """


    def __init__(self, grid_map, motion_model, visualizer=None):
        """
        Initialize the BFS graph-based planner.
        
        Args:
            grid_map: GridMap object representing the search space
            motion_model: "4n" for 4-neighbor or "8n" for 8-neighbor movement
            visualizer: Optional visualizer for real-time search visualization
        """
        super().__init__(grid_map, motion_model, visualizer)
        # Statistics for comparison and analysis
        self.expanded_count = 0
        self.expansion_map = {}


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
            if not self.grid_map.is_inside(nx, ny):
                continue
            if self.grid_map.is_obstacle(nx, ny):
                continue
            if self.grid_map.is_inflated(nx, ny):
                
                if self.visualizer:
                    self.visualizer.draw_inflated(nx, ny)
                continue         

            yield nx, ny

    def plan(self, start, goal):
        """
        Execute BFS to find a path from start to goal.
        
        BFS explores nodes level by level, guaranteeing the shortest path
        in terms of number of steps. Uses a queue (FIFO) to process nodes
        in the order they were discovered.
        
        Args:
            start: Tuple (x, y) representing the start position
            goal: Tuple (x, y) representing the goal position
            
        Returns:
            List of (x, y) tuples representing the path, or None if no path exists
        """
        sx, sy = start
        gx, gy = goal

        vis = self.visualizer
        if vis:
            vis.draw_start_goal(start, goal)
            vis.update()

        # Early exit if start equals goal
        if start == goal:
            return [start]

        # Queue for nodes to explore (FIFO - first in, first out)
        open_queue = deque([start])
        in_open = set([start])  # Fast lookup to check if node is in queue

        # Set of nodes that have been fully explored
        closed = set()
        # Dictionary to reconstruct path: parent[child] = parent_node
        parent = {}

        while open_queue:

            current = open_queue.popleft()
            cx, cy = current      

            #generation of expansion count for report only
            self.expanded_count += 1
            #generation of heatmap counter  for report only
            self.expansion_map[(cx, cy)] = self.expansion_map.get((cx, cy), 0) + 1

            closed.add(current)

            if vis:
                if self.grid_map.is_inflated(cx, cy):
                    vis.draw_inflated(cx, cy)
                else:
                    vis.draw_explored(cx, cy)

                partial = self.build_partial_path(parent, start, current)
                for i in range(len(partial) - 1):
                    x1, y1 = partial[i]
                    x2, y2 = partial[i + 1]
                    vis.draw_path_segment(x1, y1, x2, y2)
                vis.update()


            # Explore all neighbors
            for nx, ny in self.get_neighbors(cx, cy):
                v = (nx, ny)

                # Only process if not already explored and not in queue
                if (v not in closed) and (v not in in_open):
                    # Check if we reached the goal
                    if v == goal:
                        parent[v] = current
                        return self.reconstruct_path(parent, start, v)
                    # Add to queue for exploration
                    open_queue.append(v)
                    in_open.add(v)
                    parent[v] = current

                    if vis:
                        if self.grid_map.is_inflated(nx, ny):
                            vis.draw_inflated(nx, ny)
                        else:
                            vis.draw_frontier(nx, ny)
        return None  
    
    def build_partial_path(self, parent, start, current):
        """
        Build a partial path from current node back to start for visualization.
        
        Args:
            parent: Parent dictionary
            start: Start position
            current: Current position
            
        Returns:
            List of (x, y) tuples representing the partial path
        """
        path = [current]
        while current in parent:
            current = parent[current]
            path.append(current)
            if current == start:
                break
        path.reverse()
        return path

    def reconstruct_path(self, parent, start, goal):
        """
        Reconstruct the full path from goal back to start.
        
        Follows parent pointers from goal to start, then reverses to get
        the path in forward order.
        
        Args:
            parent: Parent dictionary
            start: Start position
            goal: Goal position
            
        Returns:
            List of (x, y) tuples representing the complete path
        """
        path = [goal]
        current = goal
        while current != start:
            current = parent[current]
            path.append(current)
        path.reverse()
        return path
    
    
class BFSPlanner_treesearch(Planner):
    """
    Breadth-First Search planner using tree-based approach.
    
    Tree-based BFS allows revisiting the same cell multiple times if reached
    through different paths. Uses a 3D state space (x, y, visit_id) to track
    different visits of the same cell.
    """

    def __init__(self, grid_map, motion_model, visualizer=None, max_expansions=50000):
        """
        Initialize the BFS tree-based planner.
        
        Args:
            grid_map: GridMap object representing the search space
            motion_model: "4n" for 4-neighbor or "8n" for 8-neighbor movement
            visualizer: Optional visualizer for real-time search visualization
            max_expansions: Maximum number of node expansions before stopping (default: 50000)
        """
        super().__init__(grid_map, motion_model, visualizer)
        # Unique visit ID counter for tree-based search
        self.counter = 0
        # Statistics for comparison and analysis
        self.expanded_count = 0
        self.expansion_map = {}
        self.max_expansions = max_expansions

    def get_neighbors(self, gx, gy):
        if self.motion_model == "4n":
            moves = [(1,0), (-1,0), (0,1), (0,-1)]
        else:  # default: "8n"
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

            yield nx, ny

    def plan(self, start, goal):
        """
        Execute tree-based BFS to find a path from start to goal.
        
        Tree-based BFS allows revisiting cells through different paths.
        Uses a 3D state space (x, y, visit_id) to track different visits.
        
        Args:
            start: Tuple (x, y) representing the start position
            goal: Tuple (x, y) representing the goal position
            
        Returns:
            List of (x, y) tuples representing the path, or None if no path exists
        """
        vis = self.visualizer
        if vis:
            vis.draw_start_goal(start, goal)
            vis.update()

        # 3D parent tracking: parent[state][visit_id] = (parent_state, parent_vid)
        parent = defaultdict(dict)
        # Queue of (state, visit_id) tuples
        queue = deque()

        # Initialize with start node (visit_id = 0)
        queue.append((start, 0))
        parent[start][0] = (None, None)
        self.counter = 1

        while queue:
            # Check expansion limit to prevent infinite loops
            if self.expanded_count >= self.max_expansions:
                print(f"[BFS-TREE] Expansion limit reached: {self.max_expansions}")
                return None  # Indicates limit reached, not necessarily no path

            (cur, vid) = queue.popleft()
            cx, cy = cur

            #generation of expansion count for report only
            self.expanded_count += 1
            #generation of heatmap counter  for report only
            self.expansion_map[(cx, cy)] = self.expansion_map.get((cx, cy), 0) + 1

            # Debug print
            if vis:
                parent_vid = parent[cur][vid][1]
                print(f"EXPAND {cur}: vid={vid}, parent_vid={parent_vid}, queue_size={len(queue)}")

            if vis:
                vis.draw_explored(cx, cy)
                partial = self.build_partial_path_3d(parent, start, cur, vid)
                for i in range(len(partial) - 1):
                    x1, y1 = partial[i]
                    x2, y2 = partial[i + 1]
                    vis.draw_path_segment(x1, y1, x2, y2)
                vis.update()

            if cur == goal:
                return self.reconstruct_path_3d(parent, start, goal, vid)

            # Expand all neighbors - tree search allows revisiting
            for child in self.get_neighbors(cx, cy):
                # Assign unique visit ID to this visit of the child cell
                child_vid = self.counter
                self.counter += 1

                # Store parent link in 3D space
                parent[child][child_vid] = (cur, vid)
                queue.append((child, child_vid))

                if vis:
                    vis.draw_frontier(child[0], child[1])

        return None
    
    def reconstruct_path_3d(self, parent, start, goal, vid):
        """
        Reconstruct the full path from goal back to start using 3D parent pointers.
        
        Since tree-based search uses visit IDs, we need to follow both the state
        and visit ID through the parent chain. Includes loop detection for safety.
        
        Args:
            parent: 3D parent dictionary
            start: Start position
            goal: Goal position
            vid: Visit ID of the goal node
            
        Returns:
            List of (x, y) tuples representing the complete path
        """
        path = []
        cur_state = goal
        cur_vid = vid

        visited = set()

        while True:
            path.append(cur_state)

            if cur_state == start:
                break

            if (cur_state, cur_vid) in visited:
                print("[ERROR] Loop detected in 3D final path!")
                break

            visited.add((cur_state, cur_vid))


            if cur_state not in parent or cur_vid not in parent[cur_state]:
                print("[ERROR] Missing parent link in 3D final path. Returning partial path.")
                break

            prev_state, prev_vid = parent[cur_state][cur_vid]

            cur_state = prev_state
            cur_vid = prev_vid

        return list(reversed(path))
    
    def build_partial_path_3d(self, parent, start, state, vid):
        """
        Build a partial path from the given state back to start for visualization.
        
        Used during search to show the current best path to any node being explored.
        Includes loop detection to prevent infinite loops in visualization.
        
        Args:
            parent: 3D parent dictionary
            start: Start position
            state: Current state to build path from
            vid: Visit ID of the current state
            
        Returns:
            List of (x, y) tuples representing the partial path
        """
        path = []
        cur_state = state
        cur_vid = vid
        
        visited = set()

        while True:
            path.append(cur_state)

            if cur_state == start:
                break


            if (cur_state, cur_vid) in visited:
                print("[WARN] Loop in partial 3D path — breaking early.")
                break

            visited.add((cur_state, cur_vid))

            if cur_state not in parent or cur_vid not in parent[cur_state]:
                print("[WARN] Missing parent in partial 3D path — breaking early.")
                break

            prev_state, prev_vid = parent[cur_state][cur_vid]

            cur_state = prev_state
            cur_vid = prev_vid

        return list(reversed(path))

