"""
A* Tree-Based Path Planner

This module implements the A* search algorithm using a tree-based approach.
Tree-based search allows revisiting the same grid cell multiple times if
reached through different paths. This is useful when the same cell might
be part of multiple paths with different costs. Each visit gets a unique
visit ID to track different paths through the same cell.
"""

from .planner import Planner
import heapq
import math
from collections import defaultdict

class AStarPlanner_treebased(Planner):
    """
    A* path planning algorithm using tree-based search.
    
    Unlike graph-based search, tree-based search allows the same cell to be
    visited multiple times if reached through different paths. This requires
    a 3D state space (x, y, visit_id) to track which specific visit of a cell
    we're referring to. This can be more memory efficient in some scenarios
    but may explore more nodes.
    """

    def __init__(self, grid_map, motion_model="8n", visualizer=None, debug=False):
        """
        Initialize the A* tree-based planner.
        
        Args:
            grid_map: GridMap object representing the search space
            motion_model: "4n" for 4-neighbor or "8n" for 8-neighbor movement
            visualizer: Optional visualizer for real-time search visualization
            debug: If True, print debug information during search (default: False)
        """
        super().__init__(grid_map, motion_model, visualizer)
        self.res = grid_map.resolution
        # Unique visit ID counter - needed because tree search can revisit cells
        self.counter = 0
        # Statistics for comparison and analysis
        self.expanded_count = 0
        self.expansion_map = {}
        self.debug = debug

    # ----------------------------------------
    # Heuristic
    # ----------------------------------------
    def heuristic(self, a, b):
        x1, y1 = a
        x2, y2 = b
        dx, dy = abs(x1 - x2), abs(y1 - y2)

        if self.motion_model == "4n":
            return (dx + dy) * self.res
        else:
            return (max(dx, dy) + (math.sqrt(2)-1) * min(dx, dy)) * self.res

    # ----------------------------------------
    # Neighbors
    # ----------------------------------------
    def get_neighbors(self, gx, gy):
        if self.motion_model == "4n":
            moves = [(1,0),(-1,0),(0,1),(0,-1)]
        else:
            moves = [
                (1,0),(-1,0),(0,1),(0,-1),
                (1,1),(1,-1),(-1,1),(-1,-1)
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

    # ----------------------------------------
    # Cost model
    # ----------------------------------------
    def cost(self, x1, y1, x2, y2):
        if x1 != x2 and y1 != y2:
            return math.sqrt(2) * self.res
        return 1 * self.res

    # ----------------------------------------
    # MAIN TREE-BASED A*
    # ----------------------------------------
    def plan(self, start, goal):
        """
        Execute the A* search algorithm using tree-based search.
        
        Tree-based search maintains a 3D state space (x, y, visit_id) where
        the same cell can be visited multiple times through different paths.
        This allows exploring alternative paths even after visiting a cell once.
        
        Args:
            start: Tuple (x, y) representing the start position
            goal: Tuple (x, y) representing the goal position
            
        Returns:
            List of (x, y) tuples representing the path, or None if no path exists
        """
        # Priority queue: (f_cost, g_cost, visit_id, state)
        OPEN = []

        # 3D parent tracking: parent[state][visit_id] = (parent_state, parent_vid)
        parent = defaultdict(dict)
        g_cost = defaultdict(dict)

        # Initialize with start node (visit_id = 0)
        h0 = self.heuristic(start, goal)
        g_cost[start][0] = 0.0
        heapq.heappush(OPEN, (h0, 0.0, 0, start))
        self.counter = 1  # Next visit ID will be 1

        vis = self.visualizer
        if vis:
            vis.draw_start_goal(start, goal)
            vis.update()

        while OPEN:

            f, g, vid, current = heapq.heappop(OPEN)
            cx, cy = current

            #generation of expansion count for report only
            self.expanded_count += 1
            #generation of heatmap counter  for report only
            self.expansion_map[(cx, cy)] = self.expansion_map.get((cx, cy), 0) + 1

            # Debug print
            if self.debug:
                print(
                    f"EXPAND {current}: g={g:.3f}, "
                    f"h={self.heuristic(current, goal):.3f}, f={f:.3f}"
                )

            # Goal reached → reconstruct path using correct vid chain
            if current == goal:
                return self.reconstruct_path_3d(parent, start, goal, vid)

            # Visualization
            if vis:
                vis.draw_explored(cx, cy)
                partial = self.build_partial_path_3d(parent, start, current, vid)
                for i in range(len(partial)-1):
                    x1, y1 = partial[i]
                    x2, y2 = partial[i+1]
                    vis.draw_path_segment(x1, y1, x2, y2)
                vis.update()

            # TREE SEARCH: No CLOSED set - we can revisit cells through different paths
            # Expand all neighbors and create new visit IDs for each
            for child in self.get_neighbors(cx, cy):
                nx, ny = child
                # Calculate costs for this new path
                ng = g + self.cost(cx, cy, nx, ny)
                nf = ng + self.heuristic(child, goal)

                # Assign unique visit ID to this visit of the child cell
                child_vid = self.counter
                self.counter += 1

                # Store parent link in 3D space: (parent_state, parent_vid)
                parent[child][child_vid] = (current, vid)
                g_cost[child][child_vid] = ng

                # Add to priority queue
                heapq.heappush(OPEN, (nf, ng, child_vid, child))

                if vis:
                    vis.draw_frontier(nx, ny)

        return None

    # ----------------------------------------
    # SAFE 3D reconstruction
    # ----------------------------------------
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

            # detect loops
            if (cur_state, cur_vid) in visited:
                print("[ERROR] Loop detected in 3D final path!")
                break

            visited.add((cur_state, cur_vid))

            # validate parent entry
            if cur_state not in parent or cur_vid not in parent[cur_state]:
                print("[ERROR] Missing parent link in 3D final path. Returning partial path.")
                break

            # follow parent
            prev_state, prev_vid = parent[cur_state][cur_vid]

            cur_state = prev_state
            cur_vid = prev_vid

        return list(reversed(path))
    # ----------------------------------------
    # Build partial path for visualization
    # ----------------------------------------
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

            # detect infinite loops
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