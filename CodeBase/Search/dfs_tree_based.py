"""
Depth-First Search (DFS) Tree-Based Path Planner

This module implements the DFS algorithm using a tree-based approach.
Tree-based DFS does not maintain a CLOSED set, allowing the same cell to be
visited multiple times if reached through different paths. This can be useful
when exploring alternative paths, but may lead to more node expansions.
DFS explores deeply before backtracking, using a stack (LIFO) for frontier management.
"""

from .planner import Planner

class DFSNode:
    """
    Represents a node in the DFS search tree.
    
    Each node stores its state (position) and a pointer to its parent node.
    In tree-based search, multiple nodes can have the same state but different
    parent paths, so we track nodes rather than just states.
    """
    def __init__(self, state, parent=None):
        self.state = state      # (x, y) grid coordinates
        self.parent = parent    # Parent DFSNode or None for root

class DFSPlanner_treebased(Planner):
    """
    Depth-First Search planner using tree-based approach.
    
    Key characteristics:
    - No CLOSED set: Allows revisiting the same state through different paths
    - Allows duplicates: Same state can appear multiple times in the search tree
    - Stack is LIFO: Uses Last-In-First-Out (stack) for frontier management
    - Parent pointers stored per-node: Each node has its own parent, not per-state
    """

    def __init__(self, grid_map, motion_model="8n", visualizer=None, max_expansions=50000):
        """
        Initialize the DFS tree-based planner.
        
        Args:
            grid_map: GridMap object representing the search space
            motion_model: "4n" for 4-neighbor or "8n" for 8-neighbor movement
            visualizer: Optional visualizer for real-time search visualization
            max_expansions: Maximum number of node expansions before stopping (default: 50000)
        """
        super().__init__(grid_map, motion_model, visualizer)
        # Statistics for comparison and analysis
        self.expanded_count = 0
        self.expansion_map = {}
        self.max_expansions = max_expansions

    def get_neighbors(self, gx, gy):
        """
        Generate valid neighboring cells from the current position.
        
        Returns only neighbors that are within bounds, not obstacles, and not
        blocked by inflated obstacles. The set of neighbors depends on the
        motion model (4-neighbor or 8-neighbor).
        
        Args:
            gx: Current x coordinate
            gy: Current y coordinate
            
        Yields:
            Tuple (nx, ny) for each valid neighbor
        """
        # Define movement patterns based on motion model
        if self.motion_model == "4n":
            # 4-neighbor: up, down, left, right only
            moves = [(1,0), (-1,0), (0,1), (0,-1)]
        else:
            # 8-neighbor: includes diagonal movements
            moves = [
                (1,0), (-1,0), (0,1), (0,-1),
                (1,1), (1,-1), (-1,1), (-1,-1)
            ]

        for dx, dy in moves:
            nx, ny = gx + dx, gy + dy

            # Skip if outside grid boundaries
            if not self.grid_map.is_inside(nx, ny):
                continue
            # Skip if directly on an obstacle
            if self.grid_map.is_obstacle(nx, ny):
                continue
            # Skip if blocked by inflated obstacle (robot too large to fit)
            if self.grid_map.is_inflated(nx, ny):
                if self.visualizer:
                    self.visualizer.draw_inflated(nx, ny)
                continue

            yield (nx, ny)

    # ----------------------------
    # Partial path for visualization
    # ----------------------------
    def build_partial_path(self, node: DFSNode):
        """
        Build a partial path from the given node back to the root.
        
        Follows parent pointers from the node back to the start. Includes
        loop detection since tree search can create cycles through state
        repetition (same state reached through different paths).
        
        Args:
            node: DFSNode to build the path from
            
        Returns:
            List of (x, y) tuples representing the partial path
        """
        path = []
        seen = set()  # Track node identities (memory addresses), not states

        cur = node
        while cur is not None:
            nid = id(cur)
            # Detect loops by checking if we've seen this node object before
            if nid in seen:
                print("[DFS-TREE] Loop detected in partial path reconstruction; stopping.")
                break
            seen.add(nid)

            path.append(cur.state)
            cur = cur.parent

        return list(reversed(path))

    def reconstruct_path(self, node: DFSNode):
        """
        Reconstruct the full path from goal node back to start.
        
        Same as build_partial_path, but used when the goal is reached
        to get the complete solution path.
        
        Args:
            node: The goal DFSNode
            
        Returns:
            List of (x, y) tuples representing the complete path
        """
        return self.build_partial_path(node)

    # ----------------------------
    # MAIN DFS TREE SEARCH (matches your pseudocode)
    # ----------------------------
    def plan(self, start, goal):
        """
        Execute tree-based DFS to find a path from start to goal.
        
        Tree-based DFS does not maintain a CLOSED set, so the same state can
        be visited multiple times through different paths. This uses a stack
        (LIFO) to explore deeply before backtracking.
        
        Args:
            start: Tuple (x, y) representing the start position
            goal: Tuple (x, y) representing the goal position
            
        Returns:
            List of (x, y) tuples representing the path, or None if no path exists
        """
        print(f"\n[DFS-TREE] START={start} GOAL={goal}")

        # Initialize with start node (root of search tree)
        node = DFSNode(start, parent=None)

        # Early exit if start equals goal
        if start == goal:
            return [start]

        # Initialize stack (LIFO) with start node
        O = [node]  # Stack of DFSNode objects

        vis = self.visualizer
        if vis:
            vis.draw_start_goal(start, goal)
            vis.update()

        # Main search loop - continue while stack is not empty
        while O:
            # Check expansion limit to prevent infinite loops
            if self.expanded_count >= self.max_expansions:
                print(f"[DFS-TREE] Expansion limit reached: {self.max_expansions}")
                return None  # Indicates limit reached, not necessarily no path
            
            # Pop the last node from stack (LIFO behavior)
            parent_node = O.pop()
            v = parent_node.state

            print(f"[DFS-TREE] EXPAND {v} | stack_size={len(O)}")

            # Track expansion statistics
            self.expanded_count += 1
            self.expansion_map[v] = self.expansion_map.get(v, 0) + 1

            # Visualization: show explored state and current path
            if vis:
                vis.draw_explored(v[0], v[1])

                # Build and visualize partial path from start to current node
                partial = self.build_partial_path(parent_node)
                for i in range(len(partial) - 1):
                    x1, y1 = partial[i]
                    x2, y2 = partial[i + 1]
                    vis.draw_path_segment(x1, y1, x2, y2)

                vis.update()

            # Check if we've reached the goal
            if v == goal:
                return self.reconstruct_path(parent_node)

            # Expand all neighbors - tree search allows revisiting
            for child_state in self.get_neighbors(v[0], v[1]):
                # Create new node for this child (even if state was seen before)
                child_node = DFSNode(child_state, parent=parent_node)
                O.append(child_node)

                print(f"    [DFS-TREE] PUSH {child_state}")

                if vis:
                    vis.draw_frontier(child_state[0], child_state[1])

        # No path found - stack exhausted without reaching goal
        return None
