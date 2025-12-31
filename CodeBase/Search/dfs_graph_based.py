"""
Depth-First Search (DFS) Graph-Based Path Planner

This module implements the DFS algorithm using a graph-based approach.
Graph-based DFS maintains a CLOSED set to track visited nodes, preventing
revisiting and ensuring each state is explored at most once. DFS explores
deeply into the search space before backtracking, using a stack (LIFO) for
frontier management.
"""

from .planner import Planner

# ============================================================================
# DFSPlanner_graphbased Class - Graph-based Depth-First Search Implementation
# ============================================================================
class DFSPlanner_graphbased(Planner):
    """
    Depth-First Search (DFS) — GRAPH-BASED Implementation
    
    Key Characteristics:
      - Uses CLOSED set: Tracks visited states to prevent revisiting
      - No duplicates: Each state is explored at most once
      - Stack is LIFO: Uses Last-In-First-Out (stack) for frontier management
      - Parent pointers stored per-state: Dictionary maps state to parent state
    
    Key Difference from Tree-based DFS:
      Graph-based DFS maintains a closed set to track visited states,
      ensuring each state is explored only once. This prevents infinite loops
      and is more efficient for graph search problems.
    """

    def __init__(self, grid_map, motion_model="8n", visualizer=None, debug=False):
        """
        Initialize the graph-based DFS planner.
        
        Args:
            grid_map: The grid map representing the environment
            motion_model: "4n" for 4-connected or "8n" for 8-connected movement
            visualizer: Optional visualizer for displaying search progress
            debug: If True, print debug information during search (default: False)
        """
        super().__init__(grid_map, motion_model, visualizer)
        self.expanded_count = 0  # Total number of nodes expanded during search
        self.expansion_map = {}  # Tracks how many times each state was expanded
        self.debug = debug

    # ========================================================================
    # Neighbor Generation - Successor Function
    # ========================================================================
    def get_neighbors(self, gx, gy):
        """
        Generate valid neighboring states from the current position.
        This is the successor function that defines possible moves.
        
        Args:
            gx: Current x-coordinate
            gy: Current y-coordinate
        
        Yields:
            (nx, ny): Valid neighbor coordinates that can be reached
        
        Motion Models:
            - "4n": 4-connected (up, down, left, right)
            - "8n": 8-connected (includes diagonal moves)
        """

        # Define movement patterns based on motion model
        if self.motion_model == "4n":
            # 4-connected: only cardinal directions
            moves = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        else:  # 8-connected
            # 8-connected: cardinal + diagonal directions
            moves = [
                (1, 0), (-1, 0), (0, 1), (0, -1),  # Cardinal directions
                (1, 1), (1, -1), (-1, 1), (-1, -1)  # Diagonal directions
            ]

        # Check each potential move
        for dx, dy in moves:
            nx, ny = gx + dx, gy + dy

            # Skip invalid neighbors
            if not self.grid_map.is_inside(nx, ny):
                continue
            if self.grid_map.is_obstacle(nx, ny):
                continue
            if self.grid_map.is_inflated(nx, ny):
                if self.visualizer:
                    self.visualizer.draw_inflated(nx, ny)
                continue

            yield (nx, ny)

    # ========================================================================
    # MAIN DFS GRAPH-BASED SEARCH ALGORITHM
    # ========================================================================
    def plan(self, start, goal):
        """
        Main planning function implementing graph-based Depth-First Search.
        
        Algorithm Overview:
        1. Create initial node with start state
        2. Initialize stack (LIFO) with start node
        3. Initialize empty closed set
        4. While stack is not empty:
           a. Pop node from stack (last-in-first-out)
           b. Skip if already in closed set
           c. Add to closed set
           d. Check if it's the goal
           e. Generate all valid neighbors
           f. Push unvisited neighbors onto stack
        5. Return path if goal found, None otherwise
        
        Key Characteristics:
        - Uses closed set: Prevents revisiting states
        - Each state explored at most once
        - Uses stack (LIFO) for frontier management
        - Parent dictionary maps state to parent state for path reconstruction
        
        Args:
            start: Starting position as (x, y) tuple
            goal: Goal position as (x, y) tuple
        
        Returns:
            List of coordinates representing path from start to goal, or None if no path exists
        """

        vis = self.visualizer

        # Initialize frontier stack with start state
        OPEN = [(start, None)]     # Stack of (state, parent) tuples
        OPEN_SET = {start}  # Set to track states in OPEN for O(1) lookup
        parent = {}  # Dictionary mapping state -> parent state for path reconstruction

        # Initialize closed set to track visited states
        CLOSED = set()  # Set of visited states (prevents revisiting)

        if self.debug:
            print("\n[DFS] START =", start, "GOAL =", goal)

        # Initialize visualizer if available
        if vis:
            vis.draw_start_goal(start, goal)
            vis.update()

        # Main search loop
        while OPEN:
            if self.debug:
                print("[DFS] STACK (top last):", [s for s, _ in OPEN])
            
            # Pop the last node from stack (LIFO)
            v, p = OPEN.pop()
            OPEN_SET.remove(v)
            
            if self.debug:
                print("[DFS] EXPAND:", v)

            # Skip if already expanded (duplicate in stack)
            if v in CLOSED:
                continue

            # Store parent pointer for path reconstruction
            if p is not None:
                parent[v] = p

            # Extract coordinates from state
            cx, cy = v

            # Track expansion statistics
            self.expanded_count += 1
            self.expansion_map[(cx, cy)] = (
                self.expansion_map.get((cx, cy), 0) + 1
            )

            # Update visualization
            if vis:
                vis.draw_explored(cx, cy)

                # Visualize partial path from start to current node
                partial = self.build_partial_path(parent, start, v)
                for i in range(len(partial) - 1):
                    x1, y1 = partial[i]
                    x2, y2 = partial[i + 1]
                    vis.draw_path_segment(x1, y1, x2, y2)

                vis.update()

            # Check if goal reached
            if v == goal:
                return self.reconstruct_path(parent, start, goal)

            # Mark state as visited
            CLOSED.add(v)

            # Generate and process all valid neighbors
            for child in self.get_neighbors(cx, cy):
                # Only add child if not already visited or in frontier
                if child not in CLOSED and child not in OPEN_SET:
                    if self.debug:
                        print("    [DFS] PUSH:", child)
                    OPEN.append((child, v))
                    OPEN_SET.add(child)

                    if vis:
                        vis.draw_frontier(child[0], child[1])

        # No path found
        return None

    # ========================================================================
    # Path Reconstruction Helpers
    # ========================================================================
    def reconstruct_path(self, parent, start, goal):
        """
        Reconstruct the complete path from start to goal using the parent dictionary.
        Traces backwards from goal to start by following parent pointers.
        
        Args:
            parent: Dictionary mapping state -> parent state
            start: Starting position as (x, y) tuple
            goal: Goal position as (x, y) tuple
        
        Returns:
            List of coordinates representing the complete path from start to goal
        """
        path = [goal]  # Start with goal
        cur = goal
        # Trace backwards through parent pointers until reaching start
        while cur != start:
            cur = parent[cur]  # Move to parent state
            path.append(cur)  # Add to path
        path.reverse()  # Reverse to get path from start to goal
        return path

    def build_partial_path(self, parent, start, current):
        """
        Build a partial path from start to current state for visualization.
        Used during search to show the current path being explored.
        
        Args:
            parent: Dictionary mapping state -> parent state
            start: Starting position as (x, y) tuple
            current: Current position as (x, y) tuple
        
        Returns:
            List of coordinates representing path from start to current state
        """
        path = [current]  # Start with current state
        # Trace backwards through parent pointers until reaching start
        while current in parent:
            current = parent[current]  # Move to parent state
            path.append(current)  # Add to path
            if current == start:
                break  # Stop when we reach the start
        path.reverse()  # Reverse to get path from start to current
        return path

