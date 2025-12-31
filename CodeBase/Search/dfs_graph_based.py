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

    def __init__(self, grid_map, motion_model="8n", visualizer=None):
        """
        Initialize the graph-based DFS planner.
        
        Args:
            grid_map: The grid map representing the environment
            motion_model: "4n" for 4-connected or "8n" for 8-connected movement
            visualizer: Optional visualizer for displaying search progress
        """
        super().__init__(grid_map, motion_model, visualizer)
        self.expanded_count = 0  # Total number of nodes expanded during search
        self.expansion_map = {}  # Tracks how many times each state was expanded

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
            nx, ny = gx + dx, gy + dy  # Calculate neighbor coordinates

            # Validation checks: skip invalid neighbors
            if not self.grid_map.is_inside(nx, ny):
                # Neighbor is outside the grid boundaries
                continue
            if self.grid_map.is_obstacle(nx, ny):
                # Neighbor is an obstacle (blocked cell)
                continue
            if self.grid_map.is_inflated(nx, ny):
                # Neighbor is in inflated/unsafe region
                if self.visualizer:
                    self.visualizer.draw_inflated(nx, ny)
                continue

            # Neighbor is valid - yield it for exploration
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

        # Steps 1-2: Initialize open set (stack) with start state
        # 1: node ← with s as state
        # 2: O ← node (LIFO stack)
        OPEN = [(start, None)]     # Stack of (state, parent) tuples
        parent = {}  # Dictionary mapping state -> parent state for path reconstruction

        # Step 3: Initialize closed set (empty set to track visited states)
        # 3: C ← ∅
        CLOSED = set()  # Set of visited states (prevents revisiting)

        print("\n[DFS] START =", start, "GOAL =", goal)

        # Initialize visualizer if available
        if vis:
            vis.draw_start_goal(start, goal)
            vis.update()

        # Step 4: Main search loop - continue while stack is not empty
        # 4: while O != ∅ do
        while OPEN:
            print("[DFS] STACK (top last):", [s for s, _ in OPEN])
            
            # Step 5: Pop the last node from stack (LIFO behavior)
            # 5: parent ← last node in O
            v, p = OPEN.pop()  # Remove and return last element (stack pop)
            print("[DFS] EXPAND:", v)

            # Skip if already expanded (duplicate in stack)
            # This can happen if the same state was added multiple times before being expanded
            if v in CLOSED:
                continue

            # Store parent pointer for path reconstruction
            # Maps current state to its parent state
            if p is not None:
                parent[v] = p

            # Step 6: Extract coordinates from state
            # 6: v ← parent.state
            cx, cy = v

            # Track expansion statistics for analysis
            # Bookkeeping (benchmarking only)
            self.expanded_count += 1  # Increment total expansion count
            self.expansion_map[(cx, cy)] = (
                self.expansion_map.get((cx, cy), 0) + 1  # Count expansions per state
            )

            # Visualization: Show explored state and current path being explored
            # ---- VISUALIZATION (CORRECT LOCATION) ----
            if vis:
                vis.draw_explored(cx, cy)  # Mark current state as explored

                # Build and visualize partial path from start to current node
                partial = self.build_partial_path(parent, start, v)
                for i in range(len(partial) - 1):
                    x1, y1 = partial[i]
                    x2, y2 = partial[i + 1]
                    vis.draw_path_segment(x1, y1, x2, y2)  # Draw path segments

                vis.update()  # Update visualization display

            # Step 7: Check if current state is the goal
            # 7: if v == g then return parent
            if v == goal:
                # Goal reached! Reconstruct and return the path
                return self.reconstruct_path(parent, start, goal)

            # Step 10: Add current state to closed set (mark as visited)
            # 10: C ← C ∪ {v}
            CLOSED.add(v)  # Mark state as visited to prevent revisiting

            # Step 11: Generate and process all valid neighbors
            # 11: for child in successor(v)
            for child in self.get_neighbors(cx, cy):

                # Step 13: Only add child if not already visited or in frontier
                # 13: if child not in C and child not in O
                # Check if child is not in closed set and not already in open set
                if child not in CLOSED and child not in [n for n, _ in OPEN]:

                    # Step 14: Add child to stack with current state as parent
                    # 14: add child to O
                    print("    [DFS] PUSH:", child)
                    OPEN.append((child, v))  # Push to stack (adds to end, will be popped last)

                    # Visualization: Mark neighbor as frontier (to be explored)
                    if vis:
                        vis.draw_frontier(child[0], child[1])

        # Step 18: No path found - stack exhausted without reaching goal
        # 18: return failure
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
