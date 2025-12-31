"""
A* Graph-Based Path Planner

This module implements the A* search algorithm using a graph-based approach.
Graph-based means the algorithm tracks visited nodes to avoid revisiting them,
which ensures optimal paths and prevents infinite loops. A* uses a heuristic
function to guide the search toward the goal, making it more efficient than
blind search algorithms.
"""

from .planner import Planner
from heapdict import heapdict
import math

class Node:
    """
    Represents a node in the search graph.
    
    Each node stores its position, the cost to reach it (g), the estimated
    total cost (f = g + h), and a pointer to its parent for path reconstruction.
    """
    def __init__(self, state, g=0.0, f=0.0, parent=None):
        self.state = state      # (x, y) grid coordinates
        self.g = g              # Actual cost from start to this node
        self.f = f              # Total estimated cost: f = g + h (heuristic)
        self.parent = parent    # Pointer to parent node for path reconstruction

    def __repr__(self):
        return f"Node(state={self.state}, g={self.g}, f={self.f})"

class AStarPlanner_graphbased(Planner):
    """
    A* path planning algorithm using graph-based search.
    
    Graph-based search maintains a CLOSED set of visited nodes to prevent
    revisiting. This ensures optimal paths and prevents cycles. The algorithm
    uses a priority queue (heap) ordered by f-cost (g + h) to always expand
    the most promising node first.
    """

    def __init__(self, grid_map, motion_model="8n", visualizer=None):
        """
        Initialize the A* graph-based planner.
        
        Args:
            grid_map: GridMap object representing the search space
            motion_model: "4n" for 4-neighbor or "8n" for 8-neighbor movement
            visualizer: Optional visualizer for real-time search visualization
        """
        super().__init__(grid_map, motion_model, visualizer)
        self.res = grid_map.resolution
        # Statistics for comparison and analysis
        self.expanded_count = 0    # Total number of nodes expanded
        self.expansion_map = {}     # Maps each cell to how many times it was expanded

    def heuristic(self, a, b):
        """
        Calculate the heuristic (estimated cost) from point a to point b.
        
        The heuristic must be admissible (never overestimate) for A* to find
        optimal paths. For 4-neighbor movement, we use Manhattan distance.
        For 8-neighbor movement, we use octile distance which accounts for
        diagonal movement.
        
        Args:
            a: Start position (x, y)
            b: Goal position (x, y)
            
        Returns:
            Estimated cost from a to b (scaled by grid resolution)
        """
        x1, y1 = a
        x2, y2 = b
        dx, dy = abs(x1-x2), abs(y1-y2)

        if self.motion_model == "4n":
            # Manhattan distance: sum of horizontal and vertical distances
            return (dx + dy) * self.res
        else:  
            # Octile distance: accounts for diagonal movement in 8-neighbor grids
            # This is more accurate than Euclidean for grid-based pathfinding
            return (max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)) * self.res

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

    def cost(self, x1, y1, x2, y2):
        """
        Calculate the movement cost from cell (x1, y1) to (x2, y2).
        
        Diagonal moves cost more than straight moves because they cover
        a longer distance (√2 times the grid resolution).
        
        Args:
            x1, y1: Source cell coordinates
            x2, y2: Destination cell coordinates
            
        Returns:
            Movement cost scaled by grid resolution
        """
        # Diagonal movement costs √2 times more than straight movement
        if x1 != x2 and y1 != y2:
            return math.sqrt(2) * self.res
        # Straight movement (horizontal or vertical)
        return 1 * self.res

    def plan(self, start, goal):
        """
        Execute the A* search algorithm to find a path from start to goal.
        
        The algorithm maintains an OPEN set (priority queue) of nodes to explore,
        ordered by f-cost. It expands the most promising node first, adds its
        neighbors to OPEN if they haven't been visited or if a better path is found.
        Once the goal is reached, the path is reconstructed by following parent pointers.
        
        Args:
            start: Tuple (x, y) representing the start position
            goal: Tuple (x, y) representing the goal position
            
        Returns:
            List of (x, y) tuples representing the path, or None if no path exists
        """
        # Priority queue of nodes to explore, ordered by f-cost (g + h)
        OPEN = heapdict()
        # Set of nodes that have been fully explored
        CLOSED = set()
        # Dictionary mapping states to Node objects for path reconstruction
        NODES = {} 

        # Initialize with start node
        start_node = Node(start, g=0.0, f=self.heuristic(start, goal))
        OPEN[start] = start_node.f        
        NODES[start] = start_node

        vis = self.visualizer
        if vis:
            vis.draw_start_goal(start, goal)
            vis.update()

        while OPEN:
            # Pop the node with the lowest f-cost from the priority queue
            current_state, _ = OPEN.popitem()
            current = NODES[current_state]

            # Track statistics for analysis
            self.expanded_count += 1
            x, y = current_state
            self.expansion_map[(x, y)] = self.expansion_map.get((x, y), 0) + 1

            # Debug output showing search progress
            if self.visualizer:
                print(
                    f"EXPAND {current_state}: "
                    f"g={current.g:.3f}, "
                    f"h={self.heuristic(current_state, goal):.3f}, "
                    f"f={current.f:.3f}"
                )
            
            # Check if we've reached the goal
            if current_state == goal:
                return self.reconstruct_path(current)

            # Mark current node as explored
            CLOSED.add(current_state)

            cx, cy = current_state

            if vis:
                vis.draw_explored(cx, cy)

                # draw partial path from start → current
                partial = self.build_partial_path(current)
                for i in range(len(partial) - 1):
                    x1, y1 = partial[i]
                    x2, y2 = partial[i+1]
                    vis.draw_path_segment(x1, y1, x2, y2)

                vis.update()

            # Explore all valid neighbors
            for child in self.get_neighbors(cx, cy):
                v = child 
                
                # Skip if already fully explored (graph-based: no revisiting)
                if v in CLOSED:
                    continue

                # Calculate new cost and f-value for this path
                new_g = current.g + self.cost(cx, cy, v[0], v[1])
                new_f = new_g + self.heuristic(v, goal)
            
                if v not in OPEN:
                    # New node discovered - add it to OPEN set
                    new_node = Node(v, g=new_g, f=new_f, parent=current)
                    NODES[v] = new_node
                    OPEN[v] = new_f

                    if vis:
                        vis.draw_frontier(v[0], v[1])

                else:
                    # Node already in OPEN - check if we found a better path
                    old_f = OPEN[v]
                    if new_f < old_f:
                        # Update node with better path (lower cost)
                        node = NODES[v]
                        node.f = new_f
                        node.g = new_g
                        node.parent = current

                        # Update priority in the queue
                        OPEN[v] = new_f  

                        if vis:
                            vis.draw_frontier(child[0], child[1])
        
        # No path found if we exhaust all possibilities
        return None

    def reconstruct_path(self, node):
        """
        Reconstruct the full path from goal back to start.
        
        Follows parent pointers from the goal node all the way back to the start,
        then reverses the list to get the path in forward order.
        
        Args:
            node: The goal node (or any node in the path)
            
        Returns:
            List of (x, y) tuples representing the complete path
        """
        path = []
        while node is not None:
            path.append(node.state)
            node = node.parent
        return list(reversed(path))

    def build_partial_path(self, node):
        """
        Build a partial path from the given node back to start.
        
        Used for visualization during search to show the current best path
        to any node being explored.
        
        Args:
            node: The node to build the path from
            
        Returns:
            List of (x, y) tuples representing the partial path
        """
        path = []
        while node is not None:
            path.append(node.state)
            node = node.parent
        return list(reversed(path))