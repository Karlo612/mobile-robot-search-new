# bfs.py

from .planner import Planner
from collections import deque, defaultdict

class BFSPlanner_graphbased(Planner):
    """
    Breadth-First Search (BFS) planner strictly following the provided pseudocode.
    """

    def __init__(self, grid_map, motion_model, visualizer=None):
        super().__init__(grid_map, motion_model, visualizer)
        self.expanded_count = 0
        # BFS does not use heuristic or movement costs

    def get_neighbors(self, gx, gy):
        # Same neighbor generation as in A* (respects 4n or 8n)
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
        sx, sy = start
        gx, gy = goal

        vis = self.visualizer
        if vis:
            vis.draw_start_goal(start, goal)
            vis.update()

        #s == g then return node
        if start == goal:
            return [start]

        # O ← node, an FIFO queue
        open_queue = deque([start])
        in_open = set([start])   # tracks membership in O

        # C ← ∅
        closed = set()

        # To reconstruct the path
        parent = {}

        # while O != ∅ do
        while open_queue:
            #  parent ← the first node in O
            current = open_queue.popleft()
            cx, cy = current
            self.expanded_count += 1

            # C ← parent.state 
            closed.add(current)

            # Visualization explored
            if vis:
                if self.grid_map.is_inflated(cx, cy):
                    vis.draw_inflated(cx, cy)
                else:
                    vis.draw_explored(cx, cy)
                # draw partial path for feedback
                partial = self.build_partial_path(parent, start, current)
                for i in range(len(partial) - 1):
                    x1, y1 = partial[i]
                    x2, y2 = partial[i + 1]
                    vis.draw_path_segment(x1, y1, x2, y2)
                vis.update()

            # for child in successor do
            for nx, ny in self.get_neighbors(cx, cy):
                v = (nx, ny)

                # if v is not in C 
                if (v not in closed) and (v not in in_open):
                    if v == goal:
                        parent[v] = current
                        return self.reconstruct_path(parent, start, v)
                    open_queue.append(v)
                    in_open.add(v)
                    parent[v] = current
                    # Visualization for frontier
                    if vis:
                        if self.grid_map.is_inflated(nx, ny):
                            vis.draw_inflated(nx, ny)
                        else:
                            vis.draw_frontier(nx, ny)
        return None  
    
    def build_partial_path(self, parent, start, current):
        path = [current]
        while current in parent:
            current = parent[current]
            path.append(current)
            if current == start:
                break
        path.reverse()
        return path

    def reconstruct_path(self, parent, start, goal):
        path = [goal]
        current = goal
        while current != start:
            current = parent[current]
            path.append(current)
        path.reverse()
        return path
    
    

        #-----------------------BFS TREE SEARCH-----------------


class BFSPlanner_treesearch(Planner):
    """
    Breadth-First Search (BFS) planner in tree-search style.
    """

    def __init__(self, grid_map, motion_model, visualizer=None):
        super().__init__(grid_map, motion_model, visualizer)
        self.counter = 0
        self.expanded_count = 0

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

        vis = self.visualizer
        if vis:
            vis.draw_start_goal(start, goal)
            vis.update()

        # 3D-parent storage: parent[state][vid] = (parent_state, parent_vid)
        parent = defaultdict(dict)

        # queue entries: (state, vid)
        queue = deque()

        # root
        queue.append((start, 0))
        parent[start][0] = (None,None)
        self.counter = 1

        while queue:

            (cur, vid) = queue.popleft()
            cx, cy = cur

            self.expanded_count += 1

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

            # expand children
            for child in self.get_neighbors(cx, cy):

                child_vid = self.counter
                self.counter += 1

                parent[child][child_vid] = (cur, vid)
                queue.append((child, child_vid))

                if vis:
                    vis.draw_frontier(child[0], child[1])

        return None
    
   # ----------------------------------------
    # SAFE 3D reconstruction
    # ----------------------------------------
    def reconstruct_path_3d(self, parent, start, goal, vid):
        """
        Reconstruct full final path from goal back to start.
        Works with (state, vid) pointers.
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
        Returns partial path for visualization, safe version.
        Output must be list of (x,y) states.
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

