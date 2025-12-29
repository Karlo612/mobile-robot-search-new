# CodeBase/Search/dfs.py

from .planner import Planner


class DFSPlanner_graphbased(Planner):
    """
    Depth-First Search (DFS) — GRAPH-BASED
    Matches pseudocode exactly.
    """

    def __init__(self, grid_map, motion_model="8n", visualizer=None):
        super().__init__(grid_map, motion_model, visualizer)
        self.expanded_count = 0
        self.expansion_map = {}

    # --------------------------------------------------
    # Successor function
    # --------------------------------------------------
    def get_neighbors(self, gx, gy):

        if self.motion_model == "4n":
            moves = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        else:  # 8-connected
            moves = [
                (1, 0), (-1, 0), (0, 1), (0, -1),
                (1, 1), (1, -1), (-1, 1), (-1, -1)
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

    # --------------------------------------------------
    # DFS main loop (pseudocode-accurate)
    # --------------------------------------------------
    def plan(self, start, goal):

        vis = self.visualizer

        # 1: node ← with s as state
        # 2: O ← node (LIFO stack)
        OPEN = [(start, None)]     # (state, parent)
        parent = {}

        # 3: C ← ∅
        CLOSED = set()

        print("\n[DFS] START =", start, "GOAL =", goal)

        if vis:
            vis.draw_start_goal(start, goal)
            vis.update()

        # 4: while O != ∅ do
        while OPEN:
            print("[DFS] STACK (top last):", [s for s, _ in OPEN])
            # 5: parent ← last node in O
            v, p = OPEN.pop()
            print("[DFS] EXPAND:", v)

            # Skip if already expanded
            if v in CLOSED:
                continue

            # Store parent pointer
            if p is not None:
                parent[v] = p

            # 6: v ← parent.state
            cx, cy = v

            # Bookkeeping (benchmarking only)
            self.expanded_count += 1
            self.expansion_map[(cx, cy)] = (
                self.expansion_map.get((cx, cy), 0) + 1
            )

            # ---- VISUALIZATION (CORRECT LOCATION) ----
            if vis:
                vis.draw_explored(cx, cy)

                partial = self.build_partial_path(parent, start, v)
                for i in range(len(partial) - 1):
                    x1, y1 = partial[i]
                    x2, y2 = partial[i + 1]
                    vis.draw_path_segment(x1, y1, x2, y2)

                vis.update()

            # 7: if v == g then return parent
            if v == goal:
                return self.reconstruct_path(parent, start, goal)

            # 10: C ← C ∪ {v}
            CLOSED.add(v)

            # 11: for child in successor(v)
            for child in self.get_neighbors(cx, cy):

                # 13: if child not in C and child not in O
                if child not in CLOSED and child not in [n for n, _ in OPEN]:

                    # 14: add child to O
                    print("    [DFS] PUSH:", child)
                    OPEN.append((child, v))

                    if vis:
                        vis.draw_frontier(child[0], child[1])

        # 18: return failure
        return None

    # --------------------------------------------------
    # Path helpers
    # --------------------------------------------------
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
