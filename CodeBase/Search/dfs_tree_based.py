from .planner import Planner

class DFSNode:
    def __init__(self, state, parent=None):
        self.state = state      # (x, y)
        self.parent = parent    # DFSNode or None


class DFSPlanner_treebased(Planner):
    """
    DFS Tree Search:
      - NO closed set
      - Allows duplicates (same state can appear multiple times)
      - Stack is LIFO
      - Parent pointers stored per-node (not per-state)
    """

    def __init__(self, grid_map, motion_model="8n", visualizer=None):
        super().__init__(grid_map, motion_model, visualizer)
        self.expanded_count = 0
        self.expansion_map = {}
        self.exceeded_expansion_limit = False

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
                print(f"[BLOCKED] {nx,ny} is not inside")
                continue
            if self.grid_map.is_obstacle(nx, ny):
                print(f"[BLOCKED] {nx,ny} is OBSTACLE (from {gx,gy})")
                continue
            if self.grid_map.is_inflated(nx, ny):
                print(f"[BLOCKED] {nx,ny} inflated")
                if self.visualizer:
                    self.visualizer.draw_inflated(nx, ny)
                continue

            yield (nx, ny)

    # ----------------------------
    # Partial path for visualization
    # ----------------------------
    def build_partial_path(self, node: DFSNode):
        """
        Follow parent pointers back to the root.
        Loop-safe (tree search can create cycles via state repetition).
        """
        path = []
        seen = set()  # track node identities, not states

        cur = node
        while cur is not None:
            nid = id(cur)
            if nid in seen:
                print("[DFS-TREE] Loop detected in partial path reconstruction; stopping.")
                break
            seen.add(nid)

            path.append(cur.state)
            cur = cur.parent

        return list(reversed(path))

    def reconstruct_path(self, node: DFSNode):
        # same as partial path, just used at goal
        return self.build_partial_path(node)

    # ----------------------------
    # MAIN DFS TREE SEARCH (matches your pseudocode)
    # ----------------------------
    def plan(self, start, goal):

        print(f"\n[DFS-TREE] START={start} GOAL={goal}")

        # 1: node ← with s as state
        node = DFSNode(start, parent=None)

        # 2-4: if s == g then return node
        if start == goal:
            return [start]

        # 5: O ← node, O is LIFO
        O = [node]  # stack of DFSNode objects

        vis = self.visualizer
        if vis:
            vis.draw_start_goal(start, goal)
            vis.update()

        # 6: while O != ∅ do
        while O:

            # 7: parent ← the last node from O
            parent_node = O.pop()
            v = parent_node.state

            print(f"[DFS-TREE] EXPAND {v} | stack_size={len(O)}")

            # expansions counters
            self.expanded_count += 1
            self.expansion_map[v] = self.expansion_map.get(v, 0) + 1

            # Check expansion limit to prevent infinite loops
            grid_size = self.grid_map.width * self.grid_map.height
            if self.expanded_count > grid_size:
                self.exceeded_expansion_limit = True
                print(f"[DFS-TREE] Stopped: expanded_count ({self.expanded_count}) exceeds grid size ({grid_size})")
                return None

            # visualization: explored + partial path
            if vis:
                vis.draw_explored(v[0], v[1])

                partial = self.build_partial_path(parent_node)
                for i in range(len(partial) - 1):
                    x1, y1 = partial[i]
                    x2, y2 = partial[i + 1]
                    vis.draw_path_segment(x1, y1, x2, y2)

                vis.update()

            # 8-10: if parent.state == g then return parent
            if v == goal:
                return self.reconstruct_path(parent_node)

            # 11: for child in successor(of parent) do
            for child_state in self.get_neighbors(v[0], v[1]):

                # 12: add child to O
                child_node = DFSNode(child_state, parent=parent_node)
                O.append(child_node)

                print(f"    [DFS-TREE] PUSH {child_state}")

                if vis:
                    vis.draw_frontier(child_state[0], child_state[1])

        # 15: return failure
        return None
