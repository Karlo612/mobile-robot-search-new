from Search.planner import Planner
import heapq
import math

class AStarPlanner_graphbased(Planner):

    def __init__(self, grid_map, motion_model="8n", visualizer=None):
        super().__init__(grid_map, motion_model, visualizer)
        self.res = grid_map.resolution

    def heuristic(self, a, b):
        x1, y1 = a
        x2, y2 = b
        return (abs(x1 - x2) + abs(y1 - y2)) * self.res

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
                continue
            if self.grid_map.is_obstacle(nx, ny):
                continue
            if self.grid_map.is_inflated(nx, ny):
                if self.visualizer:
                    self.visualizer.draw_inflated(nx, ny)
                continue

            yield (nx, ny)

    def cost(self, x1, y1, x2, y2):
        if x1 != x2 and y1 != y2:
            return math.sqrt(2) * self.res
        return 1 * self.res

    def plan(self, start, goal):

        sx, sy = start
        gx, gy = goal

        # openset priority queue
        OPEN = []
        heapq.heappush(OPEN, (0, start))

        # dictionary for open set membership test
        open_membership = {start: 0}

        # CLOSED set
        CLOSED = set()

        # parent tracker, g-cost
        parent = {}
        g_cost = {start: 0}

        vis = self.visualizer
        if vis:
            vis.draw_start_goal(start, goal)
            vis.update()

        while OPEN:

            f, current = heapq.heappop(OPEN)
            open_membership.pop(current, None)

            if current == goal:
                return self.reconstruct_path(parent, start, goal)

            CLOSED.add(current)

            cx, cy = current

            if vis:
                vis.draw_explored(cx, cy)

                # draw partial path from start → current
                partial = self.build_partial_path(parent, start, current)
                for i in range(len(partial) - 1):
                    x1, y1 = partial[i]
                    x2, y2 = partial[i+1]
                    vis.draw_path_segment(x1, y1, x2, y2)

                vis.update()

            for child in self.get_neighbors(cx, cy):
                
                if child in CLOSED:
                    continue

                g = g_cost[current] + self.cost(cx, cy, child[0], child[1])
                f = g + self.heuristic(child, goal)

                # child not in OPEN → NEW NODE
                if child not in open_membership:

                    g_cost[child] = g
                    parent[child] = current

                    heapq.heappush(OPEN, (f, child))
                    open_membership[child] = f

                    if vis:
                        vis.draw_frontier(child[0], child[1])

                else:
                    # improved path found
                    old_f = open_membership[child]
                    if f < old_f:

                        g_cost[child] = g
                        parent[child] = current

                        heapq.heappush(OPEN, (f, child))
                        open_membership[child] = f

                        if vis:
                            vis.draw_frontier(child[0], child[1])
        return None

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