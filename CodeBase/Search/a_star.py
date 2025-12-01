# # astar.py

# from Search.planner import Planner
# import heapq
# import math

# class AStarPlanner(Planner):
#     """
#     Textbook A* graph search (Algorithm 2) with project structure + visualization.
#     """

#     def __init__(self, grid_map, motion_model, visualizer=None):
#         super().__init__(grid_map, motion_model, visualizer)
#         self.res = grid_map.resolution

#     # -----------------------
#     # Heuristic: Manhattan
#     # -----------------------
#     def heuristic(self, a, b):
#         x1, y1 = a
#         x2, y2 = b
#         return (abs(x1 - x2) + abs(y1 - y2)) * self.res

#     # -----------------------
#     # Successor function
#     # -----------------------
#     def get_neighbors(self, gx, gy):

#         if self.motion_model == "4n":
#             moves = [
#                 (1, 0), (-1, 0),
#                 (0, 1), (0, -1),
#             ]
#         else:  # default "8n"
#             moves = [
#                 (1, 0), (-1, 0),
#                 (0, 1), (0, -1),
#                 (1, 1), (1, -1),
#                 (-1, 1), (-1, -1),
#             ]

#         for dx, dy in moves:
#             nx, ny = gx + dx, gy + dy

#             # Skip out-of-bounds
#             if not self.grid_map.is_inside(nx, ny):
#                 continue

#             # Skip hard obstacles
#             if self.grid_map.is_obstacle(nx, ny):
#                 continue

#             # Skip inflated cells (blocked by robot radius), but show them
#             if self.grid_map.is_inflated(nx, ny):
#                 if self.visualizer:
#                     self.visualizer.draw_inflated(nx, ny)
#                 continue

#             # Valid free neighbor
#             yield (nx, ny)

#     # -----------------------
#     # Movement cost
#     # -----------------------
#     def cost(self, x1, y1, x2, y2):
#         # Diagonal move in 8n
#         if x1 != x2 and y1 != y2:
#             return math.sqrt(2) * self.res
#         # Straight move
#         return 1 * self.res

#     # -----------------------
#     # Main A* (Algorithm 2)
#     # -----------------------
#     def plan(self, start, goal):
#         sx, sy = start
#         gx, gy = goal

#         # 2: OPEN as priority queue (min-heap) storing (f, (x,y))
#         open_heap = []
#         heapq.heappush(open_heap, (0.0, (sx, sy)))

#         # Track membership in OPEN with current best f
#         open_membership = {(sx, sy): 0.0}

#         # 3: CLOSED = explored states
#         closed = set()

#         # Parent pointers & g-costs
#         parent = {}
#         g_cost = {(sx, sy): 0.0}

#         vis = self.visualizer
#         if vis:
#             vis.draw_start_goal(start, goal)
#             vis.update()

#         # 4: while OPEN not empty
#         while open_heap:

#             # 5: parent ← best node in OPEN
#             f, current = heapq.heappop(open_heap)

#             # If we've already processed this with a better f, skip
#             if current in closed:
#                 continue

#             # Remove from OPEN-membership
#             open_membership.pop(current, None)

#             # 6: goal test
#             if current == goal:
#                 path = self.reconstruct_path(parent, start, goal)
#                 return path

#             # 10: add to CLOSED
#             closed.add(current)
#             cx, cy = current

#             # Visualization: explored + partial path
#             if vis:
#                 vis.draw_explored(cx, cy)
#                 partial = self.build_partial_path(parent, start, current)
#                 if len(partial) > 1:
#                     for i in range(len(partial) - 1):
#                         x1, y1 = partial[i]
#                         x2, y2 = partial[i + 1]
#                         vis.draw_path_segment(x1, y1, x2, y2)
#                 vis.update()

#             # 11: for each child in successors(parent)
#             for child in self.get_neighbors(cx, cy):
#                 if child in closed:
#                     continue

#                 nx, ny = child
#                 tentative_g = g_cost[current] + self.cost(cx, cy, nx, ny)
#                 tentative_f = tentative_g + self.heuristic(child, goal)

#                 # 13: if v not in C and child not in O  (new node)
#                 if child not in open_membership:
#                     g_cost[child] = tentative_g
#                     parent[child] = current
#                     heapq.heappush(open_heap, (tentative_f, child))
#                     open_membership[child] = tentative_f

#                     if vis:
#                         vis.draw_frontier(nx, ny)

#                 # 15–18: else if child in O and better path found
#                 else:
#                     old_f = open_membership[child]
#                     if tentative_f < old_f:
#                         g_cost[child] = tentative_g
#                         parent[child] = current
#                         heapq.heappush(open_heap, (tentative_f, child))
#                         open_membership[child] = tentative_f

#                         if vis:
#                             vis.draw_frontier(nx, ny)

#         # 22: failure
#         return None

#     # -----------------------
#     # Helper: partial path for visualization
#     # -----------------------
#     def build_partial_path(self, parent, start, current):
#         path = [current]
#         while current in parent:
#             current = parent[current]
#             path.append(current)
#             if current == start:
#                 break
#         path.reverse()
#         return path

#     # -----------------------
#     # Helper: full path reconstruction
#     # -----------------------
#     def reconstruct_path(self, parent, start, goal):
#         path = [goal]
#         current = goal

#         while current != start:
#             current = parent[current]
#             path.append(current)

#         path.reverse()
#         return path
# """"""