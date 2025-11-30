# visualizer.py

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np

class Visualizer:

    def __init__(self, grid_map, cell_size=1.0):
        self.grid_map = grid_map
        self.cell_size = cell_size

        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.ax.set_aspect("equal")

    def setup(self):
        h = self.grid_map.height
        w = self.grid_map.width

        # Grid boundaries
        self.ax.set_xlim(0, w)
        self.ax.set_ylim(0, h)

        self.ax.set_xticks(np.arange(0, w+1, 1))
        self.ax.set_yticks(np.arange(0, h+1, 1))
        self.ax.grid(True)

        self.draw_grid()

    # ---------------------------------------------------------
    # 🔹 Helper: Convert grid cell to CENTER position
    # ---------------------------------------------------------
    def _to_center(self, gx, gy):
        return gx + 0.5, gy + 0.5

    # ---------------------------------------------------------
    # DRAW BACKGROUND GRID
    # ---------------------------------------------------------
    def draw_grid(self):
        """Draw obstacles and free space squares."""
        for gy in range(self.grid_map.height):
            for gx in range(self.grid_map.width):
                color = "gray" if self.grid_map.is_obstacle(gx, gy) else "cyan"
                self._draw_cell(gx, gy, color)

    # ---------------------------------------------------------
    # DRAW START/GOAL
    # ---------------------------------------------------------
    def draw_start_goal(self, start, goal):
        sx, sy = start
        gx, gy = goal

        cx, cy = self._to_center(sx, sy)
        self.ax.scatter(cx, cy, s=300, c="red", marker="s")

        cx, cy = self._to_center(gx, gy)
        self.ax.scatter(cx, cy, s=300, c="green", marker="s")

    # ---------------------------------------------------------
    # DRAW SEARCH PROCESS
    # ---------------------------------------------------------
    def draw_explored(self, gx, gy):
        cx, cy = self._to_center(gx, gy)
        self.ax.scatter(cx, cy, s=120, c="yellow", marker="s")

    def draw_frontier(self, gx, gy):
        cx, cy = self._to_center(gx, gy)
        self.ax.scatter(cx, cy, s=120, c="blue", marker="s")

    def draw_blocked(self, gx, gy):
        cx, cy = self._to_center(gx, gy)
        self.ax.scatter(cx, cy, s=120, c="black", marker="s")

    # ---------------------------------------------------------
    # DRAW PATH
    # ---------------------------------------------------------
    def draw_path_segment(self, x1, y1, x2, y2):
        x1, y1 = self._to_center(x1, y1)
        x2, y2 = self._to_center(x2, y2)
        self.ax.plot([x1, x2], [y1, y2], "-m", linewidth=3)

    def draw_final_path(self, path):
        for i in range(len(path) - 1):
            x1, y1 = self._to_center(*path[i])
            x2, y2 = self._to_center(*path[i+1])
            self.ax.plot([x1, x2], [y1, y2], "-k", linewidth=3)

    # ---------------------------------------------------------
    # INTERNAL: Draw background grid rectangles
    # ---------------------------------------------------------
    def _draw_cell(self, gx, gy, color):
        """Draw a grid square (background)."""
        self.ax.add_patch(Rectangle(
            (gx, gy),   # bottom-left
            1.0,        # width
            1.0,        # height
            facecolor=color,
            edgecolor="none",
            alpha=0.9
        ))

    def update(self, pause=2):
        plt.pause(pause)

    def show(self):
        plt.show()