"""
Embedded Visualizer - Real-Time Path Planning Visualization

This module provides the EmbeddedVisualizer class which embeds matplotlib
visualizations into Tkinter GUI windows. It supports real-time visualization
of path planning algorithms, showing the grid, obstacles, explored nodes,
frontier nodes, and the final path. The visualizer separates static elements
(grid, legend) from dynamic elements (search progress) for efficient updates.
"""

import time
# Set backend before importing pyplot to avoid compatibility issues
import matplotlib
try:
    matplotlib.use('TkAgg')
except Exception:
    pass  # Backend might already be set
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.lines import Line2D


class EmbeddedVisualizer:
    """
    Real-time visualization of path planning algorithms.
    
    Embeds a matplotlib figure into a Tkinter frame to provide real-time
    visualization of the search process. Shows the grid map, obstacles,
    start/goal positions, explored nodes, frontier nodes, and the final path.
    
    The visualizer maintains separate lists for static elements (grid, legend,
    info text) and dynamic elements (search progress) to allow efficient
    clearing and updating of search-related visuals without affecting the
    underlying grid display.
    """
    def __init__(self, parent_frame, grid_map):
        """
        Initialize the embedded visualizer.
        
        Args:
            parent_frame: Tkinter frame to embed the visualization in
            grid_map: GridMap object to visualize
        """
        self.grid_map = grid_map
        self.parent = parent_frame

        try:
            self.fig, self.ax = plt.subplots(figsize=(6, 6))
        except Exception as e:
            # Fallback if TkAgg backend has compatibility issues
            error_msg = str(e)
            if "macOS" in error_msg or "2601" in error_msg:
                raise RuntimeError(
                    "GUI visualization requires macOS compatibility. "
                    "The TkAgg backend is not compatible with your system. "
                    "Please try:\n"
                    "1. Updating your Python version to 3.10+\n"
                    "2. Using a different Python distribution (e.g., Homebrew Python)\n"
                    "3. Running without visualization (set visualize_search: false in config)"
                ) from e
            raise
        
        self.ax.set_aspect("equal")

        try:
            self.canvas = FigureCanvasTkAgg(self.fig, master=self.parent)
            self.canvas.get_tk_widget().pack(fill="both", expand=True)
        except Exception as e:
            plt.close(self.fig)
            raise RuntimeError(f"Failed to create GUI canvas: {e}") from e

        self._grid_artist = None

        # Separate static from dynamic elements for efficient clearing
        self.static_artists = []   # env/search info + legend
        self.dynamic_artists = []  # explored/frontier/path/etc

        self.env_info_text = None
        self.search_info_text = None
        self._legend = None

    # -------------------------
    # Helpers
    # -------------------------
    def _register_dynamic(self, artist):
        self.dynamic_artists.append(artist)

    def _register_static(self, artist):
        self.static_artists.append(artist)

    def clear_dynamic_layers(self):
        """Remove search-related drawings but keep grid + legend + info boxes."""
        for artist in self.dynamic_artists:
            try:
                artist.remove()
            except Exception:
                pass
        self.dynamic_artists.clear()
        self.canvas.draw_idle()

    def _remove_static_artist(self, artist):
        if artist is not None:
            try:
                artist.remove()
            except Exception:
                pass

    # -------------------------
    # Setup grid (FAST)
    # -------------------------
    def setup(self):
        # Clear axis but do not rely on dynamic lists here
        self.ax.clear()

        h, w = self.grid_map.height, self.grid_map.width

        self._grid_artist = self.ax.imshow(
            self.grid_map.grid,
            origin="lower",
            cmap="gray_r",
            interpolation="nearest",
            extent=[0, w, 0, h]
        )

        # ticks spacing
        if w <= 20:
            step = 1
        elif w <= 50:
            step = 5
        else:
            step = 10

        self.ax.set_xticks(range(0, w + 1, step))
        self.ax.set_yticks(range(0, h + 1, step))

        self.ax.set_xlabel("Grid X")
        self.ax.set_ylabel("Grid Y")

        self.ax.grid(True, which="major", linewidth=0.3, alpha=0.5)
        self.ax.set_xlim(0, w)
        self.ax.set_ylim(0, h)

        # Make room at top for info boxes + at right for legend
        self.fig.subplots_adjust(top=0.78, right=0.80)

        # Legend outside plot (right) - High Contrast Sunset Theme
        legend_elements = [
            Line2D([0], [0], marker='s', color='#000000', markersize=10, label='Obstacle'),  # Pure black
            Line2D([0], [0], marker='s', color='#ffffff', markersize=10, label='Free'),  # Pure white
            Line2D([0], [0], marker='s', color='#ff8800', markersize=10, label='Explored'),  # Bright orange
            Line2D([0], [0], marker='s', color='#ff0000', markersize=10, label='Frontier'),  # Pure red
            Line2D([0], [0], marker='s', color='#cc0000', markersize=10, label='Inflated'),  # Dark red
            Line2D([0], [0], color='#ff00ff', lw=2, label='Partial Path'),  # Bright magenta
            Line2D([0], [0], color='#ff6600', lw=2, label='Final Path'),  # Bright orange
        ]

        if self._legend is not None:
            try:
                self._legend.remove()
            except Exception:
                pass

        self._legend = self.ax.legend(
            handles=legend_elements,
            loc="center left",
            bbox_to_anchor=(1.02, 0.5),
            fontsize=8,
            frameon=True
        )
        self._register_static(self._legend)

        # Re-draw info if already set (optional safety)
        if self.env_info_text:
            self._register_static(self.env_info_text)
        if self.search_info_text:
            self._register_static(self.search_info_text)

        self.canvas.draw_idle()

    # -------------------------
    # Info boxes (STATIC)
    # -------------------------
    def set_environment_info(self, grid_size, resolution, obstacle_radius, start, goal):
        text = (
            f"Grid: {grid_size} × {grid_size}   |   Resolution: {resolution}\n"
            f"Obstacle radius: {obstacle_radius}   |   Start: {start}   |   Goal: {goal}"
        )

        # Remove old env box
        self._remove_static_artist(self.env_info_text)

        # Use fig.text so it sits ABOVE the grid, not on it
        self.env_info_text = self.fig.text(
            0.5, 0.97, text,
            ha="center", va="top",
            fontsize=9,
            bbox=dict(boxstyle="round,pad=0.35", facecolor="white", edgecolor="black", alpha=0.95),
            zorder=50
        )
        self._register_static(self.env_info_text)
        self.canvas.draw_idle()

    def set_search_info(self, robot_radius, planner, mode, motion):
        text = (
            f"Robot radius: {robot_radius}   |   Planner: {planner}   |   Mode: {mode}   |   Motion: {motion}"
        )

        # Remove old search box
        self._remove_static_artist(self.search_info_text)

        self.search_info_text = self.fig.text(
            0.5, 0.92, text,
            ha="center", va="top",
            fontsize=9,
            bbox=dict(boxstyle="round,pad=0.35", facecolor="white", edgecolor="black", alpha=0.95),
            zorder=50
        )
        self._register_static(self.search_info_text)
        self.canvas.draw_idle()

    # -------------------------
    # Start/Goal (DYNAMIC for each run)
    # -------------------------
    def draw_start_goal(self, start, goal):
        sx, sy = start
        gx, gy = goal

        # High contrast: Start = bright green, Goal = bright red
        start_rect = Rectangle((sx, sy), 1, 1, fill=False, edgecolor="#00ff00", linewidth=3, zorder=6)
        goal_rect = Rectangle((gx, gy), 1, 1, fill=False, edgecolor="#ff0000", linewidth=3, zorder=6)

        self.ax.add_patch(start_rect)
        self.ax.add_patch(goal_rect)

        self._register_dynamic(start_rect)
        self._register_dynamic(goal_rect)

    # -------------------------
    # Drawing helpers
    # -------------------------
    def _to_center(self, gx, gy):
        return gx + 0.5, gy + 0.5

    def draw_explored(self, gx, gy):
        cx, cy = self._to_center(gx, gy)
        # High contrast: bright orange for explored
        artist = self.ax.scatter(cx, cy, s=140, c="#ff8800", marker="s", edgecolors='#cc6600', linewidths=1)
        self._register_dynamic(artist)

    def draw_frontier(self, gx, gy):
        cx, cy = self._to_center(gx, gy)
        # High contrast: pure red for frontier
        artist = self.ax.scatter(cx, cy, s=140, c="#ff0000", marker="s", edgecolors='#990000', linewidths=1)
        self._register_dynamic(artist)

    def draw_path_segment(self, x1, y1, x2, y2):
        cx1, cy1 = self._to_center(x1, y1)
        cx2, cy2 = self._to_center(x2, y2)
        # High contrast: bright magenta for partial path
        artist, = self.ax.plot([cx1, cx2], [cy1, cy2], color="#ff00ff", linewidth=3, zorder=4)
        self._register_dynamic(artist)

    def draw_final_path(self, path):
        xs, ys = [], []
        for gx, gy in path:
            cx, cy = self._to_center(gx, gy)
            xs.append(cx)
            ys.append(cy)
        # High contrast: bright orange for final path with thicker line
        artist, = self.ax.plot(xs, ys, color="#ff6600", linewidth=4, zorder=5, marker='o', markersize=5)
        self._register_dynamic(artist)
        self.canvas.draw_idle()

    def draw_inflated(self, gx, gy):
        # High contrast: dark red for inflated obstacles with higher opacity
        rect = Rectangle((gx, gy), 1, 1, facecolor="#cc0000", edgecolor="#990000", linewidth=1, alpha=0.6, zorder=2)
        self.ax.add_patch(rect)
        self._register_dynamic(rect)

    # -------------------------
    # Update / Show
    # -------------------------
    def update(self, pause=1):
        self.canvas.draw()
        self.canvas.flush_events()
        time.sleep(pause)

    def show(self):
        pass
