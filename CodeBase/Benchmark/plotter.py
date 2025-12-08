import os
from typing import List, Dict

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, BoundaryNorm


# -----------------------------------------------------------
#  Draw a single test case figure
# -----------------------------------------------------------
def plot_single_test(test: Dict):

    title       = test["title"]
    size        = test["size"]
    start       = test["start"]
    goal        = test["goal"]
    radius      = test["radius"]
    resolution  = test["resolution"]
    obstacles   = test["obstacles"]
    planners    = list(test["planners"].keys())

    # Metrics
    runtimes   = [test["planners"][p]["runtime"]    for p in planners]
    expansions = [test["planners"][p]["expansions"] for p in planners]
    path_lens  = [test["planners"][p]["path_len"]   for p in planners]
    mem_kb     = [test["planners"][p]["memory_kb"]  for p in planners]

    # Figure layout
    fig = plt.figure(figsize=(14, 12))
    gs = fig.add_gridspec(3, 3, height_ratios=[1, 4, 3], hspace=0.8)
    fig.suptitle(title, fontsize=16, fontweight="bold")

    # --------------------------------------------------------
    # Row 1: Metadata
    # --------------------------------------------------------
    ax_meta = fig.add_subplot(gs[0, :])
    ax_meta.axis("off")
    meta_lines = [
        f"Grid size: {size} x {size}",
        f"Start: {start}    Goal: {goal}",
        f"Robot radius: {radius}    Resolution: {resolution}"
    ]
    ax_meta.text(0.02, 0.5, "\n".join(meta_lines),
                 ha="left", va="center", fontsize=12)

    # --------------------------------------------------------
    # Row 2: Heatmaps
    # --------------------------------------------------------

    # Custom color map: obstacles black, unvisited white → warm colors
    colors = [
        "black",     # -1 = obstacle
        "white",     # 0 = unvisited
        "#ffffcc",   # light yellow
        "#ffcc66",   # yellow-orange
        "#ff9933",   # orange
        "#ff6600",   # warm orange
        "#cc0000",   # dark red
    ]
    bounds = [-1, 0, 1, 5, 10, 20, 50, 999999]
    norm = BoundaryNorm(bounds, len(colors))
    cmap = ListedColormap(colors)

    for idx, pname in enumerate(planners):

        ax = fig.add_subplot(gs[1, idx])

        heat = test["planners"][pname]["heatmap"]
        h, w = heat.shape

        # Copy heatmap and mark obstacles explicitly
        vis_grid = np.copy(heat)
        for (ox, oy) in obstacles:
            vis_grid[oy, ox] = -1

        # Flip vertically so grid (0,0) is bottom-left
        vis_grid = vis_grid[::-1, :]

        im = ax.imshow(vis_grid, cmap=cmap, norm=norm)

        # Draw grid lines
        ax.set_xticks(np.arange(-0.5, w, 1), minor=True)
        ax.set_yticks(np.arange(-0.5, h, 1), minor=True)
        ax.grid(which="minor", color="gray", linestyle="-", linewidth=0.3)

        ax.set_xticks([])
        ax.set_yticks([])

        # Overlay PATH
        path = test["planners"][pname]["path"]
        if path:
            px = [p[0] for p in path]
            py = [h - 1 - p[1] for p in path]   # flip y
            ax.plot(px, py, color="cyan", linewidth=2, label="Path")

        # Overlay start & goal
        sx, sy = start
        gx, gy = goal

        ax.scatter([sx], [h - 1 - sy], c="green", s=50, marker="o", label="Start")
        ax.scatter([gx], [h - 1 - gy], c="red",   s=60, marker="x", label="Goal")

        ax.set_title(pname, fontsize=11)
        ax.legend(fontsize=8, loc="upper right")
        fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04)

    # --------------------------------------------------------
    # Row 3: Metric bar charts
    # --------------------------------------------------------

    x = np.arange(len(planners))

    # Runtime
    ax_time = fig.add_subplot(gs[2, 0])
    ax_time.bar(x, runtimes)
    ax_time.set_xticks(x)
    ax_time.set_xticklabels(planners, rotation=20, ha="right")
    ax_time.set_ylabel("Time (s)")
    ax_time.set_title("Runtime")

    # Expansions
    ax_exp = fig.add_subplot(gs[2, 1])
    ax_exp.bar(x, expansions)
    ax_exp.set_xticks(x)
    ax_exp.set_xticklabels(planners, rotation=20, ha="right")
    ax_exp.set_ylabel("Nodes Expanded")
    ax_exp.set_title("Search Effort")

    # Memory + overlay path length
    ax_mem = fig.add_subplot(gs[2, 2])
    ax_mem.bar(x, mem_kb)
    ax_mem.set_xticks(x)
    ax_mem.set_xticklabels(planners, rotation=20, ha="right")
    ax_mem.set_ylabel("Peak Memory (KB)")
    ax_mem.set_title("Memory Usage")

    for i, (px, py) in enumerate(zip(x, mem_kb)):
        ax_mem.text(px, py, f"len={path_lens[i]}",
                    ha="center", va="bottom", fontsize=8)

    # --------------------------------------------------------
    # Save + show
    # --------------------------------------------------------
    os.makedirs("plots", exist_ok=True)
    safe_title = title.lower().replace(" ", "_")
    save_path = os.path.join("plots", f"{safe_title}.png")

    fig.savefig(save_path, dpi=150)
    print(f"[plotter] Saved: {save_path}")

    plt.show()


# -----------------------------------------------------------
#  Plot all tests
# -----------------------------------------------------------
def plot_all_tests(test_runs: List[Dict]):
    for test in test_runs:
        plot_single_test(test)