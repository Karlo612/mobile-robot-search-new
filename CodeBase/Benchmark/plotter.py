# CodeBase/Benchmark/plotter.py

import os
from typing import List, Dict

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, BoundaryNorm


# ================================================================
# Helper: heatmap plotting for a single test (all planners)
# ================================================================
def _plot_heatmaps_for_test(test: Dict):
    title      = test["title"]
    start      = test["start"]
    goal       = test["goal"]
    obstacles  = test["obstacles"]
    planners   = list(test["planners"].keys())

    n_planners = len(planners)
    if n_planners == 0:
        return

    # Arrange heatmaps in a grid of 3 columns
    n_cols = 3
    n_rows = int(np.ceil(n_planners / n_cols))

    fig, axes = plt.subplots(
        n_rows, n_cols,
        figsize=(5 * n_cols, 5 * n_rows),
        squeeze=False
    )
    fig.suptitle(f"{title} – Expansion Heatmaps", fontsize=16, fontweight="bold")

    # Custom colormap: obstacle = black, unvisited = white, then warm colors by expansion count
    colors = [
        "black",    # -1 => obstacle
        "white",    # 0  => unvisited
        "#ffffcc",
        "#ffdd88",
        "#ffbb55",
        "#ff9933",
        "#ff5500",
        "#cc0000",
    ]
    bounds = [-1, 0, 1, 5, 10, 20, 50, 200, 1_000_000]
    cmap = ListedColormap(colors)
    norm = BoundaryNorm(bounds, len(colors))

    sx, sy = start
    gx, gy = goal

    for idx, pname in enumerate(planners):
        r = idx // n_cols
        c = idx % n_cols
        ax = axes[r][c]

        heat = test["planners"][pname]["heatmap"]
        h, w = heat.shape

        # Build visualization grid: copy expansion counts, overlay obstacles as -1
        vis_grid = np.array(heat, dtype=float)
        for (ox, oy) in obstacles:
            if 0 <= ox < w and 0 <= oy < h:
                vis_grid[oy, ox] = -1

        # IMPORTANT: origin='lower' so row 0 appears at bottom (matches your grid style)
        im = ax.imshow(vis_grid, origin="lower", cmap=cmap, norm=norm)

        # Grid lines
        ax.set_xticks(np.arange(-0.5, w, 1), minor=True)
        ax.set_yticks(np.arange(-0.5, h, 1), minor=True)
        ax.grid(which="minor", color="gray", linestyle="-", linewidth=0.3)

        ax.set_xticks([])
        ax.set_yticks([])

        # Overlay path if available
        path = test["planners"][pname].get("path", [])
        if path:
            px = [p[0] for p in path]
            py = [p[1] for p in path]
            ax.plot(px, py, color="cyan", linewidth=2, label="Path")

        # Overlay start & goal
        ax.scatter([sx], [sy], c="green", s=50, marker="o", label="Start")
        ax.scatter([gx], [gy], c="red",   s=60, marker="x", label="Goal")

        ax.set_title(pname, fontsize=12)
        ax.legend(fontsize=8, loc="upper right")
        fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04)

    # Turn off any unused subplots
    for idx in range(n_planners, n_rows * n_cols):
        r = idx // n_cols
        c = idx % n_cols
        axes[r][c].axis("off")

    fig.tight_layout(rect=[0, 0, 1, 0.95])

    os.makedirs("plots", exist_ok=True)
    safe_title = title.lower().replace(" ", "_")
    save_path = os.path.join("plots", f"{safe_title}_heatmaps.png")
    fig.savefig(save_path, dpi=150)
    print(f"[plotter] Saved: {save_path}")
    plt.show()


# ================================================================
# Helper: generic metric bar plot (one figure, all planners)
# ================================================================
def _plot_metric_for_test(test: Dict, metric_key: str, title_suffix: str, ylabel: str, fmt: str = ".3f"):
    """
    metric_key: 'runtime', 'expansions', 'path_len', ...
    """
    title    = test["title"]
    planners = list(test["planners"].keys())

    values = [test["planners"][p][metric_key] for p in planners]

    fig, ax = plt.subplots(figsize=(8, 4))
    fig.suptitle(f"{title} – {title_suffix}", fontsize=14, fontweight="bold")

    x = np.arange(len(planners))
    ax.bar(x, values)
    ax.set_xticks(x)
    ax.set_xticklabels(planners, rotation=20, ha="right")
    ax.set_ylabel(ylabel)

    # Label bars with numeric values
    for xi, v in zip(x, values):
        if isinstance(v, (int, np.integer)):
            label = f"{v}"
        else:
            label = format(v, fmt)
        ax.text(xi, v, label, ha="center", va="bottom", fontsize=9)

    ax.grid(axis="y", linestyle="--", alpha=0.4)
    fig.tight_layout(rect=[0, 0, 1, 0.95])

    os.makedirs("plots", exist_ok=True)
    safe_title = title.lower().replace(" ", "_")
    metric_name = metric_key.lower()
    save_path = os.path.join("plots", f"{safe_title}_{metric_name}.png")
    fig.savefig(save_path, dpi=150)
    print(f"[plotter] Saved: {save_path}")
    plt.show()


# ================================================================
# Public API: plot all tests produced by Comparator
# ================================================================
def plot_all_tests(test_runs: List[Dict]):
    """
    test_runs: list of test dicts from Comparator.run_all_tests()

    For each test (e.g. "Small 10x10"):
      1) One figure: heatmaps of all planners
      2) One figure: runtime bar chart (all planners)
      3) One figure: expansions bar chart (all planners)
      4) One figure: path length bar chart (all planners)
    """
    for test in test_runs:
        # a) Heatmaps (all planners)
        _plot_heatmaps_for_test(test)

        # b) Runtime
        _plot_metric_for_test(
            test,
            metric_key="runtime",
            title_suffix="Runtime per Planner",
            ylabel="Time (s)",
            fmt=".4f",
        )

        # c) Expansions
        _plot_metric_for_test(
            test,
            metric_key="expansions",
            title_suffix="Expanded Nodes per Planner",
            ylabel="Nodes expanded",
            fmt=".0f",
        )

        # d) Path length
        _plot_metric_for_test(
            test,
            metric_key="path_len",
            title_suffix="Path Length per Planner",
            ylabel="Path length (steps)",
            fmt=".0f",
        )
