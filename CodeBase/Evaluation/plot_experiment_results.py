import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, BoundaryNorm



# ==================================================
# CONFIG (MATCH YOUR DIRECTORY)
# ==================================================
CSV_PATH = "test1.csv"
NPZ_PATH = "test1_data.npz"
OUTDIR = "plots"

MAP_SIZES = ["small", "medium", "large"]
METRICS = ["runtime_ms", "expanded_nodes", "path_len", "path_cost","memory_kb"]


# ==================================================
# UTILITIES
# ==================================================
def ensure_dir(path):
    os.makedirs(path, exist_ok=True)


# ==================================================
# LOAD DATA
# ==================================================
def load_data():
    df = pd.read_csv(CSV_PATH)

    # planner identity
    df["planner_label"] = (
        df["planner"] + "-" + df["tree"].map({True: "Tree", False: "Graph"})
    )

    heatmaps = np.load(NPZ_PATH, allow_pickle=True)
    return df, heatmaps


# ==================================================
# 1. PERFORMANCE PER MAP (BAR PLOTS)
# ==================================================
def plot_performance_per_map(df, map_size, map_id):
    subset = df[
        (df["map_size"] == map_size) &
        (df["map_id"] == map_id)
    ]

    if subset.empty:
        return

    for metric in METRICS:
        plt.figure(figsize=(7, 4))
        plt.bar(subset["planner_label"], subset[metric])
        plt.ylabel(metric.replace("_", " ").title())
        plt.title(f"{metric.replace('_',' ').title()} | {map_size} map {map_id}")
        plt.xticks(rotation=30)
        plt.tight_layout()
        plt.savefig(
            f"{OUTDIR}/{metric}_{map_size}_{map_id}.png"
        )
        plt.close()


# ==================================================
# 2. HEATMAP + OBSTACLE + PATH OVERLAY
# ==================================================
def get_expansion_colormap():
    """
    Matches the color coding from CodeBase/Benchmark/plotter.py
    """
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

    return cmap, norm

def plot_heatmap_overlay(
    heatmaps,
    map_key,
    heatmap_id,
    path_id,
    outname
):
    grid = heatmaps[f"grid/{map_key}"]
    heat = heatmaps[f"heatmap/{heatmap_id}"]

    h, w = heat.shape

    # Build visualization grid
    vis_grid = np.array(heat, dtype=float)

    # Overlay obstacles as -1
    for y in range(h):
        for x in range(w):
            if grid[y, x] != 0:
                vis_grid[y, x] = -1

    cmap, norm = get_expansion_colormap()

    plt.figure(figsize=(5, 5))

    im = plt.imshow(
        vis_grid,
        origin="lower",
        cmap=cmap,
        norm=norm
    )

    # Grid lines (same as old plotter)
    plt.gca().set_xticks(np.arange(-0.5, w, 1), minor=True)
    plt.gca().set_yticks(np.arange(-0.5, h, 1), minor=True)
    plt.grid(which="minor", color="gray", linestyle="-", linewidth=0.3)

    plt.xticks([])
    plt.yticks([])

    # Path overlay (if available)
    path_key = f"path/{path_id}"
    if path_key in heatmaps:
        path = heatmaps[path_key]
        px = [p[0] for p in path]
        py = [p[1] for p in path]
        plt.plot(px, py, color="cyan", linewidth=2)

    plt.title(heatmap_id)
    plt.colorbar(im, fraction=0.046, pad=0.04)
    plt.tight_layout()
    plt.savefig(outname, dpi=150)
    plt.close()



def plot_heatmaps_per_map(df, heatmaps, map_size, map_id):
    subset = df[
        (df["map_size"] == map_size) &
        (df["map_id"] == map_id)
    ]

    if subset.empty:
        return

    for _, row in subset.iterrows():
        map_key = f"{map_size}_{map_id}"
        heatmap_id = row["heatmap_id"]

        path_id = heatmap_id  # SAME ID FORMAT YOU STORED

        outname = f"{OUTDIR}/heatmap_{heatmap_id}.png"

        plot_heatmap_overlay(
            heatmaps,
            map_key,
            heatmap_id,
            path_id,
            outname
        )


# ==================================================
# 3. BOX PLOTS PER MAP SIZE
# ==================================================
def boxplot_metric_by_size(df, metric):
    for size in MAP_SIZES:
        subset = df[df["map_size"] == size]

        if subset.empty:
            continue

        plt.figure(figsize=(8, 4))
        subset.boxplot(
            column=metric,
            by="planner_label",
            grid=False
        )

        plt.title(f"{metric.replace('_',' ').title()} | {size} maps")
        plt.suptitle("")
        plt.ylabel(metric.replace("_", " ").title())
        plt.xticks(rotation=30)
        plt.tight_layout()

        plt.savefig(
            f"{OUTDIR}/box_{metric}_{size}.png"
        )
        plt.close()


def generate_plots(csv_path, npz_path, outdir="plots"):
    global CSV_PATH, NPZ_PATH, OUTDIR

    CSV_PATH = csv_path
    NPZ_PATH = npz_path
    OUTDIR = outdir

    ensure_dir(OUTDIR)

    df, heatmaps = load_data()

    # 1. per-map performance + heatmaps
    for size in MAP_SIZES:
        for map_id in sorted(df[df["map_size"] == size]["map_id"].unique()):
            plot_performance_per_map(df, size, map_id)
            plot_heatmaps_per_map(df, heatmaps, size, map_id)

    # 2. box plots
    for metric in METRICS:
        boxplot_metric_by_size(df, metric)



if __name__ == "__main__":
    main()
