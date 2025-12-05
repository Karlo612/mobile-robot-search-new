import matplotlib.pyplot as plt
import numpy as np


def plot_all(results):
    """
    Creates THREE grouped bar plots:
        1. Runtime
        2. Expansions
        3. Memory usage
    """

    # Extract unique test groups (Small, Medium, Large)
    titles = sorted(list({r.name.split(" - ")[0] for r in results}))

    # Extract planner names (A* Graph, A* Tree, BFS... etc)
    planners = sorted(list({r.name.split(" - ")[1] for r in results}))

    # Create lookup table: metrics[metric][title][planner]
    runtime = {t: {p: 0 for p in planners} for t in titles}
    expansions = {t: {p: 0 for p in planners} for t in titles}
    memory = {t: {p: 0 for p in planners} for t in titles}

    # Fill lookup tables
    for r in results:
        title, plan = r.name.split(" - ")
        runtime[title][plan] = r.runtime
        expansions[title][plan] = r.expansions
        memory[title][plan] = r.memory_kb

    # Convert to plotting arrays
    x = np.arange(len(titles))  # Small, Medium, Large
    width = 0.8 / len(planners)  # width of individual bars

    fig, axs = plt.subplots(3, 1, figsize=(12, 15))
    metric_data = [
        ("Runtime (s)", runtime),
        ("Expansions", expansions),
        ("Memory (KB)", memory),
    ]

    for ax, (label, dataset) in zip(axs, metric_data):

        for i, planner in enumerate(planners):
            values = [dataset[t][planner] for t in titles]
            ax.bar(x + i * width, values, width, label=planner)

        ax.set_title(label, fontsize=14)
        ax.set_xticks(x + width * (len(planners)/2))
        ax.set_xticklabels(titles)
        ax.legend()
        ax.grid(True, linestyle="--", alpha=0.4)

    plt.tight_layout()
    plt.show()
