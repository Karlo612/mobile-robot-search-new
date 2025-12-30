import tkinter as tk
import os

from tkinter import ttk, messagebox, filedialog
import threading
import csv
import numpy as np
from CodeBase.Util.heatmap_utils import expansion_map_to_array


from CodeBase.Evaluation.run_on_map import run_planner_on_map
from CodeBase.Util.random_grid_factory import create_random_grid_environment
from CodeBase.Evaluation.plot_experiment_results import generate_plots




class ExperimentGUI(tk.Toplevel):
    def __init__(self, parent):
        super().__init__(parent)

        self.title("Search Experiment Runner")
        w = self.winfo_screenwidth()
        h = self.winfo_screenheight()
        self.geometry(f"{w}x{h}")

        self.maps = {}       # size -> list[env_data]
        self.results = []    # list[dict]
        self._is_running = False
        self.heatmaps = {}   # heatmap_id -> np.ndarray
        self.grids = {}     # map_id -> np.ndarray
        self.starts = {}
        self.goals = {}
        self.paths = {}

        self._build_ui()

    # --------------------------------------------------
    # UI
    # --------------------------------------------------
    def _build_ui(self):
        left = ttk.Frame(self, width=320)
        left.pack(side="left", fill="y", padx=10, pady=10)
        left.pack_propagate(False)

        right = ttk.Frame(self)
        right.pack(side="right", fill="both", expand=True, padx=10, pady=10)

        ttk.Label(left, text="Experiment Setup", font=("Arial", 11, "bold")).pack(anchor="w", pady=(0, 8))

        # ---- Map sizes ----
        ttk.Label(left, text="Map Sizes").pack(anchor="w")
        self.use_small = tk.BooleanVar(value=True)
        self.use_medium = tk.BooleanVar(value=True)
        self.use_large = tk.BooleanVar(value=True)

        ttk.Checkbutton(left, text="Small (5 maps @ 10x10)", variable=self.use_small).pack(anchor="w")
        ttk.Checkbutton(left, text="Medium (5 maps @ 50x50)", variable=self.use_medium).pack(anchor="w")
        ttk.Checkbutton(left, text="Large (5 maps @ 100x100)", variable=self.use_large).pack(anchor="w")

        # ---- Parameters ----
        ttk.Label(left, text="Obstacle Ratio").pack(anchor="w", pady=(10, 0))
        self.obstacle_ratio = ttk.Entry(left)
        self.obstacle_ratio.insert(0, "0.2")
        self.obstacle_ratio.pack(fill="x")

        ttk.Label(left, text="Robot Radius").pack(anchor="w", pady=(6, 0))
        self.robot_radius = ttk.Entry(left)
        self.robot_radius.insert(0, "0.5")
        self.robot_radius.pack(fill="x")

        ttk.Label(left, text="Resolution").pack(anchor="w", pady=(6, 0))
        self.resolution = ttk.Entry(left)
        self.resolution.insert(0, "1.0")
        self.resolution.pack(fill="x")

        ttk.Label(left, text="Motion Model").pack(anchor="w", pady=(6, 0))
        self.motion = ttk.Combobox(left, values=["4n", "8n"], state="readonly")
        self.motion.current(0)
        self.motion.pack(fill="x")

        # ---- Planners ----
        ttk.Label(left, text="Planners", font=("Arial", 10, "bold")).pack(anchor="w", pady=(12, 4))

        self.planners = {
            "BFS Graph": tk.BooleanVar(value=True),
            "DFS Graph": tk.BooleanVar(value=True),
            "A* Graph": tk.BooleanVar(value=True),
            "BFS Tree": tk.BooleanVar(value=False),
            "DFS Tree": tk.BooleanVar(value=False),
            "A* Tree": tk.BooleanVar(value=False),
        }

        for k, v in self.planners.items():
            ttk.Checkbutton(left, text=k, variable=v).pack(anchor="w")

        # ---- Buttons ----
        ttk.Separator(left).pack(fill="x", pady=10)

        self.btn_generate = ttk.Button(left, text="Generate Maps", command=self.generate_maps)
        self.btn_generate.pack(fill="x", pady=4)

        self.btn_run = ttk.Button(left, text="Run Experiment", command=self.run_experiment)
        self.btn_run.pack(fill="x", pady=4)

        self.btn_export = ttk.Button(left, text="Export CSV", command=self.export_csv)
        self.btn_export.pack(fill="x", pady=4)

        self.btn_plot = ttk.Button(left,text="Generate Plots",command=self.generate_plots_from_files)
        self.btn_plot.pack(fill="x", pady=4)


        # ---- Progress ----
        ttk.Label(left, text="Progress").pack(anchor="w", pady=(10, 0))
        self.progress = ttk.Progressbar(left, mode="determinate")
        self.progress.pack(fill="x", pady=4)

        # ---- Log ----
        ttk.Label(right, text="Log", font=("Arial", 10, "bold")).pack(anchor="w")
        self.log = tk.Text(right, state="disabled", height=10)
        self.log.pack(fill="both", expand=True)

    # --------------------------------------------------
    # Thread-safe logging
    # --------------------------------------------------
    def log_msg(self, msg: str):
        # Must be called on UI thread
        self.log.configure(state="normal")
        self.log.insert("end", msg + "\n")
        self.log.configure(state="disabled")
        self.log.see("end")

    def log_msg_async(self, msg: str):
        # Safe from any thread
        self.after(0, lambda: self.log_msg(msg))

    def _set_running_ui(self, running: bool):
        def apply():
            self._is_running = running
            state = "disabled" if running else "normal"
            self.btn_generate.configure(state=state)
            self.btn_run.configure(state=state)
            self.btn_export.configure(state=("normal" if (not running and self.results) else state))

        self.after(0, apply)

    # --------------------------------------------------
    # Generate maps ONCE
    # --------------------------------------------------
    def generate_maps(self):
        # reset experiment state
        self.maps.clear()
        self.results.clear()

        # storage for NPZ export (one-time per map)
        self.grids = {}
        self.starts = {}
        self.goals = {}
        self.heatmaps = {}

        try:
            obstacle_ratio = float(self.obstacle_ratio.get())
            robot_radius = float(self.robot_radius.get())
            resolution = float(self.resolution.get())

            seed_txt = self.seed_entry.get().strip() if hasattr(self, "seed_entry") else ""
            base_seed = int(seed_txt) if seed_txt else None

        except Exception as e:
            messagebox.showerror("Invalid Input", str(e))
            return

        config = [
            ("small", 10, 5, self.use_small.get()),
            ("medium", 50, 5, self.use_medium.get()),
            ("large", 100, 5, self.use_large.get()),
        ]

        if not any(enabled for *_, enabled in config):
            messagebox.showwarning("No Sizes", "Select at least one map size.")
            return

        self.log_msg("[MAP GEN] Generating maps...")

        for name, size, count, enabled in config:
            if not enabled:
                continue

            self.maps[name] = []
            success = 0
            skipped = 0

            for i in range(count):

                # deterministic per-map seed
                if base_seed is not None:
                    import random, numpy as np
                    s = base_seed + (hash(name) % 10_000) + i * 97
                    random.seed(s)
                    np.random.seed(s % (2**32 - 1))

                try:
                    grid_map, world_map, robot, obstacles = create_random_grid_environment(
                        size=size,
                        obstacle_ratio=obstacle_ratio,
                        robot_radius=robot_radius,
                        resolution=resolution
                    )

                    # unique map id (stable across planners)
                    map_id = f"{name}_{i}"

                    # ---- STORE STATIC MAP DATA (ONCE) ----
                    self.grids[map_id] = grid_map.grid.copy()
                    self.starts[map_id] = (robot.sx, robot.sy)
                    self.goals[map_id] = (robot.gx, robot.gy)

                    # ---- ENV DATA FOR PLANNERS ----
                    env = {
                        "grid_map": grid_map,
                        "world_map": world_map,
                        "robot": robot,
                        "obstacles": obstacles,

                        # freeze start / goal for safety
                        "start": (robot.sx, robot.sy),
                        "goal": (robot.gx, robot.gy),

                        "meta": {
                            "map_id": map_id,
                            "size_name": name,
                            "size": size,
                            "map_index": i,
                            "obstacle_ratio": obstacle_ratio,
                            "robot_radius": robot_radius,
                            "resolution": resolution,
                            "seed": base_seed,
                        }
                    }

                    self.maps[name].append(env)
                    success += 1

                except Exception as e:
                    skipped += 1
                    self.log_msg(f"[SKIP] {name} map {i} failed: {e}")

            self.log_msg(
                f"[OK] {name} ({size}x{size}): "
                f"{success}/{count} generated, {skipped} skipped"
            )

        self.log_msg("[DONE] Map generation complete ✅")



    # --------------------------------------------------
    # Run experiment (threaded)
    # --------------------------------------------------
    def run_experiment(self):
        if self._is_running:
            return

        if not self.maps:
            messagebox.showwarning("No Maps", "Generate maps first.")
            return

        planner_defs = self._selected_planners()
        if not planner_defs:
            messagebox.showwarning("No Planners", "Select at least one planner.")
            return

        self._set_running_ui(True)
        threading.Thread(target=self._run_experiment_thread, args=(planner_defs,), daemon=True).start()

    def _selected_planners(self):
        selected = []

        # matches your pipeline: planner + use_tree_search
        if self.planners["BFS Graph"].get():
            selected.append(("BFS", False))
        if self.planners["DFS Graph"].get():
            selected.append(("DFS", False))
        if self.planners["A* Graph"].get():
            selected.append(("Astar", False))    
        if self.planners["BFS Tree"].get():
            selected.append(("BFS", True))
        if self.planners["DFS Tree"].get():
            selected.append(("DFS", True))
        if self.planners["A* Tree"].get():
            selected.append(("Astar", True)) 

        return selected

    def _run_experiment_thread(self, planner_defs):
        try:
            motion = self.motion.get()

            total_jobs = sum(len(envs) for envs in self.maps.values()) * len(planner_defs)
            self.after(0, lambda: self._init_progress(total_jobs))

            job = 0

            for size_name, envs in self.maps.items():
                for mid, env_data in enumerate(envs):

                    # stable map id (matches generate_maps)
                    map_id = env_data["meta"]["map_id"]

                    for planner_name, use_tree_search in planner_defs:
                        exec_cfg = {
                            "planner": planner_name,
                            "motion": motion,
                            "use_tree_search": use_tree_search,
                            "visualize_search": False,   # headless
                            "navigation_mode": "batch",
                        }

                        # ---- RUN PLANNER ----
                        result = run_planner_on_map(env_data, exec_cfg)

                        # capture paths

                        path = result.pop("path", None)

                        path_id = (
                            f"{map_id}_{planner_name}_{use_tree_search}_{motion}"
                        )

                        if path is not None:
                            self.paths[path_id] = path

                        # ---- HEATMAP CAPTURE ----
                        grid_map = env_data["grid_map"]
                        width, height = grid_map.width, grid_map.height

                        heatmap = expansion_map_to_array(
                            result.get("expansion_map", {}),
                            width,
                            height
                        )

                        # debug code 

                        nonzero = np.count_nonzero(heatmap)
                        maxval = heatmap.max()

                        self.log_msg_async(
                            f"[DEBUG] heatmap nonzero={nonzero}, max={maxval}"
                        )
                        # end debug code


                        heatmap_id = (
                            f"{map_id}_{planner_name}_{use_tree_search}_{motion}"
                        )

                        self.heatmaps[heatmap_id] = heatmap

                        # ---- FLATTEN RESULT FOR CSV ----
                        result.pop("heatmap", None)          # remove dict
                        result["heatmap_id"] = heatmap_id   # reference only

                        # metadata (CSV-safe)
                        result["map_size"] = size_name
                        result["map_id"] = mid
                        result["planner"] = planner_name
                        result["tree"] = use_tree_search
                        result["motion"] = motion

                        self.results.append(result)

                        job += 1
                        self.after(0, lambda v=job: self._update_progress(v))

                        self.log_msg_async(
                            f"{size_name} map {mid} | {planner_name} "
                            f"{'Tree' if use_tree_search else 'Graph'} | "
                            f"time={result.get('runtime_ms', -1):.1f}ms "
                            f"exp={result.get('expanded_nodes', 'NA')} "
                            f"path={result.get('path_len', 'NA')}"
                        )

            self.log_msg_async("[DONE] Experiment complete")

        except Exception as e:
            self.log_msg_async(f"[ERROR] {e}")

        finally:
            self._set_running_ui(False)


    def _init_progress(self, total_jobs: int):
        self.progress["value"] = 0
        self.progress["maximum"] = max(1, total_jobs)

    def _update_progress(self, value: int):
        self.progress["value"] = value
        self.update_idletasks()

    # --------------------------------------------------
    # Export
    # --------------------------------------------------
    def export_csv(self):
        if self._is_running:
            return

        if not self.results:
            messagebox.showwarning("No Data", "Run experiment first.")
            return

        path = filedialog.asksaveasfilename(defaultextension=".csv")
        if not path:
            return

        # ---- CSV ----
        cols = sorted({k for r in self.results for k in r.keys()})
        with open(path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=cols)
            writer.writeheader()
            for r in self.results:
                writer.writerow(r)

        # ---- NPZ ----
        import numpy as np

        np.savez_compressed(
            path.replace(".csv", "_data.npz"),

            # grids
            **{f"grid/{k}": v for k, v in self.grids.items()},

            # start/goal
            **{f"start/{k}": v for k, v in self.starts.items()},
            **{f"goal/{k}": v for k, v in self.goals.items()},

            # heatmaps
            **{f"heatmap/{k}": v for k, v in self.heatmaps.items()},
            
            #paths
            **{f"path/{k}": v for k, v in self.paths.items()},
        )

        self.log_msg(f"[OK] Exported CSV + NPZ to: {path}")


    def generate_plots_from_files(self):
        try:
            # 1. Ask for CSV
            csv_path = filedialog.askopenfilename(
                title="Select experiment CSV file",
                filetypes=[("CSV files", "*.csv")]
            )
            if not csv_path:
                return

            # 2. Infer NPZ path
            npz_path = csv_path.replace(".csv", "_data.npz")
            if not os.path.exists(npz_path):
                npz_path = filedialog.askopenfilename(
                    title="Select experiment NPZ file",
                    filetypes=[("NPZ files", "*.npz")]
                )
                if not npz_path:
                    return

            # 3. Output directory
            outdir = filedialog.askdirectory(
                title="Select output directory for plots"
            )
            if not outdir:
                return

            # 4. Run plotter (can be slow → thread)
            threading.Thread(
                target=self._run_plotter_thread,
                args=(csv_path, npz_path, outdir),
                daemon=True
            ).start()

        except Exception as e:
            messagebox.showerror("Plot Error", str(e))

    def _run_plotter_thread(self, csv_path, npz_path, outdir):
        try:
            self.after(0, lambda: self.btn_plot.config(state="disabled"))
            self.log_msg_async("[PLOT] Generating plots...")

            generate_plots(csv_path, npz_path, outdir)

            self.log_msg_async(f"[OK] Plots generated in: {outdir}")
            self.after(
                0,
                lambda: messagebox.showinfo(
                    "Plots Generated",
                    f"Plots successfully generated in:\n{outdir}"
                )
            )

        except Exception as e:
            self.log_msg_async(f"[ERROR] Plotting failed: {e}")
            self.after(
                0,
                lambda: messagebox.showerror("Plot Error", str(e))
            )

        finally:
            self.after(0, lambda: self.btn_plot.config(state="normal"))
