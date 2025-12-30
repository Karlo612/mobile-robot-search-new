import tkinter as tk
import os
import json
from datetime import datetime

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
        
        # Navigation state for heatmap view
        self.current_heatmap_map_idx = 0
        self.heatmap_map_list = []  # List of (map_size, map_id) tuples
        self._last_map_filter = None  # Track last filter to detect changes

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

        # ---- Seed (optional, for reproducible maps) ----
        ttk.Label(left, text="Random Seed (optional)").pack(anchor="w", pady=(6, 0))
        self.seed_entry = ttk.Entry(left)
        self.seed_entry.insert(0, "")
        self.seed_entry.pack(fill="x")
        ttk.Label(left, text="Leave empty for random", font=("Arial", 8)).pack(anchor="w", pady=(2, 0))

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

        self.btn_view_results = ttk.Button(left, text="View Results", command=self._show_results)
        self.btn_view_results.pack(fill="x", pady=4)
        self.btn_view_results.config(state="disabled")

        # ---- Progress ----
        ttk.Label(left, text="Progress").pack(anchor="w", pady=(10, 0))
        self.progress = ttk.Progressbar(left, mode="determinate")
        self.progress.pack(fill="x", pady=4)

        # ---- Create notebook for tabs (Log, Results Table, Plots, Comparison) ----
        self.notebook = ttk.Notebook(right)
        self.notebook.pack(fill="both", expand=True, pady=(5, 0))
        
        # Tab 1: Log
        log_frame = ttk.Frame(self.notebook)
        self.notebook.add(log_frame, text="Log")
        self.log = tk.Text(log_frame, state="disabled", height=10)
        self.log.pack(fill="both", expand=True)
        
        # Tab 2: Results Table
        table_frame = ttk.Frame(self.notebook)
        self.notebook.add(table_frame, text="Results Table")
        self._build_results_table(table_frame)
        
        # Tab 3: Comparison Plots
        plots_frame = ttk.Frame(self.notebook)
        self.notebook.add(plots_frame, text="Comparison Plots")
        self._build_plots_panel(plots_frame)
        
        # Tab 4: Statistics Summary
        stats_frame = ttk.Frame(self.notebook)
        self.notebook.add(stats_frame, text="Statistics")
        self._build_stats_panel(stats_frame)

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
            self.btn_view_results.configure(state=("normal" if (not running and self.results) else "disabled"))
            self.btn_plot.configure(state=("normal" if (not running and self.results) else state))

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
            
            # Update all views
            self.after(0, self._update_table)
            self.after(0, self._update_plot)
            self.after(0, self._update_stats)

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
    
    # --------------------------------------------------
    # Results Table
    # --------------------------------------------------
    def _build_results_table(self, parent):
        """Build results table with sortable columns"""
        # Toolbar
        toolbar = ttk.Frame(parent)
        toolbar.pack(fill="x", padx=5, pady=5)
        
        ttk.Label(toolbar, text="Filter by:").pack(side="left", padx=5)
        
        self.filter_planner = ttk.Combobox(toolbar, values=["All"], state="readonly", width=12)
        self.filter_planner.current(0)
        self.filter_planner.pack(side="left", padx=5)
        self.filter_planner.bind("<<ComboboxSelected>>", lambda e: self._update_table())
        
        self.filter_size = ttk.Combobox(toolbar, values=["All", "small", "medium", "large"], 
                                        state="readonly", width=12)
        self.filter_size.current(0)
        self.filter_size.pack(side="left", padx=5)
        self.filter_size.bind("<<ComboboxSelected>>", lambda e: self._update_table())
        
        ttk.Button(toolbar, text="Refresh", command=self._update_table).pack(side="left", padx=5)
        
        # Table with scrollbars
        table_container = ttk.Frame(parent)
        table_container.pack(fill="both", expand=True, padx=5, pady=5)
        
        scrollbar_y = ttk.Scrollbar(table_container, orient="vertical")
        scrollbar_x = ttk.Scrollbar(table_container, orient="horizontal")
        
        columns = ("Planner", "Mode", "Map Size", "Map ID", "Runtime (ms)", 
                   "Expanded Nodes", "Path Length", "Path Cost", "Memory (KB)", "Found")
        
        self.results_tree = ttk.Treeview(table_container, columns=columns, show="headings",
                                         yscrollcommand=scrollbar_y.set,
                                         xscrollcommand=scrollbar_x.set)
        
        scrollbar_y.config(command=self.results_tree.yview)
        scrollbar_x.config(command=self.results_tree.xview)
        
        # Configure columns
        col_widths = {"Planner": 80, "Mode": 60, "Map Size": 70, "Map ID": 60,
                     "Runtime (ms)": 100, "Expanded Nodes": 110, "Path Length": 90,
                     "Path Cost": 90, "Memory (KB)": 90, "Found": 50}
        
        for col in columns:
            self.results_tree.heading(col, text=col, command=lambda c=col: self._sort_table(c))
            self.results_tree.column(col, width=col_widths.get(col, 100), anchor="center")
        
        self.results_tree.pack(side="left", fill="both", expand=True)
        scrollbar_y.pack(side="right", fill="y")
        scrollbar_x.pack(side="bottom", fill="x")
        
        # Store sort state
        self._sort_column = None
        self._sort_reverse = False
    
    def _update_table(self):
        """Update results table with current data"""
        if not self.results:
            return
        
        # Clear existing items
        for item in self.results_tree.get_children():
            self.results_tree.delete(item)
        
        # Get filter values
        planner_filter = self.filter_planner.get()
        size_filter = self.filter_size.get()
        
        # Update planner filter options
        planners = sorted(set(r.get("planner", "") for r in self.results))
        self.filter_planner['values'] = ["All"] + planners
        if planner_filter not in ["All"] + planners:
            planner_filter = "All"
            self.filter_planner.current(0)
        
        # Filter and add rows
        for result in self.results:
            planner = result.get("planner", "")
            size = result.get("map_size", "")
            
            if planner_filter != "All" and planner != planner_filter:
                continue
            if size_filter != "All" and size != size_filter:
                continue
            
            mode = "Tree" if result.get("tree", False) else "Graph"
            found = "Yes" if result.get("found", False) else "No"
            
            values = (
                planner,
                mode,
                size,
                str(result.get("map_id", "")),
                f"{result.get('runtime_ms', 0):.2f}",
                str(result.get("expanded_nodes", "N/A")),
                str(result.get("path_len", "N/A")),
                f"{result.get('path_cost', 0):.2f}" if result.get("path_cost") != float('inf') else "N/A",
                f"{result.get('memory_kb', 0):.2f}",
                found
            )
            
            self.results_tree.insert("", "end", values=values)
    
    def _sort_table(self, column):
        """Sort table by column"""
        if self._sort_column == column:
            self._sort_reverse = not self._sort_reverse
        else:
            self._sort_reverse = False
            self._sort_column = column
        
        # Get all items
        items = [(self.results_tree.set(item, column), item) 
                for item in self.results_tree.get_children("")]
        
        # Sort
        try:
            items.sort(key=lambda x: float(x[0]) if x[0] not in ["N/A", "Yes", "No"] else 0, 
                     reverse=self._sort_reverse)
        except ValueError:
            items.sort(key=lambda x: x[0], reverse=self._sort_reverse)
        
        # Rearrange items
        for index, (val, item) in enumerate(items):
            self.results_tree.move(item, "", index)
    
    # --------------------------------------------------
    # Plots Panel
    # --------------------------------------------------
    def _build_plots_panel(self, parent):
        """Build interactive plots panel"""
        import matplotlib
        matplotlib.use('TkAgg')
        from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
        from matplotlib.figure import Figure
        
        # Control panel
        plot_controls = ttk.Frame(parent)
        plot_controls.pack(fill="x", padx=5, pady=5)
        
        ttk.Label(plot_controls, text="Metric:").pack(side="left", padx=5)
        self.plot_metric = ttk.Combobox(plot_controls, 
                                       values=["runtime_ms", "expanded_nodes", "path_len", 
                                              "path_cost", "memory_kb"],
                                       state="readonly", width=15)
        self.plot_metric.current(0)
        self.plot_metric.pack(side="left", padx=5)
        self.plot_metric.bind("<<ComboboxSelected>>", lambda e: self._update_plot())
        
        ttk.Label(plot_controls, text="View:").pack(side="left", padx=5)
        self.plot_view = ttk.Combobox(plot_controls,
                                      values=["Box Plot", 
                                             "Per Map Comparison", "Per Size Comparison", "Heatmap"],
                                      state="readonly", width=20)
        self.plot_view.current(0)
        self.plot_view.pack(side="left", padx=5)
        def on_view_change(e):
            # #region agent log
            try:
                with open('/Users/karlo/Desktop/T1AAI/aip/mobile-robot-search/.cursor/debug.log', 'a') as f:
                    f.write(json.dumps({"sessionId":"debug-session","runId":"run1","hypothesisId":"E","location":"experiment_gui.py:on_view_change","message":"View combobox changed","data":{"new_view":self.plot_view.get()},"timestamp":int(datetime.now().timestamp()*1000)}) + '\n')
            except: pass
            # #endregion
            self._toggle_map_filter()
            self._update_plot()
        self.plot_view.bind("<<ComboboxSelected>>", on_view_change)
        
        # Map filter for heatmap view
        ttk.Label(plot_controls, text="Map:").pack(side="left", padx=5)
        self.plot_map_filter = ttk.Combobox(plot_controls, values=["All"], 
                                             state="readonly", width=12)
        self.plot_map_filter.current(0)
        self.plot_map_filter.pack_forget()  # Hidden initially
        self.plot_map_filter.bind("<<ComboboxSelected>>", lambda e: (self._reset_heatmap_navigation(), self._update_plot()))
        
        # Planner filter for heatmap view
        ttk.Label(plot_controls, text="Planner:").pack(side="left", padx=5)
        self.plot_planner_filter = ttk.Combobox(plot_controls, values=["All"], 
                                                 state="readonly", width=12)
        self.plot_planner_filter.current(0)
        self.plot_planner_filter.pack_forget()  # Hidden initially
        self.plot_planner_filter.bind("<<ComboboxSelected>>", lambda e: self._update_plot())
        
        # Navigation controls for heatmap view
        self.heatmap_nav_frame = ttk.Frame(plot_controls)
        self.heatmap_nav_frame.pack_forget()  # Hidden initially
        
        # Single button to cycle through maps (loops from 4 back to 0)
        self.btn_next_map = ttk.Button(self.heatmap_nav_frame, text="Next ▶", 
                                       command=self._cycle_heatmap_map, width=10)
        self.btn_next_map.pack(side="left", padx=2)
        
        self.heatmap_map_label = ttk.Label(self.heatmap_nav_frame, text="Map: 1/5", width=15)
        self.heatmap_map_label.pack(side="left", padx=5)
        
        ttk.Button(plot_controls, text="Update Plot", command=self._update_plot).pack(side="left", padx=5)
        
        # Add a separator/spacer to ensure navigation frame is visible
        ttk.Separator(plot_controls, orient="vertical").pack(side="left", fill="y", padx=5)
        
        # Matplotlib figure
        self.plot_fig = Figure(figsize=(10, 6), dpi=100)
        self.plot_ax = self.plot_fig.add_subplot(111)
        
        self.plot_canvas = FigureCanvasTkAgg(self.plot_fig, master=parent)
        self.plot_canvas.get_tk_widget().pack(fill="both", expand=True, padx=5, pady=5)
        
        # Initial empty plot
        self.plot_ax.text(0.5, 0.5, "Run experiment to see plots", 
                         ha="center", va="center", fontsize=14)
        self.plot_ax.set_xticks([])
        self.plot_ax.set_yticks([])
        self.plot_canvas.draw()
    
    def _update_plot(self):
        """Update comparison plot"""
        if not self.results:
            return
        
        import pandas as pd
        import matplotlib
        matplotlib.use('TkAgg')
        
        df = pd.DataFrame(self.results)
        metric = self.plot_metric.get()
        view_type = self.plot_view.get()
        
        # Handle heatmap view separately
        if view_type == "Heatmap":
            self._plot_heatmaps(df)
            return
        
        # Handle per-map comparison separately
        if view_type == "Per Map Comparison":
            self._plot_per_map_comparison(df, metric)
            return
        
        # Handle per-size comparison separately
        if view_type == "Per Size Comparison":
            self._plot_per_size_comparison(df, metric)
            return
        
        if metric not in df.columns:
            return
        
        # Create planner label
        df["planner_label"] = df["planner"] + "-" + df["tree"].map({True: "Tree", False: "Graph"})
        
        self.plot_ax.clear()
        
        if view_type == "Box Plot":
            import numpy as np
            # Group by planner_label
            data_by_planner = [df[df["planner_label"] == label][metric].values 
                              for label in df["planner_label"].unique()]
            labels = df["planner_label"].unique()
            
            bp = self.plot_ax.boxplot(data_by_planner, labels=labels, patch_artist=True)
            
            # Use log scale for y-axis if metric is runtime_ms or path_len
            if metric in ["runtime_ms", "path_len"]:
                # Filter out zero or negative values for log scale
                all_positive = all(len(data) > 0 and np.all(data > 0) for data in data_by_planner)
                if all_positive:
                    self.plot_ax.set_yscale('log')
                    # Set minimum y value to avoid log(0) issues
                    min_val = min(np.min(data) for data in data_by_planner)
                    self.plot_ax.set_ylim(bottom=min_val * 0.5)  # Set bottom slightly below min
            
            self.plot_ax.set_ylabel(metric.replace("_", " ").title())
            self.plot_ax.set_title(f"{metric.replace('_', ' ').title()} Comparison")
            self.plot_ax.tick_params(axis='x', rotation=45)
        
        self.plot_fig.tight_layout()
        self.plot_canvas.draw()
    
    def _plot_per_map_comparison(self, df, metric):
        """Display individual map comparisons grouped by map size"""
        import numpy as np
        
        if metric not in df.columns:
            self.plot_fig.clear()
            ax = self.plot_fig.add_subplot(111)
            ax.text(0.5, 0.5, f"Metric '{metric}' not found", 
                   ha="center", va="center", fontsize=14)
            ax.set_xticks([])
            ax.set_yticks([])
            self.plot_fig.tight_layout()
            self.plot_canvas.draw()
            return
        
        # Create planner label
        df["planner_label"] = df["planner"] + "-" + df["tree"].map({True: "Tree", False: "Graph"})
        
        # Group by map_size and map_id
        unique_maps = df[["map_size", "map_id"]].drop_duplicates().sort_values(["map_size", "map_id"])
        
        if unique_maps.empty:
            self.plot_fig.clear()
            ax = self.plot_fig.add_subplot(111)
            ax.text(0.5, 0.5, "No maps found", 
                   ha="center", va="center", fontsize=14)
            ax.set_xticks([])
            ax.set_yticks([])
            self.plot_fig.tight_layout()
            self.plot_canvas.draw()
            return
        
        # Group by map size
        map_sizes = sorted(df["map_size"].unique())
        n_sizes = len(map_sizes)
        
        # Calculate grid: one row per map size, columns for maps within that size
        max_maps_per_size = max(len(df[df["map_size"] == size]["map_id"].unique()) 
                                for size in map_sizes) if map_sizes else 1
        
        # Clear figure and create subplots
        self.plot_fig.clear()
        
        plot_idx = 0
        for size_idx, map_size in enumerate(map_sizes):
            size_maps = unique_maps[unique_maps["map_size"] == map_size]
            
            for map_idx, (_, row) in enumerate(size_maps.iterrows()):
                map_id = row["map_id"]
                
                # Get results for this specific map
                map_results = df[(df["map_size"] == map_size) & (df["map_id"] == map_id)]
                
                if map_results.empty:
                    continue
                
                # Create subplot: rows = map sizes, cols = max maps per size
                plot_idx += 1
                ax = self.plot_fig.add_subplot(n_sizes, max_maps_per_size, plot_idx)
                
                # Get data for each planner
                planners = sorted(map_results["planner_label"].unique())
                values = []
                labels = []
                
                for planner in planners:
                    planner_data = map_results[map_results["planner_label"] == planner]
                    if not planner_data.empty:
                        metric_value = planner_data[metric].iloc[0]
                        # Handle inf values
                        if metric_value != float('inf') and not np.isnan(metric_value):
                            values.append(metric_value)
                            labels.append(planner)
                
                if values:
                    # Create bar chart
                    bars = ax.bar(range(len(labels)), values)
                    ax.set_xticks(range(len(labels)))
                    ax.set_xticklabels(labels, rotation=45, ha="right", fontsize=8)
                    ax.set_ylabel(metric.replace("_", " ").title(), fontsize=8)
                    ax.set_title(f"{map_size} Map {map_id}", fontsize=9, fontweight='bold')
                    ax.grid(True, alpha=0.3, axis='y')
                    
                    # Use log scale for y-axis if metric is runtime_ms
                    if metric == "runtime_ms":
                        # Filter out zero or negative values for log scale
                        positive_values = [v for v in values if v > 0]
                        if positive_values:
                            ax.set_yscale('log')
                            # Set minimum y value to avoid log(0) issues
                            min_val = min(positive_values)
                            ax.set_ylim(bottom=min_val * 0.5)  # Set bottom slightly below min
                    
                    # Add value labels on bars
                    for i, (bar, val) in enumerate(zip(bars, values)):
                        height = bar.get_height()
                        # For log scale, position label relative to bar height
                        if metric == "runtime_ms" and val > 0:
                            label_y = height * 1.1  # Position above bar
                        else:
                            label_y = height
                        ax.text(bar.get_x() + bar.get_width()/2., label_y,
                               f'{val:.1f}' if val < 1000 else f'{val:.0f}',
                               ha='center', va='bottom', fontsize=7)
                else:
                    ax.text(0.5, 0.5, "No data", 
                           ha="center", va="center", fontsize=8)
                    ax.set_xticks([])
                    ax.set_yticks([])
        
        # Set overall title
        self.plot_fig.suptitle(f"{metric.replace('_', ' ').title()} - Individual Map Comparison", 
                              fontsize=12, fontweight='bold')
        
        self.plot_fig.tight_layout(rect=[0, 0, 1, 0.98])  # Leave space for suptitle
        self.plot_canvas.draw()
    
    def _plot_per_size_comparison(self, df, metric):
        """Display box plots grouped by map size"""
        import numpy as np
        import matplotlib.pyplot as plt
        
        if metric not in df.columns:
            self.plot_fig.clear()
            ax = self.plot_fig.add_subplot(111)
            ax.text(0.5, 0.5, f"Metric '{metric}' not found", 
                   ha="center", va="center", fontsize=14)
            ax.set_xticks([])
            ax.set_yticks([])
            self.plot_fig.tight_layout()
            self.plot_canvas.draw()
            return
        
        # Create planner label
        df["planner_label"] = df["planner"] + "-" + df["tree"].map({True: "Tree", False: "Graph"})
        
        # Get unique map sizes
        map_sizes = sorted(df["map_size"].unique())
        
        if not map_sizes:
            self.plot_fig.clear()
            ax = self.plot_fig.add_subplot(111)
            ax.text(0.5, 0.5, "No map sizes found", 
                   ha="center", va="center", fontsize=14)
            ax.set_xticks([])
            ax.set_yticks([])
            self.plot_fig.tight_layout()
            self.plot_canvas.draw()
            return
        
        # Clear figure and create subplots (one per map size)
        n_sizes = len(map_sizes)
        self.plot_fig.clear()
        
        # Create subplots in a row
        axes = []
        for idx, map_size in enumerate(map_sizes):
            ax = self.plot_fig.add_subplot(1, n_sizes, idx + 1)
            axes.append(ax)
            
            # Get data for this map size
            size_data = df[df["map_size"] == map_size]
            
            # Group by planner_label
            planners = sorted(size_data["planner_label"].unique())
            data_by_planner = []
            labels = []
            
            for planner in planners:
                planner_data = size_data[size_data["planner_label"] == planner][metric]
                # Filter out inf and NaN values
                valid_data = planner_data[(planner_data != float('inf')) & 
                                         (~np.isnan(planner_data))].values
                if len(valid_data) > 0:
                    data_by_planner.append(valid_data)
                    labels.append(planner)
            
            if data_by_planner:
                # Create box plot
                bp = ax.boxplot(data_by_planner, labels=labels, patch_artist=True)
                
                # Color the boxes
                colors = plt.cm.Set3(np.linspace(0, 1, len(bp['boxes'])))
                for patch, color in zip(bp['boxes'], colors):
                    patch.set_facecolor(color)
                
                # Use log scale for y-axis if:
                # - metric is runtime_ms and map size is medium or large
                # - metric is path_len and map size is large
                if (metric == "runtime_ms" and map_size in ["medium", "large"]) or \
                   (metric == "path_len" and map_size == "large"):
                    # Check if we have positive values for log scale
                    all_positive = all(len(data) > 0 and np.all(data > 0) for data in data_by_planner)
                    if all_positive:
                        ax.set_yscale('log')
                        # Set minimum y value to avoid log(0) issues
                        min_val = min(np.min(data) for data in data_by_planner)
                        ax.set_ylim(bottom=min_val * 0.5)  # Set bottom slightly below min
                
                ax.set_ylabel(metric.replace("_", " ").title(), fontsize=9)
                ax.set_title(f"{map_size.title()} Maps\n(n={len(size_data)} runs)", 
                            fontsize=10, fontweight='bold')
                ax.tick_params(axis='x', rotation=45, labelsize=8)
                ax.grid(True, alpha=0.3, axis='y')
            else:
                ax.text(0.5, 0.5, "No data", 
                       ha="center", va="center", fontsize=10)
                ax.set_xticks([])
                ax.set_yticks([])
                ax.set_title(f"{map_size.title()} Maps", fontsize=10, fontweight='bold')
        
        # Set overall title
        self.plot_fig.suptitle(f"{metric.replace('_', ' ').title()} - Comparison by Map Size", 
                              fontsize=12, fontweight='bold')
        
        self.plot_fig.tight_layout(rect=[0, 0, 1, 0.95])  # Leave space for suptitle
        self.plot_canvas.draw()
    
    def _toggle_map_filter(self):
        """Show/hide map filter and navigation based on view type"""
        # #region agent log
        view_type = self.plot_view.get()
        try:
            with open('/Users/karlo/Desktop/T1AAI/aip/mobile-robot-search/.cursor/debug.log', 'a') as f:
                f.write(json.dumps({"sessionId":"debug-session","runId":"run1","hypothesisId":"A","location":"experiment_gui.py:_toggle_map_filter","message":"_toggle_map_filter called","data":{"view_type":view_type},"timestamp":int(datetime.now().timestamp()*1000)}) + '\n')
        except: pass
        # #endregion
        
        if view_type == "Heatmap":
            # #region agent log
            try:
                with open('/Users/karlo/Desktop/T1AAI/aip/mobile-robot-search/.cursor/debug.log', 'a') as f:
                    f.write(json.dumps({"sessionId":"debug-session","runId":"run1","hypothesisId":"B","location":"experiment_gui.py:_toggle_map_filter","message":"Showing heatmap controls","data":{"before_pack":True},"timestamp":int(datetime.now().timestamp()*1000)}) + '\n')
            except: pass
            # #endregion
            
            # Pack map filter on left
            self.plot_map_filter.pack(side="left", padx=5)
            # Pack navigation frame on RIGHT side to ensure it's always visible and not cut off
            self.heatmap_nav_frame.pack(side="right", padx=5)
            
            # Ensure the cycle button and label are visible
            self.btn_next_map.pack(side="left", padx=2)
            self.heatmap_map_label.pack(side="left", padx=5)
            
            # Force update to ensure visibility
            self.heatmap_nav_frame.update_idletasks()
            
            # #region agent log
            try:
                nav_visible = self.heatmap_nav_frame.winfo_viewable() if hasattr(self.heatmap_nav_frame, 'winfo_viewable') else "unknown"
                nav_width = self.heatmap_nav_frame.winfo_width() if hasattr(self.heatmap_nav_frame, 'winfo_width') else "unknown"
                button_visible = self.btn_next_map.winfo_viewable() if hasattr(self.btn_next_map, 'winfo_viewable') else "unknown"
                with open('/Users/karlo/Desktop/T1AAI/aip/mobile-robot-search/.cursor/debug.log', 'a') as f:
                    f.write(json.dumps({"sessionId":"debug-session","runId":"run1","hypothesisId":"C","location":"experiment_gui.py:_toggle_map_filter","message":"After pack - nav frame state","data":{"nav_visible":str(nav_visible),"nav_width":str(nav_width),"button_visible":str(button_visible)},"timestamp":int(datetime.now().timestamp()*1000)}) + '\n')
            except: pass
            # #endregion
            
            # Hide planner filter for heatmap (we show all planners)
            self.plot_planner_filter.pack_forget()
        else:
            # #region agent log
            try:
                with open('/Users/karlo/Desktop/T1AAI/aip/mobile-robot-search/.cursor/debug.log', 'a') as f:
                    f.write(json.dumps({"sessionId":"debug-session","runId":"run1","hypothesisId":"A","location":"experiment_gui.py:_toggle_map_filter","message":"Hiding heatmap controls","data":{"view_type":view_type},"timestamp":int(datetime.now().timestamp()*1000)}) + '\n')
            except: pass
            # #endregion
            
            self.plot_map_filter.pack_forget()
            self.plot_planner_filter.pack_forget()
            self.heatmap_nav_frame.pack_forget()
    
    def _cycle_heatmap_map(self):
        """Cycle to next map, looping back to 0 after the last map"""
        # #region agent log
        try:
            with open('/Users/karlo/Desktop/T1AAI/aip/mobile-robot-search/.cursor/debug.log', 'a') as f:
                f.write(json.dumps({"sessionId":"debug-session","runId":"run1","hypothesisId":"D","location":"experiment_gui.py:_cycle_heatmap_map","message":"Cycle button clicked","data":{"current_idx":self.current_heatmap_map_idx,"list_len":len(self.heatmap_map_list) if self.heatmap_map_list else 0},"timestamp":int(datetime.now().timestamp()*1000)}) + '\n')
        except: pass
        # #endregion
        
        if not self.heatmap_map_list:
            return
        
        # Move to next map, looping back to 0 if at the end
        self.current_heatmap_map_idx = (self.current_heatmap_map_idx + 1) % len(self.heatmap_map_list)
        self._update_plot()
    
    def _reset_heatmap_navigation(self):
        """Reset heatmap navigation when filter changes"""
        # The _plot_heatmaps will handle the reset based on filter_changed flag
        pass
    
    def _plot_heatmaps(self, df):
        """Display heatmaps in a grid layout"""
        from matplotlib.colors import ListedColormap, BoundaryNorm
        import matplotlib.pyplot as plt
        import numpy as np
        
        # Get colormap (same as plot_experiment_results.py)
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
        
        # Filter results by map size if filter is set
        map_filter = self.plot_map_filter.get()
        if map_filter != "All":
            df = df[df["map_size"] == map_filter]
        
        # Create planner label
        df["planner_label"] = df["planner"] + "-" + df["tree"].map({True: "Tree", False: "Graph"})
        
        if df.empty:
            self.plot_fig.clear()
            ax = self.plot_fig.add_subplot(111)
            ax.text(0.5, 0.5, "No heatmaps available", 
                   ha="center", va="center", fontsize=14)
            ax.set_xticks([])
            ax.set_yticks([])
            self.plot_fig.tight_layout()
            self.plot_canvas.draw()
            return
        
        # Group by map_size and map_id
        unique_maps = df[["map_size", "map_id"]].drop_duplicates().sort_values(["map_size", "map_id"])
        
        if unique_maps.empty:
            self.plot_fig.clear()
            ax = self.plot_fig.add_subplot(111)
            ax.text(0.5, 0.5, "No maps found", 
                   ha="center", va="center", fontsize=14)
            ax.set_xticks([])
            ax.set_yticks([])
            self.plot_fig.tight_layout()
            self.plot_canvas.draw()
            return
        
        # Get current map size (from filter or from current map being viewed)
        if map_filter != "All":
            current_map_size = map_filter
        else:
            # If we have a current map list, use the size of the current map
            if self.heatmap_map_list and self.current_heatmap_map_idx < len(self.heatmap_map_list):
                current_map_size, _ = self.heatmap_map_list[self.current_heatmap_map_idx]
            else:
                # Default to first map's size
                current_map_size = unique_maps.iloc[0]["map_size"] if not unique_maps.empty else None
        
        # Filter to maps of current size only (navigate within size)
        if current_map_size:
            size_maps = unique_maps[unique_maps["map_size"] == current_map_size].sort_values("map_id")
        else:
            size_maps = unique_maps
        
        if size_maps.empty:
            self.plot_fig.clear()
            ax = self.plot_fig.add_subplot(111)
            ax.text(0.5, 0.5, f"No {current_map_size} maps found", 
                   ha="center", va="center", fontsize=14)
            ax.set_xticks([])
            ax.set_yticks([])
            self.plot_fig.tight_layout()
            self.plot_canvas.draw()
            return
        
        # Store map list for navigation (only maps of current size)
        new_map_list = [(row["map_size"], row["map_id"]) 
                       for _, row in size_maps.iterrows()]
        
        # Check if lists are different by comparing their content
        lists_different = (
            len(new_map_list) != len(self.heatmap_map_list) or
            new_map_list != self.heatmap_map_list
        )
        
        # Check if filter changed
        filter_changed = self._last_map_filter != map_filter
        self._last_map_filter = map_filter
        
        # Only reset index if we're switching sizes or filter changed
        # If we're navigating within the same size, preserve the index
        if filter_changed:
            # Filter changed - reset to first map of new filter
            self.current_heatmap_map_idx = 0
        elif lists_different:
            # Map list changed - check if it's a size change
            if self.heatmap_map_list and len(self.heatmap_map_list) > 0:
                old_size = self.heatmap_map_list[0][0]
                if current_map_size != old_size:
                    # Size changed - reset index
                    self.current_heatmap_map_idx = 0
                # else: same size, different maps (shouldn't happen, but preserve index if it does)
            else:
                # No previous list - reset
                self.current_heatmap_map_idx = 0
        # else: same list, preserve index (navigating within same size)
        
        self.heatmap_map_list = new_map_list
        
        # Ensure current index is valid
        if self.current_heatmap_map_idx >= len(self.heatmap_map_list):
            self.current_heatmap_map_idx = 0
        if self.current_heatmap_map_idx < 0:
            self.current_heatmap_map_idx = 0
        
        # Get current map
        map_size, map_id = self.heatmap_map_list[self.current_heatmap_map_idx]
        map_key = f"{map_size}_{map_id}"
        
        # Update navigation label to show size context
        map_size_label = map_size.title() if map_size else "Map"
        self.heatmap_map_label.config(text=f"{map_size_label}: {self.current_heatmap_map_idx + 1}/{len(self.heatmap_map_list)}")
        
        # Update button state - always enabled if we have maps (since it loops)
        if len(self.heatmap_map_list) > 1:
            # Multiple maps available - enable cycling
            self.btn_next_map.config(state="normal")
        else:
            # Only one map - disable button
            self.btn_next_map.config(state="disabled")
        
        # #region agent log
        try:
            nav_visible = self.heatmap_nav_frame.winfo_viewable() if hasattr(self.heatmap_nav_frame, 'winfo_viewable') else "unknown"
            with open('/Users/karlo/Desktop/T1AAI/aip/mobile-robot-search/.cursor/debug.log', 'a') as f:
                f.write(json.dumps({"sessionId":"debug-session","runId":"run1","hypothesisId":"C","location":"experiment_gui.py:_plot_heatmaps","message":"Updating button state","data":{"button_state":self.btn_next_map.cget("state"),"nav_frame_visible":str(nav_visible),"current_idx":self.current_heatmap_map_idx,"list_len":len(self.heatmap_map_list)},"timestamp":int(datetime.now().timestamp()*1000)}) + '\n')
        except: pass
        # #endregion
        
        # Get grid for this map
        if map_key not in self.grids:
            self.plot_fig.clear()
            ax = self.plot_fig.add_subplot(111)
            ax.text(0.5, 0.5, f"Grid not found for {map_key}", 
                   ha="center", va="center", fontsize=14)
            ax.set_xticks([])
            ax.set_yticks([])
            self.plot_fig.tight_layout()
            self.plot_canvas.draw()
            return
        
        grid = self.grids[map_key]
        
        # Get all planners for this map
        map_results = df[(df["map_size"] == map_size) & (df["map_id"] == map_id)]
        unique_planners = sorted(map_results["planner_label"].unique())
        
        if not unique_planners:
            self.plot_fig.clear()
            ax = self.plot_fig.add_subplot(111)
            ax.text(0.5, 0.5, f"No planners found for {map_key}", 
                   ha="center", va="center", fontsize=14)
            ax.set_xticks([])
            ax.set_yticks([])
            self.plot_fig.tight_layout()
            self.plot_canvas.draw()
            return
        
        # Clear and create subplots: one row, one column per planner
        n_planners = len(unique_planners)
        self.plot_fig.clear()
        
        for planner_idx, planner_label in enumerate(unique_planners):
            ax = self.plot_fig.add_subplot(1, n_planners, planner_idx + 1)
            
            # Get heatmap for this map and planner
            planner_result = map_results[map_results["planner_label"] == planner_label]
            
            if planner_result.empty:
                ax.text(0.5, 0.5, f"No data\n{planner_label}", 
                       ha="center", va="center", fontsize=10)
                ax.set_xticks([])
                ax.set_yticks([])
                continue
            
            result = planner_result.iloc[0]
            heatmap_id = result.get("heatmap_id")
            
            if heatmap_id and heatmap_id in self.heatmaps:
                heat = self.heatmaps[heatmap_id].copy()
                h, w = heat.shape
                
                # Overlay obstacles
                vis_grid = np.array(heat, dtype=float)
                for y in range(h):
                    for x in range(w):
                        if grid[y, x] != 0:
                            vis_grid[y, x] = -1
                
                # Plot heatmap
                im = ax.imshow(vis_grid, origin="lower", cmap=cmap, norm=norm, aspect='auto')
                
                # Add path if available
                path_id = heatmap_id
                if path_id in self.paths:
                    path = self.paths[path_id]
                    px = [p[0] for p in path]
                    py = [p[1] for p in path]
                    ax.plot(px, py, color="cyan", linewidth=2, marker='o', markersize=3)
                
                # Add start/goal markers if available (show on all planners)
                if map_key in self.starts:
                    sx, sy = self.starts[map_key]
                    ax.plot(sx, sy, 'go', markersize=8, label='Start')
                if map_key in self.goals:
                    gx, gy = self.goals[map_key]
                    ax.plot(gx, gy, 'ro', markersize=8, label='Goal')
                
                # Title with planner info
                ax.set_title(planner_label, fontsize=10, fontweight='bold')
                ax.set_xticks([])
                ax.set_yticks([])
                
                # Grid lines
                ax.set_xticks(np.arange(-0.5, w, 1), minor=True)
                ax.set_yticks(np.arange(-0.5, h, 1), minor=True)
                ax.grid(which="minor", color="gray", linestyle="-", linewidth=0.2)
            else:
                ax.text(0.5, 0.5, f"No heatmap\n{planner_label}", 
                       ha="center", va="center", fontsize=10)
                ax.set_xticks([])
                ax.set_yticks([])
        
        # Add colorbar (shared for all subplots)
        axes = self.plot_fig.get_axes()
        if axes:
            sm = plt.cm.ScalarMappable(cmap=cmap, norm=norm)
            sm.set_array([])
            cbar = self.plot_fig.colorbar(sm, ax=axes, 
                                         fraction=0.046, pad=0.04)
            cbar.set_label('Expansion Count', rotation=270, labelpad=15)
        
        # Update filter options
        map_sizes = sorted(set(df["map_size"].unique()) if not df.empty else [])
        self.plot_map_filter['values'] = ["All"] + list(map_sizes)
        
        # Set overall title
        self.plot_fig.suptitle(f"Heatmap Comparison: {map_key}", 
                              fontsize=12, fontweight='bold')
        
        self.plot_fig.tight_layout(rect=[0, 0, 1, 0.95])
        self.plot_canvas.draw()
    
    # --------------------------------------------------
    # Statistics Panel
    # --------------------------------------------------
    def _build_stats_panel(self, parent):
        """Build statistics summary panel"""
        self.stats_text = tk.Text(parent, state="disabled", wrap="word", font=("Courier", 10))
        self.stats_text.pack(fill="both", expand=True, padx=5, pady=5)
        
        scrollbar = ttk.Scrollbar(parent, command=self.stats_text.yview)
        scrollbar.pack(side="right", fill="y")
        self.stats_text.config(yscrollcommand=scrollbar.set)
    
    def _update_stats(self):
        """Update statistics summary"""
        if not self.results:
            return
        
        import pandas as pd
        
        df = pd.DataFrame(self.results)
        df["planner_label"] = df["planner"] + "-" + df["tree"].map({True: "Tree", False: "Graph"})
        
        metrics = ["runtime_ms", "expanded_nodes", "path_len", "path_cost", "memory_kb"]
        
        stats_text = "=" * 60 + "\n"
        stats_text += "EXPERIMENT STATISTICS SUMMARY\n"
        stats_text += "=" * 60 + "\n\n"
        
        for planner in sorted(df["planner_label"].unique()):
            subset = df[df["planner_label"] == planner]
            stats_text += f"\n{planner}:\n"
            stats_text += "-" * 40 + "\n"
            
            for metric in metrics:
                if metric in subset.columns:
                    values = subset[metric].dropna()
                    if len(values) > 0:
                        # Filter out inf values
                        finite_values = values[values != float('inf')]
                        if len(finite_values) > 0:
                            stats_text += f"  {metric.replace('_', ' ').title()}:\n"
                            stats_text += f"    Mean: {finite_values.mean():.2f}\n"
                            stats_text += f"    Median: {finite_values.median():.2f}\n"
                            stats_text += f"    Min: {finite_values.min():.2f}\n"
                            stats_text += f"    Max: {finite_values.max():.2f}\n"
                            stats_text += f"    Std: {finite_values.std():.2f}\n"
            
            found_count = subset["found"].sum() if "found" in subset.columns else 0
            stats_text += f"  Success Rate: {found_count}/{len(subset)} ({100*found_count/len(subset):.1f}%)\n"
        
        self.stats_text.config(state="normal")
        self.stats_text.delete(1.0, "end")
        self.stats_text.insert(1.0, stats_text)
        self.stats_text.config(state="disabled")
    
    # --------------------------------------------------
    # Show Results
    # --------------------------------------------------
    def _show_results(self):
        """Switch to results tab"""
        if self.results:
            self.notebook.select(1)  # Switch to Results Table tab
            self._update_table()
            self._update_plot()
            self._update_stats()
