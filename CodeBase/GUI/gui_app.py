# CodeBase/GUI/gui_app.py

import tkinter as tk
from tkinter import ttk, messagebox

import matplotlib.pyplot as plt

from CodeBase.navigation_system import NavigationSystem
from CodeBase.Util.random_grid_factory import create_random_grid_environment
from CodeBase.Visualization.embedded_visualizer import EmbeddedVisualizer
from CodeBase.GUI.experiment_gui import ExperimentGUI



class SearchGUI(tk.Tk):
    def __init__(self):
        super().__init__()

        self.title("Search Visualization Control")
        w = self.winfo_screenwidth()
        h = self.winfo_screenheight()
        self.geometry(f"{w}x{h}")

        self.resizable(True, True)

        self.env_data = None
        self.current_config = None
        self.vis = None

        # 🔴 ENVIRONMENT DIRTY FLAG
        self.env_dirty = True

        self._build_widgets()
        self.protocol("WM_DELETE_WINDOW", self.on_close)

    # --------------------------------------------------
    # Clean shutdown
    # --------------------------------------------------
    def on_close(self):
        # Schedule matplotlib cleanup on the Tk main thread
        self.after(0, self._safe_close)

    def _safe_close(self):
        try:
            import matplotlib.pyplot as plt
            plt.close("all")
        except Exception:
            pass
        self.destroy()


    # --------------------------------------------------
    # Mark environment dirty
    # --------------------------------------------------
    def _mark_env_dirty(self, *_):
        self.env_dirty = True
        self.run_button.config(state="disabled")

    # --------------------------------------------------
    # UI Layout
    # --------------------------------------------------
    def _build_widgets(self):

        pad = {"padx": 10, "pady": 5}

        container = ttk.Frame(self)
        container.pack(fill="both", expand=True)

        # ---------------- LEFT PANEL (controls) ----------------
        control_frame = ttk.Frame(container, width=320)
        control_frame.pack(side="left", fill="y")
        control_frame.pack_propagate(False)

        ttk.Label(
            control_frame,
            text="Search Visualization Control",
            font=("Arial", 14, "bold")
        ).pack(pady=10)

        # Grid size
        ttk.Label(control_frame, text="Grid Size").pack(**pad)
        self.grid_size = ttk.Combobox(
            control_frame,
            values=["Small (10x10)", "Medium (50x50)", "Large (100x100)"],
            state="readonly"
        )
        self.grid_size.current(0)
        self.grid_size.pack(**pad)
        self.grid_size.bind("<<ComboboxSelected>>", self._mark_env_dirty)

        # Resolution
        ttk.Label(control_frame, text="Grid Resolution").pack(**pad)
        self.grid_resolution = ttk.Entry(control_frame)
        self.grid_resolution.insert(0, "1.0")
        self.grid_resolution.pack(**pad)
        self.grid_resolution.bind("<KeyRelease>", self._mark_env_dirty)

        # Obstacle percentage
        ttk.Label(control_frame, text="Obstacle Percentage").pack(**pad)
        self.obstacle_scale = tk.Scale(
            control_frame,
            from_=0.05, to=0.40,
            resolution=0.05,
            orient=tk.HORIZONTAL,
            length=220
        )
        self.obstacle_scale.set(0.20)
        self.obstacle_scale.pack(**pad)
        self.obstacle_scale.bind("<ButtonRelease-1>", self._mark_env_dirty)

        # Robot radius
        ttk.Label(control_frame, text="Robot Radius").pack(**pad)
        self.robot_radius = ttk.Entry(control_frame)
        self.robot_radius.insert(0, "0.5")
        self.robot_radius.pack(**pad)
        self.robot_radius.bind("<KeyRelease>", self._mark_env_dirty)

        # Search algorithm
        ttk.Label(control_frame, text="Search Algorithm").pack(**pad)
        self.search_type = ttk.Combobox(
            control_frame,
            values=["Astar", "BFS", "DFS"],
            state="readonly"
        )
        self.search_type.current(0)
        self.search_type.pack(**pad)

        # Search mode
        ttk.Label(control_frame, text="Search Mode").pack(**pad)
        self.search_mode = ttk.Combobox(
            control_frame,
            values=["Graph-Based", "Tree-Based"],
            state="readonly"
        )
        self.search_mode.current(0)
        self.search_mode.pack(**pad)

        # Motion model
        ttk.Label(control_frame, text="Motion Model").pack(**pad)
        self.motion_model = ttk.Combobox(
            control_frame,
            values=["4n", "8n"],
            state="readonly"
        )
        self.motion_model.current(1)
        self.motion_model.pack(**pad)

        ttk.Separator(control_frame).pack(fill="x", pady=10)

        ttk.Button(
            control_frame,
            text="Generate Grid",
            command=self.generate_grid
        ).pack(**pad)

        self.run_button = ttk.Button(
            control_frame,
            text="Run Search",
            command=self.run_search,
            state="disabled"   # 🔴 disabled until grid is generated
        )
        self.run_button.pack(**pad)

        ttk.Separator(control_frame).pack(fill="x", pady=10)

        ttk.Button(
            control_frame,
            text="Run Experiments",
            command=self.open_experiment_gui
        ).pack(**pad)


        # ---------------- RIGHT PANEL (visualization) ----------------
        self.vis_frame = ttk.Frame(container)
        self.vis_frame.pack(side="right", fill="both", expand=True)

    # --------------------------------------------------
    # Generate Grid
    # --------------------------------------------------
    def generate_grid(self):
        try:
            size_map = {
                "Small (10x10)": 10,
                "Medium (50x50)": 50,
                "Large (100x100)": 100
            }

            grid_size = size_map[self.grid_size.get()]
            obstacle_ratio = float(self.obstacle_scale.get())
            robot_radius = float(self.robot_radius.get())
            resolution = float(self.grid_resolution.get())

            try:
                grid_map, world_map, robot, obstacles = create_random_grid_environment(
                    grid_size,
                    obstacle_ratio,
                    robot_radius,
                    resolution=resolution
                )
            except Exception as e:
                messagebox.showerror("Grid Generation Failed", str(e))
                return

            print(
                "[GUI ENV]",
                "resolution =", grid_map.resolution,
                "robot_radius =", robot.radius,
                "start =", (robot.sx, robot.sy),
                "goal =", (robot.gx, robot.gy)
            )

            self.env_data = {
                "grid_map": grid_map,
                "world_map": world_map,
                "robot": robot,
                "obstacles": obstacles
            }

            # Clear old visualization
            for w in self.vis_frame.winfo_children():
                w.destroy()

            self.vis = EmbeddedVisualizer(self.vis_frame, grid_map)
            self.vis.setup()

            self.vis.set_environment_info(
                grid_size=grid_size,
                resolution=grid_map.resolution,
                obstacle_radius=grid_map.resolution / 2,
                start=(robot.sx, robot.sy),
                goal=(robot.gx, robot.gy)
            )

            self.vis.draw_start_goal(
                (robot.sx, robot.sy),
                (robot.gx, robot.gy)
            )
            self.vis.update()

            # ✅ ENVIRONMENT IS NOW VALID
            self.env_dirty = False
            self.run_button.config(state="normal")

        except Exception as e:
            messagebox.showerror("Error", str(e))

    # --------------------------------------------------
    # Run Search
    # --------------------------------------------------
    def run_search(self):

        if self.env_dirty:
            messagebox.showwarning(
                "Environment out of date",
                "Environment parameters changed.\n"
                "Please generate the grid again."
            )
            return

        if not self.env_data or not self.vis:
            messagebox.showwarning(
                "No Grid",
                "Please generate a grid first."
            )
            return

        try:
            robot = self.env_data["robot"]
            start = (robot.sx, robot.sy)
            goal = (robot.gx, robot.gy)

            exec_config = {
                "planner": self.search_type.get(),
                "motion": self.motion_model.get(),
                "use_tree_search": (self.search_mode.get() == "Tree-Based"),
                "visualize_search": True,
                "navigation_mode": "single",
            }

            self.vis.clear_dynamic_layers()

            self.vis.set_search_info(
                robot_radius=robot.radius,
                planner=self.search_type.get(),
                mode=self.search_mode.get(),
                motion=self.motion_model.get()
            )

            self.vis.draw_start_goal(start, goal)
            self.vis.update()

            nav = NavigationSystem(
                env_data=self.env_data,
                exec_config=exec_config,
                visualizer=self.vis
            )

            nav.run()

        except Exception as e:
            messagebox.showerror("Execution Error", str(e))

    def open_experiment_gui(self):
        ExperimentGUI(self)

# --------------------------------------------------
# Entry point
# --------------------------------------------------
if __name__ == "__main__":
    app = SearchGUI()
    app.mainloop()
