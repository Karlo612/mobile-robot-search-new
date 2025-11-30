# base_planner.py

class Planner:
    """
    Abstract base class for all planners.
    """
    def __init__(self, grid_map,visualizer=None):
        self.grid_map = grid_map
        self.visualizer = visualizer

    def plan(self, start, goal):
        raise NotImplementedError("plan() must be implemented by subclasses")