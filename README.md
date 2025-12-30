# mobile-robot-search
implementation of Astar, BFS and DFS in grid base environment for mobile robot


#environment setup

## For GUI Support (Recommended)

If you want to use the GUI, use Python 3.11+ with tkinter support:

1. Install Python 3.11 and tkinter via Homebrew:
   ```bash
   brew install python@3.11
   brew install python-tk@3.11
   ```

2. Create virtual environment with Python 3.11:
   ```bash
   python3.11 -m venv venv311
   source venv311/bin/activate
   ```

3. Install required libraries:
   ```bash
   pip install --upgrade pip
   pip install -r requirements.txt
   ```

## For Command-Line Only

If you only need command-line functionality:

1. go to the project directory
2. create virtual environment:
   ```bash
   python -m venv venv
   ```
3. activate virtual environment:
   ```bash
   source venv/bin/activate
   ```
4. install required library:
   ```bash
   pip install -r requirements.txt
   ```

**Note:** If you encounter GUI compatibility issues, see `GUI_FIX.md` for detailed solutions.
5. configure config.json based on what you want to run see detals below:
    a) you can run on Navigation 'mode' = 'nav'. this means it will read from excel and run only the selected planner in the config file . 

    b) you can run on benchmark "mode": = bench . in this case there will be no simulation. the code will gererate randon grid . small , medium and large . the perform test autometically for each type of search and draw a performance graph. 

6. run code from the project directory using below command:
    
    For command-line mode (using config.json):
    python -m CodeBase.main
    
    For GUI mode (interactive visualization):
    python -m CodeBase.GUI.gui_app

#config.json

{
  "map_file": "Data/maps/map16x22.xlsx",
  "resolution": 1.0,
  "origin": [0, 0],
  "robot_radius": 2, <change this to set robot radious>

  "start_grid": [2, 3], <change this to set robot position when running in nav mode>
  "goal_grid": [13, 18], <change this to set robot position when running in nav mode>
  
  "motion": "8n", <change this to set robot motion >
  "planner": "BFS", <change this to select which type of search you want to run>
  "visualize_search": true, <this is for nav mode only>
  "use_tree_search": true, <if you make this 'true' it will run in tree based mode . otherwise graphbased>
  "mode": "nav" <change this 'nav' mean single mode and 'bench' means conparisn mode>
}

#GUI Mode

The GUI application provides an interactive interface for:
- Generating random grid environments (Small 10x10, Medium 50x50, Large 100x100)
- Configuring obstacle percentage, robot radius, and grid resolution
- Selecting search algorithms (Astar, BFS, DFS)
- Choosing search mode (Graph-Based or Tree-Based)
- Selecting motion model (4n or 8n)
- Visualizing the search process in real-time
- Running batch experiments

To run the GUI:
    python -m CodeBase.GUI.gui_app

**Note**: If you encounter a macOS compatibility error when running the GUI, see `GUI_TROUBLESHOOTING.md` for solutions. The command-line mode works perfectly on all systems.

The GUI will open in a new window where you can:
1. Configure grid parameters in the left panel
2. Click "Generate Grid" to create a random environment
3. Click "Run Search" to execute the pathfinding algorithm
4. View the visualization in the right panel
5. Click "Run Experiments" to open the experiment runner for batch testing


