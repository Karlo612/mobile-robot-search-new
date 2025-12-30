# GUI Troubleshooting Guide

## Issue: GUI Won't Open - macOS Compatibility Error

If you see the error:
```
macOS 26 (2601) or later required, have instead 16 (1601) !
```

This is a known compatibility issue with the matplotlib TkAgg backend on certain macOS systems.

## Solutions

### ✅ Solution 1: Use Command-Line Mode (Recommended - Works Perfectly)

The command-line mode works without any issues:

```bash
source venv/bin/activate
python -m CodeBase.main
```

You can configure everything in `Data/configs/config.json`:
- Change planner (Astar, BFS, DFS)
- Set start/goal positions
- Configure motion model (4n, 8n)
- Enable/disable visualization

### ✅ Solution 2: Update Python Version

The GUI works better with Python 3.10 or later:

1. Install Python 3.10+ from [python.org](https://www.python.org/downloads/) or via Homebrew:
   ```bash
   brew install python@3.11
   ```

2. Create a new virtual environment:
   ```bash
   python3.11 -m venv venv
   source venv/bin/activate
   pip install -r requirements.txt
   ```

3. Run the GUI:
   ```bash
   python -m CodeBase.GUI.gui_app
   ```

### ✅ Solution 3: Use GUI Without Visualization

You can still use the GUI code structure but disable visualization:

1. Edit `Data/configs/config.json`:
   ```json
   {
     "visualize_search": false,
     ...
   }
   ```

2. Run in command-line mode (visualization disabled):
   ```bash
   python -m CodeBase.main
   ```

### ✅ Solution 4: Alternative - Use Jupyter Notebook

You can create a Jupyter notebook to interact with the code:

```python
from CodeBase.navigation_system import NavigationSystem
from CodeBase.Environment.map_loader import MapLoader
# ... etc
```

## Current Status

- ✅ **Command-line mode**: Fully functional
- ✅ **Pathfinding algorithms**: All working (A*, BFS, DFS)
- ✅ **Map loading**: Working perfectly
- ⚠️ **GUI visualization**: Has compatibility issues on some macOS systems

## What Works

All core functionality works perfectly:
- Loading maps from Excel files
- Running A*, BFS, and DFS algorithms
- Graph-based and tree-based search modes
- 4-neighbor and 8-neighbor motion models
- Path planning and obstacle inflation
- Command-line execution

The only issue is with the GUI window visualization on certain macOS configurations.

## Technical Details

The error occurs when matplotlib tries to initialize the TkAgg backend, which requires specific binary compatibility. This is a known issue with:
- Python 3.9 on some macOS systems
- Certain versions of matplotlib/numpy combinations
- TkAgg backend initialization

The core pathfinding code is completely independent of the GUI and works perfectly.

