# mobile-robot-search

Implementation of A*, BFS and DFS in grid-based environment for mobile robot path planning.

## Setup

### Prerequisites
- Python 3.11 or higher
- tkinter support (usually included with Python)

### Installation

1. **Install Python 3.11+ with tkinter** (if not already installed):

   **macOS:**
   ```bash
   brew install python@3.11 python-tk@3.11
   ```

   **Linux:**
   ```bash
   sudo apt-get install python3.11 python3.11-venv python3-tk
   ```

   **Windows:** Download from [python.org](https://www.python.org/downloads/) and check "Add Python to PATH"

2. **Create and activate virtual environment:**
   ```bash
   python3.11 -m venv venv311
   source venv311/bin/activate  # macOS/Linux
   # or
   venv311\Scripts\activate     # Windows
   ```

3. **Install dependencies:**
   ```bash
   pip install --upgrade pip
   pip install -r requirements.txt
   ```

## Running the GUI

From the project root directory (with virtual environment activated):

```bash
python -m CodeBase.GUI.gui_app
```

## Using the GUI

1. **Configure parameters** in the left panel (grid size, obstacle ratio, algorithm, etc.)
2. **Click "Generate Grid"** to create a random environment
3. **Click "Run Search"** to execute the pathfinding algorithm
4. **Click "Run Experiments"** for batch testing

## Troubleshooting

- **"No module named 'tkinter'"**: Install tkinter support (see Installation step 1)
- **Import errors**: Make sure virtual environment is activated and you're running from project root
- **GUI doesn't appear**: Check terminal for error messages, ensure Python 3.11+ is used
