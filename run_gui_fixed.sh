#!/bin/bash
# GUI Launcher Script - Uses Python 3.11 with proper tkinter support

cd "$(dirname "$0")"

# Check if venv311 exists
if [ ! -d "venv311" ]; then
    echo "Python 3.11 virtual environment not found!"
    echo ""
    echo "Please set up the GUI environment first:"
    echo "  1. brew install python@3.11 python-tk@3.11"
    echo "  2. python3.11 -m venv venv311"
    echo "  3. source venv311/bin/activate"
    echo "  4. pip install -r requirements.txt"
    echo ""
    echo "See GUI_FIX.md for detailed instructions."
    exit 1
fi

# Activate Python 3.11 environment
source venv311/bin/activate

# Check if tkinter works
python -c "import tkinter as tk; root = tk.Tk(); root.withdraw(); root.destroy()" 2>&1
if [ $? -ne 0 ]; then
    echo "ERROR: tkinter not working in this environment"
    echo "Please install python-tk@3.11: brew install python-tk@3.11"
    exit 1
fi

# Run the GUI
echo "Launching GUI..."
python -m CodeBase.GUI.gui_app

