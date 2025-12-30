# GUI Compatibility Fix

## Problem
The GUI was failing with error:
```
macOS 26 (2601) or later required, have instead 16 (1601) !
```

This was caused by an incompatible `_tkinter` binary in Python 3.9 from CommandLineTools.

## Solution
Use Python 3.11 from Homebrew which has proper tkinter support.

## Setup Instructions

1. **Install Python 3.11 and tkinter support via Homebrew:**
   ```bash
   brew install python@3.11
   brew install python-tk@3.11
   ```

2. **Create a new virtual environment with Python 3.11:**
   ```bash
   cd /Users/karlo/Desktop/T1AAI/aip/mobile-robot-search
   python3.11 -m venv venv311
   source venv311/bin/activate
   ```

3. **Install dependencies:**
   ```bash
   pip install --upgrade pip
   pip install -r requirements.txt
   ```

4. **Run the GUI:**
   ```bash
   python -m CodeBase.GUI.gui_app
   ```

## Quick Switch Script

You can create an alias or script to easily switch to the working environment:

```bash
# Add to ~/.zshrc or ~/.bashrc
alias robot-gui='cd /Users/karlo/Desktop/T1AAI/aip/mobile-robot-search && source venv311/bin/activate && python -m CodeBase.GUI.gui_app'
```

Then just run:
```bash
robot-gui
```

## Verification

Test that tkinter works:
```bash
source venv311/bin/activate
python -c "import tkinter as tk; root = tk.Tk(); root.withdraw(); print('✓ tkinter works!')"
```

## Notes

- The old `venv` (Python 3.9) still works for command-line mode
- The new `venv311` (Python 3.11) works for both GUI and command-line
- You can use either environment depending on your needs

