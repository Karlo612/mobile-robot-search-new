#!/usr/bin/env python
"""
GUI Launcher with error handling and diagnostics
"""
import sys
import os

def check_dependencies():
    """Check if all required dependencies are available"""
    issues = []
    
    try:
        import tkinter
        print("✓ tkinter available")
    except ImportError:
        issues.append("tkinter not available (usually comes with Python)")
    
    try:
        import matplotlib
        print(f"✓ matplotlib {matplotlib.__version__} available")
    except ImportError:
        issues.append("matplotlib not installed")
    
    try:
        import numpy
        print(f"✓ numpy {numpy.__version__} available")
    except ImportError:
        issues.append("numpy not installed")
    
    if issues:
        print("\n✗ Issues found:")
        for issue in issues:
            print(f"  - {issue}")
        return False
    
    return True

def try_gui():
    """Try to launch the GUI with error handling"""
    print("Attempting to launch GUI...")
    
    try:
        # Try to set backend explicitly
        import matplotlib
        matplotlib.use('TkAgg')
        
        # Import and run GUI
        from CodeBase.GUI.gui_app import SearchGUI
        
        print("✓ GUI module loaded successfully")
        print("Opening GUI window...")
        
        app = SearchGUI()
        app.mainloop()
        
    except Exception as e:
        error_type = type(e).__name__
        error_msg = str(e)
        
        print(f"\n✗ Error: {error_type}: {error_msg}")
        
        if "macOS" in error_msg or "2601" in error_msg:
            print("\n" + "="*60)
            print("GUI COMPATIBILITY ISSUE DETECTED")
            print("="*60)
            print("\nThe GUI requires TkAgg backend which has compatibility issues")
            print("on your system. Here are some solutions:\n")
            print("1. USE COMMAND-LINE MODE (works fine):")
            print("   python -m CodeBase.main")
            print("\n2. TRY UPDATING PYTHON:")
            print("   Install Python 3.10+ from python.org or Homebrew")
            print("\n3. TRY DIFFERENT BACKEND:")
            print("   The issue is with TkAgg backend compatibility")
            print("\n4. USE GUI WITHOUT VISUALIZATION:")
            print("   Set 'visualize_search': false in config.json")
            print("="*60)
        
        import traceback
        traceback.print_exc()
        return False
    
    return True

if __name__ == "__main__":
    print("="*60)
    print("Mobile Robot Search - GUI Launcher")
    print("="*60)
    print()
    
    # Check dependencies
    if not check_dependencies():
        print("\nPlease install missing dependencies:")
        print("  pip install -r requirements.txt")
        sys.exit(1)
    
    print()
    
    # Try to run GUI
    success = try_gui()
    
    if not success:
        sys.exit(1)

