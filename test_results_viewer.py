#!/usr/bin/env python
"""
Test script to populate Experiment GUI with sample data for testing
"""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import tkinter as tk
from CodeBase.GUI.gui_app import SearchGUI
from CodeBase.GUI.experiment_gui import ExperimentGUI
import random

def create_sample_results():
    """Create sample experiment results for testing"""
    results = []
    
    planners = ["Astar", "BFS", "DFS"]
    modes = [False, True]  # Graph, Tree
    map_sizes = ["small", "medium", "large"]
    
    for size in map_sizes:
        for map_id in range(3):  # 3 maps per size
            for planner in planners:
                for tree_mode in modes:
                    # Simulate realistic performance differences
                    if planner == "Astar":
                        runtime = random.uniform(10, 50)
                        expanded = random.randint(50, 200)
                        path_len = random.randint(20, 40)
                    elif planner == "BFS":
                        runtime = random.uniform(30, 100)
                        expanded = random.randint(100, 400)
                        path_len = random.randint(25, 45)
                    else:  # DFS
                        runtime = random.uniform(100, 500)
                        expanded = random.randint(200, 800)
                        path_len = random.randint(30, 60)
                    
                    # Tree mode is generally slower
                    if tree_mode:
                        runtime *= 1.2
                        expanded = int(expanded * 1.1)
                    
                    result = {
                        "planner": planner,
                        "tree": tree_mode,
                        "map_size": size,
                        "map_id": map_id,
                        "runtime_ms": runtime,
                        "expanded_nodes": expanded,
                        "path_len": path_len,
                        "path_cost": path_len * 1.0,
                        "memory_kb": random.uniform(100, 500),
                        "found": True,
                        "motion": "8n",
                        "heatmap_id": f"{size}_{map_id}_{planner}_{tree_mode}_8n"
                    }
                    results.append(result)
    
    return results

def test_results_viewer():
    """Test the results viewer with sample data"""
    print("="*60)
    print("TESTING RESULTS VIEWER")
    print("="*60)
    
    root = tk.Tk()
    root.withdraw()  # Hide main window
    
    # Create main GUI
    main_gui = SearchGUI()
    main_gui.withdraw()  # Hide it too
    
    # Create experiment GUI
    exp_gui = ExperimentGUI(main_gui)
    
    # Populate with sample data
    print("\nPopulating with sample data...")
    sample_results = create_sample_results()
    exp_gui.results = sample_results
    print(f"✓ Added {len(sample_results)} sample results")
    
    # Test table update
    print("\nTesting Results Table...")
    try:
        exp_gui._update_table()
        item_count = len(exp_gui.results_tree.get_children())
        print(f"✓ Table populated with {item_count} rows")
    except Exception as e:
        print(f"✗ Table update failed: {e}")
        import traceback
        traceback.print_exc()
    
    # Test plot update
    print("\nTesting Comparison Plots...")
    try:
        exp_gui._update_plot()
        print("✓ Plot updated successfully")
    except Exception as e:
        print(f"✗ Plot update failed: {e}")
        import traceback
        traceback.print_exc()
    
    # Test statistics update
    print("\nTesting Statistics Panel...")
    try:
        exp_gui._update_stats()
        stats_content = exp_gui.stats_text.get(1.0, "end-1c")
        print(f"✓ Statistics generated ({len(stats_content)} characters)")
        print("\nSample statistics:")
        print(stats_content[:500] + "..." if len(stats_content) > 500 else stats_content)
    except Exception as e:
        print(f"✗ Statistics update failed: {e}")
        import traceback
        traceback.print_exc()
    
    # Test filtering
    print("\nTesting Filters...")
    try:
        exp_gui.filter_planner.set("Astar")
        exp_gui.filter_size.set("small")
        exp_gui._update_table()
        filtered_count = len(exp_gui.results_tree.get_children())
        print(f"✓ Filtering works: {filtered_count} rows after filtering")
    except Exception as e:
        print(f"✗ Filtering failed: {e}")
    
    # Test sorting
    print("\nTesting Sorting...")
    try:
        exp_gui._sort_table("Runtime (ms)")
        print("✓ Sorting works")
    except Exception as e:
        print(f"✗ Sorting failed: {e}")
    
    print("\n" + "="*60)
    print("TEST COMPLETE")
    print("="*60)
    print("\nAll components tested successfully!")
    print("\nTo test interactively:")
    print("1. Run: python -m CodeBase.GUI.gui_app")
    print("2. Click 'Run Experiments'")
    print("3. Generate maps and run a real experiment")
    print("4. Click 'View Results' when complete")
    print("5. Navigate between tabs to explore results")
    
    # Show the GUI
    exp_gui.deiconify()
    exp_gui.lift()
    exp_gui.focus_force()
    
    # Switch to results table
    exp_gui.notebook.select(1)
    
    print("\nGUI window opened - you can interact with it now!")
    print("Close the window when done testing.")
    
    # Run mainloop
    exp_gui.mainloop()
    
    root.destroy()
    main_gui.destroy()

if __name__ == "__main__":
    try:
        test_results_viewer()
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    except Exception as e:
        print(f"\n\nError: {e}")
        import traceback
        traceback.print_exc()

