# Experiment GUI - Results Viewer Features

## Overview

The Experiment GUI now includes a comprehensive results viewer with tables, interactive plots, and statistics that allow you to navigate and compare different planners directly from the GUI.

## New Features

### 1. Tabbed Interface

The right panel now has 4 tabs:
- **Log**: Original experiment log output
- **Results Table**: Sortable and filterable table of all results
- **Comparison Plots**: Interactive plots comparing planners
- **Statistics**: Summary statistics for each planner

### 2. Results Table

**Features:**
- **Sortable columns**: Click any column header to sort
- **Filtering**: Filter by planner and map size
- **All metrics displayed**: Runtime, expanded nodes, path length, path cost, memory usage, success status
- **Auto-refresh**: Updates automatically when experiments complete

**Columns:**
- Planner (Astar, BFS, DFS)
- Mode (Graph/Tree)
- Map Size (small/medium/large)
- Map ID
- Runtime (ms)
- Expanded Nodes
- Path Length
- Path Cost
- Memory (KB)
- Found (Yes/No)

### 3. Comparison Plots

**Interactive Plot Types:**
- **Box Plot**: Shows distribution of metrics across all runs
- **Bar Chart**: Average performance comparison
- **Line Chart**: Performance trends across map sizes

**Metrics Available:**
- Runtime (ms)
- Expanded Nodes
- Path Length
- Path Cost
- Memory (KB)

**Features:**
- Switch between metrics and plot types dynamically
- Updates in real-time as experiments complete
- Clear visual comparison between planners

### 4. Statistics Summary

**Per-Planner Statistics:**
- Mean, Median, Min, Max, Standard Deviation for each metric
- Success rate (percentage of successful path findings)
- Easy-to-read formatted text output

### 5. Navigation

**"View Results" Button:**
- Automatically enabled when experiments complete
- Switches to Results Table tab
- Updates all views (table, plots, statistics)

## Usage

### Running Experiments

1. Configure experiment settings (map sizes, planners, parameters)
2. Click "Generate Maps"
3. Click "Run Experiment"
4. Watch progress in the Log tab

### Viewing Results

**Option 1: Automatic**
- After experiment completes, click "View Results" button
- Automatically switches to Results Table tab

**Option 2: Manual Navigation**
- Click on any tab (Results Table, Comparison Plots, Statistics)
- Views update automatically when data is available

### Filtering Results

1. Go to "Results Table" tab
2. Use dropdown filters:
   - **Planner**: Filter by specific planner (All, Astar, BFS, DFS)
   - **Map Size**: Filter by map size (All, small, medium, large)
3. Click "Refresh" or filters update automatically

### Sorting Results

- Click any column header to sort
- Click again to reverse sort order
- Works with filtered data

### Comparing Planners

1. Go to "Comparison Plots" tab
2. Select metric from dropdown (runtime_ms, expanded_nodes, etc.)
3. Select plot type (Box Plot, Bar Chart, Line Chart)
4. Click "Update Plot" or it updates automatically
5. Compare visual performance differences

### Viewing Statistics

1. Go to "Statistics" tab
2. View comprehensive statistics for each planner
3. Compare mean, median, min, max, std dev across planners
4. Check success rates

## Technical Details

### Data Structure

Results are stored in `self.results` as a list of dictionaries, each containing:
- Planner information (name, tree/graph mode)
- Map information (size, ID)
- Performance metrics (runtime, expanded nodes, path length, etc.)
- Success status

### Updates

All views update automatically when:
- Experiments complete
- "View Results" button is clicked
- Filters change (for table)
- Plot settings change (for plots)

### Performance

- Table updates are fast (handles 100+ results easily)
- Plots render in real-time
- Statistics computed on-demand
- All operations are non-blocking (GUI stays responsive)

## Benefits

1. **No need to export CSV**: View results directly in GUI
2. **Quick comparisons**: See differences between planners at a glance
3. **Interactive exploration**: Filter, sort, and plot on the fly
4. **Comprehensive statistics**: Get detailed performance breakdowns
5. **Better workflow**: Everything in one place, no file navigation needed

## Example Workflow

1. Run experiment with BFS, DFS, and A* (Graph-based)
2. Click "View Results" when complete
3. Go to "Comparison Plots" tab
4. Select "runtime_ms" and "Box Plot"
5. See that A* is fastest, DFS is slowest
6. Switch to "Statistics" tab
7. See that A* has lowest mean runtime: 15.2ms vs BFS: 45.8ms vs DFS: 120.5ms
8. Filter table to show only "large" maps
9. See how performance scales with map size

