"""
Heatmap Utilities - Expansion Map Conversion

This module provides utilities for converting sparse expansion maps (dictionaries
mapping cell coordinates to expansion counts) into dense numpy arrays suitable
for visualization as heatmaps.
"""

import numpy as np

def expansion_map_to_array(expansion_map, width, height):
    """
    Convert sparse expansion map to dense heatmap array.
    
    Takes a dictionary mapping (x, y) coordinates to expansion counts and
    converts it to a 2D numpy array where each cell contains the number of
    times it was expanded during search. This is used for visualizing search
    behavior as heatmaps.
    
    Args:
        expansion_map: Dictionary mapping (x, y) tuples to expansion counts
        width: Width of the grid
        height: Height of the grid
        
    Returns:
        2D numpy array with expansion counts, shape (height, width)
    """
    heat = np.zeros((height, width), dtype=np.int32)
    for (x, y), c in expansion_map.items():
        if 0 <= x < width and 0 <= y < height:
            heat[y, x] = c
    return heat
