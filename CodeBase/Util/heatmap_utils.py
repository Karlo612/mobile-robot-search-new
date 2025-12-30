import numpy as np

def expansion_map_to_array(expansion_map, width, height):
    """
    Convert sparse expansion_map {(x,y):count} → dense heatmap array.
    """
    heat = np.zeros((height, width), dtype=np.int32)
    for (x, y), c in expansion_map.items():
        if 0 <= x < width and 0 <= y < height:
            heat[y, x] = c
    return heat
