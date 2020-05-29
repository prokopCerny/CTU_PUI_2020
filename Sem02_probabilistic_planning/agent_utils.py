from typing import List, Tuple

import numpy as np

from maze_env import MazeAction


class Stats:
    expanded_nodes: List[int]

    def __init__(self):
        self.expanded_nodes = []
        self.replan_count = 0
        self.steps = 0


def neighbors(location: Tuple[int, int], maze: np.ndarray):
    row, col = location
    for action in MazeAction:
        row_offset, col_offset = action.value
        possible_row, possible_col = row + row_offset, col + col_offset
        if maze[possible_row, possible_col]:
            yield possible_row, possible_col


