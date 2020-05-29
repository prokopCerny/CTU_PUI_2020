from typing import Tuple, Optional, TextIO

import numpy as np


def parse_line(line: str) -> Tuple[np.ndarray, Optional[int], Optional[int]]:
    can_step_on_in_line = np.zeros(len(line), dtype=np.bool)
    start_column = None
    exit_column = None
    for idx, letter in enumerate(line):
        if letter == 'S':
            start_column = idx
            can_step_on_in_line[idx] = True
        elif letter == 'E':
            exit_column = idx
            can_step_on_in_line[idx] = True
        elif letter == ' ':
            can_step_on_in_line[idx] = True
    return can_step_on_in_line, start_column, exit_column


def load_maze(file: TextIO) -> Tuple[Tuple[int, int], Tuple[int, int], np.ndarray]:
    rows, columns = map(int, file.readline().split())
    maze = np.zeros((rows, columns), dtype=np.bool)
    start_location = None
    exit_location = None
    for row_idx in range(rows):
        maze_row, possible_start_column, possible_exit_column = parse_line(file.readline().strip())
        maze[row_idx] = maze_row
        if possible_start_column:
            start_location = (row_idx, possible_start_column)
        if possible_exit_column:
            exit_location = (row_idx, possible_exit_column)
    return start_location, exit_location, maze


def string_maze(maze, additional_symbols):
    mapping = {False: '#', True: ' '}
    lines = [[mapping[el] for el in row] for row in maze]
    for (row, col), letter in additional_symbols.items():
        lines[row][col] = letter
    return '\n'.join(''.join(line) for line in lines)


def add_padding(start_loc: Tuple[int, int],
                exit_loc: Tuple[int, int],
                maze: np.ndarray, padding: int = 1) -> Tuple[Tuple[int, int], Tuple[int, int], np.ndarray]:
    (sr, sc), (er, ec) = start_loc, exit_loc
    padded = np.zeros((maze.shape[0]+2*padding, maze.shape[1]+2*padding), dtype=maze.dtype)
    padded[padding:-padding, padding:-padding] = maze
    return (sr+padding, sc+padding), (er+padding, ec+padding), padded
