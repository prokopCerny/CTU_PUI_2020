import sys
from pathlib import Path

import numpy as np

from maze_io import load_maze, string_maze, add_padding
from maze_env import MazeEnv, MazeAction

if __name__ == '__main__':
    # np.random.seed(123)
    inpath = Path(sys.argv[1])
    with inpath.open(mode='r') as f:
        start_loc, exit_loc, maze = add_padding(*load_maze(f))
    mazeEnv = MazeEnv(start_loc, exit_loc, maze)
    print(mazeEnv.make_action(MazeAction.RIGHT))
