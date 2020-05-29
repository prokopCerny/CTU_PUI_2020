import sys
from pathlib import Path

import numpy as np

from maze_io import load_maze, string_maze, add_padding
from maze_env import MazeEnv, MazeAction
from agent import Agent, manhattan, euclid, zero_heuristic, Stats

if __name__ == '__main__':
    np.random.seed(123)
    inpath = Path(sys.argv[1])
    with inpath.open(mode='r') as f:
        start_loc, exit_loc, maze = add_padding(*load_maze(f))
    mazeEnv = MazeEnv(start_loc, exit_loc, maze)
    add_symb = {start_loc: 'S', exit_loc: 'E'}
    print(string_maze(maze, add_symb))
    agent = Agent(mazeEnv, heuristic=manhattan)
    agent.run()
    cur_stats: Stats = agent.statistics
    print(f'Avg expanded: {np.mean(cur_stats.expanded_nodes) if cur_stats.expanded_nodes else 0}\n'
          f'Total expanded: {np.sum(cur_stats.expanded_nodes) if cur_stats.expanded_nodes else 0}\n'
          f'Replan count: {cur_stats.replan_count}\n'
          f'Optimal steps: {cur_stats.optimal_length}\n'
          f'Steps taken: {cur_stats.steps}')
