from typing import Tuple, Any, Optional, List

from agent_utils import Stats, neighbors
from maze_env import MazeEnv, MazeAction
from maze_io import string_maze

import numpy as np
from collections import deque, defaultdict


class NavmeshAgent:
    env: MazeEnv
    plan: deque
    maze: np.ndarray
    exit_location: Tuple[int, int]
    current_location: Tuple[int, int]

    def __init__(self, env: MazeEnv, verbose=False, **kw):
        self.env = env
        self.current_location = env.agent_location
        self.exit_location = env.exit_location
        self.maze = env.maze
        self.plan = deque()
        self.statistics = None
        self.navmesh = None
        self.verbose = verbose

    def get_navmesh_action(self):
        min_dist = np.iinfo(np.int).max
        min_neigh = None
        for neighbor in neighbors(self.current_location, self.maze):
            if self.navmesh[neighbor] < min_dist:
                min_dist = self.navmesh[neighbor]
                min_neigh = neighbor
        return MazeAction(tuple(np.array(min_neigh) - np.array(self.current_location)))

    def run(self):
        self.statistics = Stats()
        self.navmesh, expanded_nodes = self.precompute_navmesh()
        self.statistics.expanded_nodes = [expanded_nodes]
        num = 0
        while self.current_location != self.exit_location:
            num += 1
            self.statistics.steps += 1
            if self.verbose:
                print(f'-------------\nStep {num}')

            if self.verbose:
                add_symbols = {self.env.start_location: 'S',
                               self.exit_location: 'E',
                               self.current_location: 'A',
                               self.plan[0]: '*'}
                print(string_maze(self.maze, add_symbols))

            action = self.get_navmesh_action()
            done, new_position = self.env.make_action(action)
            self.current_location = new_position

    def precompute_navmesh(self):
        visited = np.zeros_like(self.maze, dtype=np.bool)
        navmesh = (~self.maze)*np.iinfo(np.int).max
        visited[self.exit_location] = True
        queue = deque()
        queue.append((self.exit_location, 0))
        expanded_nodes = 0
        while queue:
            expanded_nodes += 1
            cur_loc, cur_cost = queue.popleft()
            for neighbor in neighbors(cur_loc, self.maze):
                if not visited[neighbor]:
                    neighbor_cost = cur_cost + 1
                    visited[neighbor] = True
                    navmesh[neighbor] = neighbor_cost
                    queue.append((neighbor, neighbor_cost))

        return navmesh, expanded_nodes

