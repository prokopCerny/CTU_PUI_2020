from typing import Tuple, Any, Optional

from maze_env import MazeEnv, MazeAction
from maze_io import string_maze

import numpy as np
from collections import deque, defaultdict


def neighbors(location: Tuple[int, int], maze: np.ndarray):
    row, col = location
    for action in MazeAction:
        row_offset, col_offset = action.value
        possible_row, possible_col = row + row_offset, col + col_offset
        if maze[possible_row, possible_col]:
            yield possible_row, possible_col


class Agent:
    env: MazeEnv
    plan: deque
    maze: np.ndarray
    exit_location: Tuple[int, int]
    current_location: Tuple[int, int]

    def __init__(self, env: MazeEnv):
        self.env = env
        self.current_location = env.agent_location
        self.exit_location = env.exit_location
        self.maze = env.maze
        self.plan = deque()

    def run(self):
        num = 1
        while self.current_location != self.exit_location:

            print(f'-------------\nStep {num}')
            num += 1

            if not self.plan or self.plan[0] != self.current_location:
                print(f'REPLAN')
                self.plan = self.bfs_plan()
            self.plan.popleft()

            add_symbols = {self.env.start_location: 'S', self.exit_location: 'E', self.current_location: 'A', self.plan[0]: '*'}
            print(string_maze(self.maze, add_symbols))

            action = MazeAction(tuple(np.array(self.plan[0]) - np.array(self.current_location)))
            done, new_position = self.env.make_action(action)
            if new_position == self.current_location:
                self.plan.appendleft(self.current_location)
            else:
                self.current_location = new_position

    def bfs_plan(self) -> Optional[deque]:
        visited = np.zeros_like(self.maze, dtype=np.bool)
        visited[self.current_location] = True
        parent = defaultdict(lambda: None)
        queue = deque()
        queue.append(self.current_location)
        while queue:
            cur = queue.popleft()
            if cur == self.exit_location:
                plan = deque()
                while cur is not None:
                    plan.appendleft(cur)
                    cur = parent[cur]
                return plan

            for neighbor in neighbors(cur, self.maze):
                if not visited[neighbor]:
                    parent[neighbor] = cur
                    visited[neighbor] = True
                    queue.append(neighbor)
        return None

