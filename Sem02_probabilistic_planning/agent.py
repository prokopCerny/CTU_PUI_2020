from typing import Tuple, Any, Optional, List

from maze_env import MazeEnv, MazeAction
from maze_io import string_maze

import numpy as np
from collections import deque, defaultdict
from heapq import heappop, heappush


class PriorityQueue:
    def __init__(self):
        self.heap = []

    def __bool__(self):
        return bool(self.heap)

    def push(self, priority, item):
        heappush(self.heap, (priority, item))

    def pop(self):
        priority, item = heappop(self.heap)
        return priority, item


class Stats:
    expanded_nodes: List[int]

    def __init__(self, optimal_length: int):
        self.expanded_nodes = []
        self.optimal_length = optimal_length
        self.replan_count = 0
        self.steps = 0


def neighbors(location: Tuple[int, int], maze: np.ndarray):
    row, col = location
    for action in MazeAction:
        row_offset, col_offset = action.value
        possible_row, possible_col = row + row_offset, col + col_offset
        if maze[possible_row, possible_col]:
            yield possible_row, possible_col


def manhattan(a: Tuple[int, int], b: Tuple[int, int]):
    a, b = np.array(a), np.array(b)
    return np.sum(np.abs(a-b), axis=0)


def euclid(a: Tuple[int, int], b: Tuple[int, int]):
    a, b = np.array(a), np.array(b)
    return np.sqrt(np.sum(np.square(a-b), axis=0))


def zero_heuristic(a: Tuple[int, int], b: Tuple[int, int]):
    return 0


class Agent:
    env: MazeEnv
    plan: deque
    maze: np.ndarray
    exit_location: Tuple[int, int]
    current_location: Tuple[int, int]

    def __init__(self, env: MazeEnv, heuristic=zero_heuristic, verbose=False):
        self.env = env
        self.current_location = env.agent_location
        self.exit_location = env.exit_location
        self.maze = env.maze
        self.plan = deque()
        self.heuristic = heuristic
        self.statistics = None
        self.verbose = verbose

    def run(self):
        self.statistics = Stats(optimal_length=len(self.a_star_plan()[0])-1)
        num = 0
        while self.current_location != self.exit_location:

            num += 1
            self.statistics.steps += 1
            if self.verbose:
                print(f'-------------\nStep {num}')

            if not self.plan or self.plan[0] != self.current_location:
                if self.verbose:
                    print(f'REPLAN')
                self.plan, expanded_nodes = self.a_star_plan()
                self.statistics.replan_count += 1
                self.statistics.expanded_nodes.append(expanded_nodes)
            self.plan.popleft()

            if self.verbose:
                add_symbols = {self.env.start_location: 'S',
                               self.exit_location: 'E',
                               self.current_location: 'A',
                               self.plan[0]: '*'}
                print(string_maze(self.maze, add_symbols))

            action = MazeAction(tuple(np.array(self.plan[0]) - np.array(self.current_location)))
            done, new_position = self.env.make_action(action)
            if new_position == self.current_location:
                self.plan.appendleft(self.current_location)
            else:
                self.current_location = new_position

    @staticmethod
    def reconstruct_plan(parent_dictionary, last_node):
        cur = last_node
        plan = deque()
        while cur is not None:
            plan.appendleft(cur)
            cur = parent_dictionary[cur]
        return plan

    def bfs_plan(self) -> Tuple[Optional[deque], int]:
        visited = np.zeros_like(self.maze, dtype=np.bool)
        visited[self.current_location] = True
        parent = defaultdict(lambda: None)
        queue = deque()
        queue.append(self.current_location)
        expanded_nodes = 0

        while queue:
            expanded_nodes += 1
            cur = queue.popleft()
            if cur == self.exit_location:
                return self.reconstruct_plan(parent, cur), expanded_nodes

            for neighbor in neighbors(cur, self.maze):
                if not visited[neighbor]:
                    parent[neighbor] = cur
                    visited[neighbor] = True
                    queue.append(neighbor)
        return None, expanded_nodes

    def a_star_plan(self) -> Tuple[Optional[deque], int]:
        queue = PriorityQueue()
        queue.push(0, self.current_location)
        parent = defaultdict(lambda: None)
        found_cost = {self.current_location: 0}
        expanded_nodes = 0

        while queue:
            expanded_nodes += 1
            _, cur = queue.pop()
            if cur == self.exit_location:
                return self.reconstruct_plan(parent, cur), expanded_nodes

            for neighbor in neighbors(cur, self.maze):
                neighbor_cost = found_cost[cur] + 1
                if neighbor not in found_cost or neighbor_cost < found_cost[neighbor]:
                    parent[neighbor] = cur
                    found_cost[neighbor] = neighbor_cost
                    neighbor_priority = neighbor_cost + self.heuristic(neighbor, self.exit_location)
                    queue.push(neighbor_priority, neighbor)

        return None, expanded_nodes
