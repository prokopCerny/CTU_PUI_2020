from typing import Tuple
from enum import Enum
import numpy as np

rotate_0 = np.eye(2, dtype=np.int)
rotate_90 = np.array([[0, -1], [1, 0]], dtype=np.int)
rotate_180 = rotate_90 @ rotate_90
rotate_270 = rotate_90 @ rotate_90 @ rotate_90

rotations = np.array([rotate_0, rotate_90, rotate_180, rotate_270])
rotation_probabilities = [0.65, 0.15, 0.05, 0.15]


class MazeAction(Enum):
    RIGHT, UP, LEFT, DOWN = [(0, 1), (-1, 0), (0, -1), (1, 0)]


class MazeEnv:
    agent_location: Tuple[int, int]
    exit_location: Tuple[int, int]
    start_location: Tuple[int, int]
    maze: np.ndarray

    def __init__(self,
                 start_location: Tuple[int, int],
                 end_location: Tuple[int, int],
                 maze: np.ndarray):
        self.exit_location = end_location
        self.start_location = start_location
        self.agent_location = start_location
        self.maze = maze

    def make_action(self, action: MazeAction) -> Tuple[bool, Tuple[int, int]]:
        rotation = rotations[np.random.choice(range(4), p=rotation_probabilities)]
        real_pos_delta = np.array(action.value) @ rotation.T
        potential_pos = tuple(np.array(self.agent_location) + real_pos_delta)
        if self.maze[potential_pos]:
            self.agent_location = potential_pos

        return self.agent_location == self.exit_location, self.agent_location


