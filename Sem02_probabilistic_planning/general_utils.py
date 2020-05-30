from astar_agent import AstarAgent, manhattan
from maze_env import MazeEnv


def get_optimal_path_length(start_loc, exit_loc, maze):
    env = MazeEnv(start_loc, exit_loc, maze)
    agent = AstarAgent(env, heuristic=manhattan, verbose=False)
    return len(agent.a_star_plan()[0])-1