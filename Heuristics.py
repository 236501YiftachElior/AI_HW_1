import numpy as np
from MazeProblem import MazeState, MazeProblem, compute_robot_direction
from Robot import UniformCostSearchRobot
from GraphSearch import NodesCollection


def tail_manhattan_heuristic(state: MazeState):
    # TODO (EX 7.2), implement heuristic, delete exception
    tail_loc = state.tail
    tail_goal = state.maze_problem.tail_goal
    return (abs(tail_loc - tail_goal).sum()) * state.maze_problem.forward_cost


def center_manhattan_heuristic(state: MazeState):
    # TODO (EX 9.2), implement heuristic, delete exception
    center_loc = (state.tail + state.head) / 2
    center_loc_goal = (state.maze_problem.tail_goal + state.maze_problem.head_goal) / 2
    return (abs(center_loc - center_loc_goal).sum()) * state.maze_problem.forward_cost


class ShorterRobotHeuristic:
    def __init__(self, maze_problem: MazeProblem, k):
        assert k % 2 == 0, "odd must be even"
        assert maze_problem.length - k >= 3, f"it is not possible to shorten a {maze_problem.length}-length robot by " \
                                             f"{k} units because robot length has to at least 3"
        self.k = k
        ################################################################################################################
        # TODO (EX. 13.2): replace all three dots, delete exception
        shorter_robot_head_goal, shorter_robot_tail_goal = self._compute_shorter_head_and_tails(maze_problem.head_goal,
                                                                                   maze_problem.tail_goal)
        self.new_maze_problem = MazeProblem(maze_map=maze_problem.maze_map,
                                            initial_head=shorter_robot_tail_goal,
                                            initial_tail=shorter_robot_head_goal,
                                            head_goal=shorter_robot_head_goal,  # doesn't matter, don't change
                                            tail_goal=shorter_robot_tail_goal)  # doesn't matter, don't change
        self.node_dists = UniformCostSearchRobot().solve(self.new_maze_problem, compute_all_dists=True)
        ################################################################################################################

        assert isinstance(self.node_dists, NodesCollection)

    def _compute_shorter_head_and_tails(self, head, tail):
        # TODO (EX. 13.1): complete code here, delete exception
        head,tail = head.copy(),tail.copy()
        if (head > tail).any():
            elongated_index = (head > tail).argmax()
            head[elongated_index] -= self.k / 2
            tail[elongated_index] += self.k / 2
        else:
            elongated_index = (tail > head).argmax()
            head[elongated_index] += self.k / 2
            tail[elongated_index] -= self.k / 2
        return head, tail

    def __call__(self, state: MazeState):
        # TODO (EX. 13.3): replace each three dots, delete exception
        # raise NotImplemented
        shorter_head_location, shorter_tail_location = self._compute_shorter_head_and_tails(state.head, state.tail)
        new_state = MazeState(self.new_maze_problem, head=shorter_tail_location, tail=shorter_head_location)
        if new_state in self.node_dists:
            node = self.node_dists.get_node(new_state)
            return node.g_value
        else:
            return 0  # what should we return in this case, so that the heuristic would be as informative as possible
            # but still admissible
    def __name__(self):
        return f"ShorterRobotHeuristic with K={self.k}"