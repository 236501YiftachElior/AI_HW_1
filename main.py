from Robot import *
from MazeProblem import *
from Animation import Animation
from Heuristics import *
from Utilities import *
from Experiments import *

if __name__ == "__main__":
    # test_robot(WAStartRobot, [99],heuristic = tail_manhattan_heuristic)
    maze_problem = create_problem(f"maze_{5}")
    for i in range(2,6):
        shorter_robot_heuristic_experiment(i)
