from Robot import *
from MazeProblem import *
from Animation import Animation
from Heuristics import *
from Utilities import *
from Experiments import *


if __name__ == "__main__":
    test_robot(WAStartRobot, [5],heuristic=center_manhattan_heuristic)
    # a = solve_and_display(UniformCostSearchRobot,2)
