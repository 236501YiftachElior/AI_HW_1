from Robot import *
from Utilities import *
from Heuristics import *
import matplotlib.pyplot as plt


def w_experiment(maze_index):
    w_values = [0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1]
    solve_times = []  # fill this list with the solving times
    solution_costs = []  # fill this list with the solution costs
    problem = create_problem(f"maze_{maze_index}")
    for w in w_values:
        ################################################################################################################
        robot_instance = WAStartRobot(heuristic=center_manhattan_heuristic,w=w)
        solution = robot_instance.solve(problem)
        solution_costs += [solution.cost]
        solve_times += [solution.solve_time]
        # raise NotImplemented

        ################################################################################################################

    plt.plot(w_values, solve_times)
    plt.xlabel("w")
    plt.ylabel("time")
    plt.title(f"wA* with center_manhattan_heuristic solving time on maze_{maze_index}")
    plt.savefig(f"plots/wAStar2 with center_manhattan_heuristic solving time on maze_{maze_index}.png")
    plt.clf()

    plt.plot(w_values, solution_costs)
    plt.xlabel("w")
    plt.ylabel("cost")
    plt.title(f"wA* with center_manhattan_heuristic solution cost on maze_{maze_index}")
    plt.savefig(f"plots/wAStar2 with center_manhattan_heuristic solution cost on maze_{maze_index}.png")
    plt.clf()

def shorter_robot_heuristic_experiment(maze_index):
    problem = create_problem(f"maze_{maze_index}")
    length = problem.length
    solve_times = []  # fill this list with the solving times
    heuristic_init_times = []  # fill this list with the heuristic initialization times
    ks = list(range(2, length - 2, 2))
    for k in ks:
        ################################################################################################################
        # heuristic = ShorterRobotHeuristic(problem,k)
        robot_instance = WAStartRobot(heuristic=ShorterRobotHeuristic, k=k)
        solution = robot_instance.solve(problem)
        heuristic_init_times += [solution.init_heuristic_time]
        solve_times += [solution.solve_time]

        ################################################################################################################

    plt.bar(ks, height=solve_times, color="b", label="total run time")
    plt.bar(ks, height=heuristic_init_times, color="g", label="heuristic initialization time")
    plt.legend()
    plt.xlabel("k")
    plt.ylabel("time")
    plt.title(f"shorter robot heuristic experiment on maze_{maze_index}.")
    plt.savefig(f"plots/shorter robot heuristic experiment on maze_{maze_index}.png")
    plt.clf()
