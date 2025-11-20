import argparse
from mbot_bridge.api import MBot
from src.graph import GridGraph, Cell
from src.graph_search import a_star_search, breadth_first_search, depth_first_search
from src.utils import generate_plan_file
import time


def cells_to_poses(path, g):
    return [[*g.cell_to_pos(c.i, c.j), 0] for c in path]


def parse_args():
    parser = argparse.ArgumentParser(description="HelloRob Path Planning on the Robot.")
    parser.add_argument("-m", "--map", type=str, default="/home/mbot/current.map")
    parser.add_argument("--goal", type=float, nargs=2, required=True)
    parser.add_argument("-r", "--collision-radius", type=float, default=15)
    parser.add_argument(
        "--algo", type=str, default="astar", choices=["bfs", "astar", "dfs"]
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    graph = GridGraph(args.map, collision_radius=args.collision_radius)
    goal = graph.pos_to_cell(*args.goal)

    robot = MBot()
    start_pose = robot.read_slam_pose()
    start = graph.pos_to_cell(*start_pose[:2])

    if args.algo == "bfs":
        path = breadth_first_search(graph, start, goal)
    elif args.algo == "astar":
        path = a_star_search(graph, start, goal)
    elif args.algo == "dfs":
        path = depth_first_search(graph, start, goal)

    print(f"Found path of length {len(path)}. Driving to the goal!")
    robot.drive_path(cells_to_poses(path, graph))

    # optional wait loop (so program doesn’t exit immediately)
    time.sleep(3)
    generate_plan_file(
        graph, start, goal, path, algo=args.algo, out_name="robot_plan.planner"
    )
