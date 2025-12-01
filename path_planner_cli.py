import argparse
from src.graph import GridGraph, Cell
from src.graph_search import a_star_search, breadth_first_search, depth_first_search
from src.utils import generate_plan_file


def parse_args():
    parser = argparse.ArgumentParser(description="HelloRob Path Planning Client.")
    parser.add_argument(
        "-m", "--map", type=str, required=True, help="Path to the map file."
    )
    parser.add_argument("--start", type=int, nargs=2, required=True, help="Start cell.")
    parser.add_argument("--goal", type=int, nargs=2, required=True, help="Goal cell.")
    parser.add_argument(
        "--algo",
        type=str,
        default="bfs",
        choices=["bfs", "dfs", "astar"],
        help="Algorithm to use.",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    graph = GridGraph(args.map, collision_radius=0.15)
    start, goal = Cell(*args.start), Cell(*args.goal)

    if args.algo == "astar":
        path = a_star_search(graph, start, goal)
    elif args.algo == "bfs":
        path = breadth_first_search(graph, start, goal)
    elif args.algo == "dfs":
        path = depth_first_search(graph, start, goal)
    else:
        print("Invalid option.")
        exit()

    generate_plan_file(graph, start, goal, path, algo=args.algo)
