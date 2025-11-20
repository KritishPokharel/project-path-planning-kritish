import json
from .graph import Cell
import numpy as np


def trace_path(cell, graph):
    """Traces a path from the given cell through its parents."""
    path = []
    while cell is not None:
        path.append(Cell(cell.i, cell.j))
        cell = graph.get_parent(cell)
    path.reverse()
    return path


def generate_plan_file(graph, start, goal, path, algo="", out_name="out.planner"):
    print(f"Saving planning data to file: {out_name}")

    # Safely convert NumPy int64 to Python int
    def to_int(val):
        if isinstance(val, np.integer):
            return int(val)
        elif isinstance(val, (list, tuple)):
            return [to_int(v) for v in val]
        return val

    plan = {
        "path": [[int(c.i), int(c.j)] for c in path],
        "visited_cells": [[int(c.i), int(c.j)] for c in graph.visited_cells],
        "dt": [],
        "map": str(graph.as_string()),
        "start": [int(start.i), int(start.j)],
        "goal": [int(goal.i), int(goal.j)],
        "planning_algo": algo,
    }

    with open(out_name, "w") as f:
        json.dump(to_int(plan), f)
