import heapq, math
from .graph import Cell
from .utils import trace_path


def depth_first_search(graph, start, goal):
    graph.init_graph()
    stack = [start]
    graph.visited[start.i, start.j] = True

    while stack:
        c = stack.pop()
        graph.visited_cells.append(Cell(c.i, c.j))

        if c.i == goal.i and c.j == goal.j:
            return trace_path(c, graph)

        for n in graph.find_neighbors(c):
            if not graph.visited[n.i, n.j]:
                graph.visited[n.i, n.j] = True
                graph.parents[n.j, n.i] = [c.j, c.i]
                stack.append(n)
    return []


def breadth_first_search(graph, start, goal):
    graph.init_graph()
    queue = [start]
    graph.visited[start.i, start.j] = True

    while queue:
        c = queue.pop(0)
        graph.visited_cells.append(Cell(c.i, c.j))

        if c.i == goal.i and c.j == goal.j:
            return trace_path(c, graph)

        for n in graph.find_neighbors(c):
            if not graph.visited[n.i, n.j]:
                graph.visited[n.i, n.j] = True
                graph.parents[n.j, n.i] = [c.j, c.i]
                queue.append(n)
    return []


def heuristic(a, b):
    return math.sqrt((a.i - b.i) ** 2 + (a.j - b.j) ** 2)


def a_star_search(graph, start, goal):
    graph.init_graph()
    open_set = []
    heapq.heappush(open_set, (0, start))
    g_score = {(start.i, start.j): 0}

    while open_set:
        _, c = heapq.heappop(open_set)
        graph.visited_cells.append(Cell(c.i, c.j))

        if c.i == goal.i and c.j == goal.j:
            return trace_path(c, graph)

        for n in graph.find_neighbors(c):
            tentative_g = g_score[(c.i, c.j)] + 1
            if (n.i, n.j) not in g_score or tentative_g < g_score[(n.i, n.j)]:
                g_score[(n.i, n.j)] = tentative_g
                f = tentative_g + heuristic(n, goal)
                graph.parents[n.j, n.i] = [c.j, c.i]
                heapq.heappush(open_set, (f, n))
    return []
