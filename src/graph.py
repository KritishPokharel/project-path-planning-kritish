import os
import numpy as np


class Cell(object):
    """Represents a single grid cell."""

    def __init__(self, i, j):
        self.i = i  # column (x)
        self.j = j  # row (y)

    def __lt__(self, other):
        if not isinstance(other, Cell):
            return NotImplemented
        return (self.i, self.j) < (other.i, other.j)


class GridGraph:
    """Occupancy grid as a graph for path planning."""

    def __init__(
        self,
        file_path=None,
        width=-1,
        height=-1,
        origin=(0, 0),
        meters_per_cell=0,
        cell_odds=None,
        collision_radius=0.05,
        threshold=0,
    ):
        if file_path is not None:
            assert self.load_from_file(file_path)
        else:
            self.width = width
            self.height = height
            self.origin = origin
            self.meters_per_cell = meters_per_cell
            self.cell_odds = cell_odds

        self.threshold = threshold
        self.set_collision_radius(collision_radius)
        self.visited_cells = []

        # Graph data
        self.parents = np.full((self.height, self.width, 2), -1, dtype=int)
        self.visited = np.zeros((self.height, self.width), dtype=bool)

    def load_from_file(self, file_path):
        """Loads the map file."""
        if not os.path.isfile(file_path):
            print(f"ERROR: Could not open {file_path}")
            return False

        with open(file_path, "r") as f:
            header = f.readline().split()
            origin_x, origin_y, self.width, self.height, self.meters_per_cell = map(
                float, header
            )
            self.origin = (origin_x, origin_y)
            self.width = int(self.width)
            self.height = int(self.height)

            self.cell_odds = np.zeros((self.height, self.width), dtype=np.int8)
            for r in range(self.height):
                row = f.readline().strip().split()
                for c in range(self.width):
                    self.cell_odds[r, c] = np.int8(row[c])
        return True

    def as_string(self):
        """Returns the map data as a string for visualization."""
        map_list = self.cell_odds.astype(str).tolist()
        rows = [" ".join(row) for row in map_list]
        cell_data = " ".join(rows)
        header = f"{self.origin[0]} {self.origin[1]} {self.width} {self.height} {self.meters_per_cell}"
        return " ".join([header, cell_data])

    def pos_to_cell(self, x, y):
        """World to grid indices."""
        i = int(np.floor((x - self.origin[0]) / self.meters_per_cell))
        j = int(np.floor((y - self.origin[1]) / self.meters_per_cell))
        return Cell(i, j)

    def cell_to_pos(self, i, j):
        """Grid to world position."""
        x = (i + 0.5) * self.meters_per_cell + self.origin[0]
        y = (j + 0.5) * self.meters_per_cell + self.origin[1]
        return x, y

    def is_cell_in_bounds(self, i, j):
        return 0 <= i < self.width and 0 <= j < self.height

    def is_cell_occupied(self, i, j):
        """Treat >0 as obstacle, <=0 as free."""
        return self.cell_odds[j, i] > 0

    def set_collision_radius(self, r):
        r_cells = int(np.ceil(r / self.meters_per_cell))
        r_indices, c_indices = np.indices((2 * r_cells - 1, 2 * r_cells - 1))
        c = r_cells - 1
        dists = (r_indices - c) ** 2 + (c_indices - c) ** 2
        self._coll_ind_j, self._coll_ind_i = np.nonzero(dists <= (r_cells - 1) ** 2)
        self.collision_radius = r
        self.collision_radius_cells = r_cells

    def check_collision(self, i, j):
        """Returns True if robot would collide at cell (i,j)."""
        j_inds = self._coll_ind_j + j - (self.collision_radius_cells - 1)
        i_inds = self._coll_ind_i + i - (self.collision_radius_cells - 1)
        in_bounds = np.bitwise_and(
            np.bitwise_and(j_inds >= 0, j_inds < self.height),
            np.bitwise_and(i_inds >= 0, i_inds < self.width),
        )
        return np.any(self.is_cell_occupied(i_inds[in_bounds], j_inds[in_bounds]))

    def find_neighbors(self, cell):
        """4-connected free neighbors."""
        nbrs = []
        directions = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        for di, dj in directions:
            ni, nj = cell.i + di, cell.j + dj
            if self.is_cell_in_bounds(ni, nj) and not self.check_collision(ni, nj):
                nbrs.append(Cell(ni, nj))
        return nbrs

    def get_parent(self, cell):
        pi, pj = self.parents[cell.j, cell.i]
        if pi == -1 or pj == -1:
            return None
        return Cell(pj, pi)

    def init_graph(self):
        """Reset all graph data before running search."""
        self.visited_cells = []
        self.parents[:, :, :] = -1
        self.visited[:, :] = False
