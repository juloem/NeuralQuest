from src.maze_cell import MazeCell
import random
import numpy as np

class Maze:
    """
    A class to represent a maze.
    Attributes:
    -----------
    rows : int
        Number of rows in the maze.
    cols : int
        Number of columns in the maze.
    grid : list of list of MazeCell
        2D grid representing the maze cells.
    Methods:
    --------
    __init__(rows, cols):
        Initializes the maze with the given number of rows and columns.
    cell_neighbors(cell):
        Returns the neighboring cells of a given cell along with the direction.
    remove_walls_between(cell1, cell2):
        Removes the walls between two adjacent cells.
    to_occupancy_map(corridor_size=1, wall_size=1):
        Converts the maze to an occupancy map with specified corridor and wall sizes.
    """
    def __init__(self, rows, cols):
        self.rows = rows
        self.cols = cols
        self.grid = [[MazeCell(r, c) for c in range(cols)] for r in range(rows)]

    def cell_neighbors(self, cell):
        directions = {
            'N': (-1, 0),
            'S': (1, 0),
            'E': (0, 1),
            'W': (0, -1)
        }
        neighbors = []
        for d, (dr, dc) in directions.items():
            nr, nc = cell.row + dr, cell.col + dc
            if 0 <= nr < self.rows and 0 <= nc < self.cols:
                neighbors.append((d, self.grid[nr][nc]))
        return neighbors

    def remove_walls_between(self, cell1, cell2):
        r_diff = cell2.row - cell1.row
        c_diff = cell2.col - cell1.col
        if r_diff == -1:  # North
            cell1.remove_wall('N')
            cell2.remove_wall('S')
        elif r_diff == 1:  # South
            cell1.remove_wall('S')
            cell2.remove_wall('N')
        elif c_diff == 1:  # East
            cell1.remove_wall('E')
            cell2.remove_wall('W')
        elif c_diff == -1:  # West
            cell1.remove_wall('W')
            cell2.remove_wall('E')

    def to_occupancy_map(self, corridor_size=1, wall_size=1):
        cell_block = corridor_size + wall_size
        map_rows = self.rows * cell_block + wall_size
        map_cols = self.cols * cell_block + wall_size
        occ_map = [[1 for _ in range(map_cols)] for _ in range(map_rows)]

        for r in range(self.rows):
            for c in range(self.cols):
                cell = self.grid[r][c]
                start_row = r * cell_block + wall_size
                start_col = c * cell_block + wall_size
                for i in range(corridor_size):
                    for j in range(corridor_size):
                        occ_map[start_row + i][start_col + j] = 0

                if not cell.has_wall('E'):
                    for i in range(corridor_size):
                        for j in range(wall_size):
                            occ_map[start_row + i][start_col + corridor_size + j] = 0
                if not cell.has_wall('S'):
                    for i in range(wall_size):
                        for j in range(corridor_size):
                            occ_map[start_row + corridor_size + i][start_col + j] = 0

        return occ_map
    
    def set_random_start_and_end(self, occ_map):
        """
        Sets random start and end points in free cells of the maze.
        Updates the `start` and `end` attributes of the maze.
        """
        free_cells = np.argwhere(np.array(occ_map) == 0)
        start_cell = tuple(free_cells[random.randint(0, len(free_cells) - 1)])
        end_cell = tuple(free_cells[random.randint(0, len(free_cells) - 1)])

        while end_cell == start_cell:
            end_cell = tuple(free_cells[random.randint(0, len(free_cells) - 1)])

        self.start = start_cell
        self.end = end_cell

    def find_path(self, planner, occupancy_map):
        """
        Uses the given planner to find a path from start to end.
        """
        if self.start is None or self.end is None:
            raise ValueError("Start and end points are not set in the maze.")
        return planner.find_path(self.start, self.end, occupancy_map)
