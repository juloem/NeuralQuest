import random
import matplotlib
matplotlib.use('Qt5Agg')
from src.cell import Cell
import matplotlib.pyplot as plt
import numpy as np

from src.utils.logger import setup_logger

class Maze:
    """
    A class to represent a maze.
    Attributes
    ----------
    width : int
        The width of the maze.
    height : int
        The height of the maze.
    grid : list
        A 2D list representing the grid of the maze.
    Methods
    -------
    create_grid():
        Creates a grid of cells for the maze.
    get_cell(row, col):
        Returns the cell at the specified row and column.
    get_unvisited_neighbors(cell):
        Returns a list of unvisited neighboring cells of the given cell.
    get_neighbors(cell):
        Returns a list of all neighboring cells of the given cell.
    generate_maze_aldous_broder():
        Generates a maze using the Aldous-Broder algorithm.
    generate_maze_recursive_backtracker():
        Generates a maze using the Recursive Backtracker algorithm.
    to_points():
        Converts the maze to a list of points for drawing.
    reset_maze():
        Resets the maze by marking all cells as unvisited and restoring all walls.
    draw_maze():
        Draws the maze using matplotlib.
    """
    def __init__(self, width, height):
        """
        Initializes a new instance of the maze with the specified width and height.

        Args:
            width (int): The width of the maze.
            height (int): The height of the maze.
        """
        self._logger = setup_logger(self.__class__.__name__)
        self.width = width
        self.height = height
        self.grid = self.create_grid()
        self.start = None
        self.end = None

    def create_grid(self):
        """
        Creates a grid of Cell objects for the maze.

        The grid is a 2D list where each element is an instance of the Cell class,
        representing a cell in the maze.

        Returns:
            list: A 2D list of Cell objects representing the maze grid.
        """
        grid = [[Cell(row, col) for col in range(self.width)] for row in range(self.height)]
        self._logger.debug(f"Created {self.width}x{self.height} maze")
        return grid

    def get_cell(self, row, col):
        """
        Retrieve the cell at the specified row and column in the grid.

        Args:
            row (int): The row index of the cell.
            col (int): The column index of the cell.

        Returns:
            Cell: The cell object located at the specified row and column.
        """
        return self.grid[row][col]

    def get_unvisited_neighbors(self, cell):
        """
        Get a list of unvisited neighboring cells and their corresponding direction indices.

        Args:
            cell (Cell): The current cell for which to find unvisited neighbors.

        Returns:
            List[Tuple[Cell, int]]: A list of tuples, each containing an unvisited neighboring cell 
                                    and its direction index (0: Up, 1: Right, 2: Down, 3: Left).
        """
        neighbors = []
        directions = [(-1, 0), (0, 1), (1, 0), (0, -1)]  # [Up, Right, Down, Left]
        for idx, (dr, dc) in enumerate(directions):
            r, c = cell.row + dr, cell.col + dc
            if 0 <= r < self.height and 0 <= c < self.width:
                neighbor = self.get_cell(r, c)
                if not neighbor.visited:
                    neighbors.append((neighbor, idx))
        return neighbors

    def get_neighbors(self, cell):
        """
        Get the neighboring cells of a given cell in the maze.

        Args:
            cell (Cell): The cell for which to find the neighbors.

        Returns:
            list of tuple: A list of tuples where each tuple contains a neighboring cell and the direction index.
                           The direction index corresponds to:
                           0 - Up, 1 - Right, 2 - Down, 3 - Left.
        """
        neighbors = []
        directions = [(-1, 0), (0, 1), (1, 0), (0, -1)]  # [Up, Right, Down, Left]
        for idx, (dr, dc) in enumerate(directions):
            r, c = cell.row + dr, cell.col + dc
            if 0 <= r < self.height and 0 <= c < self.width:
                neighbor = self.get_cell(r, c)
                neighbors.append((neighbor, idx))
        return neighbors

    def generate_maze_aldous_broder(self):
        """
        Generates a maze using the Aldous-Broder algorithm.
        The Aldous-Broder algorithm is a random walk algorithm that guarantees
        an unbiased maze. It starts from a random cell and performs a random walk
        until all cells have been visited. During the walk, it removes walls between
        the current cell and a randomly chosen neighboring cell if the neighbor has
        not been visited yet.
        This method modifies the maze in place.
        Attributes:
            total_cells (int): The total number of cells in the maze.
            visited_cells (int): The number of cells that have been visited.
            current_cell (Cell): The current cell being visited.
            neighbors (list): A list of neighboring cells and the direction to them.
            neighbor_cell (Cell): A randomly chosen neighboring cell.
            direction (str): The direction to the neighboring cell.
        """
        total_cells = self.width * self.height
        visited_cells = 1

        current_cell = self.get_cell(random.randint(0, self.height - 1), random.randint(0, self.width - 1))
        current_cell.visited = True

        while visited_cells < total_cells:
            neighbors = self.get_neighbors(current_cell)
            neighbor_cell, direction = random.choice(neighbors)

            if not neighbor_cell.visited:
                current_cell.remove_wall(neighbor_cell, direction)
                neighbor_cell.visited = True
                visited_cells += 1

            current_cell = neighbor_cell

    def generate_maze_wilson(self):
        unvisited_cells = set()
        for row in self.grid:
            for cell in row:
                unvisited_cells.add(cell)

        # Choose a random cell and add it to the maze (tree)
        initial_cell = random.choice(tuple(unvisited_cells))
        initial_cell.visited = True
        unvisited_cells.remove(initial_cell)

        while unvisited_cells:
            # Choose a random unvisited cell to start a random walk
            walk_start = random.choice(tuple(unvisited_cells))
            path = [walk_start]
            cell_in_tree = False
            cell_positions = {(walk_start.row, walk_start.col): 0}

            while not cell_in_tree:
                current_cell = path[-1]
                neighbors = self.get_neighbors(current_cell)
                neighbor_cell, direction = random.choice(neighbors)
                # Loop-Erasure
                if neighbor_cell in path:
                    # Erase the loop
                    loop_start = cell_positions[(neighbor_cell.row, neighbor_cell.col)]
                    path = path[:loop_start + 1]
                    path_positions = path[:loop_start + 1]
                    cell_positions = {(cell.row, cell.col): idx for idx, cell in enumerate(path_positions)}
                else:
                    path.append(neighbor_cell)
                    cell_positions[(neighbor_cell.row, neighbor_cell.col)] = len(path) - 1
                    if neighbor_cell.visited:
                        cell_in_tree = True

            # Add the loop-erased path to the maze
            for i in range(len(path) - 1):
                cell = path[i]
                next_cell = path[i + 1]
                direction = self.get_direction(cell, next_cell)
                cell.remove_wall(next_cell, direction)
                if not cell.visited:
                    cell.visited = True
                    unvisited_cells.remove(cell)
                if not next_cell.visited:
                    next_cell.visited = True
                    unvisited_cells.discard(next_cell)  # Use discard in case next_cell is the final visited cell

    def get_direction(self, cell1, cell2):
        # Returns the direction from cell1 to cell2
        dr = cell2.row - cell1.row
        dc = cell2.col - cell1.col
        if dr == -1 and dc == 0:
            return 0  # Up
        elif dr == 0 and dc == 1:
            return 1  # Right
        elif dr == 1 and dc == 0:
            return 2  # Down
        elif dr == 0 and dc == -1:
            return 3  # Left
        else:
            return -1  # Not a neighboring cell

    def generate_maze_recursive_backtracker(self):
        """
        Generates a maze using the recursive backtracker algorithm.
        This method starts from a random cell, marks it as visited, and uses a stack to keep track of the path.
        It continues to visit unvisited neighboring cells, removing walls between the current cell and the chosen neighbor.
        If a cell has no unvisited neighbors, it backtracks to the previous cell in the stack.
        The algorithm continues until all cells have been visited.
        Returns:
            None
        """
        stack = []

        current_cell = self.get_cell(random.randint(0, self.height - 1), random.randint(0, self.width - 1))
        current_cell.visited = True
        stack.append(current_cell)

        while stack:
            current_cell = stack[-1]
            unvisited_neighbors = self.get_unvisited_neighbors(current_cell)
            if unvisited_neighbors:
                neighbor_cell, direction = random.choice(unvisited_neighbors)
                current_cell.remove_wall(neighbor_cell, direction)
                neighbor_cell.visited = True
                stack.append(neighbor_cell)
            else:
                stack.pop()  # Backtrack

    def to_points(self):
        """
        Converts the maze grid into a list of points representing the walls.
        This method iterates through each cell in the grid and checks for the presence
        of walls on each side of the cell (top, right, bottom, left). For each wall
        that exists, it adds the corresponding points to the list.
        Returns:
            list: A list of points where each point is represented as a list of two integers [x, y].
        """
        points = []
        for row in self.grid:
            for cell in row:
                x, y = cell.col, self.height - cell.row - 1

                if cell.walls[0]:  # Top wall
                    points.append([x, y + 1])
                    points.append([x + 1, y + 1])
                if cell.walls[1]:  # Right wall
                    points.append([x + 1, y])
                    points.append([x + 1, y + 1])
                if cell.walls[2]:  # Bottom wall
                    points.append([x, y])
                    points.append([x + 1, y])
                if cell.walls[3]:  # Left wall
                    points.append([x, y])
                    points.append([x, y + 1])
        return points

    def reset_maze(self):
        """
        Resets the maze to its initial state.

        This method iterates through each cell in the maze grid and sets the 
        'visited' attribute of each cell to False and the 'walls' attribute 
        to a list of four True values, indicating that all walls are intact.
        """
        for row in self.grid:
            for cell in row:
                cell.visited = False
                cell.walls = [True, True, True, True]

    def draw_maze(self):
        """
        Draws the maze using matplotlib.
        This method creates a visual representation of the maze by plotting the walls of each cell.
        It uses matplotlib to create a figure and axis, and then iterates through the grid of cells,
        drawing lines for each wall that exists.
        The maze is displayed with the origin (0, 0) at the bottom-left corner, and the y-axis is inverted
        to match the typical maze representation where the top row is at the highest y-coordinate.
        The walls of each cell are represented as follows:
        - Top wall: horizontal line from (x, y + 1) to (x + 1, y + 1)
        - Right wall: vertical line from (x + 1, y) to (x + 1, y + 1)
        - Bottom wall: horizontal line from (x, y) to (x + 1, y)
        - Left wall: vertical line from (x, y) to (x, y + 1)
        The method does not return any value. It displays the maze plot using plt.show().
        Parameters:
        None
        """
        _, ax = plt.subplots(figsize=(10, 10))
        ax.set_aspect('equal')
        plt.axis('off')

        for row in self.grid:
            for cell in row:
                x, y = cell.col, self.height - cell.row - 1

                if cell.walls[0]:
                    ax.plot([x, x + 1], [y + 1, y + 1], color='black')
                if cell.walls[1]:
                    ax.plot([x + 1, x + 1], [y, y + 1], color='black')
                if cell.walls[2]:
                    ax.plot([x, x + 1], [y, y], color='black')
                if cell.walls[3]:
                    ax.plot([x, x], [y, y + 1], color='black')

        plt.show()

    def draw_environment_points(self):
        environment_points = self.to_points()
        environment_points = np.array(environment_points)
        
        plt.figure(figsize=(10, 10))
        plt.scatter(environment_points[:, 0], environment_points[:, 1], s=1, color='black')
        plt.title("Environment Points Visualization")
        plt.axis('equal')
        plt.axis('off')
        plt.show()

    def draw_maze(self, ax=None, show_environment_points=False,
                  environment_points=None, basis_points=None, encoding=None, path=None, path_label='Path'):
        if ax is None:
            fig, ax = plt.subplots(figsize=(10, 10))
        else:
            fig = ax.get_figure()

        ax.set_aspect('equal')
        #plt.axis('off')
        ax.axis('off')

        # Draw the maze walls
        for row in self.grid:
            for cell in row:
                x, y = cell.col, self.height - cell.row - 1

                if cell.walls[0]:
                    ax.plot([x, x + 1], [y + 1, y + 1], color='black')
                if cell.walls[1]:
                    ax.plot([x + 1, x + 1], [y, y + 1], color='black')
                if cell.walls[2]:
                    ax.plot([x, x + 1], [y, y], color='black')
                if cell.walls[3]:
                    ax.plot([x, x], [y, y + 1], color='black')

        # Plot the start and end points if available
        if self.start:
            ax.scatter(self.start.col + 0.5, self.height - self.start.row - 0.5, s=100, color='green', marker='P', label='Start')
        if self.end:
            ax.scatter(self.end.col + 0.5, self.height - self.end.row - 0.5, s=100, color='red', marker='X', label='End')

        # Overlay environment points if requested
        if show_environment_points and environment_points is not None:
            environment_points = np.array(environment_points)
            ax.scatter(environment_points[:, 0], environment_points[:, 1], s=1, color='blue', label='Environment Points')

        # Overlay basis points and encoding if provided
        if basis_points is not None and encoding is not None:
            scatter = ax.scatter(basis_points[:, 0], basis_points[:, 1],
                                c=encoding, cmap='viridis', s=20, label='Basis Points')
            fig.colorbar(scatter, ax=ax, label='Encoding Value')

        # Plot the path if available
        if path:
            path_points = [[cell.col + 0.5, self.height - cell.row - 0.5] for cell in path]
            path_points = np.array(path_points)
            ax.plot(path_points[:, 0], path_points[:, 1], color='orange',
                    linewidth=2, label=path_label)
       

        ax.legend()
        plt.show()

    def set_start_end(self):
        """
        Randomly selects and sets the start and end cells for the maze.
        This method selects two distinct cells from the maze grid to be the start and end points.
        It ensures that the start and end points are not the same.
        Attributes:
            self.start: The starting cell of the maze.
            self.end: The ending cell of the maze.
        """
        import random

        all_cells = [cell for row in self.grid for cell in row]
        self.start = random.choice(all_cells)
        self.end = random.choice(all_cells)
        while self.end == self.start:
            self.end = random.choice(all_cells)
