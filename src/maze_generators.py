import random
from abc import ABC, abstractmethod


class MazeGenerator(ABC):
    """
    Abstract base class for maze generators.

    This class defines the interface that all maze generator classes must implement.
    Subclasses must provide an implementation for the `generate` method.

    Methods:
        generate(maze):
            Abstract method to generate a maze. Must be implemented by subclasses.
    """
    @abstractmethod
    def generate(self, maze):
        pass


class RecursiveBacktrackerGenerator(MazeGenerator):
    """
    A maze generator that uses the recursive backtracking algorithm.
    Methods
    -------
    generate(maze)
        Generates a maze using the recursive backtracking algorithm.
    """
    def generate(self, maze):
        stack = []
        start = maze.grid[random.randint(0, maze.rows - 1)][random.randint(0, maze.cols - 1)]
        visited = {(start.row, start.col)}
        stack.append(start)

        while stack:
            current = stack[-1]
            unvisited_neighbors = [(d, n) for (d, n) in maze.cell_neighbors(current)
                                   if (n.row, n.col) not in visited]
            if unvisited_neighbors:
                _, nxt = random.choice(unvisited_neighbors)
                maze.remove_walls_between(current, nxt)
                visited.add((nxt.row, nxt.col))
                stack.append(nxt)
            else:
                stack.pop()
