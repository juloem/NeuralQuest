class MazeCell:
    """
    A class to represent a cell in a maze.
    Attributes:
    ----------
    row : int
        The row position of the cell in the maze.
    col : int
        The column position of the cell in the maze.
    walls : dict
        A dictionary indicating the presence of walls in the four cardinal directions (N, S, E, W).
    Methods:
    -------
    remove_wall(direction):
        Removes the wall in the specified direction.
    has_wall(direction):
        Checks if there is a wall in the specified direction.
    """
    def __init__(self, row, col):
        self.row = row
        self.col = col
        self.walls = {
            'N': True,
            'S': True,
            'E': True,
            'W': True
        }

    def remove_wall(self, direction):
        self.walls[direction] = False

    def has_wall(self, direction):
        return self.walls[direction]
