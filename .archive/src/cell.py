from src.utils.logger import setup_logger

class Cell:
    """
    A class to represent a cell in a maze.
    Attributes:
        row (int): The row position of the cell in the maze.
        col (int): The column position of the cell in the maze.
        walls (list of bool): A list representing the presence of walls around the cell.
                              The order is [Top, Right, Bottom, Left].
        visited (bool): A flag indicating whether the cell has been visited.
    Methods:
        remove_wall(other, wall): Remove the wall between this cell and the neighboring cell.
    """
    
    def __init__(self, row, col):
        """
        Initializes a Cell object with a specified row and column.

        Args:
            row (int): The row index of the cell.
            col (int): The column index of the cell.

        Attributes:
            _logger (Logger): Logger instance for the Cell class.
            row (int): The row index of the cell.
            col (int): The column index of the cell.
            walls (list): A list of boolean values indicating the presence of walls 
                          [Top, Right, Bottom, Left].
            visited (bool): A flag indicating whether the cell has been visited.
        """
        self._logger = setup_logger(self.__class__.__name__)
        self.row = row
        self.col = col
        self.walls = [True, True, True, True]  # [Top, Right, Bottom, Left]
        self.visited = False

    def __lt__(self, other):
        return (self.row, self.col) < (other.row, other.col)

    def remove_wall(self, other, wall):
        """
        Remove the wall between this cell and the neighboring cell.

        Parameters:
        - other: The neighboring Cell object.
        - wall: The index of the wall to remove.
        """
        try:
            self.walls[wall] = False
            other.walls[(wall + 2) % 4] = False
        except IndexError:
            print("Invalid wall index.")
            return
