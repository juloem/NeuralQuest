import unittest
from src.maze import Maze
from src.cell import Cell

class TestMaze(unittest.TestCase):

    def setUp(self):
        self.maze = Maze(5, 5)

    def test_create_grid(self):
        self.assertEqual(len(self.maze.grid), 5)
        self.assertEqual(len(self.maze.grid[0]), 5)
        self.assertIsInstance(self.maze.grid[0][0], Cell)

    def test_get_cell(self):
        cell = self.maze.get_cell(0, 0)
        self.assertIsInstance(cell, Cell)
        self.assertEqual(cell.row, 0)
        self.assertEqual(cell.col, 0)

    def test_get_unvisited_neighbors(self):
        cell = self.maze.get_cell(0, 0)
        neighbors = self.maze.get_unvisited_neighbors(cell)
        self.assertIsInstance(neighbors, list)

    def test_get_neighbors(self):
        cell = self.maze.get_cell(0, 0)
        neighbors = self.maze.get_neighbors(cell)
        self.assertIsInstance(neighbors, list)
        self.assertEqual(len(neighbors), 2)

    def test_generate_maze_aldous_broder(self):
        self.maze.generate_maze_aldous_broder()
        visited_cells = sum(cell.visited for row in self.maze.grid for cell in row)
        self.assertEqual(visited_cells, 25)

    def test_generate_maze_recursive_backtracker(self):
        self.maze.generate_maze_recursive_backtracker()
        visited_cells = sum(cell.visited for row in self.maze.grid for cell in row)
        self.assertEqual(visited_cells, 25)

    def test_to_points(self):
        points = self.maze.to_points()
        self.assertIsInstance(points, list)

    def test_reset_maze(self):
        self.maze.generate_maze_aldous_broder()
        self.maze.reset_maze()
        for row in self.maze.grid:
            for cell in row:
                self.assertFalse(cell.visited)
                self.assertEqual(cell.walls, [True, True, True, True])

if __name__ == '__main__':
    unittest.main()