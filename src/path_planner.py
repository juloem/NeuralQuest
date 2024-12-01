# path_planner.py

import heapq
import random

class PathPlanner:
    import heapq
import random
from itertools import count  # For unique sequence count

class PathPlanner:
    def __init__(self, maze):
        self.maze = maze
        self.start = maze.start
        self.end = maze.end
        if not self.start or not self.end:
            raise ValueError("Start or end point not set in the maze.")
        self.path = None

    def a_star_search(self):
        open_set = []
        came_from = {}
        g_score = {cell: float('inf') for row in self.maze.grid for cell in row}
        g_score[self.start] = 0
        f_score = {cell: float('inf') for row in self.maze.grid for cell in row}
        f_score[self.start] = self.heuristic(self.start, self.end)
        counter = count()  # Unique sequence count

        # Push the start node onto the heap
        heapq.heappush(open_set, (f_score[self.start], next(counter), self.start))

        while open_set:
            current = heapq.heappop(open_set)[2]  # Access the cell
            if current == self.end:
                self.path = self.reconstruct_path(came_from, current)
                return self.path

            for neighbor, direction in self.maze.get_neighbors(current):
                if not current.walls[direction]:  # Check if path exists
                    tentative_g_score = g_score[current] + 1  # Assuming uniform cost
                    if tentative_g_score < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g_score
                        f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, self.end)
                        # Check if neighbor is already in open_set
                        if neighbor not in [item[2] for item in open_set]:
                            heapq.heappush(open_set, (f_score[neighbor], next(counter), neighbor))
        return None  # No path found

    def heuristic(self, cell1, cell2):
        # Use Manhattan distance as heuristic
        return abs(cell1.row - cell2.row) + abs(cell1.col - cell2.col)

    def reconstruct_path(self, came_from, current):
        path = []
        while current in came_from:
            path.append(current)
            current = came_from[current]
        path.append(self.start)
        path.reverse()
        return path

    def rrt_search(self, max_iterations=10000, goal_sample_rate=0.5):
        if not self.start or not self.end:
            raise ValueError("Start or end point not set.")

        tree = {self.start: None}  # Node: Parent
        for _ in range(max_iterations):
            # Decide whether to sample the goal or a random point
            if random.random() < goal_sample_rate:
                random_cell = self.end
            else:
                # Randomly select a cell
                random_cell = self.maze.get_cell(random.randint(0, self.maze.height - 1), random.randint(0, self.maze.width - 1))
                # Find nearest node in the tree
                nearest_cell = min(tree.keys(), key=lambda cell: self.distance(cell, random_cell))
                # Attempt to move towards random_cell from nearest_cell
                path_segment = self.get_path_segment(nearest_cell, random_cell)
                if path_segment:
                    for i in range(1, len(path_segment)):
                        current_cell = path_segment[i - 1]
                        next_cell = path_segment[i]
                        if next_cell not in tree:
                            tree[next_cell] = current_cell
                            if next_cell == self.end:
                                self.path = self.reconstruct_rrt_path(tree, next_cell)
                                return self.path
        return None  # No path found
    
    def get_path_segment(self, from_cell, to_cell):
        path = [from_cell]
        current_cell = from_cell
        while current_cell != to_cell:
            dr = to_cell.row - current_cell.row
            dc = to_cell.col - current_cell.col
            # Decide direction to move
            if abs(dr) > abs(dc):
                step_row = current_cell.row + (1 if dr > 0 else -1)
                step_col = current_cell.col
            elif dc != 0:
                step_row = current_cell.row
                step_col = current_cell.col + (1 if dc > 0 else -1)
            else:
                # Reached the target cell
                break

            if 0 <= step_row < self.maze.height and 0 <= step_col < self.maze.width:
                next_cell = self.maze.get_cell(step_row, step_col)
                direction = self.maze.get_direction(current_cell, next_cell)
                if not current_cell.walls[direction]:
                    path.append(next_cell)
                    current_cell = next_cell
                    if current_cell == to_cell:
                        break
                else:
                    # Obstacle encountered
                    break
            else:
                # Out of bounds
                break
        return path if len(path) > 1 else None

    def distance(self, cell1, cell2):
        return abs(cell1.row - cell2.row) + abs(cell1.col - cell2.col)

    def get_next_cell(self, from_cell, to_cell):
        # Determine direction to move
        dr = to_cell.row - from_cell.row
        dc = to_cell.col - from_cell.col
        if abs(dr) > abs(dc):
            step = (1 if dr > 0 else -1, 0)
        else:
            step = (0, 1 if dc > 0 else -1)
        next_row = from_cell.row + step[0]
        next_col = from_cell.col + step[1]
        if 0 <= next_row < self.maze.height and 0 <= next_col < self.maze.width:
            next_cell = self.maze.get_cell(next_row, next_col)
            direction = self.maze.get_direction(from_cell, next_cell)
            if not from_cell.walls[direction]:
                return next_cell
        return None

    def reconstruct_rrt_path(self, tree, current):
        path = [current]
        while current in tree and tree[current]:
            current = tree[current]
            path.append(current)
        path.reverse()
        return path
