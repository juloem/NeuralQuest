import random
from math import atan2, cos, sin, sqrt
from src.bresenham import bresenham
from random import randrange
import matplotlib.pyplot as plt
from abc import ABC, abstractmethod
from collections import deque

#####################################################
# Maze Generator Interfaces and Implementations
#####################################################

class MazeGenerator(ABC):
    @abstractmethod
    def generate(self, width, height):
        pass

class RecursiveBacktracker(MazeGenerator):
    def generate(self, width, height):
        maze = self._init_maze(width, height)
        stack = [(0,0)]
        maze[0][0]['visited'] = True

        while stack:
            x, y = stack[-1]
            neighbors = self._unvisited_neighbors(x, y, maze, width, height)
            if not neighbors:
                stack.pop()
            else:
                nx, ny = random.choice(neighbors)
                self._remove_wall(maze, x, y, nx, ny)
                maze[ny][nx]['visited'] = True
                stack.append((nx, ny))
        return maze

    def _init_maze(self, width, height):
        maze = [[{'visited': False, 'walls': {'N': True, 'S': True, 'E': True, 'W': True}}
                  for _ in range(width)] for _ in range(height)]
        return maze

    def _unvisited_neighbors(self, x, y, maze, width, height):
        neighbors = []
        if x > 0 and not maze[y][x-1]['visited']:
            neighbors.append((x-1, y))
        if x < width-1 and not maze[y][x+1]['visited']:
            neighbors.append((x+1, y))
        if y > 0 and not maze[y-1][x]['visited']:
            neighbors.append((x, y-1))
        if y < height-1 and not maze[y+1][x]['visited']:
            neighbors.append((x, y+1))
        return neighbors

    def _remove_wall(self, maze, x1, y1, x2, y2):
        if x2 == x1 + 1:
            maze[y1][x1]['walls']['E'] = False
            maze[y2][x2]['walls']['W'] = False
        elif x2 == x1 - 1:
            maze[y1][x1]['walls']['W'] = False
            maze[y2][x2]['walls']['E'] = False
        elif y2 == y1 + 1:
            maze[y1][x1]['walls']['S'] = False
            maze[y2][x2]['walls']['N'] = False
        elif y2 == y1 - 1:
            maze[y1][x1]['walls']['N'] = False
            maze[y2][x2]['walls']['S'] = False

class Node:
    def __init__(self, coordinates, parent=None):
        # coordinates: [x, y]
        self.coordinates = coordinates
        self.parent = parent

#####################################################
# Maze Class
#####################################################

class Maze:
    def __init__(self, width, height, generator: MazeGenerator):
        self.width = width
        self.height = height
        self.generator = generator
        self.grid = None

    def generate_maze(self):
        self.grid = self.generator.generate(self.width, self.height)

    def get_grid(self):
        return self.grid


#####################################################
# Occupancy Grid Map
#####################################################

class OccupancyGridMap:
    def __init__(self, maze: Maze):
        self.maze = maze
        self.grid = self._maze_to_occupancy()

    def _maze_to_occupancy(self):
        maze_grid = self.maze.get_grid()
        height = self.maze.height
        width = self.maze.width

        occ_width = 2*width+1
        occ_height = 2*height+1
        occ_grid = [[1 for _ in range(occ_width)] for _ in range(occ_height)]

        for y in range(height):
            for x in range(width):
                cell = maze_grid[y][x]
                cx, cy = 2*x+1, 2*y+1
                occ_grid[cy][cx] = 0
                if cell['walls']['N'] == False:
                    occ_grid[cy-1][cx] = 0
                if cell['walls']['S'] == False:
                    occ_grid[cy+1][cx] = 0
                if cell['walls']['E'] == False:
                    occ_grid[cy][cx+1] = 0
                if cell['walls']['W'] == False:
                    occ_grid[cy][cx-1] = 0
        return occ_grid

    def get_occupancy_grid(self):
        return self.grid


#####################################################
# Path Planning
#####################################################

class PathPlanner(ABC):
    @abstractmethod
    def plan(self, start, goal):
        pass

class AStarPlanner(PathPlanner):
    def __init__(self, occupancy_grid, progress_callback=None):
        self.grid = occupancy_grid
        self.progress_callback = progress_callback

    def plan(self, start, goal):
        open_set = [(self._heuristic(start, goal), 0, start, None)]
        closed_set = set()
        came_from = {}

        step_count = 0
        while open_set:
            open_set.sort(key=lambda x: x[0])
            f, g, current, parent = open_set.pop(0)
            if current in closed_set:
                continue
            came_from[current] = parent

            if current == goal:
                path = self._reconstruct_path(came_from, current)
                if self.progress_callback:
                    self.progress_callback(path=path, open_set=open_set, closed_set=closed_set)
                return path

            closed_set.add(current)

            # Update progress visualization every N steps if callback is active
            if self.progress_callback and step_count % 10 == 0:
                self.progress_callback(open_set=open_set, closed_set=closed_set)

            for neighbor in self._get_neighbors(current):
                if neighbor in closed_set:
                    continue
                tentative_g = g + 1
                f_score = tentative_g + self._heuristic(neighbor, goal)
                open_set.append((f_score, tentative_g, neighbor, current))
            
            step_count += 1

        return []

    def _heuristic(self, a, b):
        return abs(a[0]-b[0]) + abs(a[1]-b[1])

    def _get_neighbors(self, cell):
        (r, c) = cell
        neighbors = []
        directions = [(1,0),(-1,0),(0,1),(0,-1)]
        for dr, dc in directions:
            nr, nc = r+dr, c+dc
            if 0 <= nr < len(self.grid) and 0 <= nc < len(self.grid[0]):
                if self.grid[nr][nc] == 0:
                    neighbors.append((nr,nc))
        return neighbors

    def _reconstruct_path(self, came_from, current):
        path = []
        while current is not None:
            path.append(current)
            current = came_from[current]
        path.reverse()
        return path


class RRTPlanner:
    def __init__(self, occupancy_grid, max_iterations=10000, max_branch_length=20, goal_tolerance=20, progress_callback=None):
        self.grid = occupancy_grid
        self.height = len(occupancy_grid)
        self.width = len(occupancy_grid[0]) if self.height > 0 else 0
        self.max_iterations = max_iterations
        self.max_branch_length = max_branch_length
        self.goal_tolerance = goal_tolerance
        self.progress_callback = progress_callback

    def plan(self, start, goal):
        """
        Plan a path using RRT from start to goal.
        start: (row, col) or (y, x)
        goal: (row, col) or (y, x)

        We'll convert to (x, y) for internal consistency.
        """
        start_pt = [start[1], start[0]]  # (x, y)
        goal_pt = [goal[1], goal[0]]     # (x, y)

        # Ensure start and goal are free
        if not self._is_free_cell(start_pt) or not self._is_free_cell(goal_pt):
            print("Start or goal is blocked. No path possible.")
            return []

        root_node = Node(start_pt)
        nodes = [root_node]

        for i in range(self.max_iterations):
            # Random point in the grid
            random_point = [randrange(self.width), randrange(self.height)]

            # Find closest node to this random point
            closest_node = self.find_closest_node(random_point, nodes)

            # Create a new point extending from closest_node towards random_point
            candidate_point = self.create_new_branch_point(closest_node.coordinates, random_point, self.max_branch_length)

            # Check if path from closest_node to candidate_point is free
            if not self.collision_detected(closest_node.coordinates, candidate_point):
                new_node = Node(candidate_point, closest_node)
                nodes.append(new_node)

                # Show progress every so often if callback exists
                if self.progress_callback and i % 100 == 0:
                    # Convert nodes to tree for visualization if needed
                    tree = {(n.coordinates[1], n.coordinates[0]): (n.parent.coordinates if n.parent else None) for n in nodes}
                    # Progress callback might need adaptation. For simplicity:
                    self.progress_callback(rrt_tree={node.coordinates: (node.parent.coordinates if node.parent else None) for node in nodes})

                # Check if goal reached
                if self.test_goal(new_node.coordinates, goal_pt, self.goal_tolerance):
                    # Reconstruct path
                    path = self._reconstruct_path(new_node, goal_pt)
                    if self.progress_callback:
                        self.progress_callback(path=[(p[1], p[0]) for p in path])
                    # Convert (x,y) back to (r,c)
                    return [(p[1], p[0]) for p in path]

        print("RRT: Max iterations exceeded, no path found.")
        return []

    def _is_free_cell(self, pt):
        x, y = pt
        if 0 <= y < self.height and 0 <= x < self.width:
            return self.grid[y][x] == 0
        return False

    def _reconstruct_path(self, node, goal_pt):
        path = [goal_pt]
        current = node
        while current:
            path.append(current.coordinates)
            current = current.parent
        path.reverse()
        return path

    # Helper functions similar to the provided RRT code

    def calculate_distance(self, p1, p2):
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        return sqrt(dx*dx + dy*dy)

    def calculate_angle(self, p1, p2):
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        return atan2(dy, dx)

    def collision_detected(self, p1, p2):
        # p1, p2 are [x,y]
        # Use bresenham to iterate through cells between p1 and p2
        line_cells = list(bresenham(p1[0], p1[1], p2[0], p2[1]))
        for (x, y) in line_cells:
            if not (0 <= x < self.width and 0 <= y < self.height):
                return True  # Out of bounds considered collision
            if self.grid[y][x] == 1:
                return True
        return False

    def find_closest_node(self, random_pt, node_list):
        nearest_distance = float('inf')
        nearest_node = None
        for n in node_list:
            dist = self.calculate_distance(random_pt, n.coordinates)
            if dist < nearest_distance:
                nearest_node = n
                nearest_distance = dist
        return nearest_node

    def create_new_branch_point(self, p1, p2, max_distance):
        # p1, p2 are [x,y]
        d = self.calculate_distance(p1, p2)
        theta = self.calculate_angle(p1, p2)

        if max_distance > d:
            max_distance = d

        new_x = p1[0] + int(max_distance * cos(theta))
        new_y = p1[1] + int(max_distance * sin(theta))

        return [new_x, new_y]

    def test_goal(self, p1, p_goal, tolerance):
        return self.calculate_distance(p1, p_goal) < tolerance


#####################################################
# Visualization
#####################################################

class MazeVisualizer:
    def __init__(self, occupancy_grid, path=None, wall_thickness=1, show_path=True, start=None, goal=None, show_progress=False, delay=0.5):
        self.grid = occupancy_grid
        self.path = path
        self.wall_thickness = wall_thickness
        self.show_path = show_path
        self.start = start
        self.goal = goal
        self.show_progress = show_progress
        self.delay = delay  # Added delay for slower animation

        if self.show_progress:
            plt.ion()

        self.fig, self.ax = plt.subplots(figsize=(len(self.grid[0])/5, len(self.grid)/5))
        self._draw()

    def _draw(self):
        self.ax.clear()
        self.ax.imshow(self.grid, cmap='binary', origin='upper')

        if self.show_path and self.path:
            y_coords = [p[0] for p in self.path]
            x_coords = [p[1] for p in self.path]
            self.ax.plot(x_coords, y_coords, color='red', linewidth=self.wall_thickness)

        if self.start is not None:
            sr, sc = self.start
            self.ax.plot(sc, sr, 'o', color='green', markersize=8, markeredgecolor='black')

        if self.goal is not None:
            gr, gc = self.goal
            self.ax.plot(gc, gr, marker='*', color='yellow', markersize=12, markeredgecolor='black')

        self.ax.set_xticks([])
        self.ax.set_yticks([])
        self.ax.set_aspect('equal')
        self.fig.canvas.draw()
        if self.show_progress:
            plt.pause(self.delay)  # Use the specified delay

    def update_path(self, path):
        self.path = path
        self._draw()

    def update_progress(self, path=None, open_set=None, closed_set=None, rrt_tree=None):
        if not self.show_progress:
            return

        self.ax.clear()
        self.ax.imshow(self.grid, cmap='binary', origin='upper')

        if self.start is not None:
            sr, sc = self.start
            self.ax.plot(sc, sr, 'o', color='green', markersize=8, markeredgecolor='black')

        if self.goal is not None:
            gr, gc = self.goal
            self.ax.plot(gc, gr, marker='*', color='yellow', markersize=12, markeredgecolor='black')

        if open_set is not None:
            for _, _, cell, _ in open_set:
                self.ax.plot(cell[1], cell[0], 'o', color='blue', markersize=4)

        if closed_set is not None:
            for cell in closed_set:
                self.ax.plot(cell[1], cell[0], 's', color='gray', markersize=2)

        if rrt_tree is not None:
            for node, parent in rrt_tree.items():
                if parent is not None:
                    self.ax.plot([parent[1], node[1]], [parent[0], node[0]], color='magenta', linewidth=1)

        if path:
            y_coords = [p[0] for p in path]
            x_coords = [p[1] for p in path]
            self.ax.plot(x_coords, y_coords, color='red', linewidth=self.wall_thickness)

        self.ax.set_xticks([])
        self.ax.set_yticks([])
        self.ax.set_aspect('equal')
        self.fig.canvas.draw()
        plt.pause(self.delay)  # Use the specified delay here as well


#####################################################
# Helper Functions
#####################################################

def get_random_start_goal(occupancy_grid):
    free_cells = [(r, c) for r in range(len(occupancy_grid))
                          for c in range(len(occupancy_grid[0]))
                          if occupancy_grid[r][c] == 0]
    if len(free_cells) < 2:
        raise ValueError("Not enough free cells to choose start and goal.")
    start = random.choice(free_cells)
    free_cells.remove(start)
    goal = random.choice(free_cells)
    return start, goal


#####################################################
# Example Main Usage
#####################################################

if __name__ == "__main__":
    width, height = 10, 10
    generator = RecursiveBacktracker()
    maze = Maze(width, height, generator)
    maze.generate_maze()

    occ_map = OccupancyGridMap(maze)
    occ_grid = occ_map.get_occupancy_grid()

    start, goal = get_random_start_goal(occ_grid)
    print("Start:", start, "Goal:", goal)

    # Show progress and set a delay of 1 second per update:
    show_progress = False
    visualization_delay = 1.0  # Increase this value to slow down the animation

    viz = MazeVisualizer(occ_grid, wall_thickness=2, show_path=True, start=start, goal=goal, show_progress=show_progress, delay=visualization_delay)

    def progress_callback(path=None, open_set=None, closed_set=None, rrt_tree=None):
        viz.update_progress(path=path, open_set=open_set, closed_set=closed_set, rrt_tree=rrt_tree)

    #planner = AStarPlanner(occ_grid, progress_callback=progress_callback if show_progress else None)
    planner = RRTPlanner(occ_grid, max_iterations=50000, max_branch_length=1, goal_tolerance=1)
    path = planner.plan(start, goal)
    print("Path found:", path)

    viz.update_path(path)
    plt.ioff()
    plt.show()
