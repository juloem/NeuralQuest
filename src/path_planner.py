from abc import ABC, abstractmethod
import heapq
import random
from math import sqrt, atan2, cos, sin
from src.bresenham import bresenham
from src.heuristic import euclidean_distance

class PathPlanner(ABC):
    """
    Abstract base class for all path planning algorithms.
    """
    @abstractmethod
    def find_path(self, start, end, occupancy_map):
        """
        Finds a path from start to end in the given occupancy map.
        """
        pass


class Node:
    """
    Represents a node in the grid for path planning.
    """
    def __init__(self, position, g=0, h=0, parent=None):
        self.position = position  # (row, col)
        self.g = g  # Cost from start to current node
        self.h = h  # Heuristic cost to the goal
        self.f = g + h  # Total cost (f = g + h)
        self.parent = parent  # Parent node for path reconstruction
        self.cost = 0   # For algorithms that use cost

    def __lt__(self, other):
        return self.f < other.f  # Priority queue compares based on f-cost

class AStarPlanner(PathPlanner):
    """
    A* path planning algorithm.
    """
    def __init__(self, heuristic=None, allow_diagonal=False):
        # Movement directions
        self.directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # Up, Down, Left, Right
        if allow_diagonal:
            self.directions += [(-1, -1), (-1, 1), (1, -1), (1, 1)]

        if heuristic is None:
            self.heuristic = lambda current, goal: abs(current[0] - goal[0]) + abs(current[1] - goal[1])
        else:
            self.heuristic = heuristic

    def find_path(self, start, end, occupancy_map):
        """
        Finds the shortest path from start to end using A* algorithm.
        """
        # Priority queue for open list
        open_list = []
        heapq.heappush(open_list, Node(start, h=self.heuristic(start, end)))

        # Set for closed list
        closed_set = set()

        # Keep track of visited nodes
        visited = {}

        while open_list:
            # Get the node with the lowest f-cost
            current_node = heapq.heappop(open_list)
            current_pos = current_node.position

            # Add to closed list
            closed_set.add(current_pos)

            # Check if goal is reached
            if current_pos == end:
                return self.reconstruct_path(current_node)

            # Explore neighbors
            for direction in self.directions:
                neighbor_pos = (current_pos[0] + direction[0], current_pos[1] + direction[1])

                # Check bounds
                if not (0 <= neighbor_pos[0] < len(occupancy_map) and 0 <= neighbor_pos[1] < len(occupancy_map[0])):
                    continue

                # Skip if neighbor is a wall or already in closed set
                if occupancy_map[neighbor_pos[0]][neighbor_pos[1]] == 1 or neighbor_pos in closed_set:
                    continue

                # Calculate costs
                g_cost = current_node.g + 1
                h_cost = self.heuristic(neighbor_pos, end)
                f_cost = g_cost + h_cost

                # Check if the neighbor has been visited with a lower f-cost
                if neighbor_pos in visited and visited[neighbor_pos].f <= f_cost:
                    continue

                # Add neighbor to open list
                neighbor_node = Node(neighbor_pos, g=g_cost, h=h_cost, parent=current_node)
                heapq.heappush(open_list, neighbor_node)
                visited[neighbor_pos] = neighbor_node

        # No path found
        return []

    def reconstruct_path(self, node):
        """
        Reconstructs the path from end to start using the parent references.
        """
        path = []
        while node:
            path.append(node.position)
            node = node.parent
        return path[::-1]  # Reverse the path
    
class RRTPlanner(PathPlanner):
    """
    Rapidly-exploring Random Tree (RRT) path planning algorithm.
    """
    def __init__(self, max_iterations=20000, max_step_size=10, goal_tolerance=2):
        """
        Initializes the RRT planner.
        """
        self.max_iterations = max_iterations
        self.max_step_size = max_step_size
        self.goal_tolerance = goal_tolerance
        self.nodes = []  # Store nodes for visualization

    def find_path(self, start, goal, occupancy_map):
        """
        Finds a path from start to goal using the RRT algorithm.
        """
        height, width = len(occupancy_map), len(occupancy_map[0])

        # Initialize the tree with the start node
        root_node = Node(start)
        self.nodes = [root_node]

        for iteration in range(self.max_iterations):
            if iteration == self.max_iterations - 1:
                print("Max iterations reached")
                return []
            
            # Generate a random point in the map
            rand_point = (random.randint(0, height - 1), random.randint(0, width - 1))

            # Find the nearest node in the tree
            nearest_node = self.find_nearest_node(rand_point, self.nodes)

            # Create a new node in the direction of rand_point, limited by max_step_size
            new_node_position = self.steer(nearest_node.position, rand_point, self.max_step_size)

            # Check for collision between nearest_node and new_node_position
            if not self.collision_detected(nearest_node.position, new_node_position, occupancy_map):
                # Create new node and add to tree
                new_node = Node(new_node_position, parent=nearest_node)
                self.nodes.append(new_node)

                # Check if goal is within goal_tolerance
                if self.calc_distance(new_node.position, goal) <= self.goal_tolerance:
                    # Goal reached, construct path
                    goal_node = Node(goal, parent=new_node)
                    return self.construct_path(goal_node)

        # Failed to find a path
        return []

    def find_nearest_node(self, point, nodes):
        """
        Finds the nearest node in the tree to the given point.
        """
        nearest_dist = float('inf')
        for node in nodes:
            dist = self.calc_distance(point, node.position)
            if dist < nearest_dist:
                nearest_dist = dist
                nearest_node = node
        return nearest_node

    def steer(self, from_pos, to_pos, max_distance):
        """
        Returns a new position moved from from_pos towards to_pos, limited by max_distance.
        """
        new_pos = list(from_pos)
        distance = self.calc_distance(from_pos, to_pos)
        theta = self.calc_angle(from_pos, to_pos)

        # If distance to closes node is less than maximum branch length
        if max_distance > distance:
            max_distance = distance

        new_pos[0] += int(max_distance * cos(theta))
        new_pos[1] += int(max_distance * sin(theta))

        return new_pos

        # from_row, from_col = from_pos
        # to_row, to_col = to_pos

        # dx = to_col - from_col
        # dy = to_row - from_row
        # distance = sqrt(dx**2 + dy**2)

        # if distance == 0:
        #     return from_pos

        # ratio = min(max_distance / distance, 1.0)
        # new_row = int(from_row + dy * ratio)
        # new_col = int(from_col + dx * ratio)
        # return (new_row, new_col)

    def collision_detected(self, from_pos, to_pos, occupancy_map):
        """
        Checks if a path from from_pos to to_pos collides with any obstacles.
        """
        width = len(occupancy_map[0])
        line_cells = self.bresenham_line(from_pos, to_pos)
        for cell in line_cells:
            row, col = cell
            if not (0 <= row < len(occupancy_map) and 0 <= col < len(occupancy_map[0])):
                return True  # Out of bounds is considered a collision
            if occupancy_map[row][col] == 1:
                return True
        # No collision
        return False
    
    def bresenham_line(self, start, end):
        """
        Generates the points along a line between start and end using Bresenham's algorithm.
        """
        x0, y0 = start[1], start[0]
        x1, y1 = end[1], end[0]

        points = []

        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy

        while True:
            points.append((y0, x0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                if x0 == x1:
                    break
                err += dy
                x0 += sx
            if e2 <= dx:
                if y0 == y1:
                    break
                err += dx
                y0 += sy

        return points

    def calc_distance(self, p1, p2):
        """
        Calculates the Euclidean distance between two points.
        """
        return euclidean_distance(p1, p2)
    
    def calc_angle(self, p1, p2):
        """
        Calculates the angle between two points.
        """
        dx = p2[1] - p1[1]
        dy = p2[0] - p1[0]
        return atan2(dy, dx)

    def construct_path(self, goal_node):
        """
        Constructs the path from start to goal by traversing parent links.
        """
        path = []
        node = goal_node
        while node.parent:
            path.append(node.position)
            node = node.parent
        path.append(node.position)
        path.reverse()
        return path
