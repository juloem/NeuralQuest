import numpy as np
import random
import math
import matplotlib.pyplot as plt

class RRTMazePathfinderViz:
    def __init__(self, maze, max_iterations=1000, step_size=0.5, goal_bias=0.1, visualize=False):
        """
        Initialize RRT Pathfinder with visualization support.

        Args:
            maze (Maze): The maze object to navigate
            max_iterations (int): Maximum number of iterations for RRT
            step_size (float): Maximum distance between nodes
            goal_bias (float): Probability of sampling the goal instead of random point
            visualize (bool): Enable step-by-step visualization
        """
        self.maze = maze
        self.max_iterations = max_iterations
        self.step_size = step_size
        self.goal_bias = goal_bias
        self.visualize = visualize
        
        # Visualization setup
        self.fig, self.ax = None, None
        self.node_scatter = None
        self.path_lines = []
        
        # Validate start and end points
        if not maze.start or not maze.end:
            raise ValueError("Maze must have start and end points set. Call set_start_end() first.")
        
        self.start = (maze.start.col + 0.5, maze.height - maze.start.row - 0.5)
        self.goal = (maze.end.col + 0.5, maze.height - maze.end.row - 0.5)
        
        # Tree storage
        self.nodes = [self.start]
        self.parent = {self.start: None}
        
        # Visualization preparation
        if self.visualize:
            self._setup_visualization()
    
    def _setup_visualization(self):
        """
        Set up the visualization environment for RRT exploration.
        """
        plt.ion()  # Turn on interactive mode
        self.fig, self.ax = plt.subplots(figsize=(12, 10))
        
        # Draw the maze background
        self.maze.draw_maze(ax=self.ax)
        
        # Plot start and goal points
        self.ax.scatter(self.start[0], self.start[1], color='green', s=100, marker='o', label='Start')
        self.ax.scatter(self.goal[0], self.goal[1], color='red', s=100, marker='x', label='Goal')
        
        # Initialize node scatter plot
        self.node_scatter = self.ax.scatter([], [], color='blue', s=10, alpha=0.5, label='RRT Nodes')
        
        self.ax.set_title('RRT Maze Exploration')
        self.ax.legend()
        plt.pause(0.1)
    
    def _update_visualization(self, new_node, nearest_node):
        """
        Update the visualization with new nodes and connections.
        
        Args:
            new_node (tuple): Newly added node
            nearest_node (tuple): Nearest node it connects to
        """
        if not self.visualize:
            return
        
        # Update nodes scatter plot
        nodes_array = np.array(self.nodes)
        self.node_scatter.set_offsets(nodes_array)
        
        # Draw connection line
        self.ax.plot([nearest_node[0], new_node[0]], 
                     [nearest_node[1], new_node[1]], 
                     color='blue', linewidth=1, alpha=0.3)
        
        plt.pause(0.01)
    
    def _finalize_visualization(self, path=None):
        """
        Finalize the visualization, optionally highlighting the path.
        
        Args:
            path (list, optional): Solution path to highlight
        """
        if not self.visualize:
            return
        
        plt.ioff()  # Turn off interactive mode
        
        if path:
            # Convert path points to numpy array for plotting
            path_points = np.array(path)
            self.ax.plot(path_points[:, 0], path_points[:, 1], 
                         color='red', linewidth=3, label='Solution Path')
            self.ax.legend()
        
        plt.show()
    
    def distance(self, point1, point2):
        """Calculate Euclidean distance between two points."""
        return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)
    
    def is_collision_free(self, point1, point2):
        """
        Check if the path between two points is collision-free.
        
        Args:
            point1 (tuple): Starting point (x, y)
            point2 (tuple): Ending point (x, y)
        
        Returns:
            bool: True if path is collision-free, False otherwise
        """
        # Interpolate points along the line
        num_checks = max(int(self.distance(point1, point2) / 0.1), 2)
        for i in range(1, num_checks):
            t = i / (num_checks - 1)
            x = point1[0] + t * (point2[0] - point1[0])
            y = point1[1] + t * (point2[1] - point1[1])
            
            # Convert to cell coordinates
            col = int(x)
            row = self.maze.height - 1 - int(y)
            
            # Check if point is within maze bounds
            if 0 <= row < self.maze.height and 0 <= col < self.maze.width:
                cell = self.maze.get_cell(row, col)
                
                # Check adjacent cells for walls
                neighbors = self.maze.get_neighbors(cell)
                for neighbor, direction in neighbors:
                    # If there's a wall between current cell and neighbor, path is blocked
                    if cell.walls[direction]:
                        return False
            else:
                return False
        
        return True
    
    def find_nearest_node(self, point):
        """
        Find the nearest node in the tree to a given point.
        
        Args:
            point (tuple): Point to find nearest node to
        
        Returns:
            tuple: Nearest node in the tree
        """
        return min(self.nodes, key=lambda node: self.distance(node, point))
    
    def steer(self, from_point, to_point):
        """
        Steer from one point towards another, limiting distance.
        
        Args:
            from_point (tuple): Starting point
            to_point (tuple): Target point
        
        Returns:
            tuple: New point in the direction of to_point, limited by step size
        """
        dist = self.distance(from_point, to_point)
        if dist <= self.step_size:
            return to_point
        
        # Scale the direction vector
        theta = math.atan2(to_point[1] - from_point[1], to_point[0] - from_point[0])
        return (
            from_point[0] + self.step_size * math.cos(theta),
            from_point[1] + self.step_size * math.sin(theta)
        )
    
    def solve(self):
        """
        Solve the maze using RRT algorithm with optional visualization.
        
        Returns:
            list or None: Path from start to goal, or None if no path found
        """
        for iteration in range(self.max_iterations):
            # Decide whether to sample goal or random point
            if random.random() < self.goal_bias:
                sample = self.goal
            else:
                # Sample random point within maze bounds
                x = random.uniform(0, self.maze.width)
                y = random.uniform(0, self.maze.height)
                sample = (x, y)
            
            # Find nearest node in tree
            nearest = self.find_nearest_node(sample)
            
            # Steer towards the sample point
            new_point = self.steer(nearest, sample)
            
            # Check for collision-free path
            if self.is_collision_free(nearest, new_point):
                # Add new point to tree
                self.nodes.append(new_point)
                self.parent[new_point] = nearest
                
                # Update visualization
                self._update_visualization(new_point, nearest)
                
                # Check if goal reached
                if self.distance(new_point, self.goal) <= self.step_size:
                    # Reconstruct path
                    path = self._get_path(new_point)
                    
                    # Finalize visualization
                    self._finalize_visualization(path)
                    
                    return path
        
        # Finalize visualization if no path found
        self._finalize_visualization()
        return None
    
    def _get_path(self, last_point):
        """
        Reconstruct the path from start to goal.
        
        Args:
            last_point (tuple): Final point reached
        
        Returns:
            list: Path of points from start to goal
        """
        path = [last_point]
        current = last_point
        
        while current != self.start:
            current = self.parent[current]
            path.append(current)
        
        path.reverse()
        
        return path

def find_maze_path_viz(maze, max_iterations=1000, step_size=0.5, goal_bias=0.1):
    """
    Convenience function to find a path in a maze using RRT with visualization.
    
    Args:
        maze (Maze): Maze object to solve
        max_iterations (int): Maximum RRT iterations
        step_size (float): Step size for node expansion
        goal_bias (float): Probability of sampling goal
    
    Returns:
        list or None: Path of points from start to goal, or None if no path found
    """
    # Ensure start and end points are set
    if not maze.start or not maze.end:
        maze.set_start_end()
    
    # Create RRT solver with visualization
    rrt_solver = RRTMazePathfinderViz(
        maze, 
        max_iterations=max_iterations, 
        step_size=step_size, 
        goal_bias=goal_bias, 
        visualize=True
    )
    
    # Solve the maze
    path = rrt_solver.solve()
    
    return path