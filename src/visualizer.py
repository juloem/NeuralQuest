import matplotlib.pyplot as plt

class MazeVisualizer:
    """
    A class used to visualize a maze and its occupancy map.
    Methods
    -------
    visualize_maze(maze)
        Static method that visualizes the maze structure using matplotlib.
    visualize_occupancy_map(occ_map)
        Static method that visualizes the occupancy map using matplotlib.
    """
    @staticmethod
    def visualize_maze(maze):
        fig, ax = plt.subplots()
        ax.set_aspect("equal")
        ax.set_xlim(0, maze.cols)
        ax.set_ylim(0, maze.rows)
        ax.invert_yaxis()

        for r in range(maze.rows):
            for c in range(maze.cols):
                cell = maze.grid[r][c]
                x, y = c, r
                if cell.has_wall('N'):
                    ax.plot([x, x + 1], [y, y], color='black')
                if cell.has_wall('W'):
                    ax.plot([x, x], [y, y + 1], color='black')
                if r == maze.rows - 1 and cell.has_wall('S'):
                    ax.plot([x, x + 1], [y + 1, y + 1], color='black')
                if c == maze.cols - 1 and cell.has_wall('E'):
                    ax.plot([x + 1, x + 1], [y, y + 1], color='black')

        plt.title("Maze")

    @staticmethod
    def visualize_occupancy_map(occ_map, start=None, end=None):
        fig, ax = plt.subplots()
        ax.imshow(occ_map, cmap="binary", origin="upper", vmin=0, vmax=1)
        ax.axis("off")

        # Mark start and end points
        if start:
            ax.scatter(start[1], start[0], marker='o', color='green', s=50)
        if end:
            ax.scatter(end[1], end[0], marker='*', color='orange', s=100)
        
        plt.title("Occupancy Map")

    @staticmethod
    def visualize_path(occ_map, path, start, end):
        """
        Visualizes the occupancy map with the calculated path.
        """
        fig, ax = plt.subplots()
        ax.imshow(occ_map, cmap="binary", origin="upper", vmin=0, vmax=1)
        ax.axis("off")

        # Add start point
        ax.scatter(start[1], start[0], c='green', marker='o', label='Start', s=100)

        # Add end point
        ax.scatter(end[1], end[0], c='orange', marker='*', label='End', s=150)

        # Plot the path
        if path:
            path_x = [pos[1] for pos in path]
            path_y = [pos[0] for pos in path]
            ax.plot(path_x, path_y, color='red', linewidth=3, label='Path')

        # Add legend
        ax.legend(loc='upper left', bbox_to_anchor=(1.05, 1), borderaxespad=0.)

        plt.title("Path Planning Visualization")

    @staticmethod
    def visualize_rrt(occ_map, nodes, path, start, goal):
        """
        Visualizes the occupancy map, RRT tree, and the path found.
        """
        fig, ax = plt.subplots()
        ax.imshow(occ_map, cmap="binary", origin="upper", vmin=0, vmax=1)
        ax.axis("off")

        # Plot the RRT tree
        for node in nodes:
            if node.parent is not None:
                n_pos = (node.position[1], node.position[0])
                p_pos = (node.parent.position[1], node.parent.position[0])
                ax.plot([n_pos[0], p_pos[0]], [n_pos[1], p_pos[1]], color='gray', linewidth=0.5)

        # Plot the path
        if path:
            path_x = [pos[1] for pos in path]
            path_y = [pos[0] for pos in path]
            ax.plot(path_x, path_y, color='blue', linewidth=2, label='Path')

        # Plot start and goal
        ax.scatter(start[1], start[0], c='green', marker='o', label='Start', s=100)
        ax.scatter(goal[1], goal[0], c='red', marker='*', label='Goal', s=150)

        # Add legend
        ax.legend(loc='upper left', bbox_to_anchor=(1.05, 1), borderaxespad=0.)

        plt.title("RRT Path Planning Visualization")
        plt.show()

    @staticmethod
    def show():
        plt.show()
