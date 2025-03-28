import numpy as np
import matplotlib.pyplot as plt
import random
from scipy.spatial import cKDTree

# Cell class definition
class Cell:
    def __init__(self):
        self.walls = [True, True, True, True]  # [Top, Right, Bottom, Left]
        self.visited = False

# Create the grid
def create_grid(width, height):
    grid = [[Cell() for _ in range(width)] for _ in range(height)]
    return grid

# Aldous-Broder maze generation
def aldous_broder_maze(grid, width, height):
    total_cells = width * height
    visited_cells = 1

    current_row = random.randint(0, height - 1)
    current_col = random.randint(0, width - 1)
    grid[current_row][current_col].visited = True

    while visited_cells < total_cells:
        neighbors = []
        moves = [(-1, 0, 0, 2),  # Up
                 (0, 1, 1, 3),   # Right
                 (1, 0, 2, 0),   # Down
                 (0, -1, 3, 1)]  # Left

        for move in moves:
            nr, nc = current_row + move[0], current_col + move[1]
            if 0 <= nr < height and 0 <= nc < width:
                neighbors.append((nr, nc, move[2], move[3]))

        nr, nc, wall_current, wall_neighbor = random.choice(neighbors)
        neighbor_cell = grid[nr][nc]

        if not neighbor_cell.visited:
            grid[current_row][current_col].walls[wall_current] = False
            neighbor_cell.walls[wall_neighbor] = False
            neighbor_cell.visited = True
            visited_cells += 1

        current_row, current_col = nr, nc

# Convert maze to points
def maze_to_points(grid, width, height):
    points = []
    for row in range(height):
        for col in range(width):
            cell = grid[row][col]
            x, y = col, height - row - 1

            if cell.walls[0]:
                points.append([x, y + 1])
                points.append([x + 1, y + 1])
            if cell.walls[1]:
                points.append([x + 1, y])
                points.append([x + 1, y + 1])
            if cell.walls[2]:
                points.append([x, y])
                points.append([x + 1, y])
            if cell.walls[3]:
                points.append([x, y])
                points.append([x, y + 1])
    return np.array(points)

# BPS encoding function
def compute_bps_encoding(environment_points, basis_points, feature_type='distance', **kwargs):
    tree = cKDTree(environment_points)
    distances, _ = tree.query(basis_points, k=1)
    if feature_type == 'distance':
        max_distance = kwargs.get('max_distance', distances.max())
        encoding = distances / max_distance
    elif feature_type == 'occupancy':
        threshold = kwargs.get('threshold', 0.1)
        encoding = (distances < threshold).astype(float)
    else:
        raise ValueError("Unsupported feature_type")
    return encoding

# Visualization functions
def draw_maze(grid, width, height):
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.set_aspect('equal')
    plt.axis('off')

    for row in range(height):
        for col in range(width):
            cell = grid[row][col]
            x, y = col, height - row - 1

            if cell.walls[0]:
                ax.plot([x, x + 1], [y + 1, y + 1], color='black')
            if cell.walls[1]:
                ax.plot([x + 1, x + 1], [y, y + 1], color='black')
            if cell.walls[2]:
                ax.plot([x, x + 1], [y, y], color='black')
            if cell.walls[3]:
                ax.plot([x, x], [y, y + 1], color='black')

    plt.show()

# Main code
if __name__ == "__main__":
    width, height = 20, 20
    grid = create_grid(width, height)
    aldous_broder_maze(grid, width, height)
    draw_maze(grid, width, height)

    environment_points = maze_to_points(grid, width, height)

    # Generate basis points
    N = 256
    min_coords = environment_points.min(axis=0)
    max_coords = environment_points.max(axis=0)
    basis_points = np.random.rand(N, 2) * (max_coords - min_coords) + min_coords

    # Compute BPS encoding
    bps_encoding = compute_bps_encoding(environment_points, basis_points, feature_type='distance')

    # Visualize BPS encoding
    plt.scatter(basis_points[:, 0], basis_points[:, 1], c=bps_encoding, cmap='viridis')
    plt.colorbar(label='Encoding Value')
    plt.title("BPS Encoding Visualization")
    plt.show()
