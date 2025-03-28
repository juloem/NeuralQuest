import time

from src.maze import Maze
from src.maze_generators import RecursiveBacktrackerGenerator
from src.visualizer import MazeVisualizer
from src.path_planner import AStarPlanner as AStar
from src.path_planner import RRTPlanner as RRT
from src.heuristic import manhattan_distance, euclidean_distance, chebyshev_distance

def main():
    ROWS, COLS = 5, 5
    CORRIDOR_SIZE, WALL_SIZE = 10, 1

    gen = RecursiveBacktrackerGenerator()

    # Create the maze and generate it
    maze = Maze(ROWS, COLS)
    gen.generate(maze)

    # Convert to occupancy map
    occupancy_map = maze.to_occupancy_map(corridor_size=CORRIDOR_SIZE, wall_size=WALL_SIZE)

    # Set start and end points
    maze.set_random_start_and_end(occupancy_map)

    # Find shortest path
    heuristic = euclidean_distance
    #planner = AStar(heuristic=heuristic)
    planner = RRT()
    start_time = time.time()
    path = maze.find_path(planner, occupancy_map)
    end_time = time.time()
    print(f"A* Path found in {end_time - start_time:.5f} seconds")

    # Debug path
    print("Path: ", path)

    # Visualize
    #MazeVisualizer.visualize_maze(maze)
    #MazeVisualizer.visualize_occupancy_map(occupancy_map, maze.start, maze.end)
    #MazeVisualizer.visualize_path(occupancy_map, path, maze.start, maze.end)
    MazeVisualizer.visualize_rrt(occupancy_map, planner.nodes, path, maze.start, maze.end)
    MazeVisualizer.show()

if __name__ == "__main__":
    main()