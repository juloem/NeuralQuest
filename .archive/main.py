import logging
import matplotlib.pyplot as plt
import numpy as np

from src.utils.logger import configure_root_logger, setup_logger
from src.maze import Maze
from src.bps_encode import BPS_Encoder
from src.path_planner import PathPlanner


def main():
    # Initialize logging
    configure_root_logger(
        log_file='.log/NeuralQuest.log',
        level=logging.DEBUG,
        console_logging=False
    )
    logger = setup_logger(__name__)
    logger.info("==== Begin NeuralQuest ====")
    
    WIDTH, HEIGHT = 10, 10

    # Generate a maze
    maze = Maze(WIDTH, HEIGHT)
    maze.generate_maze_aldous_broder()
    maze.set_start_end()

    # Convert maze to environment points
    environment_points = maze.to_points()
    environment_points = np.array(environment_points)

    # BPS encoding
    bps_encoder = BPS_Encoder(environment_points, num_basis_points=16)
    bps_encoder.compute_encoding(feature_type='distance', basis_point_method='grid')

    # Initialize PathPlanner
    planner = PathPlanner(maze)

    # Perform A* search
    path = planner.a_star_search()
    if path is None:
        print(f"No path found.")
    else:
        print(f"Path found!")

    # Perform RRT search
    # path = planner.rrt_search()
    # if path is None:
    #     print(f"No path found.")
    # else:
    #     print(f"Path found!")

    # Visualize the maze with environment points and BPS encoding
    maze.draw_maze(show_environment_points=True,
                   environment_points=environment_points,
                   basis_points=bps_encoder.basis_points,
                   encoding=bps_encoder.encoding,
                   path=path,
                   path_label="Path")

    logger.info("==== End NeuralQuest ====")

if __name__ == "__main__":
    main()
