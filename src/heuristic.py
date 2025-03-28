def manhattan_distance(p1, p2):
    """
    Manhattan distance heuristic.
    """
    return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])

def euclidean_distance(p1, p2):
    """
    Euclidean distance heuristic.
    """
    return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5

def chebyshev_distance(p1, p2):
    """
    Chebyshev distance heuristic.
    """
    return max(abs(p1[0] - p2[0]), abs(p1[1] - p2[1]))