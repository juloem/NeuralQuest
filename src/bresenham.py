"""
Implementation of Bresenham's line drawing algorithm
See en.wikipedia.org/wiki/Bresenham's_line_algorithm
Source: The bresenham module https://github.com/encukou/bresenham
Copyright © 2016 Petr Viktorin
encukou/bresenham is licensed under the MIT License
Usage:
For example, the coordinates of a line from (-1, -4) to (3, 2), are:
list(bresenham(-1, -4, 3, 2))
[(-1, -4), (0, -3), (0, -2), (1, -1), (2, 0), (2, 1), (3, 2)]
"""


def bresenham(start, end):
    """
    Generate points in a line from start to end using Bresenham's algorithm.
    Parameters:
    start (tuple): The starting point of the line (y, x).
    end (tuple): The ending point of the line (y, x).
    Yields:
    tuple: The next point (y, x) in the line from start to end.
    """

    x0, y0 = start[1], start[0]
    x1, y1 = end[1], end[0]

    dx = x1 - x0
    dy = y1 - y0

    xsign = 1 if dx > 0 else -1
    ysign = 1 if dy > 0 else -1

    dx = abs(dx)
    dy = abs(dy)

    if dx > dy:
        xx, xy, yx, yy = xsign, 0, 0, ysign
    else:
        dx, dy = dy, dx
        xx, xy, yx, yy = 0, ysign, xsign, 0

    D = 2*dy - dx
    y = 0

    for x in range(dx + 1):
        yield x0 + x*xx + y*yx, y0 + x*xy + y*yy
        if D >= 0:
            y += 1
            D -= 2*dx
        D += 2*dy
