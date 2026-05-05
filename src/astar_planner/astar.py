import heapq
from typing import List, Tuple, Dict

Grid = List[List[int]]  # 0 = free, 1 = obstacle
Point = Tuple[int, int]


def heuristic(a: Point, b: Point) -> float:
    # Manhattan distance (good for grid maps)
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def get_neighbors(node: Point, grid: Grid) -> List[Point]:
    x, y = node
    neighbors = []

    directions = [(1,0), (-1,0), (0,1), (0,-1)]

    for dx, dy in directions:
        nx, ny = x + dx, y + dy
        if 0 <= nx < len(grid) and 0 <= ny < len(grid[0]):
            if grid[nx][ny] == 0:
                neighbors.append((nx, ny))

    return neighbors


def reconstruct_path(came_from: Dict[Point, Point], current: Point):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


def astar(grid: Grid, start: Point, goal: Point):
    open_set = []
    heapq.heappush(open_set, (0, start))

    came_from = {}
    g_score = {start: 0}

    f_score = {start: heuristic(start, goal)}

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            return reconstruct_path(came_from, current)

        for neighbor in get_neighbors(current, grid):
            tentative_g = g_score[current] + 1

            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f, neighbor))

    return None  # no path found