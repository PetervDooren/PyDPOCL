import numpy as np
from shapely.geometry import Point
from heapq import heappush, heappop

from typing import List, Set, Tuple
from shapely import Point, Polygon, LineString

# visualization
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPolygon


def create_visibility_graph(polygon: Polygon):
    # Collect all vertices from exterior and interior rings
    exterior_coords = list(polygon.exterior.coords)[:-1]
    interior_coords = [list(interior.coords)[:-1] for interior in polygon.interiors]
    all_coords = exterior_coords + [pt for ring in interior_coords for pt in ring]

    visibility_graph = {i: [] for i in range(len(all_coords))}

    for i, p1 in enumerate(all_coords):
        for j, p2 in enumerate(all_coords):
            if j <= i:
                continue
            segment = LineString([p1, p2])

            # Check if segment is inside polygon and doesn't cross any holes
            if polygon.covers(segment):
            #if polygon.contains(segment) and all(not segment.crosses(hole) for hole in hole_boundaries):
                visibility_graph[i].append(j)
                visibility_graph[j].append(i)

    return visibility_graph, all_coords

def add_point_to_visibility_graph(points, visibility_graph, polygon, new_point):
    i_new_point = len(points)
    points.append(new_point)
    visibility_graph[i_new_point] = []

    for i, p in enumerate(points):
        if i == i_new_point:
            continue
        segment = LineString([new_point, p])

        # Check if segment is inside polygon and doesn't cross any holes
        if polygon.covers(segment):
            visibility_graph[i].append(i_new_point)
            visibility_graph[i_new_point].append(i)
    return visibility_graph, points

def find_path_visibility_graph(start: Point, goal: Point, free_space: Polygon):
    visibility_graph, points = create_visibility_graph(free_space)
    start_index = len(points)
    visibility_graph, points = add_point_to_visibility_graph(points, visibility_graph, free_space, (start.x, start.y))
    goal_index = len(points)
    visibility_graph, points = add_point_to_visibility_graph(points, visibility_graph, free_space, (goal.x, goal.y))
    
    #helper_visualize_visibility_graph(free_space, points, visibility_graph, start, goal)
    
    # perform A* on the visibility graph
    def distance(a, b):
        return np.hypot(a[0] - b[0], a[1] - b[1])
     # A* search
    open_set = []
    heappush(open_set, (distance(points[start_index], points[goal_index]), 0, start_index, [start_index]))
    closed_set = set()
    cost_to_come = {start_index: 0}

    while open_set:
        _, cost, current, path = heappop(open_set)
        #helper_visualize_visibility_graph(free_space, points, {}, start, goal, open_set, closed_set, path)
        if current == goal_index:
            # Return path as list of (x, y) tuples
            return LineString([points[i] for i in path])
        if current in closed_set:
            continue
        closed_set.add(current)
        for neighbor in visibility_graph[current]:
            cost_neighbor = cost + distance(points[current], points[neighbor])
            if neighbor not in cost_to_come or cost_neighbor < cost_to_come[neighbor]:
                cost_to_come[neighbor] = cost_neighbor
                heappush(open_set, (cost_neighbor + distance(points[neighbor], points[goal_index]), cost_neighbor, neighbor, path + [neighbor]))

    # No path found
    return None

def helper_visualize_visibility_graph(free_space: Polygon, points: List[Tuple], visibility_graph: dict = {}, start = None, goal = None, open_set = [], closed_set = [], current_path=[]):
    plt.figure(2)
    fig = plt.gcf()  # get current figure
    fig.clf()
    ax = fig.gca()   # get current axes
    
    coords = list(free_space.exterior.coords)
    poly = MplPolygon(coords, closed=True, facecolor='green', edgecolor='green', alpha=0.5, fill=True)
    ax.add_patch(poly)
    for hole in free_space.interiors:
        hole_coords = list(hole.coords)
        hole_poly = MplPolygon(hole_coords, closed=True, facecolor='white', edgecolor='green', alpha=1, fill=True)
        ax.add_patch(hole_poly)

    # Plot vertices
    for i, (x, y) in enumerate(points):
        ax.plot(x, y, 'o', color='black')

    # Plot visibility edges
    for i, visible in visibility_graph.items():
        for j in visible:
            p1 = points[i]
            p2 = points[j]
            line = LineString([p1, p2])
            ax.plot(*line.xy, color='black', alpha=1)

    def plot_path(ax, path, color='black'):
        if len(path) < 2:
            return
        xp = [points[p][0] for p in path]
        yp = [points[p][1] for p in path]
        ax.plot(xp, yp, color=color)

    for point in closed_set:
        ax.plot(points[point][0], points[point][1], marker='o', color='blue')
    for _, _, point, path in open_set:
        ax.plot(points[point][0], points[point][1], marker='o', color='magenta')
        plot_path(ax, path)
    # highlight current path
    plot_path(ax, current_path, color='cyan')
    # plot start and goal nodes
    if start is not None:
        ax.plot(start.x, start.y, 'o', color='red')
    if goal is not None:
        ax.plot(goal.x, goal.y, 'o', color='red')

    ax.set_aspect('equal')
    plt.title("Visibility Graph")
    plt.show(block=False)
    plt.pause(0.001)



def find_path_Astar(start: Point, goal: Point, free_space: Polygon):
    """Apply a discretised A* planner to find a path from start to goal. All points in the path lie in the free space.

    Args:
        start (Point): _description_
        goal (Point): _description_
        free_space (Polygon): _description_
    
    Returns: Line
    """
    # Discretize the eroded polygon into a grid of points
    # Parameters for grid resolution
    minx, miny, maxx, maxy = free_space.bounds
    grid_res = 0.1  # meters (adjust as needed)
    grid_digits = 4 # rounding accuracy
    x_coords = np.arange(minx, maxx, grid_res)
    y_coords = np.arange(miny, maxy, grid_res)
    x_coords = [round(x, ndigits=grid_digits) for x in x_coords]
    y_coords = [round(y, ndigits=grid_digits) for y in y_coords]

    # Build set of valid points inside the free_space polygon
    valid_points = set()
    for x in x_coords:
        for y in y_coords:
            pt = Point(x, y)
            if free_space.contains(pt):
                valid_points.add((x, y))
    if len(valid_points) == 0:
        # Free space is very small
        return None

    # Helper: get neighbors (4-connected grid)
    def get_neighbors(node):
        x, y = node
        neighbors = [
            (round(x + grid_res, ndigits=grid_digits), y),
            (round(x- grid_res, ndigits=grid_digits), y),
            (x, round(y + grid_res, ndigits=grid_digits)),
            (x, round(y - grid_res, ndigits=grid_digits))
        ]
        return [n for n in neighbors if n in valid_points]

    # Helper: heuristic (Euclidean distance)
    def heuristic(a, b):
        return np.hypot(a[0] - b[0], a[1] - b[1])

    # Find closest grid point to start and goal
    def closest_grid_point(pt):
        return min(valid_points, key=lambda p: np.hypot(p[0] - pt.x, p[1] - pt.y))

    start_node = closest_grid_point(start)
    goal_node = closest_grid_point(goal)

    # A* search
    open_set = []
    heappush(open_set, (heuristic(start_node, goal_node), 0, start_node, [start_node]))
    closed_set = set()
    g_score = {start_node: 0}

    while open_set:
        #helper_visualize_Astart(free_space, valid_points, start_node, goal_node, open_set, closed_set)
        _, cost, current, path = heappop(open_set)
        if current == goal_node:
            # Return path as list of (x, y) tuples
            if len(path) < 2:
                return LineString([(start.x, start.y), (goal.x, goal.y)])
            return LineString(path)
        if current in closed_set:
            continue
        closed_set.add(current)
        for neighbor in get_neighbors(current):
            tentative_g = cost + heuristic(current, neighbor)
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                g_score[neighbor] = tentative_g
                heappush(open_set, (tentative_g + heuristic(neighbor, goal_node), tentative_g, neighbor, path + [neighbor]))

    # No path found
    return None

def helper_visualize_Astart(eroded: Polygon = None,
                            points: Set[Tuple[float, float]] = [],
                            start_point: Tuple[float, float] = None,
                            goal_point: Tuple[float, float] = None,
                            open_set: List[Tuple] = [],
                            closed_set: List[Tuple[float, float]] = []):
    plt.figure(2)
    fig = plt.gcf()  # get current figure (figure 1)
    fig.clf()
    ax = fig.gca()   # get current axes

    def plot_area(ax, area: Polygon, color='lightgray', edgecolor='black', alpha=0.5, fill = True, label=None):
        coords = list(area.exterior.coords)
        poly = MplPolygon(coords, closed=True, facecolor=color, edgecolor=edgecolor, alpha=alpha, fill=fill, label=label)
        ax.add_patch(poly)
        for hole in area.interiors:
            hole_coords = list(hole.coords)
            hole_poly = MplPolygon(hole_coords, closed=True, facecolor='white', edgecolor=edgecolor, alpha=1, fill=fill, label=label)
            ax.add_patch(hole_poly)
        if label:
            # Place label at centroid
            xs, ys = zip(*coords)
            centroid = (sum(xs)/len(xs), sum(ys)/len(ys))
            ax.text(centroid[0], centroid[1], label, ha='center', va='center', fontsize=8)
    if eroded is not None:
        plot_area(ax, eroded, color = 'green', label='eroded')

    def plot_point(ax, point, color='black'):
        ax.plot(point[0], point[1], marker='o', color=color)  # single point
    
    def plot_path(ax, path, color='black'):
        if len(path) < 2:
            return
        xp = [p[0] for p in path]
        yp = [p[1] for p in path]
        ax.plot(xp, yp, color=color)

    for point in points:
        plot_point(ax, point)
    for _, _, point, path in open_set:
        plot_point(ax, point, 'magenta')
        plot_path(ax, path)
    for point in closed_set:
        plot_point(ax, point, 'blue')
    if start_point is not None:
        plot_point(ax, start_point, color='green')
    if goal_point is not None:
        plot_point(ax, goal_point, color='red')
    plt.show(block=False)

def find_path(start: Point, goal: Point, free_space: Polygon):
    return find_path_visibility_graph(start, goal, free_space)
    #return find_path_Astar(start, goal, free_space)