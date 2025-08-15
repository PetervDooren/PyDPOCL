from PyPOCL.GPlan import GPlan
from PyPOCL.Ground_Compiler_Library.Element import Operator, Argument

from typing import List
from shapely import Polygon, MultiPolygon, difference, within, intersects

# visualization
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPolygon


def check_connections_in_plan(plan: GPlan) -> bool:
    """verify that for every move action in the plan. There exists a path from the start to the goal position in that configuration.

    Args:
        plan (GPlan): _description_
    """
    static_objs = []
    # find static objects
    for obj in plan.variableBindings.objects:
        if plan.variableBindings.is_type(obj, 'physical_item'):
            for causal_link in plan.CausalLinkGraph.edges:
                if causal_link.label.source.name == "within":
                    if obj == causal_link.label.source.Args[0]:
                        break
            else:
                static_objs.append(obj)
    
    for step in plan.steps:
        if step.schema != 'movemono':
            continue
        obst_areas = []
        # find all placelocs that could occur simultaneously.
        for causal_link in plan.CausalLinkGraph.edges:
            if causal_link.label.source.name != "within":
                continue
            if plan.OrderingGraph.isPath(step, causal_link.source) or plan.OrderingGraph.isPath(causal_link.sink, step):
                continue
            sourceloc = causal_link.label.source.Args[1]
            if causal_link.source.schema == 'dummy_init': # if the link is grounded in the initial condition, the source area is not a variable.
                sourceloc = causal_link.label.sink.Args[1]
            obst_areas.append(sourceloc)
        
        start_area = plan.variableBindings.geometric_vb.get_assigned_area(step.Args[2])
        goal_area = plan.variableBindings.geometric_vb.get_assigned_area(step.Args[3])
        robot_obj = plan.variableBindings.symbolic_vb.get_const(step.Args[0])
        reach_area = plan.variableBindings.geometric_vb.defined_areas[plan.variableBindings.reach_areas[robot_obj]]
        object_width, object_length = plan.variableBindings.geometric_vb.object_dimensions[plan.variableBindings.symbolic_vb.get_const(step.Args[1])]
        
        # check if the start and goal areas are connected
        is_connected = False
        available_space = reach_area
        for obj in static_objs:
            area_arg = plan.variableBindings.initial_positions[obj]
            area = plan.variableBindings.geometric_vb.defined_areas[area_arg]    
            available_space = difference(available_space, area)
        for loc in obst_areas:
            area = plan.variableBindings.geometric_vb.get_area(loc)
            available_space = difference(available_space, area)
        erosion_dist = 0.5*min(object_width, object_length)
        eroded = available_space.buffer(-erosion_dist)

        #helper_visualize_connection(step, plan, static_objs, obst_areas, eroded)

        if type(eroded) == Polygon: # eroded space is not separated. therefore there is a path from start to goal
            is_connected = True
            find_corridor(step, plan, eroded) # test this method
            continue
        elif type(eroded) == MultiPolygon:
            start_centroid = start_area.centroid # middle of the start area
            goal_centroid = goal_area.centroid # middle of the goal area
            for poly in eroded.geoms:
                if within(start_centroid, poly) and within(goal_centroid, poly):
                    is_connected = True
                    break
            else:
                # no polygon contains both start and goal. Therefore they are separated in the reachable space
                is_connected = False
                find_movable_obstacles(eroded, static_objs, obst_areas, erosion_dist, start_centroid, goal_centroid, plan)
                return False
        else: # eroded has an unexpected type
            print(f"eroded has an unexpected type: {type(eroded)}")
            is_connected = False
            return False
        
        if not is_connected:
            return False
    # no step was found to not have a connection between start and end
    return True

def find_movable_obstacles(eroded, static_objs, area_args, erosion_dist, start, goal, plan: GPlan) -> List[Argument]:
    """ Find which objects can be moved to create a path between start and end

    Args:
        eroded (MultiPolygon): _description_
        static_objs (List[Argument]): list of static objects which are defined by their initial positions.
        area_args (List[Argument]): list of area args which can coexist with the planned step.
        erosion_dist (_type_): _description_
        start (Point): the centroid of the start area
        goal (Point): the centroid of the goal area
        plan (GPlan): the plan in which this operation takes place

    Returns:
        List[List[Arguments]]: Each item in the first list is a collection of objects which,
                               if they are all moved. Provide a path between start and goal. Each collection has the same cost.
    """
    all_areas = [plan.variableBindings.initial_positions[obj] for obj in static_objs]
    all_areas.extend(area_args)
    # Find connections between areas
    poly_args = dict()
    connections = dict()
    for poly in eroded.geoms:
        arg = Argument(name="eroded_polygon")
        poly_args[arg] = poly
        connections[arg] = []
    for arg in all_areas:
        connections[arg] = []

    areas = [plan.variableBindings.geometric_vb.get_area(a) for a in all_areas]
    inflated_areas = [area.buffer(erosion_dist) for area in areas]
    for i in range(len(all_areas)):
        area_arg_i = all_areas[i]
        area_i = inflated_areas[i]
        # check connections to eroded
        for poly_arg in poly_args:
            poly = poly_args[poly_arg]
            if intersects(poly, area_i):
                connections[area_arg_i].append(poly_arg)
                connections[poly_arg].append(area_arg_i)
        # check connections to other objects
        for j in range(i, len(all_areas)):
            area_arg_j = all_areas[j]
            area_j = inflated_areas[j]
            if intersects(area_i, area_j):
                # add connection between the two
                connections[area_arg_i].append(area_arg_j)
                connections[area_arg_j].append(area_arg_i)
    # run dijkstra algorithm over the connections
    cost_dict = dict()
    predecessor_dict = dict()
    for poly_arg in poly_args:
        cost_dict[poly_arg] = 100
        predecessor_dict[poly_arg] = []
    for arg in all_areas:
        cost_dict[arg] = 100
        predecessor_dict[arg] = []
    
    # find start poly
    for poly_arg in poly_args:
        poly = poly_args[poly_arg]
        if within(start, poly):
            start_arg = poly_arg
            break
    else:
        print("start not found in eroded")
        raise
    # find goal poly
    for poly_arg in poly_args:
        poly = poly_args[poly_arg]
        if within(goal, poly):
            goal_arg = poly_arg
            break
    else:
        print("goal not found in eroded")
        raise
    cost_dict[start_arg] = 0
    open_list = [start_arg]
    closed_list = []

    while True:
        open_list.sort(key=lambda x: cost_dict[x], reverse=True)
        node = open_list.pop()
        closed_list.append(node)
        if node == goal_arg:
            break
        node_cost = cost_dict[node]
        for connection in connections[node]:
            if connection not in open_list and connection not in closed_list:
                open_list.append(connection)
            if node_cost + 1 < cost_dict[connection]:
                predecessor_dict[connection] = [node]
                cost_dict[connection]=node_cost + 1
            elif node_cost + 1 == cost_dict[connection]:
                predecessor_dict[connection].append(node)
    #backtrace the objects that should be moved
    def recursive_backtrace(arg, connection_dict):
        if len(connection_dict[arg]) == 0:
            if arg.name == "eroded_polygon":
                return [[]]
            else:
                return [[arg]]
        else:
            arg_list = []
            for connection in connection_dict[arg]:
                subarglist = recursive_backtrace(connection, connection_dict)
                for l in subarglist:
                    if arg.name != "eroded_polygon":
                        l.append(arg)
                    arg_list.append(l)
        return arg_list

    movable_object_sets = recursive_backtrace(goal_arg, predecessor_dict)
    return movable_object_sets

def find_corridor(step: Operator, plan: GPlan, eroded: Polygon):
    """Apply a discretised A* planner to find a path from A to B. All points in the path lie in the eroded Polygon.

    Args:
        step (Operator): _description_
        plan (GPlan): _description_
        eroded (Polygon): _description_
    """
    start_area = plan.variableBindings.geometric_vb.get_assigned_area(step.Args[2])
    goal_area = plan.variableBindings.geometric_vb.get_assigned_area(step.Args[3])
    start_centroid = start_area.centroid # middle of the start area
    goal_centroid = goal_area.centroid # middle of the goal area

    # Discretize the eroded polygon into a grid of points
    import numpy as np
    from shapely.geometry import Point
    from heapq import heappush, heappop

    # Parameters for grid resolution
    minx, miny, maxx, maxy = eroded.bounds
    grid_res = 0.1  # meters (adjust as needed)
    x_coords = np.arange(minx, maxx, grid_res)
    y_coords = np.arange(miny, maxy, grid_res)

    # Build set of valid points inside eroded polygon
    valid_points = set()
    for x in x_coords:
        for y in y_coords:
            pt = Point(x, y)
            if eroded.contains(pt):
                valid_points.add((x, y))

    # Helper: get neighbors (4-connected grid)
    def get_neighbors(node):
        x, y = node
        neighbors = [
            (x + grid_res, y),
            (x - grid_res, y),
            (x, y + grid_res),
            (x, y - grid_res)
        ]
        return [n for n in neighbors if n in valid_points]

    # Helper: heuristic (Euclidean distance)
    def heuristic(a, b):
        return np.hypot(a[0] - b[0], a[1] - b[1])

    # Find closest grid point to start and goal
    def closest_grid_point(pt):
        return min(valid_points, key=lambda p: np.hypot(p[0] - pt.x, p[1] - pt.y))

    start_node = closest_grid_point(start_centroid)
    goal_node = closest_grid_point(goal_centroid)

    # A* search
    open_set = []
    heappush(open_set, (heuristic(start_node, goal_node), 0, start_node, [start_node]))
    closed_set = set()
    g_score = {start_node: 0}

    while open_set:
        #helper_visualize_Astart(eroded, valid_points, start_node, goal_node, open_set, closed_set)
        _, cost, current, path = heappop(open_set)
        if current == goal_node:
            # Return path as list of (x, y) tuples
            return path
        if current in closed_set:
            continue
        closed_set.add(current)
        for neighbor in get_neighbors(current):
            tentative_g = cost + heuristic(current, neighbor)
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                g_score[neighbor] = tentative_g
                heappush(open_set, (tentative_g + heuristic(neighbor, goal_node), tentative_g, neighbor, path + [neighbor]))

    # No path found
    return []

def helper_visualize_Astart(eroded, points, start_point, goal_point, open_set = [], closed_set=[]):
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
    plot_point(ax, start_point, color='green')
    plot_point(ax, goal_point, color='red')
    plt.show(block=False)


def helper_visualize_connection(step: Operator, plan: GPlan, static_objs: List[Argument] = [], obstacle_areas: List[Polygon] = [], eroded: Polygon = None) -> None:
    """ 
    Create an image of showing the process of checking wether a connection exists.
    """
    # Give each object a unique color
    objcolors = ["red",
                 "blue",
                 "green",
                 "yellow",
                 "purple",
                 "cyan",
                 "orange",
                 "white"]
    objcolor_dict = {}
    i = 0
    for obj in plan.variableBindings.symbolic_vb.objects:
        if plan.variableBindings.is_type(obj, 'physical_item'):
            objcolor_dict[obj] = objcolors[i]
            i = i+1
            if i >= len(objcolors):
                i = len(objcolors)-1

    fig, ax = plt.subplots(figsize=(8, 6))

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
        if type(eroded) == Polygon: # eroded space is not separated. therefore there is a path from start to goal
            plot_area(ax, eroded, color = 'green', label='eroded')
        if type(eroded) == MultiPolygon:
            for poly in eroded.geoms:
                plot_area(ax, poly, color = 'green', label='eroded')
        else: # eroded has an unexpected type
            print(f"eroded has an unexpected type: {type(eroded)}")

    # plot start area:
    start = plan.variableBindings.geometric_vb.get_assigned_area(step.Args[2])
    goal = plan.variableBindings.geometric_vb.get_assigned_area(step.Args[3])
    plot_area(ax, start, color = 'magenta', label='start')
    plot_area(ax, goal, color = 'magenta', label='goal')

    # plot all obstacles:
    for obj in static_objs:
        area_arg = plan.variableBindings.initial_positions[obj]
        area = plan.variableBindings.geometric_vb.defined_areas[area_arg]    
        plot_area(ax, area, color='black', label=obj.name)
    for loc in obstacle_areas:
        area = plan.variableBindings.geometric_vb.get_area(loc)
        obj = plan.variableBindings.symbolic_vb.get_const(plan.variableBindings.geometric_vb.placelocs[loc].object)
        objcolor = objcolor_dict[obj]
        plot_area(ax, area, color=objcolor, label=obj.name)    
    
    ax.set_aspect('equal')
    ax.autoscale()
    ax.set_title(f'Connection Visualization')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.show(block=False)
