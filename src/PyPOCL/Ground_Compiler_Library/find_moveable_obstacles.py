from PyPOCL.Ground_Compiler_Library.Element import Argument
from PyPOCL.GPlan import GPlan

from typing import List
from shapely import Polygon, MultiPolygon, difference, within, intersects

# visualization
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPolygon

def find_movable_obstacles(plan: GPlan, pathvar: Argument) -> List[List[Argument]]:
    """ Find which objects can be moved to ground the path variable.
    The disjunctions of pathvar should already be set.

    Args:
        plan (GPlan): the plan in which to find the movable obstacles
        pathvar (Argument): the path variable which cannot be grounded

    Returns:
        List[List[Arguments]]: Each item in the first list is a collection of objects which,
                               if they are all moved. Provide a path between start and goal. Each collection has the same cost.
    """
    geo_vb = plan.variableBindings.geometric_vb
    start_arg = geo_vb.paths[pathvar].start_area
    start_area = geo_vb.placelocs[start_arg].area_assigned
    start = start_area.centroid
    goal_arg = geo_vb.paths[pathvar].goal_area
    goal_area = geo_vb.placelocs[goal_arg].area_assigned
    goal = goal_area.centroid
    object_width = geo_vb.paths[pathvar].object_width
    object_length = geo_vb.paths[pathvar].object_length

    available_space = geo_vb.defined_areas[geo_vb.base_area]
    for within_var in geo_vb.within_mapping[pathvar]:
        if within_var in geo_vb.defined_areas.keys():
            available_space = available_space.intersection(geo_vb.defined_areas[within_var])
        else: # within_var is a variable
            print(f"path {pathvar} should not be within placement location {within_var}!")
    # check if the start and goal areas are connected
    # remove all areas that are disjunct from the area_max
    disjunct_args = geo_vb.disjunctions[pathvar]
    for d_area in disjunct_args:
        if d_area in geo_vb.defined_areas:
            available_space = difference(available_space, geo_vb.defined_areas[d_area])
        elif geo_vb.placelocs[d_area].area_assigned is not None:
            available_space = difference(available_space, geo_vb.placelocs[d_area].area_assigned)
        else:
            print(f"problem: disjunct area {d_area} is not defined")
    # erode available space with the size of the object
    erosion_dist = 0.5*min(object_width, object_length)
    eroded = available_space.buffer(-erosion_dist)
    
    # check that the available space is the correct type:
    if type(eroded) == Polygon: # eroded space is not separated. therefore there is a path from start to goal
        print("start and end are already connected")
        return []
    elif type(eroded) != MultiPolygon:
        print(f"available space has unknown type {type(available_space)}")
        raise
    all_areas = disjunct_args
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
    
    # create dict for visualization
    obst_area_dict = {}
    for i in range(len(areas)):
        obst_area_dict[all_areas[i]] = areas[i]
    #helper_visualize_moveable_obstacles(poly_args, obst_area_dict, connections, cost_dict, predecessor_dict)
    
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
            #check if we enter free space, in which case cost does not increase
            if connection in poly_args:
                if node_cost < cost_dict[connection]:
                    predecessor_dict[connection] = [node]
                    cost_dict[connection]=node_cost
                elif node_cost == cost_dict[connection]:
                    predecessor_dict[connection].append(node)
                continue
            # otherwise cost increases by 1
            else:
                if node_cost + 1 < cost_dict[connection]:
                    predecessor_dict[connection] = [node]
                    cost_dict[connection]=node_cost + 1
                elif node_cost + 1 == cost_dict[connection]:
                    predecessor_dict[connection].append(node)
                continue
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
    #helper_visualize_moveable_obstacles(poly_args, obst_area_dict, connections, cost_dict, predecessor_dict)
    return movable_object_sets

def helper_visualize_moveable_obstacles(poly_args, obst_areas, connections, cost_list, predecessor_list, start=None, goal=None):
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
    for poly in poly_args.values():
        plot_area(ax, poly, color = 'green')
    for obst in obst_areas.values():
        plot_area(ax, obst, color = 'gray')

    if start is not None:
        plot_area(ax, start, color='red', label='start')
        ax.plot(start.centroid.x, start.centroid.y, marker='o', color='black')  # single point
    if goal is not None:
        plot_area(ax, goal, color='red', label='goal')
        ax.plot(goal.centroid.x, goal.centroid.y, marker='o', color='black')
    for node, node_connections in connections.items():
        if node in poly_args:
            p1 = poly_args[node].centroid
        elif node in obst_areas:
            p1 = obst_areas[node].centroid
        for connection in node_connections:
            if connection in poly_args:
                p2 = poly_args[connection].centroid
            elif connection in obst_areas:
                p2 = obst_areas[connection].centroid
            if connection in predecessor_list[node] or node in predecessor_list[connection]:
                ax.plot([p1.x, p2.x], [p1.y, p2.y], color='Blue')
            else:
                ax.plot([p1.x, p2.x], [p1.y, p2.y], color='black')
        colors = ['green', 'yellow', 'red']
        if cost_list[node] < 3:
            color = colors[cost_list[node]]
        else:
            color = 'black'
        ax.plot(p1.x, p1.y, marker='o', color=color)  # single point
    plt.show()