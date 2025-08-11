from PyPOCL.GPlan import GPlan
from PyPOCL.Ground_Compiler_Library.Element import Operator, Argument

from typing import List
from shapely import Polygon, MultiPolygon, difference, within

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
            area = plan.variableBindings.geometric_vb.get_assigned_area(loc)
            available_space = difference(available_space, area)
        erosion_dist = 0.5*min(object_width, object_length)
        eroded = available_space.buffer(-erosion_dist)

        #helper_visualize_connection(step, plan, static_objs, obst_areas, eroded)

        if type(eroded) == Polygon: # eroded space is not separated. therefore there is a path from start to goal
            is_connected = True
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
                return False
        else: # eroded has an unexpected type
            print(f"eroded has an unexpected type: {type(eroded)}")
            is_connected = False
            return False
        
        if not is_connected:
            return False
    # no step was found to not have a connection between start and end
    return True

def helper_visualize_connection(step: Operator, plan: GPlan, static_objs: List[Argument] = [], obstacle_areas: List[Polygon] = [], eroded: Polygon = None) -> None:
    """ 
    Create an image of showing the process of checking wether a connection exists.
    """
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon as MplPolygon

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
        plot_area(ax, area, color='black', label='static_obstacle')
    for loc in obstacle_areas:
        area = plan.variableBindings.geometric_vb.get_assigned_area(loc)
        obj = plan.variableBindings.symbolic_vb.get_const(plan.variableBindings.geometric_vb.placelocs[loc].object)
        objcolor = objcolor_dict[obj]
        plot_area(ax, area, color=objcolor, label=obj.name)    
    
    ax.set_aspect('equal')
    ax.autoscale()
    ax.set_title(f'Connection Visualization')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.show()
