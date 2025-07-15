from PyPOCL.GPlan import GPlan

from typing import List
from shapely import Polygon, MultiPolygon, difference, within

def check_connections_in_plan(plan: GPlan) -> bool:
    """verify that for every move action in the plan. There exists a path from the start to the goal position in that configuration.

    Args:
        plan (GPlan): _description_
    """
    static_obst_areas = []
    # find static objects
    for obj in plan.variableBindings.objects:
        if plan.variableBindings.is_type(obj, 'physical_item'):
            for causal_link in plan.CausalLinkGraph.edges:
                if causal_link.label.source.name == "within":
                    if obj == causal_link.label.source.Args[0]:
                        break
            else:
                area_arg = plan.variableBindings.initial_positions[obj]
                area = plan.variableBindings.geometric_vb.defined_areas[area_arg]
                static_obst_areas.append(area)
    
    for step in plan.steps:
        if step.schema != 'movemono':
            continue
        obst_areas = static_obst_areas
        # find all placelocs that could occur simultaneously.
        for causal_link in plan.CausalLinkGraph.edges:
            if causal_link.label.source.name != "within":
                continue
            if not plan.OrderingGraph.isPath(step, causal_link.source) and not plan.OrderingGraph.isPath(causal_link.sink, step):
                   continue
            sourceloc = causal_link.label.source.Args[1]
            if causal_link.source.schema == 'dummy_init': # if the link is grounded in the initial condition, the source area is not a variable.
                area = plan.variableBindings.geometric_vb.defined_areas[sourceloc]
            else:
                area = plan.variableBindings.geometric_vb.get_assigned_area(sourceloc)
            obst_areas.append(area)
        
        start_area = plan.variableBindings.geometric_vb.get_assigned_area(step.Args[2])
        goal_area = plan.variableBindings.geometric_vb.get_assigned_area(step.Args[3])
        robot_obj = plan.variableBindings.symbolic_vb.get_const(step.Args[0])
        reach_area = plan.variableBindings.geometric_vb.defined_areas[plan.variableBindings.reach_areas[robot_obj]]
        object_width, object_length = plan.variableBindings.geometric_vb.object_dimensions[plan.variableBindings.symbolic_vb.get_const(step.Args[1])]
        if not check_is_connected(start_area, goal_area, reach_area, obst_areas, object_width, object_length):
            print(f"No connection between start and end of {step}.")
            return False
    # no step was found to not have a connection between start and end
    return True

def check_is_connected(start: Polygon, goal: Polygon, reach: Polygon, obstacle_areas: List[Polygon], object_width: float, object_length: float) -> bool:
    if not within(start, reach):
        print(f"start area {start} is not within the reachable space {reach}")
        return False
    if not within(goal, reach):
        print(f"goal area {goal} is not within the reachable space {reach}")
        return False

    available_space = reach
    for obst in obstacle_areas:
        available_space = difference(available_space, obst)
    erosion_dist = 0.5*min(object_width, object_length)
    eroded = available_space.buffer(-erosion_dist)

    if type(eroded) == Polygon: # eroded space is not separated. therefore there is a path from start to goal
        return True
    if type(eroded) == MultiPolygon:
        start_centroid = start.centroid # middle of the start area
        goal_centroid = goal.centroid # middle of the goal area
        for poly in eroded.geoms:
            if within(start_centroid, poly) and within(goal_centroid, poly):
                return True
        # no polygon contains both start and goal. Therefore they are separated in the reachable space
        return False
    else: # eroded has an unexpected type
        print(f"eroded has an unexpected type: {type(eroded)}")
        return False
