from typing import List
from shapely import Polygon, MultiPolygon, difference, within
   
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
