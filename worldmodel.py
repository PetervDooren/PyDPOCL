import json
from shapely import Polygon

def load_worldmodel(file):
    with open(file, 'r') as file:
        data = json.load(file)
        # load areas
        areas = {}
        for a in data['areas']:
            areas[a["name"]] = Polygon(a["coords"])
        # load robot info
        robot_reach = {}
        for r in data["robots"]:
            robot_reach[r["name"]] = r["reach"]
            if r["reach"] not in areas.keys():
                print(f"robot {r['name']} specifies reach area {r['reach']} but this is not defined. Existing areas are {areas.keys()}")
    return robot_reach, areas

def link_areas(objects, areas):
    area_objects = [a for a in objects if a.typ=='area']
    linked_areas = {}
    for a in area_objects:
        linked_areas[a] = areas[a.name]
    return linked_areas