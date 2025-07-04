import os
import json
from collections import namedtuple
from shapely import Polygon
from dataclasses import dataclass
from uuid import uuid4

from PyPOCL.Ground_Compiler_Library.GElm import GLiteral, Operator
from PyPOCL.Ground_Compiler_Library import Ground, precompile
from PyPOCL.Ground_Compiler_Library.Element import Argument

Domain = namedtuple('Domain', ['name', 'conditions', 'object_types', 'operators'])
Problem = namedtuple('Problem', ['name', 'domain', 'objects', 'base_area', 'areas', 'initial_positions', 'init', 'goal', 'robot_reach'])

def just_compile(domain_file, problem_file):
	GL = Ground.GLib(domain_file, problem_file)
	ground_step_list = precompile.deelementize_ground_library(GL)
	return ground_step_list, GL.objects, GL.object_types

@dataclass
class world_object:
    name: str
    width: float # dimension x
    length: float # dimension y
    pos_x: float
    pos_y: float

def load_worldmodel(file, objects):
    """Load a geometric worldmodel and link it to the symbolic worldmodel

    Args:
        file (sting): json file containing the worldmodel description
        objects (set(Arguments)): list of symbolic objects in the planning problem

    Returns:
        _type_: _description_
    """
    with open(file, 'r') as file:
        data = json.load(file)
        # load areas
        areas = {}
        for a in data['areas']:
            areas[a["name"]] = Polygon(a["coords"])
        # get base area
        base_area = data["base_area"]
        # load robot info
        robot_reach = {}
        for r in data["robots"]:
            robot_reach[r["name"]] = r["reach"]
            if r["reach"] not in areas.keys():
                print(f"robot {r['name']} specifies reach area {r['reach']} but this is not defined. Existing areas are {areas.keys()}")
        # load objects
        geo_objects = {}
        for o in data["objects"]:
            geo_objects[o["name"]] = world_object(o["name"], o["width"], o["length"], o["initial_pose"][0], o["initial_pose"][1])

    area_mapping = {} # mapping of arguments to a polygon
    object_mapping = {} # mapping of arguments to world_object
    object_area_mapping = {} # mapping of object argument to inital area argument
    robot_reach_mapping = {} # mapping between a robot argument and an area argument representing its reach

    # link areas to their arguments
    area_objects = [a for a in objects if a.typ=='area']
    for a in area_objects:
        area_mapping[a] = areas[a.name]

    # link objects to their arguments
    physical_objects = [a for a in objects if a.typ=='item']
    for o in physical_objects:
        object_mapping[o] = geo_objects[o.name]
        # create arguments to represent the initial positions of the object
        argname = o.name + "_init_pos"
        area_arg = Argument(uuid4(), "area", argname, None)
        objects.add(area_arg)
        object_area_mapping[o] = area_arg
        # create an area to represent the initial position of the object
        obj = geo_objects[o.name]
        x_min = obj.pos_x - 0.5*obj.width
        x_max = obj.pos_x + 0.5*obj.width
        y_min = obj.pos_y - 0.5*obj.length
        y_max = obj.pos_y + 0.5*obj.length
        object_poly = Polygon([[x_min, y_min],
                              [x_min, y_max],
                              [x_max, y_max],
                              [x_max, y_min]])
        # add inital area to the set of objects
        area_mapping[area_arg] = object_poly

    # link robots to their reach
    robot_objects = [a for a in objects if a.typ=='robot']
    for r in robot_objects:
        reach_area = [a for a in area_objects if a.name == robot_reach[r.name]]
        robot_reach_mapping[r] = reach_area[0] 

    return objects, area_mapping, object_mapping, object_area_mapping, robot_reach_mapping, base_area

def update_init_state(init_state, area_mapping, object_area_mapping):
    """update the truth conditions of the initial state using the geometry

    Args:
        init_state (_type_): initial state to be modified
        area_mapping (dict(Argument: Polygon)): mapping of arguments to a polygon
        object_area_mapping (dict(Argument:Argument)): mapping of object argument to inital area argument

    Returns:
        _type_: initial state with updated truth values on its effects.
    """
    # add conditions for intial positions
    for obj in object_area_mapping.keys():
        for area in object_area_mapping.values():
            cond = GLiteral('within', [obj, area], False, uuid4(), False)
            init_state.effects.append(cond)

    for cond in init_state.effects:
        if cond.name == 'within':
            init_area_arg = object_area_mapping[cond.Args[0]]
            object_poly = area_mapping[init_area_arg]
            area_poly = area_mapping[cond.Args[1]]
            if object_poly.within(area_poly):
                cond.truth = True
            else:
                cond.truth = False
    return init_state

def pre_process_operators(operators):
		"""pre processes operators with the relations between them.
		updates properties cndts, cndt_map, threat_map, and threats

		Args:
			operators (List(Operator)): list of operators, including the initial and goal state.
		"""
		
		for op1 in operators:
			# clear existing data
			op1.cndts = []
			op1.cndt_map = dict()
			op1.threats = []
			op1.threat_map = dict()

			for pre in op1.preconds:
				print('... Processing antecedents for {} \t\tof step {}'.format(pre, op1))
				op1.cndt_map[pre.ID] = []
				op1.threat_map[pre.ID] = []
				for op2 in operators:
					for eff_i in range(len(op2.effects)):
						eff = op2.effects[eff_i]
						if eff.name != pre.name:
							continue # effect is not based on the same predicate as the precondition
						if eff.truth != pre.truth: # the effect undoes the precondition. Add to threat list
							if op2.stepnumber not in op1.threats:
								op1.threats.append(op2.stepnumber)
							op1.threat_map[pre.ID].append((op2.stepnumber, eff_i))
						else: # the effect is identical and therefore fulfills the precondition
							if op2.stepnumber not in op1.cndts:
								op1.cndts.append(op2.stepnumber)
							op1.cndt_map[pre.ID].append((op2.stepnumber, eff_i))


def load_domain_and_problem(domain_file, problem_file, worldmodel_file):
    """
    Create a domain with the given initial and goal states.

    Args:
        init (str): Initial state of the domain.
        goal (str): Goal state of the domain.

    Returns:
        Domain: A named tuple representing the domain with init and goal states.
    """
    ground_steps, objects, object_types = just_compile(domain_file, problem_file)
    operators = ground_steps[:-2]  # all except init and goal
    init_state = ground_steps[-2]  # second last step is the initial state
    goal_state = ground_steps[-1]  # last step is the goal state

    if worldmodel_file is None:
        # no worldmodel, so the domain contains no geometric variables
        area_mapping = {}
        object_mapping = {}
        object_area_mapping = {}
        robot_reach = {}
        base_area = None
    else:
        # load worldmodel
        objects, area_mapping, object_mapping, object_area_mapping, robot_reach, base_area = load_worldmodel(worldmodel_file, objects)
        init_state = update_init_state(init_state, area_mapping, object_area_mapping)
        pre_process_operators(ground_steps)

    # domain and problem names
    domain_name = os.path.splitext(os.path.basename(domain_file))[0]
    problem_name = os.path.splitext(os.path.basename(problem_file))[0]
    domain = Domain(name = domain_name,
                    conditions=[], # not used in the planner, but technically part of the domain definition
                    object_types=object_types,
                    operators=operators)
    problem = Problem(name = problem_name,
                      domain=domain_name,
                      objects=objects,
                      base_area=base_area,
                      areas=area_mapping,
                      initial_positions=object_area_mapping,
                      init=init_state,
                      goal=goal_state,
                      robot_reach=robot_reach)
    return domain, problem