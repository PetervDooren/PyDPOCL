from PyPOCL.GPlan import GPlan
from PyPOCL.worldmodel import Domain, Problem
from PyPOCL.Ground_Compiler_Library.Element import Operator
from PyPOCL.Ground_Compiler_Library.pathPlanner import check_connections_in_plan

import graphviz
from shapely import within, Polygon, MultiPolygon, overlaps, difference
from uuid import UUID
import json
import yaml

# visualization
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPolygon

MARGIN_OF_ERROR = 1e-7 # buffer for nummerical problems

def check_plan(plan: GPlan) -> None:
    """Check if the plan is complete and valid. Does not use the tracked flaws, but checks the plan structure directly."""
    if not plan.isInternallyConsistent():
        print("Plan is not internally consistent")
        return False
    if not plan.variableBindings.isInternallyConsistent():
        print("Variable bindings are not internally consistent")
        return False
    # check that all variables are ground
    if not plan.variableBindings.is_fully_ground():
        print("Variable bindings are not ground")
        return False
    # check that all preconditions of steps are supported by causal links
    for step in plan.steps:
        for pre in step.preconds:
            for edge in plan.CausalLinkGraph.edges:
                if edge.label.sink.ID == pre.ID and edge.sink.ID == step.ID:
                    break
            else:
                # if we did not break, then no edge was found
                print(f"Step {step.ID} has unsupported precondition {pre.ID}")
                return False
    # check that no causal links are threatened
    for edge in plan.CausalLinkGraph.edges:
        threatening_operators = [t[0] for t in  edge.sink.threat_map[edge.label.sink.ID]]
        for step in plan.steps:
            if not step.stepnum in threatening_operators:
                continue
            if step == edge.source or step == edge.sink:
                # if the step is the source or sink of the causal link, it cannot threaten it
                continue
            if not plan.OrderingGraph.isPath(edge.source, step) or not plan.OrderingGraph.isPath(step, edge.sink):
                # if the step is not in the path between source and sink, it cannot threaten the causal link
                continue
            for t in [t for t in edge.sink.threat_map[edge.label.sink.ID] if t[0] == step.stepnum]:
                if plan.variableBindings.is_unified(edge.label.sink, step.effects[t[1]]):
                    # if the edge is unified with the threatening effect, it threatens it
                    print(f"Causal link {edge} is threatened by step {step.ID}")
                    return False
    # check that all causal links condition arguments are unified
    for edge in plan.CausalLinkGraph.edges:
        # check that the effect of the edge is unified with the precondition of the edge
        if not plan.variableBindings.is_unified(edge.label.source, edge.label.sink):
            print(f"Causal link {edge} does not have unified effect: {edge.label.source} and precondition {edge.label.sink}")
            return False
        

    # check that place locations are large enough for the objects
    for area_id in plan.variableBindings.geometric_vb.variables:
        placeloc = plan.variableBindings.geometric_vb.placelocs[area_id]
        area = placeloc.area_assigned
        if area is None:
            print(f"Area {area_id} is not assigned")
            return False
        # check that the object is defined
        if placeloc.object_width == 0 or placeloc.object_length == 0:
            print(f"Area {area_id} has no object defined")
            return False
        # check that the object fits within the assigned area
        if area.area - placeloc.object_width * placeloc.object_length < -MARGIN_OF_ERROR:
            print(f"Area {area_id} is too small for the object with width {placeloc.object_width} and length {placeloc.object_length}")
            return False

    # check that all reach constraints of steps are satisfied in the assigned area
    for step in plan.steps:
        for rc in step.reach_constraints:
            # check that the reach constraint is satisfied in the assigned area
            area = plan.variableBindings.geometric_vb.get_assigned_area(rc[0])
            if area is None:
                print(f"Step {step.ID} has reach constraint {rc} but no assigned area")
                return False
            robot_arg = plan.variableBindings.symbolic_vb.get_const(rc[1])
            reach_area_arg = plan.variableBindings.reach_areas[robot_arg]
            reach_area = plan.variableBindings.geometric_vb.defined_areas[reach_area_arg]
            if not within(area, reach_area):
                print(f"Step {step.ID} has unsatisfied reach constraint {rc}")
                return False
    # check that all other areas that can occur simultaneously do not overlap
    static_objs = []
    for obj in plan.variableBindings.objects:
        if plan.variableBindings.is_type(obj, 'physical_item'):
            for causal_link in plan.CausalLinkGraph.edges:
                if causal_link.label.source.name == "within":
                    if obj == causal_link.label.source.Args[0]:
                        break
            else:
                static_objs.append(obj)

    for causal_link in plan.CausalLinkGraph.edges:
        if causal_link.label.source.name != "within":
            continue
        object = causal_link.label.source.Args[0]
        sourceloc = causal_link.label.source.Args[1]
        if causal_link.source.schema == 'dummy_init': # if the link is grounded in the initial condition, the source area is not a variable.
            area = plan.variableBindings.geometric_vb.defined_areas[sourceloc]
        else:
            area = plan.variableBindings.geometric_vb.get_assigned_area(sourceloc)
        
        # check overlap with static items
        for obj in static_objs:
            other_area_arg = plan.variableBindings.initial_positions[obj]
            other_area = plan.variableBindings.geometric_vb.defined_areas[other_area_arg]
            if overlaps(area, other_area):
                print(f"area: {sourceloc}, set by {causal_link.source} overlaps with static object {obj}, at {other_area_arg}.")
                return False		

        # check overlap with moving items
        for other_link in plan.CausalLinkGraph.edges:
            if other_link.label.source.name != "within":
                continue
            other_object = other_link.label.source.Args[0]
            if other_object == object:
                continue
            other_loc = other_link.label.source.Args[1]
            if other_link.source.schema == 'dummy_init': # if the link is grounded in the initial condition, the source area is not a variable.
                other_area = plan.variableBindings.geometric_vb.defined_areas[other_loc]
            else:
                other_area = plan.variableBindings.geometric_vb.get_assigned_area(other_loc)
            # check if the causal links overlap.
            if plan.OrderingGraph.isPath(causal_link.sink, other_link.source) or plan.OrderingGraph.isPath(other_link.sink, causal_link.source):
                # One causal link is stricly before another. Therefore the location described in it cannot be occupied at the same time.
                continue
            if overlaps(area, other_area):
                print(f"area: {sourceloc}, set by {causal_link.source} overlaps with area {other_loc}, set by {other_link.source}, both areas can be occupied at the same time.")
                return False
    if not check_plan_execution(plan):
        return False	
    return True

def check_plan_execution(plan: GPlan) -> bool:
    """Simulate an execution of a plan and check that it does not deadlock.

    Args:
        plan (GPlan): _description_
    """
    # create an open list of actions which can be performed at this stage

    closed_list = [plan.dummy.init]
    # create a state object
    state = plan.variableBindings.initial_positions

    while True:
        # determine which steps can be executed
        open_list = []
        for step in plan.steps:
            if step in closed_list:
                continue
            # for all prerequisites of step. check that they are in the closed list
            for edge in plan.OrderingGraph.edges:
                if edge.sink == step and edge.source not in closed_list:
                    break
            else:
                # all previous steps are in the closed list
                if step == plan.dummy.goal:
                    return True # reached the end of the plan.
                open_list.append(step)

        # check if any of the open steps can be executed
        for step in open_list:
            obj = plan.variableBindings.symbolic_vb.get_const(step.Args[1])
            goal_arg = step.Args[3]
            if check_connection(step, state, plan):
                # update state
                state[obj] = goal_arg
                # add step to the closed list
                closed_list.append(step)
                # reset loop.
                break
            else:
                print(f"Could not execute step {step} in the current state")
        else:
            # no step could be executed
            print(f"No step could be executed in the current state!")
            return False

def check_connection(step: Operator, state: dict, plan: GPlan) -> bool:
    # get the grounded information from step
    robot_obj = plan.variableBindings.symbolic_vb.get_const(step.Args[0])
    reach_area = plan.variableBindings.geometric_vb.defined_areas[plan.variableBindings.reach_areas[robot_obj]]
    
    moved_obj = plan.variableBindings.symbolic_vb.get_const(step.Args[1])
    object_width, object_length = plan.variableBindings.geometric_vb.object_dimensions[moved_obj]

    start_area = plan.variableBindings.geometric_vb.get_assigned_area(step.Args[2])
    goal_area = plan.variableBindings.geometric_vb.get_assigned_area(step.Args[3])
    
    # create a map of the available space
    available_space = reach_area
    for obj, area_arg in state.items():
        if obj == moved_obj:
            continue
        if area_arg in plan.variableBindings.geometric_vb.defined_areas:
            area = plan.variableBindings.geometric_vb.defined_areas[area_arg]
        else:
            area = plan.variableBindings.geometric_vb.get_assigned_area(area_arg)
        available_space = difference(available_space, area)
    erosion_dist = 0.5*min(object_width, object_length)
    eroded = available_space.buffer(-erosion_dist)

    # determine if both start and end lie in the available space
    is_connected = False
    if type(eroded) == Polygon: # eroded space is not separated. therefore there is a path from start to goal
        is_connected = True
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
    else: # eroded has an unexpected type
        print(f"eroded has an unexpected type: {type(eroded)}")
        is_connected = False

    #helper_show_state(step, state, plan, eroded, is_connected)
    return is_connected

def helper_show_state(step: Operator, state: dict, plan: GPlan, eroded: Polygon = None, connected: bool = None) -> bool:
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
    mainobj = plan.variableBindings.symbolic_vb.get_const(step.Args[1])
    start = plan.variableBindings.geometric_vb.get_assigned_area(step.Args[2])
    goal = plan.variableBindings.geometric_vb.get_assigned_area(step.Args[3])
    mainobjcolor = objcolor_dict[mainobj]
    edgecolor = 'black' if connected is None else 'green' if connected else 'red'
    plot_area(ax, start, color = mainobjcolor, edgecolor=edgecolor, alpha=0.8, label='start')
    plot_area(ax, goal, color = mainobjcolor, edgecolor=edgecolor, alpha=0.8, label='goal')

    # plot the current state:
    for obj, area_arg in state.items():
        if obj == mainobj:
            continue
        objcolor = objcolor_dict[obj]
        if area_arg in plan.variableBindings.geometric_vb.defined_areas:
            area = plan.variableBindings.geometric_vb.defined_areas[area_arg]
        else:
            area = plan.variableBindings.geometric_vb.get_assigned_area(area_arg)
        plot_area(ax, area, color=objcolor, label=obj.name)    
    
    ax.set_aspect('equal')
    ax.autoscale()
    ax.set_title(f'Connection Visualization')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.show()


def plan_to_dot(plan: GPlan, filepath_dot: str = None, filepath_svg: str = None, show=True) -> None:
    """
    Write the plan's ordering and causal link graphs to a Graphviz .dot file.

    Args:
        plan (GPlan): The plan to export.
        filepath_dot (str): The path to the output .dot file.
        filepath_svg (str): The path to the output .svg file.
        show (bool): show the figure 
    """
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

    dot = graphviz.Digraph()

    # add nodes for each step
    for step in plan.steps:
        if step.schema == 'movemono':
            obj = plan.variableBindings.symbolic_vb.get_const(step.Args[1])
            objcolor = objcolor_dict[obj]
        else:
            objcolor = "white"
        dot.node(f"{step.ID}", f"{step.schema}", style="filled", fillcolor=objcolor)

    for edge in plan.OrderingGraph.edges:
        dot.edge(f"{edge.source.ID}", f"{edge.sink.ID}", color="black", style="dashed")

    for edge in plan.CausalLinkGraph.edges:
        effect = edge.label.source.name
        dot.edge(f"{edge.source.ID}", f"{edge.sink.ID}", label=effect, color='red')

    dot.render(filename=filepath_dot, view=show, outfile=filepath_svg)

def visualize_plan(plan: GPlan, show=True, filepath: str = None) -> None:
    """ Create an image of the plan geometry. 

    Args:
        plan (_type_): _description_
        filepaht (str): file to write the image to. None = no image will be saved.
    """

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

    geo_vb = plan.variableBindings.geometric_vb

    # plot base area:
    plot_area(ax, geo_vb.defined_areas[geo_vb.base_area], alpha = 1.0)

    # plot all defined-areas:
    for key, area in geo_vb.defined_areas.items():
        if key == geo_vb.base_area:
            continue
        plot_area(ax, area, fill=False, label=key.name)

    # plot all actions:
    for step in plan.steps:
        # skip any step that does not move objects:
        if not step.schema == 'movemono':
            continue
        obj = plan.variableBindings.symbolic_vb.get_const(step.Args[1])
        startarea = geo_vb.get_assigned_area(step.Args[2])
        goalarea = geo_vb.get_assigned_area(step.Args[3])
        path = geo_vb.get_path(step.Args[4])
        patharea = geo_vb.get_area(step.Args[4])
        objcolor = objcolor_dict[obj]

        plot_area(ax, patharea, color=objcolor, alpha=0.3)
        # plot the path from start to goal
        ax.plot(*zip(*path.coords), color='black')
        start = path.coords[-2]
        end = path.coords[-1]
        ax.annotate('', xy=end, xytext=start,
            arrowprops=dict(arrowstyle='->', color='black', lw=2)
        )
        plot_area(ax, startarea, color=objcolor, edgecolor='blue')
        plot_area(ax, goalarea, color=objcolor, edgecolor='red')
        
    ax.set_aspect('equal')
    ax.autoscale()
    ax.set_title(f'Plan Visualization: {plan.problem}')
    plt.xlabel('X')
    plt.ylabel('Y')
    if filepath:
        fig.savefig(filepath)
    if show:
        plt.show()

def plot_area(ax, area: Polygon, color='lightgray', edgecolor='black', alpha=0.5, fill = True, label=None):
    coords = list(area.exterior.coords)
    poly = MplPolygon(coords, closed=True, facecolor=color, edgecolor=edgecolor, alpha=alpha, fill=fill, label=label)
    ax.add_patch(poly)
    if label:
        # Place label at centroid
        xs, ys = zip(*coords)
        centroid = (sum(xs)/len(xs), sum(ys)/len(ys))
        ax.text(centroid[0], centroid[1], label, ha='center', va='center', fontsize=8)

def plan_to_json(plan, filepath: str) -> None:
    """
    Serialize the current plan to a JSON file.

    Args:
        filepath (str): The path to the output JSON file.

    The JSON will include:
        - Plan metadata (ID, name, cost, heuristic, depth)
        - Steps with their arguments and properties
        - Orderings and causal links
        - Variable bindings (if serializable)
        - Flaws (if serializable)

    Note:
        Some complex objects may require custom serialization logic.
    """
    def condition_to_dict(literal):
        return {
            "ID": str(literal.ID),
            "name": literal.name, # not required, but useful for debugging
            "Args": [str(a.ID) for a in literal.Args],
            "trudom": literal.truth, # not required, but useful debugging
        }

    def step_to_dict(step):
        return {
            "ID": str(step.ID),
            "schema": str(getattr(step, "schema", "")),
            "Args": [str(a.ID) for a in getattr(step, "Args", [])],
            "preconds": [condition_to_dict(p) for p in getattr(step, "preconds", [])],
            "effects": [condition_to_dict(p) for p in getattr(step, "effects", [])],
            "stepnum": getattr(step, "stepnum", None), # not required, but useful for debugging
            "height": getattr(step, "height", None), # not required, but useful for debugging
        }

    def ordering_to_dict(edge):
        return {
            "source": str(getattr(edge.source, "ID", "")),
            "sink": str(getattr(edge.sink, "ID", "")),
        }

    def causal_link_to_dict(edge):
        return {
            "source": str(getattr(edge.source, "ID", "")),
            "sink": str(getattr(edge.sink, "ID", "")),
            "effect": str(getattr(edge.label.source, "ID", "")),
            "precondition": str(getattr(edge.label.sink, "ID", "")),
        }
    
    def variable_bindings_to_dict(vb):
        vb_dict = {"symbolic": {}, "geometric": {}}
        for a in vb.symbolic_vb.variables:
            vb_dict["symbolic"][str(a.ID)] = vb.symbolic_vb.get_const(a).name
        for a in vb.geometric_vb.variables:
            vb_dict["geometric"][str(a.ID)] = list(vb.geometric_vb.get_assigned_area(a).exterior.coords)
        return vb_dict

    plan_dict = {
        "ID": str(plan.ID),
        "name": plan.name,
        "domain": plan.domain,
        "problem": plan.problem,
        "solved": plan.solved,
        "cost": plan.cost,
        "heuristic": plan.heuristic,
        "depth": plan.depth,
        "init_state": step_to_dict(plan.dummy.init),
        "goal_state": step_to_dict(plan.dummy.goal),
        "steps": [step_to_dict(step) for step in plan.steps],
        "orderings": [ordering_to_dict(edge) for edge in plan.OrderingGraph.edges],
        "causal_links": [causal_link_to_dict(edge) for edge in plan.CausalLinkGraph.edges],
        "variableBindings": variable_bindings_to_dict(plan.variableBindings),
        # "flaws": str(self.flaws),
    }

    with open(filepath, "w") as f:
        json.dump(plan_dict, f, indent=2)

def plan_to_yaml(plan, filepath: str) -> None:
    """write a plan to a yaml file such that it can be executed. Plan will be collapsed to total order in this process.

    Args:
        filepath (str): _description_
    """
    # conversions between planning terms and yaml terms:
    robot_names = {"left_panda": 0,
                    "right_panda": 1}
    parcel_names = {"boxa": 0,
                    "boxb": 1,
                    "boxc": 2}

    plan_dict = {"actions": []}
    for step in plan.OrderingGraph.topoSort():
        if step.schema == "movemono": # we can only do one type now. And we should exclude init and goal.
            robot_name = plan.variableBindings.symbolic_vb.get_const(step.Args[0]).name
            parcel_name = plan.variableBindings.symbolic_vb.get_const(step.Args[1]).name
            target_area = plan.variableBindings.geometric_vb.get_assigned_area(step.Args[3])
            target_coords = [{"x": t[0], "y": t[1]} for t in target_area.exterior.coords]
            step_dict = {"type": "singleArm", # only have one type right now
                "robot": robot_names[robot_name],
                "parcel": parcel_names[parcel_name],
                "target": target_coords
            }
            plan_dict["actions"].append(step_dict)
    with open(filepath, "w") as f:
        yaml.dump(plan_dict, f, default_flow_style=False, sort_keys=False)

def plan_from_json(domain: Domain, problem: Problem, filepath: str) -> GPlan:
    """
    Load a plan from a JSON file.

    Args:
        filepath (str): The path to the input JSON file.

    This method will populate the current instance with the data from the JSON file.
    """
    plan = GPlan.make_root_plan(domain, problem)

    with open(filepath, "r") as f:
        plan_dict = json.load(f)

    # Check if the domain and problem match
    if plan_dict.get("domain") != domain.name:
        raise ValueError(f"Plans domain name {plan_dict.get('domain')} does not match provided domain {domain.name}")
    if plan_dict.get("problem") != problem.name:
        raise ValueError(f"Plans problem name {plan_dict.get('problem')} does not match provided problem {problem.name}")
    
    plan.name = plan_dict["name"]
    plan.ID = UUID(plan_dict["ID"])
    plan.solved = plan_dict["solved"]
    plan.cost = plan_dict["cost"]
    plan.heuristic = plan_dict["heuristic"]
    plan.depth = plan_dict["depth"]

    # track IDs between the saved plan and the new plan
    id_map = {}
    id_map["steps"] = {}
    id_map["args"] = {}
    id_map["preconds"] = {}
    id_map["effects"] = {}
    id_map["objects"] = {}

    # get the IDs of the init and goal steps
    id_map["steps"][plan_dict["init_state"]["ID"]] = plan.dummy.init.ID
    for i in range(len(plan.dummy.init.effects)):
        id_map["effects"][plan_dict["init_state"]["effects"][i]["ID"]] = plan.dummy.init.effects[i].ID
    id_map["steps"][plan_dict["goal_state"]["ID"]] = plan.dummy.goal.ID
    for i in range(len(plan.dummy.goal.preconds)):
        id_map["preconds"][plan_dict["goal_state"]["preconds"][i]["ID"]] = plan.dummy.goal.preconds[i].ID

    # Add steps to the plan
    for step_data in plan_dict["steps"]:
        if step_data["schema"] == "dummy_init" or step_data["schema"] == "dummy_goal":
            # Skip the dummy init and goal steps as they are already created
            continue
        # find the operator schema in the domain
        op_schema = next((op for op in domain.operators if op.schema == step_data["schema"]), None)
        if not op_schema:
            raise ValueError(f"Operator schema {step_data['schema']} not found in domain {domain.name}")
        step = op_schema.instantiate()
        # Add the step to the plan
        plan.insert(step)

        # map the step ids
        id_map["steps"][step_data["ID"]] = step.ID
        # map the argument IDs
        for i in range(len(step_data["Args"])):
            id_map["args"][step_data["Args"][i]] = step.Args[i].ID
        # map the precondition IDs
        for i in range(len(step_data["preconds"])):
            id_map["preconds"][step_data["preconds"][i]["ID"]] = step.preconds[i].ID
        for i in range(len(step_data["effects"])):
            id_map["effects"][step_data["effects"][i]["ID"]] = step.effects[i].ID
    
    # add ordering constraints to the plan
    for edge_data in plan_dict["orderings"]:
        source_id = edge_data["source"]
        sink_id = edge_data["sink"]
        source = plan.get(id_map["steps"][source_id])
        sink = plan.get(id_map["steps"][sink_id])
        plan.OrderingGraph.addEdge(source, sink)
    
    # add causal links to the plan
    for edge_data in plan_dict["causal_links"]:
        source_id = edge_data["source"]
        sink_id = edge_data["sink"]
        effect_id = edge_data["effect"]
        precondition_id = edge_data["precondition"]
        source = plan.get(id_map["steps"][source_id])
        sink = plan.get(id_map["steps"][sink_id])
        effect = next((e for e in source.effects if e.ID == id_map["effects"][effect_id]))
        precondition = next((e for e in sink.preconds if e.ID == id_map["preconds"][precondition_id]))
        if source and sink:
            plan.CausalLinkGraph.addEdge(source, sink, effect, precondition)
    
    # add variable bindings to the plan
    for var, obj in plan_dict["variableBindings"]["symbolic"].items():
        if var not in id_map["args"]:
            continue  # this is a constant, not a variable
        var_arg = next((a for a in plan.variableBindings.symbolic_vb.variables if a.ID == id_map["args"][var]), None)
        if not var_arg:
            raise ValueError(f"Variable {var} not found in plan {plan.name}")
        obj_arg = next((o for o in problem.objects if o.name == obj), None)
        if not obj_arg:
            raise ValueError(f"Object {obj} not found in problem {problem.name}")
        plan.variableBindings.symbolic_vb.add_codesignation(var_arg, obj_arg)
    for var, area in plan_dict["variableBindings"]["geometric"].items():
        if var not in id_map["args"]:
            continue  # this is a constant, not a variable
        var_arg = next((a for a in plan.variableBindings.geometric_vb.variables if a.ID == id_map["args"][var]), None)
        if not var_arg:
            raise ValueError(f"Variable {var} not found in plan {plan.name}")
        area_poly = Polygon(area)
        if not area_poly:
            raise ValueError(f"Area {var} has invalid polygon {area}")
        plan.variableBindings.geometric_vb.set_assigned_area(var_arg, area_poly)
    return plan