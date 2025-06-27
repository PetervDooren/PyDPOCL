from __future__ import annotations
from Ground_Compiler_Library.GElm import GLiteral, Operator
from uuid import uuid4
from Flaws import FlawLib, OPF, TCLF
from Ground_Compiler_Library.OrderingGraph import OrderingGraph, CausalLinkGraph
from Ground_Compiler_Library.VariableBindings import VariableBindings
import copy
from collections import namedtuple, defaultdict
import math
import json

dummyTuple = namedtuple('dummyTuple', ['init', 'goal'])
# class dummyTuple:
# 	def __init__(self, init, final):
# 		self.init = init
# 		self.final = final

class GPlan:
	"""
	Plan in plan space
    ...

    Attributes
    ----------
    ID : uuid
        unique identifier of the plan
    OrderingGraph : OrderingGraph
        ???
	CausalLinkGraph: CausalLinkGraph
		???
	variableBindings: VariableBindings
		structure to solve the constraint satisfaction problem of variable bindings
	flaws: FlawLib
		list of open flaws
	solved: bool
		true if the plan is without flaws
	dummy: dummyTuple
		tuple containing the steps representing the initial state and the goal
	init: List(GLiteral)
		list of conditions present in the initial state
	goal: List(GLiteral)
		list of goal conditions
	steps: List(Operator)
		list of steps that make the plan
	cndt_map: ???
		???
	threat_map: ???
		???
	heuristic: float
		heuristic estimating the cost to complete the plan
	name: string
		???
    cost: float
		cost of the plan thus far
	depth: int
		???

	Methods
    -------
	__init__(?, ?)
    make_root_plan():
	index():
	instantiate():
	isInternallyConsistent():
	insert():
	insert_primitive():
	insert_decomp():
	resolve():
	resolve_with_primitive():
	resolve_with_decomp():
	"""

	def __init__(self):
		"""Initialize a GPlan with empty graphs and no steps"""
		# plan data
		self.ID = uuid4()
		self.steps = []
		self.OrderingGraph = OrderingGraph()
		self.CausalLinkGraph = CausalLinkGraph()
		self.variableBindings = VariableBindings()
		self.init = []
		self.goal = []

		# planning metadata
		self.flaws = FlawLib()
		self.solved = False
		self.dummy = None
		self.cndt_map = None
		self.threat_map = None
		self.heuristic = float('inf')
		self.name = ''
		self.cost = 0
		self.depth = 0
		self.potential_tclf = []

	def __len__(self):
		return len(self.steps)

	def __getitem__(self, pos):
		return self.steps[pos]

	def __setitem__(self, item, pos):
		self.steps[pos] = item

	@staticmethod
	def make_root_plan(dummy_init_constructor, dummy_goal_constructor, objects, object_types, area_mapping, robot_reach, base_area='table') -> GPlan:
		"""Helper function to create a root plan at the start of planning

		Args:
			dummy_init_constructor (_type_): _description_
			dummy_goal_constructor (_type_): _description_
			objects (set(Argument)): list of objects in the world
			object_types(defaultdict(str, set(str)))
		"""
		root_plan = GPlan()

		root_plan.dummy = dummyTuple(dummy_init_constructor, dummy_goal_constructor)

		root_plan.init = root_plan.dummy.init.preconds
		root_plan.goal = root_plan.dummy.goal.preconds
		root_plan.steps = [root_plan.dummy.init, root_plan.dummy.goal]
		# check if any existing steps are choices (instances of cndts of open conditions)
		root_plan.dummy.goal.update_choices(root_plan)
		# add required orderings
		root_plan.OrderingGraph.addOrdering(root_plan.dummy.init, root_plan.dummy.goal)

		# register parameters
		root_plan.variableBindings.set_objects(objects, object_types)
		root_plan.variableBindings.set_areas(area_mapping)
		# base area is table:
		table_areas = [a for a in area_mapping if a.name==base_area]
		root_plan.variableBindings.geometric_vb.set_base_area(table_areas[0])
		root_plan.variableBindings.set_reach(robot_reach)
		# for condition in root_plan.init:
		# 	for a in condition.Args:
		# 		root_plan.variableBindings.register_variable(a)
		# for condition in root_plan.goal:
		# 	for a in condition.Args:
		# 		root_plan.variableBindings.register_variable(a)

		# add open precondition flaws for the goal
		for p in root_plan.dummy.goal.open_preconds:
			root_plan.flaws.insert(root_plan, OPF(root_plan.dummy.goal, p, 100000))

		return root_plan

	def index(self, step):
		for i, s in enumerate(self.steps):
			if s.ID == step.ID:
				return i
		print('{} {} {}'.format(self.name, step, step.ID))
		for i, s in enumerate(self.steps):
			print('{} {} {}'.format(i, s, s.ID))
		raise ValueError('{} with ID={} not found in plan {}'.format(step, step.ID, self.name))

	def instantiate(self, add_to_name):
		new_self = copy.deepcopy(self)
		new_self.ID = uuid4()
		new_self.name += add_to_name
		# refresh attributes
		return new_self

	# @property
	# def cost(self):
	# 	return len(self.steps) - 2

	def isInternallyConsistent(self):
		return self.OrderingGraph.isInternallyConsistent() and self.CausalLinkGraph.isInternallyConsistent()

	# Insert Methods #

	def insert(self, step):
		# baseline condition:
		# self.cost += 1
		# self.cost += (2 * 2 + 1) - (step.height * step.height)
		if step.height > 0:
			self.insert_decomp(step)
		else:
			self.insert_primitive(step)


	def insert_primitive(self, new_step):
		self.steps.append(new_step)

		# global orderings
		self.OrderingGraph.addEdge(self.dummy.init, new_step)
		self.OrderingGraph.addEdge(new_step, self.dummy.goal)

		# add variables of the new step
		for a in new_step.Args:
			self.variableBindings.register_variable(a)
		for within_condition in new_step.preconds:
			if within_condition.name == 'within': 
				self.variableBindings.link_area_to_object(within_condition.Args[0], within_condition.Args[1])
		for ne in new_step.nonequals:
			self.variableBindings.add_non_codesignation(new_step.Args[ne[0]], new_step.Args[ne[1]])
		for rc in new_step.reach_constraints:
			self.variableBindings.add_reach_constraint(rc[0], rc[1])

		# add open conditions for new step
		for pre in new_step.open_preconds:
			self.flaws.insert(self, OPF(new_step, pre))

		# check for causal link threats
		for edge in self.CausalLinkGraph.edges:
			# source, sink, condition = edge
			if edge.source.ID == new_step.ID:
				continue
			if edge.sink.ID == new_step.ID:
				continue
			if self.OrderingGraph.isPath(new_step, edge.source):
				continue
			if self.OrderingGraph.isPath(edge.sink, new_step):
				continue
			if new_step.stepnum in [tup[0] for tup in edge.sink.threat_map[edge.label.ID]]:
				self.potential_tclf.append(TCLF(new_step, edge))

	def insert_decomp(self, new_step):
		raise DeprecationWarning("decomposition is no longer supported")
		# magic happens here
		swap_dict = dict()

		# sub dummy init
		d_i = new_step.dummy.init.instantiate()
		d_i.depth = new_step.depth
		swap_dict[new_step.dummy.init.ID] = d_i
		self.steps.append(d_i)
		# add flaws for each new_step precondition, but make s_need d_i and update cndt_map/ threat_map
		d_i.swap_setup(new_step.cndts, new_step.cndt_map, new_step.threats, new_step.threat_map)
		for pre in new_step.open_preconds:
			self.flaws.insert(self, OPF(d_i, pre, new_step.height))
		preconds = list(new_step.open_preconds)
		d_i.preconds = preconds
		d_i.open_preconds = preconds

		self.OrderingGraph.addEdge(self.dummy.init, d_i)
		self.OrderingGraph.addEdge(d_i, self.dummy.goal)


		# sub dummy goal
		d_f = new_step.dummy.goal.instantiate(default_None_is_to_refresh_open_preconds=False)
		d_f.depth = new_step.depth
		swap_dict[new_step.dummy.goal.ID] = d_f
		# d_f will be primitive, to allow any heighted applicable steps
		self.insert(d_f)

		# added this 2017-08-09
		self.OrderingGraph.addEdge(d_i, d_f)
		self.OrderingGraph.addEdge(d_f, self.dummy.goal)
		self.OrderingGraph.addEdge(self.dummy.init, d_f)

		# decomposition links
		# self.HierarchyGraph.addOrdering(new_step, d_i)
		# self.HierarchyGraph.addOrdering(new_step, d_f)
		# for sb_step in new_step.sub_steps:
		# 	self.HierarchyGraph.addOrdering(new_step, sb_step)

		# log who your family is
		new_step.dummy = dummyTuple(d_i, d_f)
		d_i.sibling = d_f
		d_f.sibling = d_i

		# sub steps
		for substep in new_step.sub_steps:
			new_substep = substep.instantiate(default_None_is_to_refresh_open_preconds=False)
			swap_dict[substep.ID] = new_substep

			# INCREMENT DEPTH
			new_substep.depth = new_step.depth + 1
			if new_substep.depth > self.depth:
				self.depth = new_substep.depth

			self.insert(new_substep)

			# if your substeps have children, make those children fit between your init and
			if new_substep.height > 0:
				self.OrderingGraph.addEdge(new_substep.dummy.goal, d_f)
				self.OrderingGraph.addEdge(d_i, new_substep.dummy.init)

		# sub orderings
		for edge in new_step.sub_orderings.edges:
			source, sink = swap_dict[edge.source.ID], swap_dict[edge.sink.ID]
			if source.height > 0:
				source = source.dummy.goal
			if sink.height > 0:
				sink = sink.dummy.init
			self.OrderingGraph.addEdge(source, sink)

		# sub links
		for edge in new_step.sub_links.edges:
			# instantiating a GLiteral does not give it new ID (just returns deep copy)
			source, sink, label = swap_dict[edge.source.ID], swap_dict[edge.sink.ID], edge.label.instantiate()
			if source.height > 0:
				source = source.dummy.goal
			if sink.height > 0:
				sink = sink.dummy.init

			clink = self.CausalLinkGraph.addEdge(source, sink, label)

			# check if this link is threatened
			for substep in new_step.sub_steps:
				new_substep = swap_dict[substep.ID]
				if new_substep.ID in {clink.source.ID, clink.sink.ID}:
					continue
				if new_substep.stepnum not in clink.sink.threat_map[clink.label.ID]:
					continue
				if new_substep.height > 0:
					# decomp step compared to its dummy init and dummy goal steps
					if self.OrderingGraph.isPath(new_substep.dummy.goal, clink.source):
						continue
					if self.OrderingGraph.isPath(clink.sink, new_substep.dummy.init):
						continue
					self.flaws.insert(self, TCLF(new_substep.dummy.goal, clink))
				else:
					# primitive step gets the primitive treatment
					if self.OrderingGraph.isPath(new_substep, clink.source):
						continue
					if self.OrderingGraph.isPath(clink.sink, new_substep):
							continue
					self.flaws.insert(self, TCLF(new_substep, clink))

	# Resolve Methods #

	def resolve(self, new_step, s_need, p):
		if new_step.height > 0:
			self.resolve_with_decomp(new_step, s_need, p)
		else:
			self.resolve_with_primitive(new_step, s_need, p)

	def resolve_with_primitive(self, new_step, mutable_s_need, mutable_p):
		"""resolve required precondition 'mutable_p' of step 'mutable_s_need' using step 'new_step' 

		Args:
			new_step (_type_): _description_
			mutable_s_need (_type_): _description_
			mutable_p (_type_): _description_
		"""
		# operate on cloned plan
		mutable_s_need.fulfill(mutable_p)

		# add orderings
		self.OrderingGraph.addEdge(new_step, mutable_s_need)

		# add causal link
		#TODO uniquely identify the provider condition in the causal link
		c_link = self.CausalLinkGraph.addEdge(new_step, mutable_s_need, mutable_p)

		mutable_s_need.update_choices(self)

		# check if this link is threatened
		ignore_these = {mutable_s_need.ID, new_step.ID}
		# ignore_these = {mutable_s_need.stepnum, new_step.stepnum}
		for step in self.steps:
			if step.ID in ignore_these:
				continue
			if step.stepnum not in [tup[0] for tup in mutable_s_need.threat_map[mutable_p.ID]]:
				continue
			if self.OrderingGraph.isPath(mutable_s_need, step):
				continue
			# only for reuse case, otherwise this check is superfluous
			if self.OrderingGraph.isPath(step, new_step):
				continue
			self.potential_tclf.append(TCLF(step, c_link))

		# # check if adding this step threatens other causal links
		# for cl in self.CausalLinkGraph.edges:
		# 	if cl == c_link:
		# 		continue
		# 	if new_step.stepnum not in cl.sink.threat_map[cl.label.ID]:
		# 		continue
		# 	if self.OrderingGraph.isPath(new_step, cl.source):
		# 		continue
		# 	if self.OrderingGraph.isPath(cl.sink, new_step):
		# 		continue
		# 	self.flaws.insert(self, TCLF(new_step, cl))

	def resolve_with_decomp(self, new_step, mutable_s_need, mutable_p):
		raise DeprecationWarning("decomposition is no longer supported")
		d_i, d_f = new_step.dummy

		# operate on cloned plan
		# mutable_s_need = self[s_index]
		# mutable_p = mutable_s_need.preconds[p_index]
		mutable_s_need.fulfill(mutable_p)

		# add ordering
		self.OrderingGraph.addEdge(d_f, mutable_s_need)

		# add causal link
		c_link = self.CausalLinkGraph.addEdge(d_f, mutable_s_need, mutable_p)

		mutable_s_need.update_choices(self)

		# check if df -> s_need is threatened
		ignore_these = {mutable_s_need.ID, d_f.ID, d_i.ID}
		for step in self.steps:
			# reminder: existing steps are primitive

			if step.ID in ignore_these:
				continue

			### NOT SUFFICIENT: needs to not be a threat to any sub-step added... ###
			if step.stepnum not in mutable_s_need.threat_map[mutable_p.ID]:
				continue
			# check only for d_f, in case this step occurs between d_i and d_f
			if self.OrderingGraph.isPath(step, d_f):
				continue
			if self.OrderingGraph.isPath(mutable_s_need, step):
				continue
			self.flaws.insert(self, TCLF(step, c_link))

		# # check if adding this step threatens other causal links
		# for cl in self.CausalLinkGraph.edges:
		# 	# all causal links are between primitive steps
		# 	if cl == c_link:
		# 		continue
		# 	if new_step.stepnum not in cl.sink.threat_map[cl.label.ID]:
		# 		continue
		# 	if self.OrderingGraph.isPath(d_f, cl.source):
		# 		continue
		# 	if self.OrderingGraph.isPath(cl.sink, d_f):  # LOOK HERE TODO: DECIDE
		# 		continue
		# 	self.flaws.insert(self, TCLF(d_f, cl))
	def check_plan(self):
		"""Check if the plan is internally consistent and has no flaws"""
		if not self.isInternallyConsistent():
			print("Plan is not internally consistent")
			return False
		if len(self.flaws) > 0:
			print("Plan has open flaws")
			return False
		if not self.variableBindings.isInternallyConsistent():
			print("Variable bindings are not internally consistent")
			return False
		# check that all variables are ground
		if not self.variableBindings.is_fully_ground():
			print("Variable bindings are not ground")
			return False
		# check that all preconditions of steps are supported by causal links
		for step in self.steps:
			for pre in step.preconds:
				for edge in self.CausalLinkGraph.edges:
					if edge.label.ID == pre.ID and edge.sink.ID == step.ID:
						break
				else:
					# if we did not break, then no edge was found
					print(f"Step {step.ID} has unsupported precondition {pre.ID}")
					return False
		# check that no causal links are threatened
		for edge in self.CausalLinkGraph.edges:
			threatening_operators = [t[0] for t in  edge.sink.threat_map[edge.label.ID]]
			for step in self.steps:
				if not step.stepnum in threatening_operators:
					continue
				if not self.OrderingGraph.isPath(edge.source, step) or not self.OrderingGraph.isPath(step, edge.sink):
					# if the step is not in the path between source and sink, it cannot threaten the causal link
					continue
				for t in [t for t in edge.sink.threat_map[edge.label.ID] if t[0] == step.stepnum]:
					if self.variableBindings.is_unified(edge.label, step.effects[t[1]]):
						# if the edge is unified with the threatening effect, it threatens it
						print(f"Causal link {edge} is threatened by step {step.ID}")
						return False
		return True

	def to_json(self, filepath: str):
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
		def literal_to_dict(literal):
			return {
				"ID": str(literal.ID),
				"name": literal.name,
				"Args": [str(a) for a in literal.Args],
				"trudom": literal.truth,
			}

		def step_to_dict(step):
			return {
				"ID": str(step.ID),
				"schema": str(getattr(step, "schema", "")),
				"Args": [str(a) for a in getattr(step, "Args", [])],
				"preconds": [literal_to_dict(p) for p in getattr(step, "preconds", [])],
				"effects": [literal_to_dict(p) for p in getattr(step, "effects", [])],
				#"nonequals": [ne for ne in getattr(step, "nonequals", [])],
				#"reach_constraints": [rc for rc in getattr(step, "reach_constraints", [])],
				#"open_preconds": [str(p) for p in getattr(step, "open_preconds", [])],
				"stepnum": getattr(step, "stepnum", None),
				"height": getattr(step, "height", None),
			}

		def edge_to_dict(edge):
			return {
				"source": str(getattr(edge.source, "ID", "")),
				"sink": str(getattr(edge.sink, "ID", "")),
				"label": str(getattr(edge, "label", "")),
			}
		
		def variable_bindings_to_dict(vb):
			# Assuming VariableBindings has a method to serialize itself
			return vb.to_dict() if hasattr(vb, 'to_dict') else str(vb)

		plan_dict = {
			"ID": str(self.ID),
			"name": self.name,
			"cost": self.cost,
			"heuristic": self.heuristic,
			"depth": self.depth,
			"init_state": step_to_dict(self.dummy.init),
			"goal_state": step_to_dict(self.dummy.goal),
			"steps": [step_to_dict(step) for step in self.steps],
			"orderings": [edge_to_dict(edge) for edge in self.OrderingGraph.edges],
			"causal_links": [edge_to_dict(edge) for edge in self.CausalLinkGraph.edges],
			"variableBindings": variable_bindings_to_dict(self.variableBindings),
			# "flaws": str(self.flaws),
		}

		with open(filepath, "w") as f:
			json.dump(plan_dict, f, indent=2)
	
	@staticmethod
	def from_json(filepath: str) -> GPlan:
		"""
		Load a plan from a JSON file.

		Args:
			filepath (str): The path to the input JSON file.

		This method will populate the current instance with the data from the JSON file.
		"""
		def literal_from_dict(literal_dict):
			return GLiteral(
				pred_name=literal_dict["name"],
				arg_tup=(literal_dict["Args"][0], literal_dict["Args"][1]),
				trudom=literal_dict["trudom"],
				_id=literal_dict["ID"],
				is_static=False, # static property should no longer be used
				)

		with open(filepath, "r") as f:
			plan_dict = json.load(f)
		# Create a new GPlan instance
		init = Operator(
			operator=plan_dict["init_state"]["schema"],
			args=plan_dict["init_state"]["Args"],
			preconditions=[literal_from_dict(p) for p in plan_dict["init_state"]["preconds"]],
			effects=[literal_from_dict(p) for p in plan_dict["init_state"]["effects"]],
			stepnum=plan_dict["init_state"].get("stepnum", None),
			height=plan_dict["init_state"].get("height", 0),
			nonequals=[]
		)
		# Set the ID for the init state
		init.ID = plan_dict["init_state"]["ID"]
		goal = Operator(
			operator=plan_dict["goal_state"]["schema"],
			args=plan_dict["goal_state"]["Args"],
			preconditions=[literal_from_dict(p) for p in plan_dict["goal_state"]["preconds"]],
			effects=[literal_from_dict(p) for p in plan_dict["goal_state"]["effects"]],
			stepnum=plan_dict["goal_state"].get("stepnum", None),
			height=plan_dict["goal_state"].get("height", 0),
			nonequals=[]
		)
		# Set the ID for the goal state
		goal.ID = plan_dict["goal_state"]["ID"]
		# Create the GPlan instance
		plan = GPlan()

		plan.ID = plan_dict["ID"]
		plan.name = plan_dict["name"]
		plan.cost = plan_dict["cost"]
		plan.heuristic = plan_dict["heuristic"]
		plan.depth = plan_dict["depth"]
		# Populate steps
		for step_data in plan_dict["steps"]:
			if step_data["schema"] == "dummy_init" or step_data["schema"] == "dummy_goal":
				# Skip the dummy init and goal steps as they are already created
				continue
			step = Operator(
				operator=step_data["schema"],
				args=[str(a) for a in step_data["Args"]],
				preconditions=[literal_from_dict(p) for p in step_data["preconds"]],
				effects=[literal_from_dict(p) for p in step_data["effects"]],
				stepnum=step_data.get("stepnum", None),
				height=step_data.get("height", 0),
				nonequals=[]
			)
			step.ID = step_data["ID"]
			# Add the step to the plan
			plan.steps.append(step)
			plan.OrderingGraph.elements.add(step)
			plan.CausalLinkGraph.elements.add(step)
		# Populate orderings
		for edge_data in plan_dict["orderings"]:
			source = next((s for s in plan.OrderingGraph.elements if str(s.ID) == edge_data["source"]), None)
			sink = next((s for s in plan.OrderingGraph.elements if str(s.ID) == edge_data["sink"]), None)
			if source and sink:
				plan.OrderingGraph.addEdge(source, sink)
		# Populate causal links
		for edge_data in plan_dict["causal_links"]:
			source = next((s for s in plan.CausalLinkGraph.elements if str(s.ID) == edge_data["source"]), None)
			sink = next((s for s in plan.CausalLinkGraph.elements if str(s.ID) == edge_data["sink"]), None)
			label = GLiteral(ID=edge_data["label"])
			if source and sink:
				plan.CausalLinkGraph.addEdge(source, sink, label)
		# Populate variable bindings
		if "variableBindings" in plan_dict:
			vb_data = plan_dict["variableBindings"]
			# Assuming VariableBindings has a method to load from a dict
			plan.variableBindings = VariableBindings.from_dict(vb_data) if hasattr(VariableBindings, 'from_dict') else VariableBindings()
		else:
			plan.variableBindings = VariableBindings()
		return plan

	def __lt__(self, other):
		# if self.cost / (1 + math.log2(self.depth+1)) + self.heuristic != other.cost / (1 + math.log2(other.depth+1)) + other.heuristic:
		# 	return self.cost / (1 + math.log2(self.depth+1)) + self.heuristic < other.cost / (1 + math.log2(other.depth+1)) + other.heuristic
		# if self.cost - math.log2(self.depth+1) + self.heuristic != other.cost - math.log2(other.depth+1) + other.heuristic:
		# 	return self.cost - math.log2(self.depth+1) + self.heuristic < other.cost - math.log2(other.depth+1) + other.heuristic
		# if self.cost - self.depth + self.heuristic != other.cost - other.depth + other.heuristic:
		# 	return self.cost - self.depth + self.heuristic < other.cost - other.depth + other.heuristic
		if self.cost + self.heuristic != other.cost + other.heuristic:
			return (self.cost + self.heuristic) < (other.cost + other.heuristic)
		elif self.cost != other.cost:
			return self.cost < other.cost
		elif self.heuristic != other.heuristic:
			return self.heuristic < other.heuristic
		elif len(self.flaws) != len(other.flaws):
			return len(self.flaws) < len(other.flaws)
		elif len(self.CausalLinkGraph.edges) != len(other.CausalLinkGraph.edges):
			return len(self.CausalLinkGraph.edges) > len(other.CausalLinkGraph.edges)
		elif len(self.OrderingGraph.edges) != len(other.OrderingGraph.edges):
			return len(self.OrderingGraph.edges) > len(other.OrderingGraph.edges)
		elif sum([step.stepnum for step in self]) != sum([step.stepnum for step in other]):
			return sum([step.stepnum for step in self]) < sum([step.stepnum for step in other])
		else:
			return self.OrderingGraph < other.OrderingGraph
		
	def print(self):
		"""Display the plan steps in a human readable format. Substitute constants where possible"""
		print(f"Plan: {self}")

		for step in self.OrderingGraph.topoSort():
			args = str([self.variableBindings.repr_arg(arg) for arg in step.Args])
			print(str(step.schema) + args + '_{}'.format(str(step.ID)[-4:]))
		print('\n')

	def __str__(self):
		return self.name
		# return 'GPlan{} c={} h={}\t'.format(self.ID[-4:], self.cost, self.heuristic) + \
		# 		str(self.steps) + '\n' + str(self.OrderingGraph) + '\n' + str(self.CausalLinkGraph)

	def __repr__(self):
		return self.__str__()


def topoSort(ordering_graph):
	L =[]
	# ogr = copy.deepcopy(ordering_graph)
	ogr = OrderingGraph()
	init_dummy = Operator(name='init_dummy')
	ogr.elements.add(init_dummy)
	for elm in list(ordering_graph.elements):
		ogr.addOrdering(init_dummy, elm)
	S = {init_dummy}

	#L = list(graph.Steps)
	while len(S) > 0:
		n = S.pop()
		if n not in L:
			L.append(n)
		for m_edge in ogr.getIncidentEdges(n):
			ogr.edges.remove(m_edge)
			#if the sink has no other ordering sources, add it to the visited
			if len({edge for edge in ogr.getParents(m_edge.sink)}) == 0:
				S.add(m_edge.sink)
	return L

if __name__ == '__main__':
	pass