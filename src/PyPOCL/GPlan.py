from __future__ import annotations
from PyPOCL.Ground_Compiler_Library.GElm import GLiteral, Operator
from uuid import UUID, uuid4
from PyPOCL.Flaws import FlawLib, OPF, TCLF
from PyPOCL.Ground_Compiler_Library.OrderingGraph import OrderingGraph, CausalLinkGraph
from PyPOCL.Ground_Compiler_Library.VariableBindings import VariableBindings
from PyPOCL.worldmodel import Domain, Problem
from shapely import within, Polygon
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
	domain: str
		name of the domain this plan is for
	problem: str
		name of the problem this plan is for
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
		self.domain = None
		self.problem = None
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
	def make_root_plan(domain: Domain, problem: Problem) -> GPlan:
		"""Helper function to create a root plan at the start of planning

		Args:
			dummy_init_constructor (_type_): _description_
			dummy_goal_constructor (_type_): _description_
			objects (set(Argument)): list of objects in the world
			object_types(defaultdict(str, set(str)))
		"""
		if domain.name != problem.domain:
			raise ValueError(f"Domain name {domain.name} does not match problem domain {problem.domain}")
		root_plan = GPlan()
		root_plan.domain = domain.name
		root_plan.problem = problem.name

		root_plan.dummy = dummyTuple(problem.init, problem.goal)

		root_plan.init = root_plan.dummy.init.preconds
		root_plan.goal = root_plan.dummy.goal.preconds
		root_plan.steps = [root_plan.dummy.init, root_plan.dummy.goal]
		# check if any existing steps are choices (instances of cndts of open conditions)
		root_plan.dummy.goal.update_choices(root_plan)
		# add required orderings
		root_plan.OrderingGraph.addOrdering(root_plan.dummy.init, root_plan.dummy.goal)

		# register parameters
		root_plan.variableBindings.set_objects(problem.objects, domain.object_types, problem.object_dimensions)
		root_plan.variableBindings.set_areas(problem.areas)
		# set base area
		base_areas = [a for a in problem.areas if a.name==problem.base_area]
		if len(base_areas) == 0:
			if len(problem.areas) == 0:
				print(f"Problem has defined no areas. Assuming problem is purely symbolic.")
			else:
				raise ValueError(f"Base area {problem.base_area} not found in problem areas {problem.areas}")
		else:
			root_plan.variableBindings.geometric_vb.set_base_area(base_areas[0])
		
		root_plan.variableBindings.set_reach(problem.robot_reach)

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
	
	def get(self, id):
		for i, s in enumerate(self.steps):
			if s.ID == id:
				return s
		for i, s in enumerate(self.steps):
			print('{} {} {}'.format(i, s, s.ID))
		raise ValueError('ID={} not found in plan {}'.format(id, self.name))

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
		for within_condition in new_step.effects:
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

	def resolve(self, provider, consumer, precondition):
		if provider.height > 0:
			self.resolve_with_decomp(provider, consumer, precondition)
		else:
			self.resolve_with_primitive(provider, consumer, precondition)

	def resolve_with_primitive(self, provider: Operator, consumer: Operator, precondition: GLiteral) -> None:
		"""resolve precondition of the consumer using an effect of the provider 

		Args:
			provider (Operator): action which provides the effect
			consumer (Operator): action which requires the effect for its precondition
			precondition (GLiteral): precondition of the consumer that is fulfilled by the provider's effect
		"""
		# operate on cloned plan
		consumer.fulfill(precondition)

		# add orderings
		self.OrderingGraph.addEdge(provider, consumer)

		# add causal link
		#TODO uniquely identify the provider condition in the causal link
		c_link = self.CausalLinkGraph.addEdge(provider, consumer, precondition)

		consumer.update_choices(self)

		# check if this link is threatened
		ignore_these = {consumer.ID, provider.ID}
		# ignore_these = {mutable_s_need.stepnum, new_step.stepnum}
		for step in self.steps:
			if step.ID in ignore_these:
				continue
			if step.stepnum not in [tup[0] for tup in consumer.threat_map[precondition.ID]]:
				continue
			if self.OrderingGraph.isPath(consumer, step):
				continue
			# only for reuse case, otherwise this check is superfluous
			if self.OrderingGraph.isPath(step, provider):
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

	def resolve_with_decomp(self, provider, consumer, precondition):
		raise DeprecationWarning("decomposition is no longer supported")
		d_i, d_f = provider.dummy

		# operate on cloned plan
		# mutable_s_need = self[s_index]
		# mutable_p = mutable_s_need.preconds[p_index]
		consumer.fulfill(precondition)

		# add ordering
		self.OrderingGraph.addEdge(d_f, consumer)

		# add causal link
		c_link = self.CausalLinkGraph.addEdge(d_f, consumer, precondition)

		consumer.update_choices(self)

		# check if df -> s_need is threatened
		ignore_these = {consumer.ID, d_f.ID, d_i.ID}
		for step in self.steps:
			# reminder: existing steps are primitive

			if step.ID in ignore_these:
				continue

			### NOT SUFFICIENT: needs to not be a threat to any sub-step added... ###
			if step.stepnum not in consumer.threat_map[precondition.ID]:
				continue
			# check only for d_f, in case this step occurs between d_i and d_f
			if self.OrderingGraph.isPath(step, d_f):
				continue
			if self.OrderingGraph.isPath(consumer, step):
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
		"""Check if the plan is complete and valid. Does not use the tracked flaws, but checks the plan structure directly."""
		if not self.isInternallyConsistent():
			print("Plan is not internally consistent")
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
				if step == edge.source or step == edge.sink:
					# if the step is the source or sink of the causal link, it cannot threaten it
					continue
				if not self.OrderingGraph.isPath(edge.source, step) or not self.OrderingGraph.isPath(step, edge.sink):
					# if the step is not in the path between source and sink, it cannot threaten the causal link
					continue
				for t in [t for t in edge.sink.threat_map[edge.label.ID] if t[0] == step.stepnum]:
					if self.variableBindings.is_unified(edge.label, step.effects[t[1]]):
						# if the edge is unified with the threatening effect, it threatens it
						print(f"Causal link {edge} is threatened by step {step.ID}")
						return False
		#TODO check that all causal links condition arguments are unified
		# cannot be done right now as causal links only contain info on the sink-precondition, not the source effect

		# check that place locations are large enough for the objects
		for area_id in self.variableBindings.geometric_vb.variables:
			placeloc = self.variableBindings.geometric_vb.placelocs[area_id]
			area = placeloc.area_assigned
			if area is None:
				print(f"Area {area_id} is not assigned")
				return False
			# check that the object is defined
			if placeloc.object_width == 0 or placeloc.object_length == 0:
				print(f"Area {area_id} has no object defined")
				return False
			# check that the object fits within the assigned area
			if area.area < placeloc.object_width * placeloc.object_length:
				print(f"Area {area_id} is too small for the object with width {placeloc.object_width} and length {placeloc.object_length}")
				return False

		# check that all reach constraints of steps are satisfied in the assigned area
		for step in self.steps:
			for rc in step.reach_constraints:
				# check that the reach constraint is satisfied in the assigned area
				area = self.variableBindings.geometric_vb.get_assigned_area(rc[0])
				if area is None:
					print(f"Step {step.ID} has reach constraint {rc} but no assigned area")
					return False
				robot_arg = self.variableBindings.symbolic_vb.get_const(rc[1])
				reach_area_arg = self.variableBindings.reach_areas[robot_arg]
				reach_area = self.variableBindings.geometric_vb.defined_areas[reach_area_arg]
				if not within(area, reach_area):
					print(f"Step {step.ID} has unsatisfied reach constraint {rc}")
					return False
		# check that all other areas that can occur simultaneously do not overlap
		
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
				"sink-precondition": str(getattr(edge.label, "ID", "")),
			}
		
		def variable_bindings_to_dict(vb):
			vb_dict = {"symbolic": {}, "geometric": {}}
			for a in vb.symbolic_vb.variables:
				vb_dict["symbolic"][str(a.ID)] = vb.symbolic_vb.get_const(a).name
			for a in vb.geometric_vb.variables:
				vb_dict["geometric"][str(a.ID)] = list(vb.geometric_vb.get_assigned_area(a).exterior.coords)
			return vb_dict

		plan_dict = {
			"ID": str(self.ID),
			"name": self.name,
			"domain": self.domain,
			"problem": self.problem,
			"solved": self.solved,
			"cost": self.cost,
			"heuristic": self.heuristic,
			"depth": self.depth,
			"init_state": step_to_dict(self.dummy.init),
			"goal_state": step_to_dict(self.dummy.goal),
			"steps": [step_to_dict(step) for step in self.steps],
			"orderings": [ordering_to_dict(edge) for edge in self.OrderingGraph.edges],
			"causal_links": [causal_link_to_dict(edge) for edge in self.CausalLinkGraph.edges],
			"variableBindings": variable_bindings_to_dict(self.variableBindings),
			# "flaws": str(self.flaws),
		}

		with open(filepath, "w") as f:
			json.dump(plan_dict, f, indent=2)
	
	@staticmethod
	def from_json(domain: Domain, problem: Problem, filepath: str) -> GPlan:
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
			sink_precondition_id = edge_data["sink-precondition"]
			source = plan.get(id_map["steps"][source_id])
			sink = plan.get(id_map["steps"][sink_id])
			sink_precondition = next((e for e in sink.preconds if e.ID == id_map["preconds"][sink_precondition_id]))
			if source and sink:
				plan.CausalLinkGraph.addEdge(source, sink, sink_precondition)
		
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