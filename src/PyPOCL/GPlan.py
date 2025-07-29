from __future__ import annotations
from PyPOCL.Ground_Compiler_Library.GElm import GLiteral, Operator
from PyPOCL.deterministic_uuid import duuid4
from PyPOCL.Flaws import FlawLib, OPF, TCLF, UGSV, UGGV
from PyPOCL.Ground_Compiler_Library.OrderingGraph import OrderingGraph, CausalLinkGraph
from PyPOCL.Ground_Compiler_Library.VariableBindings import VariableBindings
from PyPOCL.worldmodel import Domain, Problem
import copy
from collections import namedtuple


dummyTuple = namedtuple('dummyTuple', ['init', 'goal'])


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
		self.ID = duuid4()
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

		root_plan.init = root_plan.dummy.init.effects
		root_plan.goal = root_plan.dummy.goal.preconds
		root_plan.steps = [root_plan.dummy.init, root_plan.dummy.goal]
		# check if any existing steps are choices (instances of cndts of open conditions)
		root_plan.dummy.goal.update_choices(root_plan)
		# add required orderings
		root_plan.OrderingGraph.addOrdering(root_plan.dummy.init, root_plan.dummy.goal)

		# register parameters
		root_plan.variableBindings.set_objects(problem.objects, domain.object_types, problem.object_dimensions, problem.initial_positions)
		root_plan.variableBindings.set_areas(problem.areas)
		# set base area
		if problem.base_area is None:
			print(f"Problem has defined no areas. Assuming problem is purely symbolic.")
		else:
			root_plan.variableBindings.geometric_vb.set_base_area(problem.base_area)
		
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
		new_self.ID = duuid4()
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
		
		# add ungrounded variables for new step
		for arg in new_step.Args:
			if self.variableBindings.is_type(arg, 'area'):
				self.flaws.insert(self, UGGV(arg))
			elif self.variableBindings.is_type(arg, 'symbol'):
				self.flaws.insert(self, UGSV(arg))

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
			if new_step.stepnum in [tup[0] for tup in edge.sink.threat_map[edge.label.sink.ID]]:
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

	def resolve(self, provider: Operator, consumer: Operator, effect: GLiteral, precondition: GLiteral) -> None:
		"""resolve precondition of the consumer using an effect of the provider 

		Args:
			provider (Operator): action which provides the effect
			consumer (Operator): action which requires the effect for its precondition
			effect (GLiteral): effect of the provider that fulfills the precondition of the consumer
			precondition (GLiteral): precondition of the consumer that is fulfilled by the provider's effect
		"""
		if provider.height > 0:
			self.resolve_with_decomp(provider, consumer, effect, precondition)
		else:
			self.resolve_with_primitive(provider, consumer, effect, precondition)

	def resolve_with_primitive(self, provider: Operator, consumer: Operator, effect: GLiteral, precondition: GLiteral) -> None:
		"""resolve precondition of the consumer using an effect of the provider. Where the provider is a primitive step.

		Args:
			provider (Operator): action which provides the effect
			consumer (Operator): action which requires the effect for its precondition
			effect (GLiteral): effect of the provider that fulfills the precondition of the consumer
			precondition (GLiteral): precondition of the consumer that is fulfilled by the provider's effect
		"""
		# operate on cloned plan
		consumer.fulfill(precondition)

		# add orderings
		self.OrderingGraph.addEdge(provider, consumer)

		# add causal link
		c_link = self.CausalLinkGraph.addEdge(provider, consumer, effect, precondition)

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

	def resolve_with_decomp(self, provider: Operator, consumer: Operator, effect: GLiteral, precondition: GLiteral) -> None:
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

	def set_disjunctions(self, var):
		"""
		Add disjunctions to the geometric variable var representing a placement location such that they do not overlap.
		"""
		# Find all objects that are static
		static_objs = []
		for obj in self.variableBindings.objects:
			if self.variableBindings.is_type(obj, 'physical_item'):
				for causal_link in self.CausalLinkGraph.edges:
					if causal_link.label.source.name == "within":
						if obj == causal_link.label.source.Args[0]:
							break
				else:
					static_objs.append(obj)		
		for obj in static_objs:
			obj_area_arg = self.variableBindings.initial_positions[obj]
			self.variableBindings.geometric_vb.add_disjunction(var, obj_area_arg)

		# find the causal link that places the object at the goal location represented by var
		for causal_link in self.CausalLinkGraph.edges:
			if causal_link.label.source.name != "within":
				continue
			if causal_link.label.source.Args[1] == var:
				break
			if causal_link.label.sink.Args[1] == var:
				# var is a startarea. No disjunctions need to be set. #TODO check if correct
				return True
		else:
			raise LookupError(f"Could not find {var} in the causal links")
		
		# add disjunctions between var and moving object locations.
		object = causal_link.label.source.Args[0]

		for other_link in self.CausalLinkGraph.edges:
			if other_link.label.source.name != "within":
				continue
			other_object = other_link.label.source.Args[0]
			if other_object == object:
				continue
			other_loc = other_link.label.source.Args[1]

			# check if the causal links overlap.
			if self.OrderingGraph.isPath(causal_link.sink, other_link.source) or self.OrderingGraph.isPath(other_link.sink, causal_link.source):
				# One causal link is stricly before another. Therefore the location described in it cannot be occupied at the same time.
				continue
			if not self.variableBindings.geometric_vb.can_intersect(var, other_loc):
				# the areas do not overlap. No need to add an explicit disjunction.
				continue
			self.variableBindings.geometric_vb.add_disjunction(var, other_loc)	
		return True

	def set_disjunctions_all(self):
		"""
		Add disjunctions to the geometric variables representing placement locations such that they do not overlap.
		"""
		# Find all objects that are static
		static_objs = []
		for obj in self.variableBindings.objects:
			if self.variableBindings.is_type(obj, 'physical_item'):
				for causal_link in self.CausalLinkGraph.edges:
					if causal_link.label.source.name == "within":
						if obj == causal_link.label.source.Args[0]:
							break
				else:
					static_objs.append(obj)
		for var in self.variableBindings.geometric_vb.variables:
			for obj in static_objs:
				obj_area_arg = self.variableBindings.initial_positions[obj]
				self.variableBindings.geometric_vb.add_disjunction(var, obj_area_arg)

		# add disjunctions between moving object locations.
		for causal_link in self.CausalLinkGraph.edges:
			if causal_link.label.source.name != "within":
				continue
			object = causal_link.label.source.Args[0]
			sourceloc = causal_link.label.source.Args[1]

			for other_link in self.CausalLinkGraph.edges:
				if other_link.label.source.name != "within":
					continue
				other_object = other_link.label.source.Args[0]
				if other_object == object:
					continue
				other_loc = other_link.label.source.Args[1]

				# check if the causal links overlap.
				if self.OrderingGraph.isPath(causal_link.sink, other_link.source) or self.OrderingGraph.isPath(other_link.sink, causal_link.source):
					# One causal link is stricly before another. Therefore the location described in it cannot be occupied at the same time.
					continue
				self.variableBindings.geometric_vb.add_disjunction(sourceloc, other_loc)	
		return True
	
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