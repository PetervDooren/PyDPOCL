from PyPOCL.Ground_Compiler_Library.OrderingGraph import OrderingGraph, CausalLinkGraph
from PyPOCL.Ground_Compiler_Library.Flaws_unused import Flaw, FlawLib, TCLF
from PyPOCL.deterministic_uuid import duuid4
from PyPOCL.Ground_Compiler_Library.Element import Argument, Element, Operator, Literal
from PyPOCL.Ground_Compiler_Library.Graph import Edge
from PyPOCL.Ground_Compiler_Library.ElementGraph import ElementGraph
from PyPOCL.Ground_Compiler_Library.PlanElementGraph import Condition
import copy
from collections import namedtuple
# import json
# import jsonpickle
dummyTuple = namedtuple('dummyTuple', ['init', 'final'])

# class dummyTuple:
# 	def __init__(self, init, final):
# 		self.init = init
# 		self.final = final

class Operator:
	"""
	Read-Only Operator
	...

    Attributes
    ----------
    schema : string
        name of the operator
	Args : list(Argument)
        arguments of the operator of the action
	ID : uuid
        unique identifier of the step
	preconds : list(GLiteral)
        ?
	effects : list(GLiteral)
		?
	stepnum : int
        step number
	stepnumber : int
        step number (identical to stepnum?)
	nonequals : set(tuple(int, int))
		pairs of argument indices indicating which arguments may not codesignate.
	reach_constraints : set(tuple(int, int))
		pairs of argument indices indicating in_reach(1, 2) conditions on arguments.
	height : int
        ?
	sub_steps : list(?)
		?
	sub_orderings : OrderingGraph
        ?
	sub_links : CausalLinkGraph
        ?
	dummy : dummyTuple(Operator, Operator)
        ?
	depth : int
        ?
	cndts : list(int)
        list of antecedents of this step. given in terms of stepnumber
	condt_map : dict(uuid: list(tuple(int, int)))
        mapping between preconditions of this step and the (stepnr, effnr) in the planners steplist which can provide that. 
	threat_map : dict(uuid: list(tuple(int, int)))
        mapping between preconditions of this step and the (stepnr, effnr) in the planners steplist which threaten that precondition
	threats : list(int)
        list of steps which threaten this step
	instantiable : bool
        ?
	risks : list(?)
        ?
	choices : list(?)
        ?
	choice_map : dict(?:?)
        ?
	open_preconds : list(GLiteral)
        ?
	
    Methods
    -------
    setup():
	swap_setup(plan):
	swap_substeps():
	instantiate():
	fulfill():
	update_choices():
	is_cndt():
	is_threat():
	"""

	def __init__(self, operator, args, preconditions, effects, stepnum, height, nonequals):

		# READ-ONLY ATTRIBUTES #
		# schema refers to the name of the operator
		self.schema = operator
		# Args are Argument or Actor "Element" types
		self.Args = args
		# ID used as "instance ID"
		self.ID = duuid4()
		# preconds is a list of GCond
		self.preconds = [p for p in preconditions if p.name != 'in_reach'] # exclude in reach preconds
		self.effects = effects
		# stepnum is the ground step constructor type
		self.stepnum = stepnum
		self.stepnumber = stepnum
		# which arguments should be unequal
		self.nonequals = nonequals
		# in-reach constraints for the arms
		self.reach_constraints = [(p.Args[0], p.Args[1]) for p in preconditions if p.name == 'in_reach']
		# height is 0 when primitive
		self.height = height

		if height > 0:
			self.sub_steps = []
			self.sub_orderings = OrderingGraph()
			self.sub_links = CausalLinkGraph()
			self.dummy = dummyTuple(None, None)

		# depth starts at 0 and takes on value during planning
		self.depth = 0

		self.cndts = None
		self.cndt_map = None
		self.threat_map = None
		self.threats = None

		self.instantiable = True

		# INSTANCE ATTRIBUTES #
		# risks are number of threat instances
		self.risks = list()
		self.choices = list()
		# choices are instances of cndt antecedents
		self.choice_map = dict()
		# self.num_choices = 0
		# open preconditions which need causal link
		open_preconds = [p for p in self.preconds if p.name != 'in_reach']
		self.open_preconds = list(open_preconds)

	# def to_json(self):
	# 	return '{}:{}, {}'
		# return json.dumps(self, default=lambda o: o.__dict__, sort_keys=True, indent=4)

	# public methods #

	# def default(self):
	# 	def default(self, obj):
	# 		if hasattr(obj, 'to_json'):
	# 			return obj.to_json()
	# 		return json.JSONEncoder.default(self, obj)

	def setup(self, step_to_cndt, precond_to_effect, step_to_threat, precond_to_threat):
		"""
		setup the metadata of this operator. noting how it relates to other operators in a list.

		#TODO docstring deprecated
		:param step_to_cndt: dict of form Operator -> Operator^k such as D[stepnum] -> [cndt antecedent step nums]
		:param precond_to_effect: dict of form GLiteral -> Operator^k such as D[pre.ID] -> [cndt antecedent step nums]
		:param step_to_threat: dict of form GLiteral -> Operator^k such as D[stepnum] -> [cndt threat step nums]
		"""
		self.cndts = list(step_to_cndt[self.stepnum])
		self.cndt_map = {pre.ID: list(precond_to_effect[pre.ID]) for pre in self.preconds}
		self.threats = list(step_to_threat[self.stepnum])
		self.threat_map = {pre.ID: list(precond_to_threat[pre.ID]) for pre in self.preconds}

	def swap_setup(self, cndts, cndtmap, threats, threatmap):
		self.cndts = cndts
		self.cndt_map = cndtmap
		self.threats = threats
		self.threat_map = threatmap

	def swap_substeps(self, gsteps, decomp_step, num_GL_steps):
		change_dict = {step: gsteps[step.stepnumber].instantiate() for step in decomp_step.ground_subplan.Steps}
		self.sub_steps = list(change_dict.values())
		for edge in decomp_step.ground_subplan.OrderingGraph.edges:
			self.sub_orderings.addEdge(change_dict[edge.source], change_dict[edge.sink])
		for edge in decomp_step.ground_subplan.CausalLinkGraph.edges:
			new_sink = change_dict[edge.sink]
			# Condition.subgraph(subplan, edge.label)
			g_label = GLiteral(edge.label.name, edge.label.Args, edge.label.truth, -1, None)
			for p in new_sink.preconds:
				if p != g_label:
					continue
				self.sub_links.addEdge(change_dict[edge.source], new_sink, p)
				self.sub_orderings.addEdge(change_dict[edge.source], new_sink)
				new_sink.fulfill(p)
				break

		# set these babes to not be instantiable "fo' life"
		gsteps[decomp_step.sub_dummy_init.stepnumber].instantiable = False
		gsteps[decomp_step.sub_dummy_goal.stepnumber].instantiable = False
		init_step = gsteps[decomp_step.sub_dummy_init.stepnumber].instantiate()
		final_step = gsteps[decomp_step.sub_dummy_goal.stepnumber].instantiate()

		for step in self.sub_steps:
			self.sub_orderings.addEdge(init_step, step)
			self.sub_orderings.addEdge(step, final_step)
		self.sub_orderings.addEdge(init_step, final_step)

		# reconfigure init step to be top cndt for all steps and goal

		for step in self.sub_steps:
			for other_step in self.sub_steps:
				if other_step == step:
					continue
				prioritize_cndt(other_step, step)
			prioritize_cndt(init_step, step)
			prioritize_cndt(step, final_step)
		prioritize_cndt(init_step, final_step)

		# add init_step as top cndt for all

		self.dummy = dummyTuple(init_step, final_step)

	def instantiate(self, default_refresh=None, default_None_is_to_refresh_open_preconds=None):
		new_self = copy.deepcopy(self)
		new_self.ID = duuid4()
		arg_mapping = dict()
		for a in self.Args:
			arg_mapping[a.ID] = duuid4()
		# give args new ids. Due to linking will also change refs in preconds and effects
		for a in new_self.Args:
			a.ID = arg_mapping[a.ID]
		self.choice_map = dict()

		if default_refresh is None:
			self.risks = list()
			self.choices = list()

		if default_None_is_to_refresh_open_preconds is None:
			open_preconds = [p for p in self.preconds if p.name != 'in_reach']
			self.open_preconds = list(open_preconds)

		return new_self

	def fulfill(self, pre):
		if self.cndt_map is None:
			raise AttributeError('Cndt Map not found; run setup(xyz) first')
		if pre.ID not in self.cndt_map:
			raise ValueError('{} not found in cndt_map, id={}'.format(pre, pre.ID))
		if pre not in self.preconds:
			raise ValueError('{} found in cndt_map w/ id={}, but {} not found in preconds'.format(pre, pre.ID, pre))
		# remove precondition from open precond
		if pre in self.open_preconds:
			self.open_preconds.remove(pre)
		else:
			print('pre: {} not found in {} to remove, allowed in some cases'.format(pre, self))

	def update_choices(self, plan):
		choices = set()
		for pre in self.open_preconds:
			choice_nums = self.cndt_map[pre.ID]
			for step in plan.steps:
				if self.ID == step.ID:
					continue
				if plan.OrderingGraph.isPath(self, step):
					continue
				if step.stepnum in choice_nums:
					choices.add(step)
		self.choices = list(choices)

	def is_cndt(self, other):
		return other.stepnum in self.cndts

	def is_threat(self, other):
		return other.stepnum in self.threats

	# private hooks #

	def __hash__(self):
		return hash(self.ID)

	def __eq__(self, other):
		return self.ID == other.ID

	def __str__(self):
		args = str([arg.arg_name if arg.name is None else arg.name if not isinstance(arg, ElementGraph) else arg for arg in self.Args])
		return str(self.schema) + args + '_{}'.format(str(self.ID)[-4:])

	def __repr__(self):
		return self.__str__()


class GLiteral:
	"""
	A READ-ONLY Ground Literal / Condition
	"""
	def __init__(self, pred_name, arg_tup, trudom, _id, is_static):
		self.name = pred_name
		self.Args = list(arg_tup)
		self.truth = trudom
		self.ID = _id
		self.is_static = is_static

	def instantiate(self):
		return copy.deepcopy(self)

	# def to_json(self):
	# 	return json.dumps(self, default=lambda o: o.__dict__, sort_keys=True, indent=4)
	# 	# return 'u{name}: {}}'
	#
	# def from_json(self):
	# 	pass

	def __hash__(self):
		return hash(self.ID)

	def __len__(self):
		return len(self.Args)

	def __eq__(self, other):
		if not isinstance(other, GLiteral):
			return False
		return self.name == other.name and self.Args == other.Args and self.truth == other.truth

	def __repr__(self):
		args = str([arg.arg_name if arg.name is None else arg.name if not isinstance(arg, ElementGraph) else arg for arg in self.Args])
		t = ''
		if not self.truth:
			t = 'not-'
		return '{}{}'.format(t, self.name) + args


#@clock
def test(step, causal_link):
	for eff in step.Effects:
		if eff.isOpposite(causal_link.label):
			return True
	return False


def topoSort(plan):
	OG = copy.deepcopy(plan.OrderingGraph)
	L =[]
	S = {plan.initial_dummy_step}
	while len(S) > 0:
		n = S.pop()
		L.append(n)
		for m_edge in OG.getIncidentEdges(n):
			OG.edges.remove(m_edge)
			if len({edge for edge in OG.getParents(m_edge.sink)}) == 0:
				S.add(m_edge.sink)
	return L


def checkHeight(listActions, height):
	for a in listActions:
		if a.height == height:
			return True
	return False


def prioritize_cndt(cndt, whose):
	if cndt.stepnum in whose.cndts:
		whose.cndts.remove(cndt.stepnum)
		whose.cndts.insert(0, cndt.stepnum)
		for pre in whose.preconds:
			if cndt.stepnum not in whose.cndt_map[pre.ID]:
				continue
			whose.cndt_map[pre.ID].remove(cndt.stepnum)
			whose.cndt_map[pre.ID].insert(0, cndt.stepnum)