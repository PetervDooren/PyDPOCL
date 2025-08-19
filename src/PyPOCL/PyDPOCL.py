from typing import Set, List
from collections import namedtuple
from PyPOCL.GPlan import GPlan
from PyPOCL.Ground_Compiler_Library.GElm import GLiteral
from PyPOCL.Ground_Compiler_Library.find_moveable_obstacles import find_movable_obstacles
from PyPOCL.Flaws import Flaw, OPF, TCLF, GTF, GPTF, UGSV, UGGV, UGPV
from PyPOCL.worldmodel import Domain, Problem
from PyPOCL.deterministic_uuid import duuid4
import math
import graphviz
from heapq import heappush, heappop
import time

from PyPOCL.plan_utility import visualize_plan
import matplotlib.pyplot as plt

REPORT = 1
RRP = 0

# graphviz colors:
OPEN_NODE = 'cyan'
CLOSED_NODE = 'white'
LEAF_NODE = 'red'
GOAL_NODE = 'green'

PlanningReport = namedtuple("PlanningReport", ["planning_time", "expanded", "visited", "terminated", "plans_found"])

class Frontier:

	def __init__(self):
		self._frontier = []

	def __len__(self):
		return len(self._frontier)

	def __getitem__(self, position):
		return self._frontier[position]

	def pop(self):
		return heappop(self._frontier)

	def insert(self, plan):
		heappush(self._frontier, plan)

	def extend(self, itera):
		for item in itera:
			self.insert(item)

	def __repr__(self):
		k = 'frontier plans\n'
		for plan in self._frontier:
			'{}:{} c={} h={} steps:\n{}\n'.format(k, str(plan.ID), plan.cost, plan.heuristic, plan.steps)
		return k


class POCLPlanner:
	"""
	Plan space planner, only instantiate once per planner, starts with ground steps
    ...

    Attributes
    ----------
    gsteps : set(Operator)
        Grounded steps of the planning problem. The last two are the initial state and goal state respectively.
    ID : uuid
        unique identifier of the planner
    h_step_dict : dict(?,?)
        ???
	h_lit_dict : dict(GLiteral: float)
		cached mapping between predicates and their heuristic cost
	frontier : Frontier
		frontier of partial plans to be explored
	_h_visited : List(GLiteral)
		list of predicates that have been visited by the heurisitic function
	plan_num : int
		number of plans opened
	max_height : int
		maximum height of the operators in the planner

    Methods
    -------
    pop():
	insert(plan):
	solve():
	add_step():
	reuse_step():
	resolve_threat():
	
	Heuristic Methods
	-------
	h_condition():
	h_step():
	h_plan():
	h_subplan():
	"""

	def __init__(self, domain: Domain, problem: Problem, log=False, plangraph_name=None) -> None:
		"""construct planner

		Args:
			domain (Domain): domain of the planning problem
			problem (Problem): problem of the planning problem
		"""	
		self.ID = duuid4()
		self.log = log # defines log level
		self.save_plangraph = plangraph_name is not None # create a planning graph
		if self.save_plangraph:
			self.plangraph_name = plangraph_name
			self.dot = graphviz.Digraph()

		self.domain = domain
		self.problem = problem

		# get data from domain and problem
		steplist = domain.operators
		steplist.append(problem.init)  # add initial state
		steplist.append(problem.goal)  # add goal state
		self.gsteps = steplist
		self.h_step_dict = dict()
		self.h_lit_dict = dict()

		self._frontier = Frontier()
		self.plan_num = 0
		self.opened = 0 # number of opened plans
		root_plan = GPlan.make_root_plan(domain, problem)
		root_plan.log = log
		self.insert(root_plan)
		self._h_visited = []
		self.max_height = self.gsteps[-3].height

	# Private Hooks #

	def __len__(self):
		return len(self._frontier)

	def __getitem__(self, position):
		return self._frontier[position]

	# Methods #
	def pop(self) -> GPlan:
		return self._frontier.pop()

	def insert(self, plan: GPlan, parent_plan: GPlan=None, label=None) -> None:
		plan.heuristic = self.h_plan(plan)
		self.log_message('>\tadd plan to frontier: {} with cost {} and heuristic {}\n'.format(plan.name, plan.cost, plan.heuristic))
		if self.save_plangraph:
			self.dot.node(f"{plan.ID}", f"plan_{self.opened}", style="filled", fillcolor=OPEN_NODE)
			if parent_plan is not None:
				self.dot.edge(f"{parent_plan.ID}", f"{plan.ID}", label=label)
		self._frontier.insert(plan)
		self.plan_num += 1
		self.opened += 1

	# @clock
	def solve(self, k: int=4, cutoff: int=60) -> List[GPlan]:
		# find k solutions to problem

		completed = []
		expanded = 0
		leaves = 0
		tclf_visits = 0

		geometry_fig = plt.figure()

		t0 = time.time()
		t_report = time.time()
		print('k={}'.format(str(k)))
		print('time\texpanded\tvisited\tterminated\tdepth\tcost\ttrace')
		while len(self) > 0:
			if time.time() - t_report > 1: # report every second
				elapsed = time.time() - t0
				delay = str('%0.8f' % elapsed)
				print(f'{delay}\t{expanded}\t{self.opened}\t{leaves}')
				t_report = time.time()
			if cutoff > 0 and time.time() - t0 > cutoff:
				elapsed = time.time() - t0
				delay = str('%0.8f' % elapsed)
				print(f'timedout: {delay}\t {expanded}\t{self.opened}\t{leaves}')

				if self.save_plangraph:
						self.dot.render(filename=f"{self.plangraph_name}.dot", outfile=f"{self.plangraph_name}.svg")

				planning_report = PlanningReport(delay, expanded, self.opened, leaves, len(completed))
				return [], planning_report

			plan = self.pop()
			expanded += 1
			self.plan_num = 0 # reset branch counter

			if self.save_plangraph:
				self.dot.node(f"{plan.ID}", f"visited_{expanded}", style="filled", fillcolor=CLOSED_NODE)

			if not plan.isInternallyConsistent():
				# if plan.name[-3] == 'a':
				# 	print('stop')
				if self.save_plangraph:
					self.dot.node(f"{plan.ID}", f"leaf_{leaves}", style="filled", fillcolor=LEAF_NODE)

				self.log_message('prune {}'.format(plan.name))
				leaves += 1
				continue

			# debugging:
			# plan_schemata = Counter(step.schema for step in plan.steps)
			# Counter(plan_schemata)
			# for item, value in plan_schemata.items():
			# 	if value > 3:
			# 		print('check here')
			# if len(plan_schemata) > len(set(plan_schemata)):
			# 	print('check here')

			self.log_message('Plan {} selected cost={} heuristic={}'.format(plan.name, plan.cost, plan.heuristic))
			if self.log:
				plan.print()
				#visualize_plan(plan, fig=geometry_fig)

			plan.update_flaws()

			if len(plan.flaws) == 0:
				plan.solved = True
				# success
				elapsed = time.time() - t0
				delay = str('%0.8f' % elapsed)
				completed.append(plan)

				trace = math.floor(len(plan.name.split('['))/2)
				print('{}\t{}\t{}\t{}\t{}\t{}\t{}'.format(delay, expanded, len(self) + expanded, leaves, str(plan.depth), plan.cost, trace))
				if REPORT:
					print(f"solution {len(completed)} found at {expanded} nodes expanded and {len(self)+expanded} nodes visited and {leaves} branches terminated")
					plan.print()
				if self.save_plangraph:
					self.dot.node(f"{plan.ID}", f"goal_{len(completed)}", style="filled", fillcolor=GOAL_NODE)

				if len(completed) == k:
					if self.save_plangraph:
						self.dot.render(filename=f"{self.plangraph_name}.dot", outfile=f"{self.plangraph_name}.svg")
					planning_report = PlanningReport(delay, expanded, len(self)+expanded, leaves, len(completed))
					return completed, planning_report
				continue

			# Select Flaw
			flaw = plan.flaws.next()
			plan.name += '[' + str(flaw.flaw_type)[0] + ']'
			self.log_message('{} selected : {}\n'.format(flaw.name, flaw))

			if isinstance(flaw, TCLF):
				tclf_visits += 1
				self.resolve_threat(plan, flaw)
			elif isinstance(flaw, GTF):
				self.resolve_geometric_threat(plan, flaw)
			elif isinstance(flaw, GPTF):
				self.resolve_geometric_path_threat(plan, flaw)
			elif isinstance(flaw, UGSV):
				if not self.ground_variable(plan, flaw):
					if self.save_plangraph:
						self.dot.node(f"{plan.ID}", f"leaf_{leaves}", style="filled", fillcolor=LEAF_NODE)
					self.log_message(f"could not ground symbolic arg {flaw.arg}. pruning")
					leaves += 1
			elif isinstance(flaw, UGGV):
				if not self.ground_geometric_variable(plan, flaw):
					if self.save_plangraph:
						self.dot.node(f"{plan.ID}", f"leaf_{leaves}", style="filled", fillcolor=LEAF_NODE)
					self.log_message(f"could not resolve geometric arg {flaw.arg}. pruning")
					leaves += 1
			elif isinstance(flaw, UGPV):
				if not self.ground_path_variable(plan, flaw):
					if self.save_plangraph:
						self.dot.node(f"{plan.ID}", f"leaf_{leaves}", style="filled", fillcolor=LEAF_NODE)
					self.log_message(f"could not ground symbolic arg {flaw.arg}. pruning")
					leaves += 1
			elif isinstance(flaw, OPF):
				self.add_step(plan, flaw)
				self.reuse_step(plan, flaw)
				self.ground_in_init(plan, flaw)
			else:
				raise ValueError(f"Unknown flaw type. Dont know how to resolve: flaw: {flaw} of type {type(flaw)}")

		# frontier is empty
		print(f'FAIL: No more plans to visit with {expanded} nodes expanded')
		elapsed = time.time() - t0
		delay = str('%0.8f' % elapsed)
		planning_report = PlanningReport(delay, expanded, len(self)+expanded, leaves, len(completed))
		return [], planning_report

	def add_step(self, plan: GPlan, flaw: Flaw) -> None:
		"""add a new step to resolve a flaw in the plan. Will add one or more plans to the

		Args:
			plan (GPlan): plan which is incremented
			flaw (Flaw): flaw of that plan to be resolved (must be open precondition)
		"""

		consumer, precondition = flaw.flaw
		candidates = consumer.cndt_map[precondition.ID]

		if len(candidates) == 0:
			return

		# need indices
		consumer_index = plan.index(consumer)
		precondition_index = consumer.preconds.index(precondition)
		for candidate_operator, candidate_effect in candidates:

			if RRP:
				# the Recursive Repair Policty: only let steps with <= height repair open conditions.
				if self.gsteps[candidate_operator].height > flaw.level:
					continue

			# cannot add a step which is the inital step
			if not self.gsteps[candidate_operator].instantiable:
				continue
			# clone plan and new step

			new_plan = plan.instantiate(str(self.plan_num) + '[a] ')

			# use indices befoer inserting new steps
			new_plan_consumer = new_plan[consumer_index]
			new_plan_precondition = new_plan_consumer.preconds[precondition_index]

			# instantiate new step
			new_step = self.gsteps[candidate_operator].instantiate()

			# pass depth to new Added step.
			new_step.depth = new_plan_consumer.depth

			# recursively insert new step and substeps into plan, adding orderings and flaws
			new_plan.insert(new_step)

			# check that provided condition can be codesignated with the required(consumed) condition
			new_plan_effect = new_step.effects[candidate_effect]
			if not new_plan.variableBindings.unify(new_plan_effect, new_plan_precondition):
				continue

			self.log_message(f'Add step {new_step} to plan {new_plan.name} to satisfy precondition {new_plan_precondition} of {new_plan_consumer} with effect {new_plan_effect}.')

			# resolve s_need with the new step
			new_plan.resolve(new_step, new_plan_consumer, new_plan_effect, new_plan_precondition)

			new_plan.cost += ((self.max_height*self.max_height)+1) - (new_step.height*new_step.height)
			# new_plan.cost += self.max_height + 1 - new_step.height
			# new_plan.cost += 1
			# self.max_height + 1 - new_step.height

			# insert our new mutated plan into the frontier
			self.insert(new_plan, plan, 'OPF: add_step')

	def reuse_step(self, plan: GPlan, flaw: Flaw) -> None:
		consumer, precondition = flaw.flaw

		choices = []
		for step in plan.steps:
			if step.schema == 'dummy_init':
				continue
			if step.stepnum in [tup[0] for tup in consumer.cndt_map[precondition.ID]] and not consumer.ID == step.ID and not plan.OrderingGraph.isPath(consumer, step):
				for stepnr, effnr in consumer.cndt_map[precondition.ID]:
					if stepnr == step.stepnum:
						choices.append((step, effnr))
		if len(choices) == 0:
			return

		# consumer indices
		consumer_index = plan.index(consumer)
		precondition_index = consumer.preconds.index(precondition)
		for candidate_action, effect_nr in choices:
			# clone plan and new step
			new_plan = plan.instantiate(str(self.plan_num) + '[r] ')

			# use indices before inserting new steps
			new_plan_consumer = new_plan[consumer_index]
			new_plan_precondition = new_plan_consumer.preconds[precondition_index]

			# use index to find old step
			new_plan_provider = new_plan.steps[plan.index(candidate_action)]

			# check that provided condition can be codesignated with the required(consumed) condition
			new_plan_effect = new_plan_provider.effects[effect_nr]
			if not new_plan.variableBindings.unify(new_plan_effect, new_plan_precondition):
				continue
			self.log_message(f'Reuse step {new_plan_provider} to plan {new_plan.name} to satisfy precondition {new_plan_precondition} of {consumer} with effect {new_plan_effect}.')
			# resolve open condition with old step
			new_plan.resolve(new_plan_provider, new_plan_consumer, new_plan_effect, new_plan_precondition)

			# insert mutated plan into frontier
			self.insert(new_plan, plan, 'OPF: reuse step')

	def ground_in_init(self, plan: GPlan, flaw: Flaw) -> None:
		"""Similar to reuse step. But specifically for grounding an unsupported condition in the initial state.

		Args:
			plan (GPlan): root plan to iterate on
			flaw (Flaw): flaw to resolve
		"""
		consumer, precondition = flaw.flaw

		choices = []
		step = plan.dummy.init
		if step.stepnum in [tup[0] for tup in consumer.cndt_map[precondition.ID]]:
			for stepnr, effnr in consumer.cndt_map[precondition.ID]:
				if stepnr == step.stepnum:
					choices.append((step, effnr))
		if len(choices) == 0:
			return

		# consumer indices
		consumer_index = plan.index(consumer)
		precondition_index = consumer.preconds.index(precondition)
		if precondition.name == "within":
			# clone plan and new step
			new_plan = plan.instantiate(str(self.plan_num) + '[r] ')

			# use indices before inserting new steps
			new_plan_consumer = new_plan[consumer_index]
			new_plan_precondition = new_plan_consumer.preconds[precondition_index]

			# use index to find old step
			new_plan_provider = new_plan.dummy.init

			# find the initial position of the object
			obj_var = precondition.Args[0]
			obj = new_plan.variableBindings.symbolic_vb.get_const(obj_var)
			if obj is None:
				print("Fuck! the object is not grounded yet!")
			init_pos = new_plan.variableBindings.initial_positions[obj]
			init_pos_effect = next((lit for lit in new_plan.init if lit.Args[0] == obj and lit.Args[1] == init_pos and lit.name == "within"))

			# check that provided condition can be codesignated with the required(consumed) condition
			if new_plan.variableBindings.unify(init_pos_effect, new_plan_precondition):
				self.log_message(f'Ground {new_plan_precondition} of {consumer} in the initial state with effect {init_pos_effect}.')
				# resolve open condition with old step
				new_plan.resolve(new_plan_provider, new_plan_consumer, init_pos_effect, new_plan_precondition)

				# insert mutated plan into frontier
				self.insert(new_plan, plan, 'OPF: reuse init')
		else:
			for candidate_action, effect_nr in choices:
				# clone plan and new step
				new_plan = plan.instantiate(str(self.plan_num) + '[r] ')

				# use indices before inserting new steps
				new_plan_consumer = new_plan[consumer_index]
				new_plan_precondition = new_plan_consumer.preconds[precondition_index]

				# use index to find old step
				new_plan_provider = new_plan.steps[plan.index(candidate_action)]

				# check that provided condition can be codesignated with the required(consumed) condition
				new_plan_effect = new_plan_provider.effects[effect_nr]
				if not new_plan.variableBindings.unify(new_plan_effect, new_plan_precondition):
					continue
				self.log_message(f'Ground {new_plan_precondition} of {consumer} in the initial state with effect {new_plan_effect}.')
				# resolve open condition with old step
				new_plan.resolve(new_plan_provider, new_plan_consumer, new_plan_effect, new_plan_precondition)

				# insert mutated plan into frontier
				self.insert(new_plan, plan, 'OPF: reuse init')

	def resolve_threat(self, plan: GPlan, tclf: TCLF) -> None:
		threat_index = plan.index(tclf.threat)
		src_index = plan.index(tclf.link.source)
		snk_index = plan.index(tclf.link.sink)

		# Promotion
		new_plan = plan.instantiate(str(self.plan_num)+ '[tp] ')
		threat = new_plan[threat_index]
		sink = new_plan[snk_index]
		new_plan.OrderingGraph.addEdge(sink, threat)
		if hasattr(threat, 'sibling'):
			new_plan.OrderingGraph.addEdge(sink, threat.sibling)
		if hasattr(sink, 'sibling'):
			new_plan.OrderingGraph.addEdge(sink.sibling, threat)
		threat.update_choices(new_plan)
		self.insert(new_plan, plan, 'TCLF: promote')
		self.log_message('promote {} in front of {} in plan {}'.format(threat, sink, new_plan.name))


		# Demotion
		new_plan = plan.instantiate(str(self.plan_num) + '[td] ')
		threat = new_plan[threat_index]
		source = new_plan[src_index]
		new_plan.OrderingGraph.addEdge(threat, source)
		if hasattr(threat, 'sibling'):
			new_plan.OrderingGraph.addEdge(source, threat.sibling)
		if hasattr(source, 'sibling'):
			new_plan.OrderingGraph.addEdge(source.sibling, threat)
		threat.update_choices(new_plan)
		self.insert(new_plan, plan, 'TCLF: demote')
		self.log_message('demotion {} behind {} in plan {}'.format(threat, source, new_plan.name))
	
	def resolve_geometric_threat(self, plan: GPlan, gtf: GTF) -> None:
		# find out if the threatening area is a static object or not.
		threat_source, threat_sink = GPlan.find_place_in_plan(plan, gtf.threat)
		
		area_is_static = threat_source == plan.dummy.init and threat_sink == plan.dummy.goal
		if area_is_static:
			self.resolve_geometric_threat_static(plan, gtf)
		else:
			self.resolve_geometric_threat_dynamic(plan, gtf, threat_source, threat_sink)

	def resolve_geometric_threat_static(self, plan: GPlan, gtf: GTF) -> None:
		threatened_area = gtf.area
		# find the causal link corresponding to the threatened area
		source, sink = GPlan.find_place_in_plan(plan, threatened_area)
		consumer_index = plan.index(source)
		precondition_index = None # free (yadayada) is not an explicit condition

		threatening_area = gtf.threat
		for obj, area in plan.variableBindings.initial_positions.items():
			if area == threatening_area:
				break
		else:
			raise LookupError(f"Could not find object belonging to initial area {threatening_area}")
		threatening_obj = obj

		# add a step to the plan to move the object
		candidates = []
		for o in self.gsteps:
			# cannot add a step which is the inital step
			if not o.instantiable:
				continue
			for e in o.effects:
				if e.name == 'within' and e.truth:
					candidates.append((o.stepnum, o.effects.index(e)))

		if len(candidates) == 0:
			return

		# need indices
		for candidate_operator, candidate_effect in candidates:
			# clone plan and new step

			new_plan = plan.instantiate(str(self.plan_num) + '[a] ')

			# use indices befoer inserting new steps
			new_plan_consumer = new_plan[consumer_index]
			#new_plan_precondition = new_plan_consumer.preconds[precondition_index]

			# instantiate new step
			new_step = self.gsteps[candidate_operator].instantiate()

			# pass depth to new Added step.
			new_step.depth = new_plan_consumer.depth

			# recursively insert new step and substeps into plan, adding orderings and flaws
			new_plan.insert(new_step)

			new_plan.OrderingGraph.addEdge(new_step, new_plan_consumer)
			#new_plan.CausalLinkGraph.addEdge(new_step, new_plan_consumer, new_step.effects[candidate_effect], None)
			new_goal_object = new_step.effects[candidate_effect].Args[0]
			new_goal_area = new_step.effects[candidate_effect].Args[1]
			new_plan.variableBindings.add_codesignation(new_goal_object, threatening_obj)
			new_plan.variableBindings.geometric_vb.add_disjunction(threatened_area, new_goal_area)

			self.log_message(f'Add step {new_step} to plan {new_plan.name} to satisfy area conflict of {new_plan_consumer}.')

			new_plan.cost += ((self.max_height*self.max_height)+1) - (new_step.height*new_step.height)

			# insert our new mutated plan into the frontier
			self.insert(new_plan, plan, 'GTF: add step')

	def resolve_geometric_threat_dynamic(self, plan: GPlan, gtf: GTF, threat_source, threat_sink) -> None:
		threatened_area = gtf.area
		# find the causal link corresponding to the threatened area
		src, snk = GPlan.find_place_in_plan(plan, threatened_area)
		src_index_1 = plan.index(src)
		snk_index_1 = plan.index(snk)
		src_index_2 = plan.index(threat_source)
		snk_index_2 = plan.index(threat_sink)

		# Promotion place 2 before 1
		new_plan = plan.instantiate(str(self.plan_num)+ '[tp] ')
		source_1 = new_plan[src_index_1]
		sink_2 = new_plan[snk_index_2]
		new_plan.OrderingGraph.addEdge(sink_2, source_1)
		if hasattr(source_1, 'sibling'):
			new_plan.OrderingGraph.addEdge(sink_2, source_1.sibling)
		if hasattr(sink_2, 'sibling'):
			new_plan.OrderingGraph.addEdge(sink_2.sibling, source_1)
		sink_2.update_choices(new_plan) # check if needed and/or if the same should be done for source_1
		self.insert(new_plan, plan, 'GTF: promote')
		self.log_message('promote {} in front of {} in plan {}'.format(sink_2, source_1, new_plan.name))


		# Demotion place 2 after 1
		new_plan = plan.instantiate(str(self.plan_num) + '[td] ')
		source_2 = new_plan[src_index_2]
		sink_1 = new_plan[snk_index_1]
		new_plan.OrderingGraph.addEdge(sink_1, source_2)
		if hasattr(source_2, 'sibling'):
			new_plan.OrderingGraph.addEdge(sink_1, source_2.sibling)
		if hasattr(sink_1, 'sibling'):
			new_plan.OrderingGraph.addEdge(sink_1.sibling, source_2)
		sink_1.update_choices(new_plan) #TODO check if needed?
		self.insert(new_plan, plan, 'GTF: demote')
		self.log_message('demotion {} behind {} in plan {}'.format(sink_1, source_2, new_plan.name))
	
	def resolve_geometric_path_threat(self, plan: GPlan, gptf: GPTF) -> None:
		# find out if the threatening area is a static object or not.
		threat_source, threat_sink = GPlan.find_place_in_plan(plan, gptf.threat)
		
		area_is_static = threat_source == plan.dummy.init and threat_sink == plan.dummy.goal
		if area_is_static:
			self.resolve_geometric_path_threat_static(plan, gptf)
		else:
			self.resolve_geometric_path_threat_dynamic(plan, gptf, threat_source, threat_sink)
	
	def resolve_geometric_path_threat_static(self, plan: GPlan, gptf: GTF) -> None:
		threatened_path = gptf.path
		# find the step corresponding to the threatened path
		for step in plan.steps:
			if threatened_path in step.Args:
				break
		else:
			print(f"Could not find path {threatened_path} in plan")

		consumer_index = plan.index(step)
		precondition_index = None # free (yadayada) is not an explicit condition

		threatening_area = gptf.threat
		for obj, area in plan.variableBindings.initial_positions.items():
			if area == threatening_area:
				break
		else:
			raise LookupError(f"Could not find object belonging to initial area {threatening_area}")
		threatening_obj = obj

		# add a step to the plan to move the object
		candidates = []
		for o in self.gsteps:
			# cannot add a step which is the inital step
			if not o.instantiable:
				continue
			for e in o.effects:
				if e.name == 'within' and e.truth:
					candidates.append((o.stepnum, o.effects.index(e)))

		if len(candidates) == 0:
			return

		# need indices
		for candidate_operator, candidate_effect in candidates:
			# clone plan and new step

			new_plan = plan.instantiate(str(self.plan_num) + '[a] ')

			# use indices befoer inserting new steps
			new_plan_consumer = new_plan[consumer_index]
			#new_plan_precondition = new_plan_consumer.preconds[precondition_index]

			# instantiate new step
			new_step = self.gsteps[candidate_operator].instantiate()

			# pass depth to new Added step.
			new_step.depth = new_plan_consumer.depth

			# recursively insert new step and substeps into plan, adding orderings and flaws
			new_plan.insert(new_step)

			new_plan.OrderingGraph.addEdge(new_step, new_plan_consumer)
			#new_plan.CausalLinkGraph.addEdge(new_step, new_plan_consumer, new_step.effects[candidate_effect], None)
			new_goal_object = new_step.effects[candidate_effect].Args[0]
			new_goal_area = new_step.effects[candidate_effect].Args[1]
			new_plan.variableBindings.add_codesignation(new_goal_object, threatening_obj)
			new_plan.variableBindings.geometric_vb.add_disjunction(threatened_path, new_goal_area)

			self.log_message(f'Add step {new_step} to plan {new_plan.name} to satisfy area conflict of {new_plan_consumer}.')

			new_plan.cost += ((self.max_height*self.max_height)+1) - (new_step.height*new_step.height)

			# insert our new mutated plan into the frontier
			self.insert(new_plan, plan, 'GPTF: add step')

	def resolve_geometric_path_threat_dynamic(self, plan: GPlan, gptf: GTF, threat_source, threat_sink) -> None:
		threatened_path = gptf.path
		# find the step corresponding to the threatened path
		for step in plan.steps:
			if threatened_path in step.Args:
				break
		else:
			print(f"Could not find path {threatened_path} in plan")
		step_index_1 = plan.index(step)
		src_index_2 = plan.index(threat_source)
		snk_index_2 = plan.index(threat_sink)

		# Promotion place 2 before 1
		new_plan = plan.instantiate(str(self.plan_num)+ '[tp] ')
		source_1 = new_plan[step_index_1]
		sink_2 = new_plan[snk_index_2]
		new_plan.OrderingGraph.addEdge(sink_2, source_1)
		if hasattr(source_1, 'sibling'):
			new_plan.OrderingGraph.addEdge(sink_2, source_1.sibling)
		if hasattr(sink_2, 'sibling'):
			new_plan.OrderingGraph.addEdge(sink_2.sibling, source_1)
		sink_2.update_choices(new_plan) # check if needed and/or if the same should be done for source_1
		self.insert(new_plan, plan, 'GTF: promote')
		self.log_message('promote {} in front of {} in plan {}'.format(sink_2, source_1, new_plan.name))


		# Demotion place 2 after 1
		new_plan = plan.instantiate(str(self.plan_num) + '[td] ')
		source_2 = new_plan[src_index_2]
		sink_1 = new_plan[step_index_1]
		new_plan.OrderingGraph.addEdge(sink_1, source_2)
		if hasattr(source_2, 'sibling'):
			new_plan.OrderingGraph.addEdge(sink_1, source_2.sibling)
		if hasattr(sink_1, 'sibling'):
			new_plan.OrderingGraph.addEdge(sink_1.sibling, source_2)
		sink_1.update_choices(new_plan) #TODO check if needed?
		self.insert(new_plan, plan, 'GTF: demote')
		self.log_message('demotion {} behind {} in plan {}'.format(sink_1, source_2, new_plan.name))
	
	def ground_variable(self, plan: GPlan, flaw: UGSV):
		""" create branch plans by grounding a symbolic variable.

		Args:
			plan (GPlan): _description_
			flaw (UGSV): _description_

		Returns:
			bool: True if at least one branch was made.
		"""
		plan.name += '[ugsv]'

		# ground the symbolic variable
		arg = flaw.arg
		if plan.variableBindings.is_ground(arg):
			obj = plan.variableBindings.symbolic_vb.get_const(arg)
			self.log_message(f'Variable {arg} is already ground to object {obj}.')
			new_plan = plan.instantiate(str(self.plan_num) + '[ag] ')
			self.insert(new_plan, plan, 'UGSV: already ground')
			return True

		grounding_success = False # should be True if at least one branch is created
		for obj in plan.variableBindings.objects:
			if not plan.variableBindings.can_codesignate(arg, obj):
				continue
			# add potential plan with codesignation
			new_plan = plan.instantiate(str(self.plan_num) + '[g] ')
			if not new_plan.variableBindings.add_codesignation(arg, obj): # due to the geometric consequences of grounding variables can_codesignate is no longer complete.
				continue
			self.log_message(f'Grounding variable {arg} to object {obj}.')
			self.insert(new_plan, plan, f'UGSV: ground variable')
			grounding_success = True
		return grounding_success
		
	def ground_geometric_variable(self, plan: GPlan, flaw: UGSV) -> bool:
		""" create branch plans by grounding a geometric variable. Will create only one branch at most.

		Args:
			plan (GPlan): _description_
			flaw (UGSV): _description_

		Returns:
			bool: True if at least one branch was made.
		"""
		plan.name += '[uggv]'

		# ground the geometric variable
		arg = flaw.arg
		if plan.variableBindings.geometric_vb.is_ground(arg):
			self.log_message(f'Variable {arg} is already ground. This should not happen!')
			new_plan = plan.instantiate(str(self.plan_num) + '[ag] ')
			self.insert(new_plan, plan, 'UGGV: already ground')
			return True
		new_plan = plan.instantiate(str(self.plan_num) + '[g] ')
		new_plan.set_disjunctions(arg)
		if new_plan.variableBindings.geometric_vb.resolve(arg):
			self.log_message(f'Grounding variable {arg}.')
			self.insert(new_plan, plan, 'UGGV: ground')
			return True
		else:
			offending_areas = new_plan.variableBindings.geometric_vb.disjunctions[arg]
			self.log_message(f"could not ground variable {arg}, it conflicts with areas: {offending_areas}")
			new_new_plan = plan.instantiate(str(self.plan_num) + '[g] ')
			if new_new_plan.variableBindings.geometric_vb.resolve(arg):
				moved_areas = []
				for area in offending_areas:
					if new_new_plan.variableBindings.is_type(area, 'area'):
						if new_new_plan.variableBindings.geometric_vb.can_intersect(area, arg):
							new_new_plan.flaws.insert(new_new_plan, GTF(area, arg))
							moved_areas.append(area)
					elif new_new_plan.variableBindings.is_type(area, 'path'):
						if new_new_plan.variableBindings.geometric_vb.intersect(area, arg):
							return False # there is no way to resolve this

				self.log_message(f"grounding variable {arg}. Moving areas {moved_areas}")
				self.insert(new_new_plan, plan, 'UGGV: ground with threats')
				return True
			return False
	
	def ground_path_variable(self, plan: GPlan, flaw: UGPV):
		""" create branch plans by grounding a path variable.

		Args:
			plan (GPlan): _description_
			flaw (UGPV): _description_

		Returns:
			bool: True if at least one branch was made.
		"""
		plan.name += '[ugpv]'

		arg = flaw.arg
		new_plan = plan.instantiate(str(self.plan_num) + '[g] ')
		new_plan.set_disjunctions_path(arg)
		if new_plan.variableBindings.geometric_vb.resolve_path(arg):
			self.log_message(f'Grounding path variable {arg}.')
			self.insert(new_plan, plan, 'UGPV: ground')
			return True
		movable_obstacle_sets = find_movable_obstacles(new_plan, arg)
		if len(movable_obstacle_sets) < 1:
			print(f"Could not find objects to remove for {arg}. This should not be possible")
			# repeat methods for debugging
			new_plan.variableBindings.geometric_vb.resolve_path(arg)
			movable_obstacle_sets = find_movable_obstacles(new_plan, arg)
			return False
		for obst_set in movable_obstacle_sets:
			new_new_plan = new_plan.instantiate(str(self.plan_num) + '[g] ')
			for obst in obst_set:
				new_new_plan.variableBindings.geometric_vb.remove_disjunction(arg, obst)
				new_new_plan.flaws.insert(new_new_plan, GPTF(obst, arg))
			if not new_new_plan.variableBindings.geometric_vb.resolve_path(arg):
				print(f"Could not ground {arg} after removing objects. This should not happen")
				continue
			self.log_message(f"grounding path variable {arg}. Moving areas {obst_set}")	
			self.insert(new_new_plan, plan, 'UGPV: ground with threats')
		return True

	# Heuristic Methods #

	def h_condition(self, plan: GPlan, stepnum: int, precond: GLiteral) -> float:
		if precond.is_static:
			return 0
		if precond in plan.init:
			return 0
		if precond in self.h_lit_dict.keys():
			return self.h_lit_dict[precond]
		if precond in self._h_visited:
			return 0

		self._h_visited.append(precond)

		min_so_far = float('inf')
		# if the following is true, then we have an "sub-init" step in our mist
		if len(self.gsteps[stepnum].cndts) == 0:
			stepnum += 2

		for cndt, eff in self.gsteps[stepnum].cndt_map[precond.ID]:
			if not self.gsteps[cndt].instantiable:
				continue
			if not self.gsteps[cndt].height == 0:
				continue
			cndt_heuristic = self.h_step(plan, cndt)
			if cndt_heuristic < min_so_far:
				min_so_far = cndt_heuristic

		self.h_lit_dict[precond] = min_so_far
		return min_so_far

	def h_step(self, plan: GPlan, stepnum: int) -> float:
		if stepnum in self.h_step_dict.keys():
			return self.h_step_dict[stepnum]
		if stepnum == plan.dummy.init.stepnum:
			return 1
		if stepnum in self._h_visited:
			return 1

		self._h_visited.append(stepnum)
		sumo = 1
		for pre in self.gsteps[stepnum].preconds:
			sumo += self.h_condition(plan, stepnum, pre)

		if self.gsteps[stepnum].height > 0:
			sumo += self.h_subplan(plan, self.gsteps[stepnum])

		self.h_step_dict[stepnum] = sumo
		return sumo

	def h_plan(self, plan: GPlan) -> float:
		sumo = 0

		self._h_visited = []
		self.h_lit_dict = dict()
		# flaw_gen = plan.flaws.OC_gen()
		for flaw in plan.flaws.OC_gen():
			# num_flaws += 1
			# exists_choice = False
			# if len(flaw.s_need.choices) > 0:
			# 	exists_choice = True


			if len(flaw.s_need.choices) == 0:
				sumo += self.h_condition(plan, flaw.s_need.stepnum, flaw.p)

		return sumo

	def h_subplan(self, plan, abstract_step):
		sumo = 0
		for sub_step in abstract_step.sub_steps:
			for pre in sub_step.open_preconds:
				if pre in abstract_step.preconds or pre in plan.init:
					continue
				sumo += self.h_condition(plan, sub_step.stepnum, pre)
		for pre in abstract_step.dummy.final.open_preconds:
			if pre in abstract_step.preconds or pre in plan.init:
				continue
			sumo += self.h_condition(plan, abstract_step.dummy.final.stepnum, pre)
		return sumo

	# logging
	def log_message(self, message):
		if self.log:
			print(message)