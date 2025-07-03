from typing import Set, List
from PyPOCL.GPlan import GPlan, math
from PyPOCL.Ground_Compiler_Library.GElm import GLiteral, Operator
from PyPOCL.Flaws import Flaw, TCLF
from PyPOCL.worldmodel import Domain, Problem
from uuid import uuid4
from heapq import heappush, heappop
import time
LOG = 1
REPORT = 1
RRP = 0

def log_message(message):
	if LOG:
		print(message)


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

	def __init__(self, domain: Domain, problem: Problem) -> None:
		"""construct planner

		Args:
			domain (Domain): domain of the planning problem
			problem (Problem): problem of the planning problem
		"""		
		self.ID = uuid4()

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
		root_plan = GPlan.make_root_plan(domain, problem)
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

	def insert(self, plan: GPlan) -> None:
		plan.heuristic = self.h_plan(plan)
		log_message('>\tadd plan to frontier: {} with cost {} and heuristic {}\n'.format(plan.name, plan.cost, plan.heuristic))
		self._frontier.insert(plan)
		self.plan_num += 1

	# @clock
	def solve(self, k: int=4, cutoff: int=6000) -> List[GPlan]:
		# find k solutions to problem

		completed = []
		expanded = 0
		leaves = 0
		tclf_visits = 0

		t0 = time.time()
		print('k={}'.format(str(k)))
		print('time\texpanded\tvisited\tterminated\tdepth\tcost\ttrace')
		while len(self) > 0:
			plan = self.pop()
			self.plan_num = 0 # reset branch counter

			if not plan.isInternallyConsistent():
				# if plan.name[-3] == 'a':
				# 	print('stop')
				log_message('prune {}'.format(plan.name))
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

			log_message('Plan {} selected cost={} heuristic={}'.format(plan.name, plan.cost, plan.heuristic))
			if LOG:
				plan.print()

			# check if any potential TCLFs are fully initialised
			for pot_tclf in plan.potential_tclf:
				link_args = pot_tclf.link.label.Args
				# hotfix. store this info in the causal link
				## find matching condition #TODO make more efficient
				matching_conditions = [e for e in pot_tclf.threat.effects if e.name == pot_tclf.link.label.name and e.truth != pot_tclf.link.label.truth]
				if len(matching_conditions) < 1:
					print(f"Error, TCLF threat: {pot_tclf.threat} contains no effect which matches {pot_tclf.link.label}")
					continue
				if len(matching_conditions) > 1:
					print(f"warning more than one matching condition matching {pot_tclf.link.label}, namely: {matching_conditions}")
				threat_args = matching_conditions[0].Args
				for i in range(len(link_args)):
					if not plan.variableBindings.is_codesignated(link_args[i], threat_args[i]):
						break
				else: # all arguments codesignate. the link is threatened
					log_message(f"TCLF found {pot_tclf}!")
					plan.flaws.insert(plan, pot_tclf)
					plan.potential_tclf.remove(pot_tclf)

			if len(plan.flaws) == 0:
				if plan.variableBindings.symbolic_vb.is_fully_ground():
					if plan.variableBindings.geometric_vb.resolve():
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
							plan.check_plan()
							plan.to_json("plan_{}.json".format(len(completed)))

						if len(completed) == k:
							return completed
						# if len(completed) == 6:
						# 	print('check here')
						continue
				else: # variables are not fully ground
					self.ground_variable(plan)
					continue

			if time.time() - t0 > cutoff:
				print('timedout: {}\t{}\t{}'.format(expanded, len(self) + expanded, leaves))
				return

			# Select Flaw
			flaw = plan.flaws.next()
			plan.name += '[' + str(flaw.flaw_type)[0] + ']'
			log_message('{} selected : {}\n'.format(flaw.name, flaw))

			if isinstance(flaw, TCLF):
				tclf_visits += 1
				self.resolve_threat(plan, flaw)
			else: # must be open precondition flaw (OPF)
				# only count expanded nodes as those that resolve open conditions
				expanded += 1
				self.add_step(plan, flaw)
				self.reuse_step(plan, flaw)			

		raise ValueError('FAIL: No more plans to visit with {} nodes expanded'.format(expanded))

	def add_step(self, plan: GPlan, flaw: Flaw) -> None:
		"""add a new step to resolve a flaw in the plan. Will add one or more plans to the

		Args:
			plan (GPlan): plan which is incremented
			flaw (Flaw): flaw of that plan to be resolved (must be open precondition)
		"""

		s_need, p = flaw.flaw
		cndts = s_need.cndt_map[p.ID]

		if len(cndts) == 0:
			return

		# need indices
		s_index = plan.index(s_need)
		p_index = s_need.preconds.index(p)
		for cndt_step, cndt_eff in cndts:

			if RRP:
				# the Recursive Repair Policty: only let steps with <= height repair open conditions.
				if self.gsteps[cndt_step].height > flaw.level:
					continue

			# cannot add a step which is the inital step
			if not self.gsteps[cndt_step].instantiable:
				continue
			# clone plan and new step

			new_plan = plan.instantiate(str(self.plan_num) + '[a] ')

			# use indices befoer inserting new steps
			mutable_s_need = new_plan[s_index]
			mutable_p = mutable_s_need.preconds[p_index]

			# instantiate new step
			new_step = self.gsteps[cndt_step].instantiate()

			# pass depth to new Added step.
			new_step.depth = mutable_s_need.depth

			# recursively insert new step and substeps into plan, adding orderings and flaws
			new_plan.insert(new_step)

			# check that provided condition can be codesignated with the required(consumed) condition
			provider = new_step.effects[cndt_eff]
			consumer = mutable_p
			if not new_plan.variableBindings.unify(provider, consumer):
				continue

			log_message(f'Add step {new_step} to plan {new_plan.name} to satisfy precondition {mutable_p} of {s_need} with effect {provider}.')

			# resolve s_need with the new step
			new_plan.resolve(new_step, mutable_s_need, mutable_p)

			new_plan.cost += ((self.max_height*self.max_height)+1) - (new_step.height*new_step.height)
			# new_plan.cost += self.max_height + 1 - new_step.height
			# new_plan.cost += 1
			# self.max_height + 1 - new_step.height

			# insert our new mutated plan into the frontier
			self.insert(new_plan)

	def reuse_step(self, plan: GPlan, flaw: Flaw) -> None:
		s_need, p = flaw.flaw

		choices = []
		for step in plan.steps:
			if step.stepnum in [tup[0] for tup in s_need.cndt_map[p.ID]] and not s_need.ID == step.ID and not plan.OrderingGraph.isPath(s_need, step):
				for stepnr, effnr in s_need.cndt_map[p.ID]:
					if stepnr == step.stepnum:
						choices.append((step, effnr))
		if len(choices) == 0:
			return

		# need indices
		s_index = plan.index(s_need)
		p_index = s_need.preconds.index(p)
		for choice, eff_nr in choices:
			# clone plan and new step
			new_plan = plan.instantiate(str(self.plan_num) + '[r] ')

			# use indices before inserting new steps
			mutable_s_need = new_plan[s_index]
			mutable_p = mutable_s_need.preconds[p_index]

			# use index to find old step
			old_step = new_plan.steps[plan.index(choice)]

			# check that provided condition can be codesignated with the required(consumed) condition
			provider = old_step.effects[eff_nr]
			consumer = mutable_p
			if not new_plan.variableBindings.unify(provider, consumer):
				continue
			log_message(f'Reuse step {old_step} to plan {new_plan.name} to satisfy precondition {mutable_p} of {s_need} with effect {provider}.')
			# resolve open condition with old step
			new_plan.resolve(old_step, mutable_s_need, mutable_p)

			# insert mutated plan into frontier
			self.insert(new_plan)

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
		self.insert(new_plan)
		log_message('promote {} in front of {} in plan {}'.format(threat, sink, new_plan.name))


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
		self.insert(new_plan)
		log_message('demotion {} behind {} in plan {}'.format(threat, source, new_plan.name))

	def ground_variable(self, plan: GPlan):
		plan.name += '[ug]'

		# ground a symbolic parameter
		for var in plan.variableBindings.get_var_per_group():
			if plan.variableBindings.is_ground(var):
				continue
			for obj in plan.variableBindings.objects:
				if not plan.variableBindings.can_codesignate(var,obj):
					continue
				# add potential plan with codesignation
				new_plan = plan.instantiate(str(self.plan_num) + '[g] ')
				if not new_plan.variableBindings.add_codesignation(var, obj): # due to the geometric consequences of grounding variables can_codesignate is no longer complete.
					continue
				log_message(f'Grounding variable {var} to object {obj}.')
				self.insert(new_plan)

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
