
import itertools
from typing import Set, List
import copy
import pickle
from collections import namedtuple, defaultdict
from PyPOCL.Ground_Compiler_Library.PlanElementGraph import Condition, Action
from PyPOCL.clockdeco import clock
from PyPOCL.Ground_Compiler_Library.Plannify import Plannify
from PyPOCL.Ground_Compiler_Library.Element import Argument, Actor, Operator, Literal
from PyPOCL.Ground_Compiler_Library.pddlToGraphs import parseDomAndProb
from PyPOCL.Ground_Compiler_Library.Graph import Edge
from PyPOCL.Ground_Compiler_Library.Flaws_unused import FlawLib
import hashlib

#GStep = namedtuple('GStep', 'action pre_dict pre_link')
Antestep = namedtuple('Antestep', 'action eff_link')


def groundStoryList(operators: Set[Action], objects, obtypes) -> List[Action]:
	"""

	:param operators: non-ground operator schemas
	:param objects: constants/values
	:param obtypes: object type ontology
	:return: primitive ground steps
	"""
	stepnum = 0
	gsteps = []
	print('...Creating Primitive Ground Steps')
	for op in operators:
		op.updateArgs()
		# cndts = [[obj for obj in objects if arg.typ == obj.typ or arg.typ in obtypes[obj.typ]] for arg in op.Args]
		# tuples = itertools.product(*cndts)
		# for t in tuples:

		# 	# check for inconsistent tuple of arg types
		# 	legaltuple = True
		# 	for (u,v) in op.nonequals:
		# 		if t[u] == t[v]:
		# 			legaltuple = False
		# 			break
		# 	if not legaltuple:
		# 		continue

		gstep = copy.deepcopy(op)
		print('Creating ground step {}'.format(gstep))

		# replace the ID of the internal elements
		gstep._replaceInternals()

		# assign the step number (only one of the following should be necessary)
		gstep.root.stepnumber = stepnum
		gstep.root.arg_name = stepnum
		stepnum += 1

		# swap the leaves of the step with the objects in tuple "t"
		#gstep.replaceArgs(t)

		# append the step to our growin glist
		gsteps.append(gstep)

		# not sure why one would need the following:
		# gstep.replaceInternals()

		# assign height of the step to the root element and
		gstep.height = 0
		gstep.root.height = 0

	return gsteps

def groundDecompStepList(doperators, GL, stepnum=0, height=0):
	gsteps = []
	print('...Creating Ground Decomp Steps')
	for op in doperators:
		#Subplans = Plannify(op.subplan, GL)
		print('processing operator: {}'.format(op))
		try:
			sub_plans = Plannify(op.subplan, GL, height)
		except:
			continue
		for sp in sub_plans:

			GDO = copy.deepcopy(op)
			GDO.is_decomp = True

			if not rewriteElms(GDO, sp, op):
				continue

			GDO.root.is_decomp = True

			# swap constructor IDs and replaced_IDs
			GDO._replaceInternals()
			GDO.replaceInternals()

			# Now create dummy init step and goal step
			dummy_init = Action(name='begin:' + str(GDO.name))
			dummy_init.has_cndt = False
			dummy_init.root.stepnumber = stepnum
			for condition in GDO.Preconditions:
				dummy_init.edges.add(Edge(dummy_init.root, condition.root, 'effect-of'))
				dummy_init.edges.update(condition.edges)
				dummy_init.elements.update(condition.elements)
			gsteps.append(dummy_init)
			stepnum+=1

			dummy_goal = Action(name='finish:' + str(GDO.name))
			dummy_goal.is_cndt = False
			dummy_goal.root.stepnumber = stepnum
			for condition in GDO.Effects:
				dummy_goal.edges.add(Edge(dummy_goal.root, condition.root, 'precond-of'))
				dummy_goal.edges.update(condition.edges)
				dummy_goal.elements.update(condition.elements)
			gsteps.append(dummy_goal)
			stepnum+=1

			GDO.sub_dummy_init = dummy_init
			GDO.sub_dummy_goal = dummy_goal

			GDO.ground_subplan = sp
			GDO.root.stepnumber = stepnum
			sp.root = GDO.root
			stepnum += 1
			GDO.height = height + 1
			GDO.root.height = height + 1

			# important to add init and goal steps first
			gsteps.append(GDO)


	return gsteps

def rewriteElms(GDO, sp, op):

	for elm in sp.elements:
		EG = elm
		if elm.typ in {'Action', 'Condition'}:
			EG = eval(elm.typ).subgraph(sp, elm)

		assignElmToContainer(GDO, EG, list(op.elements))
	GDO.updateArgs()
	for (u,v) in op.nonequals:
		if GDO.Args[u] == GDO.Args[v]:
			return False
	for arg in GDO.Args:
		if isinstance(arg, Argument):
			if arg.name is None:
				return False
	return True

def assignElmToContainer(GDO, EG, ex_elms):

	for ex_elm in ex_elms:
		if ex_elm.arg_name is None:
			continue
		if EG.arg_name != ex_elm.arg_name:
			if isinstance(EG, Argument):
				if EG.name != ex_elm.name:
					continue
			else:
				continue

		GDO.assign(ex_elm, EG)

import re
@clock
def upload(GL, name):
	# n = re.sub('[^A-Za-z0-9]+', '', name)
	print(name)
	with open(name, 'wb') as afile:
		pickle.dump(GL, afile)

@clock
def reload(name):
	n = re.sub('[^A-Za-z0-9]+', '', name)
	afile = open(n, "rb")
	GL = pickle.load(afile)
	afile.close()
	FlawLib.non_static_preds = GL.non_static_preds

	return GL


class GLib:
	""" a class to represent and configure a planning problem

	 Attributes
    ----------
    operators : set(Action)
        unground operators
	non_static_preds : set(Tuple)
		?
	object_types : defaultdict(str: set(str))
		?
	objects : set(Argument)
		?
	_gsteps : List(Action)
		fully ground operators
	ante_dict : defaultdict(int : set(int))
		dictionary mapping a step A to the set of their antecedents B. i.e. step A has a precondition which is an effect of B.
		Where steps are represented by their stepnumber in g_steps
	threat_dict : defaultdict(int: set(int))
		dictionary mapping a step A to the set of steps which pose a threat to their preconditions
		Where steps are represented by their stepnumber in g_steps
	flaw_threat_dict : defaultdict(uuid: set((int, int)))
		dictionary mapping a precondition(pre.replaced_ID) to the set of tuple(stepnr, effectnr) which pose a threat to it.
		the threatening step represented by their stepnumber in g_steps
	id_dict : defaultdict(uuid: set(int))s
		dictionary mapping preconditions (pre.replaced_ID) to the set of steps which have that condition as an effect. i.e. their providers/antecedents
	eff_dict : defaultdict(uuid: set((int, int)))
		dictionary mapping preconditions (pre.replaced_ID) to the set of tuple(stepnr, effectnr) which can act as their providers
	name : str
		name of the domain+problem

	"""
	def __init__(self, domain, problem):
		operators, dops, objects, obtypes, init_action, goal_action = parseDomAndProb(domain, problem)
		self.non_static_preds = FlawLib.non_static_preds
		self.object_types = obtypes
		self.objects = objects

		# primitive steps
		self._gsteps = groundStoryList(operators, self.objects, obtypes)

		#dictionaries
		# a candidate map is a dictionary such that cndt_map[step_id][pre_id] = [(s_1, e_1),...,(s_k, e_k)] values are steps whose effect is same
		# self.cndt_map = defaultdict(lambda x: defaultdict(list))
		# self.threat_map = defaultdict(lambda x: defaultdict(list))
		# cndts (key is step number, value is set of step numbers)
		self.ante_dict = defaultdict(set)
		# threats (key is step number, value is set of step numbers)
		self.threat_dict = defaultdict(set)
		self.flaw_threat_dict = defaultdict(set)
		# id_dict is just by precondition ID
		self.id_dict = defaultdict(set)
		self.eff_dict = defaultdict(set)

		print('...Creating PlanGraph base level')
		self.loadAll()

		for i in range(3):
			print('...Creating PlanGraph decompositional level {}'.format(i+1))
			try:
				D = groundDecompStepList(dops, self, stepnum=len(self._gsteps), height=i)
			except:
				break
			if not D or len(D) == 0:
				break
			self.loadPartition(D)

		init_action.root.stepnumber = len(self._gsteps)
		# replacing internal replaced_IDs
		init_action._replaceInternals()
		# replace IDs
		init_action.replaceInternals()
		init_action.instantiable = False

		goal_action.root.stepnumber = len(self._gsteps) + 1
		# replace internal replaced_IDs
		goal_action._replaceInternals()
		# replace IDs
		goal_action.replaceInternals()
		goal_action.reusable = False

		# check if init and goal have potential causal relationships
		self.loadPartition([init_action, goal_action])

		print('{} ground steps created'.format(len(self)))
		print('uploading')
		d_name = domain.split('/')[-1].split('.')[0]
		p_name = problem.split('/')[-1].split('.')[0]
		self.name = d_name + '.' + p_name

	def insert(self, _pre, antestep, eff_i):
		self.id_dict[_pre.replaced_ID].add(antestep.stepnumber)
		self.eff_dict[_pre.replaced_ID].add((antestep.stepnumber, eff_i))

	def loadAll(self):
		self.load(self._gsteps, self._gsteps)

	def loadPartition(self, particles):
		#print('... for each decompositional operator ')
		self.load(particles, self._gsteps)
		self.load(self._gsteps, particles)
		self.load(particles, particles)
		self._gsteps.extend(particles)

	def load(self, antecedents, consequents):
		for ante in antecedents:
			# steps which have no preconditions needn't have any candidates
			if not ante.has_cndt:
				continue
			for pre in ante.Preconditions:
				print('... Processing antecedents for {} \t\tof step {}'.format(pre, ante))
				self._loadAntecedentPerConsequent(consequents, ante, pre)

	def _loadAntecedentPerConsequent(self, antecedents: Set[Action], _step: Action, _pre) -> None:
		"""check if the steps in antecedents are antecedents to a step with a certain precondition

		Args:
			antecedents (_type_): _description_
			_step (_type_): _description_
			_pre (_type_): _description_
		"""
		for gstep in antecedents:
			# skip steps which cannever be a candidate (such as goal)
			if not gstep.is_cndt:
				continue
			if self._parseEffects(gstep, _step, _pre) > 0:
				self.ante_dict[_step.stepnumber].add(gstep.stepnumber)

	def _parseEffects(self, step1, step2, pre):
		"""process the effects that the condition pre from step 1 has on step2. Assuming step1 precedes step2

		Args:
			step1 (Action): step whose effects are to be analysed
			step2 (Action): step which has condition pre
			pre (GLiteral): condition of step2 which is subject to the effects of step1

		Returns:
			int: number of effects of step1 that fulfill pre of step2
		"""
		count = 0
		for i in range(len(step1.Effects)):
			Eff = step1.Effects[i]
			if Eff.name != pre.name:
				continue # effect is not based on the same predicate as the precondition
			if Eff.truth != pre.truth: # the effect undoes the precondition. Add to threat list
				self.threat_dict[step2.stepnumber].add(step1.stepnumber)
				self.flaw_threat_dict[pre.replaced_ID].add((step1.stepnumber, i))
			else: # the effect is identical and therefore fulfills the precondition
				self.insert(pre, step1.deepcopy(replace_internals=True), i)
				count += 1
		return count

	def getPotentialEffectLinkConditions(self, src, snk):
		"""
		Given source and sink steps, return {eff(src) \cap pre(snk)}
		But, let those conditions be those of the src.
		"""
		cndts = []
		for eff in self[src.stepnumber].effects:
			for pre in self[snk.stepnumber].preconditions:
				if eff.replaced_ID not in self.id_dict[pre.replaced_ID]:
					continue
				cndts.append(Edge(src, snk, copy.deepcopy(eff)))

		return cndts

	def getConsistentEffect(self, S_Old, precondition):
		effect_token = None
		for eff in S_Old.effects:
			if eff.replaced_ID in self.eff_dict[precondition.replaced_ID] or self.eff_dict[eff.replaced_ID] == \
					self.eff_dict[precondition.replaced_ID]:
				effect_token = eff
				break
		if effect_token is None:
			raise AttributeError('story_GL.eff_dict empty but id_dict has antecedent')
		return effect_token

	def hasConsistentPrecondition(self, Sink, effect):
		for pre in Sink.preconditions:
			if effect.replaced_ID in self.eff_dict[pre.replaced_ID]:
				return True
		return False

	def getConsistentPrecondition(self, Sink, effect):
		pre_token = None
		for pre in Sink.preconditions:
			if effect.replaced_ID in self.eff_dict[pre.replaced_ID]:
				pre_token = pre
				break
		if pre_token is None:
			raise AttributeError('effect {} not in story_GL.eff_Dict for Sink {}'.format(effect, Sink))
		return pre_token

	def __len__(self):
		return len(self._gsteps)

	def __getitem__(self, position):
		return self._gsteps[position]

	def __contains__(self, item):
		return item in self._gsteps

	def __repr__(self):
		return 'Grounded Step Library: \n' +  str([step.__repr__() for step in self._gsteps])


if __name__ ==  '__main__':
	domain_file = 'domains/ark-domain.pddl'
	problem_file = 'domains/ark-problem.pddl'

	operators, objects, object_types, initAction, goalAction = parseDomAndProb(domain_file, problem_file)
