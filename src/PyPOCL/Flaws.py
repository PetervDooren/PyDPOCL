from collections import deque
from bisect import bisect_left

"""
	Flaws for plan element graphs
"""

class Flaw:
	"""
    Represents flaw in a plan

    This base class may be extended for different kinds of flaws like OPF or TCLF

    Attributes:
        name (str): The name or identifier of the flaw.
        flaw (tuple): A tuple representing the flaw's components, interpretation varies by flaw type.
        cndts (int): Number of steps in the plan which could provide the condition in this flaw.
        risks (int): Number of steps in the plan which may threaten the condition in this flaw.
        criteria (int): A value used for comparing the severity of the flaw (higher = more severe).
        tiebreaker (int): An arbitraty value used to resolve ties between flaws when comparing their severity (default is 0).
        flaw_type (str or None): The type of the flaw, which could help in specific categorization (default is None).

    Methods:
        __lt__(self, other):
            Compares this flaw with another flaw to determine its order based on risk, criteria, and tiebreaker values.
            The comparison prioritizes the following (in order):
                - If the flaw type is 'unsafe', the comparison is based on the 'risks' attribute.
                - If the risk values are equal, the comparison is based on the 'criteria' attribute.
                - If both risk and criteria are equal, the comparison is based on the 'tiebreaker' attribute.

        __repr__(self):
            Returns a string representation of the flaw instance, showing the flaw components, criteria, and tiebreaker values.

    Args:
        f (tuple): A tuple representing the components of the flaw, usually containing elements that define the flaw.
        name (str): The name or type of the flaw, typically a descriptive label.
    """
	def __init__(self, f, name):
		self.name = name
		self.flaw = f
		self.cndts = 0
		self.risks = 0
		self.criteria = 0
		self.tiebreaker = 0
		self.flaw_type = None

	#For comparison via bisect
	def __lt__(self, other):
		if self.flaw_type == 'unsafe':
			if self.risks != other.risks:
				return self.risks < other.risks
		elif self.criteria != other.criteria:
			return self.criteria < other.criteria
		else:
			return self.tiebreaker < other.tiebreaker

	def __repr__(self):
		return 'Flaw({}, criteria={}, tb={})'.format(self.flaw, self.criteria, self.tiebreaker)


class OPF(Flaw):

	def __init__(self, s_need, pre, level=0):
		super(OPF, self).__init__((s_need, pre), 'opf')
		self.s_need = s_need
		self.p = pre
		self.level = level
		# self.criteria = hash(s_need.stepnum) ^ hash(pre.name) ^ hash(pre.truth)
		self.criteria = len(str(s_need.schema)) + len(str(pre.name)) + len(str(pre.truth))
		self.tiebreaker = sum(len(str(arg.name)) for arg in pre.Args)

	def __hash__(self):
		return hash(self.flaw[0].ID) ^ hash(self.flaw[1].ID)


class TCLF(Flaw):
	"""
    Represents a specific type of flaw in a partial order causal link plan, namely the TCLF (Threatened Causal Link Flaw). 
    A TCLF occurs when a step (threatening step) could potentially undo the condition established by a causal link

    Attributes:
        threat (Step): The step identified as the threatening step, which could potentially break the causal link.
        link (CausalLink): The causal link being threatened by the threatening step.
        criteria (int): A computed metric used for evaluating the severity of the flaw, based on the schema of the threatening step
                        and the label of the causal link.
        tiebreaker (int): A computed metric used to break ties between multiple TCLFs, based on the causal link's label and the
                          sink and source of the link.

    Methods:
        __hash__(): Computes a unique hash value for the TCLF instance, combining the hash of the threat, link source, sink,
                    and link label to ensure the uniqueness of the instance in hash-based collections.

    Args:
        threatening_step (Step): The step that threatens the validity of the causal link.
        causal_link_edge (CausalLink): The causal link that is being threatened by the step.
    """
	def __init__(self, threatening_step, causal_link_edge):
		super(TCLF, self).__init__((threatening_step, causal_link_edge), 'tclf')
		self.threat = self.flaw[0]
		self.link = self.flaw[1]
		self.criteria = len(str(self.threat.schema)) + len(str(self.link.label.sink.name)) + len(str(self.link.label.sink.truth))
		self.tiebreaker = len(str(self.link.label.sink.truth)) + len(str(self.link.label.sink.name)) \
		                  + len(str(causal_link_edge.sink.schema)) - len(causal_link_edge.source.preconds)

	def __hash__(self):
		return hash(self.threat.ID) ^ hash(self.link.source.ID) ^ hash(self.link.sink.ID) ^ hash(self.link.label.source.ID ^ hash(self.link.label.sink.ID))

# class DTCLF(Flaw):
# 	def __init__(self, dummy_init, dummy_final, causal_link_edge):
# 		super(DTCLF, self).__init__(((dummy_init, dummy_final), causal_link_edge), 'tclf')
# 		self.anterior = dummy_init
# 		self.posterior = dummy_final
# 		self.link = self.flaw[1]
# 		self.criteria = self.anterior.stepnum ^ self.posterior.stepnum ^ self.link.source.stepnum ^ self.link.sink.stepnum
# 		self.tiebreaker = hash(self.link.label.name) ^ hash(self.link.label.truth) ^ sum(hash(arg) for arg in self.link.label.Args)

	# def __hash__(self):
	# 	return hash(self.anterior.ID) ^ hash(self.posterior.ID) ^ hash(self.link.source.ID) ^ hash(self.link.sink.ID) ^ hash(self.link.label.ID)

class DCF(Flaw):
	def __init__(self, f, name):
		super(DCF, self).__init__(f, name)
		self.criteria = len(f.Steps)
		self.tiebreaker = f.root.stepnumber
	def __repr__(self):
		steps = [''.join(str(step) + ', ' for step in self.flaw.Step_Graphs)]
		return 'DCF(' + ''.join(['{}'.format(step) for step in steps]) + 'criteria ={}, tb={})'.format(
			self.criteria, self.tiebreaker)

class Flawque:
	""" A deque which pretends to be a set, and keeps everything sorted, highest-value first"""

	def __init__(self, name=None):
		self._flaws = deque()
		self.name = name

	def add(self, flaw):
		flaw.flaw_type = self.name
		self.insert(flaw)

	def update(self, iter):
		for flaw in iter:
			self.add(flaw)

	def __contains__(self, item):
		return item in self._flaws

	def __len__(self):
		return len(self._flaws)

	def removeDuplicates(self):
		self._flaws = deque(set(self._flaws))

	def head(self):
		return self._flaws.popleft()

	def tail(self):
		return self._flaws.pop()

	def pop(self):
		return self._flaws.pop()

	def peek(self):
		return self._flaws[-1]

	def insert(self, flaw):
		index = bisect_left(self._flaws, flaw)
		self._flaws.rotate(-index)
		self._flaws.appendleft(flaw)
		self._flaws.rotate(index)

	def __getitem__(self, position):
		return self._flaws[position]

	def __repr__(self):
		return str(self._flaws)


class FlawTypes:
	""" 
	A collection of all flaws in order of priority.
	"""
	def __init__(self, statics, inits, threats, unsafe, reusable, nonreusable):
		self._list = [statics, inits, threats, unsafe, reusable, nonreusable]

	def __len__(self):
		return len(self._list)
	def __getitem__(self, item):
		return self._list[item]


class FlawLib:
	"""
    A container for managing and categorizing different types of flaws that arise in a plan. 

    Attributes:
        non_static_preds (set): A set of predicates that are not static.
        statics (Flawque): A queue of OPF flaws related to static conditions (unchangeable elements).
        inits (Flawque): A queue of OPF flaws whose condition is true in the initial state.
        decomps (Flawque): A queue for decompositional ground subplans that need to be added.
        threats (Flawque): A queue for TCLF flaws related to causal link dependencies that are undone.
        unsafe (Flawque): A queue for OPF flaws whose condition may create a TCLF.
        reusable (Flawque): A queue for OPF flaws whose condition is consistent with at least one existing effect.
        nonreusable (Flawque): A queue for OPF flaws whose condition is inconsistent with any existing effect.
        typs (FlawTypes): An collection of all the above flaw queues.
        restricted_names (list): A list of flaw set names that are restricted from certain operations, e.g., 'threats' and 'decomps'.

    Methods:
        __len__(self):
            Returns the total number of flaws across all categories in the flaw library.

        __contains__(self, flaw):
            Checks if a specific flaw exists in any of the flaw sets.

        flaws(self):
            Returns a list of all flaws from all sets, excluding restricted categories like 'threats' and 'decomps'.

        counts_for_heuristic(self, flaw_set):
            Determines whether a flaw set should be considered for heuristics based on its name and size.

        OC_gen(self):
            A generator that yields open conditions (flaws with a height of 0) for heuristic evaluation.

        next(self):
            Returns the flaw with the highest priority (i.e., the first flaw in any of the non-empty flaw sets) and removes it from the set.

        addCndtsAndRisks(self, plan, action):
            For each effect of the given action, updates the open-condition mapping by incrementing the conditions (`cndts`) 
            or risks for corresponding flaws, based on the action's relationships with other steps in the plan.

        insert(self, plan, flaw):
            Inserts a flaw into the appropriate flaw set based on its type, adjusting its `cndts` (conditions) and `risks` (threats) 
            based on the relationships between steps in the plan and the flaw's requirements.

        __repr__(self):
            Returns a string representation of the `FlawLib` instance, displaying the flaw sets and their contents.

    Args:
        None (this class is initialized with default flaw sets).
    """
	non_static_preds = set()

	def __init__(self):
		#static = unchangeable (should do oldest first.)
		self.statics = Flawque('statics')

		#init = established by initial state
		self.inits = Flawque('inits')

		#decomps - decompositional ground subplans to add
		self.decomps = Flawque('decomps')

		#threat = causal link dependency undone
		self.threats = Flawque('threats')

		#unsafe = existing effect would undo sorted by number of cndts
		self.unsafe = Flawque('unsafe')

		#reusable = open conditions consistent with at least one existing effect sorted by number of cndts
		self.reusable = Flawque('reusable')

		#nonreusable = open conditions inconsistent with existing effect sorted by number of cndts
		self.nonreusable = Flawque('nonreusable')

		self.typs = FlawTypes(self.statics, self.threats, self.inits, self.unsafe, self.reusable, self.nonreusable)
		self.restricted_names = ['threats', 'decomps']

	def __len__(self):
		return sum(len(flaw_set) for flaw_set in self.typs)

	def __contains__(self, flaw):
		for flaw_set in self.typs:
			if flaw in flaw_set:
				return True
		return False

	@property
	def flaws(self):
		""" Return all OPF flaws. i.e. every flaw not in 'restricted_names' """
		return [flaw for i, flaw_set in enumerate(self.typs) for flaw in flaw_set if flaw_set.name not in
				self.restricted_names]

	def counts_for_heuristic(self, flaw_set):
		if len(flaw_set) == 0:
			return False
		if flaw_set.name in self.restricted_names:
			return False
		return True

	def OC_gen(self):
		''' Generator for open conditions'''
		# for flaw_set in self.typs:
		# 	if not self.counts_for_heuristic(flaw_set):
		return [flaw for flaw_set in self.typs for flaw in flaw_set
		        if self.counts_for_heuristic(flaw_set) and flaw.flaw[0].height == 0]
		# for i, flaw_set in enumerate(self.typs):
		# 	if len(flaw_set) == 0:
		# 		continue
		# 	if flaw_set.name in self.restricted_names:
		# 		continue
		# 	# if flaw_set.name == 'statics':
		# 	# 	continue
		# 	return (flaw for flaw in flaw_set if flaw.flaw[0].height == 0)
			# return(g)

	def next(self):
		''' Returns flaw with highest priority, and removes'''
		for flaw_set in self.typs:
			if len(flaw_set) > 0:
				return flaw_set.pop()
		return None

	#@clock
	# I think this method is unused. 
	def addCndtsAndRisks(self, plan, action):
		""" For each effect of Action, add to open-condition mapping if consistent"""

		for oc in self.OCs():
			s_need, pre = oc.flaw

			# step numbers of antecdent types
			if plan.OrderingGraph.isPath(s_need, action):
				continue

			if action.stepnumber in action.cndt_map[pre.ID]:
				oc.cndts += 1

			# step numbers of threatening steps
			elif action.stepnumber in action.threats:
				oc.risks += 1

	#@clock
	def insert(self, plan, flaw):
		''' for each effect of an existing step, check and update mapping to consistent effects'''
		# TODO make a better way of distinguishing the flaws.
		if flaw.name == 'tclf':
			#if flaw not in self.threats:
			self.threats.add(flaw)
			return

		# if flaw.name == 'dcf':
		# 	self.decomps.add(flaw)
		# 	return

		# flaw is an open precondition. Evaluate it in context for use in heuristics and ordering.
		#unpack flaw
		s_need, pre = flaw.flaw
		# use height to determine which steps are to be considered

		#if pre.predicate is static
		if pre.is_static:
			self.statics.add(flaw)
			return

		for step in plan.steps:
			if step.ID == s_need.ID:
				continue
			if plan.OrderingGraph.isPath(s_need, step):
				continue
			if step.stepnum in [tup[0] for tup in s_need.cndt_map[pre.ID]]:
				flaw.cndts += 1
			if step.stepnum in [tup[0] for tup in s_need.threat_map[pre.ID]]:
				flaw.risks += 1

		if pre in plan.init:
			self.inits.add(flaw)
			return

		if flaw.risks > 0:
			self.unsafe.add(flaw)
			return

		#if not static but has cndts, then reusable
		if flaw.cndts > 0:
			self.reusable.add(flaw)
			return

		#last, must be nonreusable
		self.nonreusable.add(flaw)

	def __repr__(self):
		F = [('|' + ''.join([str(flaw) + '\n|' for flaw in T]) , T.name) for T in self.typs if len(T) > 0]
		return 'Flaw Lib\n' + '\n'.join(['{}: {}'.format(name, flaws) for flaws, name in F])

# import unittest
#
#
# class TestOrderingGraphMethods(unittest.TestCase):
#
# 	def test_flaw_counter(self):
# 		assert True
#
# if __name__ ==  '__main__':
# 	unittest.main()