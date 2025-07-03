import unittest

from PyPOCL.GPlan import GPlan
from PyPOCL.worldmodel import load_domain_and_problem

class TestPlanManipulation(unittest.TestCase):
    def setUp(self):
        domain_file = 'tests/domains/test-domain.pddl'
        problem_file = 'tests/domains/test-problem.pddl'
        worldmodel_file = 'tests/domains/test-worldmodel.json'

        # load domain and problem
        domain, problem = load_domain_and_problem(domain_file, problem_file, worldmodel_file)

        self.root_plan = GPlan.make_root_plan(domain, problem)
        self.operators = domain.operators

    def test_add_step(self):
        # find movemono step
        ops = [o for o in self.operators if o.schema=='movemono']
        action = ops[0].instantiate()

        new_plan = self.root_plan.instantiate('1[a]')
        new_plan.insert(action)

        # test action is added
        self.assertIn(action, new_plan)

        # test action cannot be before start or after end
        self.assertTrue(new_plan.OrderingGraph.isPath(new_plan.dummy.init, action))
        self.assertFalse(new_plan.OrderingGraph.isPath(new_plan.dummy.goal, action))
        self.assertFalse(new_plan.OrderingGraph.isPath(action, new_plan.dummy.init))
        self.assertTrue(new_plan.OrderingGraph.isPath(action, new_plan.dummy.goal))

        # test all arguments are in the variablebindings
        for a in action.Args:
            self.assertIn(a, new_plan.variableBindings)

if __name__ == '__main__':
    unittest.main()