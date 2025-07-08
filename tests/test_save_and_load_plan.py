import unittest
import json
from PyPOCL.GPlan import GPlan
from PyPOCL.worldmodel import load_domain_and_problem
from PyPOCL.plan_utility import check_plan, plan_from_json

class DummyStep:
    def __init__(self, ID, name):
        self.ID = ID
        self.name = name
        self.Args = []
        self.preconds = []
        self.effects = []

class TestGPlanJson(unittest.TestCase):
    def test_from_json(self):
        domain_file = 'tests/domains/test-domain.pddl'
        problem_file = 'tests/domains/test-problem.pddl'
        worldmodel_file = 'tests/domains/test-worldmodel.json'

        # load domain and problem
        domain, problem = load_domain_and_problem(domain_file, problem_file, worldmodel_file)

        plan = plan_from_json(domain, problem, "tests/plans/test-problem-plan_0.json")
        self.assertEqual(plan.name, "[u]0[a] [u]0[a] [t]1[td] [u]1[r] [ug]1[g] [ug]0[g] ")
        self.assertEqual(plan.cost, 2)
        self.assertEqual(plan.depth, 0)
        self.assertEqual(len(plan.steps), 4)
        #self.assertTrue(check_plan(plan))

if __name__ == '__main__':
    unittest.main()