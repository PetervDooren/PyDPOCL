import unittest
import json
from PyPOCL.GPlan import GPlan
from PyPOCL.worldmodel import load_domain_and_problem

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

        plan = GPlan.from_json(domain, problem, "tests/test_plan.json")
        self.assertEqual(plan.name, "test-plan")
        self.assertEqual(plan.cost, 1)
        self.assertEqual(plan.depth, 0)
        self.assertEqual(len(plan.steps), 3)

if __name__ == '__main__':
    unittest.main()