import unittest

from PyPOCL.PyDPOCL import POCLPlanner
from PyPOCL.worldmodel import load_domain_and_problem

class TestBenchmarks(unittest.TestCase):
    def test_benchmark(self):
        domain_file = 'tests/benchmarks/manipulation-domain/manipulation-domain.pddl'
        problem_file = 'tests/benchmarks/manipulation-domain/manipulation-problem.pddl'
        worldmodel_file = 'tests/benchmarks/manipulation-domain/manipulation-worldmodel.json'

        # load domain and problem
        domain, problem = load_domain_and_problem(domain_file, problem_file, worldmodel_file)

        planner = POCLPlanner(domain, problem)
        plans = planner.solve(k=1)
        self.assertGreater(len(plans), 0, "No plans found")
        for plan in plans:
            self.assertTrue(plan.check_plan(), "Plan is not valid")
    
    def test_ark_benchmark(self):
        domain_file = 'tests/benchmarks/ark-domain/ark-domain.pddl'
        problem_file = 'tests/benchmarks/ark-domain/ark-problem.pddl'
        worldmodel_file = None  # ark problem has no geometric variables

        # load domain and problem
        domain, problem = load_domain_and_problem(domain_file, problem_file, worldmodel_file)

        planner = POCLPlanner(domain, problem)
        plans = planner.solve(k=1)
        self.assertGreater(len(plans), 0, "No plans found")
        for plan in plans:
            self.assertTrue(plan.check_plan(), "Plan is not valid")

if __name__ == '__main__':
    unittest.main()