import os
import unittest

from PyPOCL.PyDPOCL import POCLPlanner
from PyPOCL.worldmodel import load_domain_and_problem
from PyPOCL.plan_utility import check_plan

class TestBenchmarks(unittest.TestCase):
    #def test_all_benchmarks(self):
    #    benchmark_dir = 'tests/benchmarks'
    #    domains = [name for name in os.listdir(benchmark_dir) if os.path.isdir(os.path.join(benchmark_dir, name))]
    #
    #    for domain in domains:
    #        domain_file = os.path.join(benchmark_dir, domain, "domain.pddl")
    #        problem_file = os.path.join(benchmark_dir, domain, "problem.pddl")
    #        worldmodel_file = os.path.join(benchmark_dir, domain, "worldmodel.json")
    #        if not os.path.exists(os.path.join(benchmark_dir, domain, "worldmodel.json")):
    #            worldmodel_file = None
    #        
    #        self.assertTrue(os.path.exists(domain_file) and os.path.exists(problem_file), f"Domain or problem file not found for {domain}: {domain_file}, {problem_file}")
    #        self.run_benchmark(domain_file, problem_file, worldmodel_file)

    def test_ark_domain(self):
        domain_file = "tests/benchmarks/ark-domain/domain.pddl"
        problem_file = "tests/benchmarks/ark-domain/problem.pddl"
        worldmodel_file = None
        self.run_benchmark(domain_file, problem_file, worldmodel_file)
    
    def test_manipulation_domain(self):
        domain_name = "manipulation-domain"
        domain_file = f"tests/benchmarks/{domain_name}/domain.pddl"
        problem_file = f"tests/benchmarks/{domain_name}/problem.pddl"
        worldmodel_file = f"tests/benchmarks/{domain_name}/worldmodel.json"
        self.run_benchmark(domain_file, problem_file, worldmodel_file)

    def test_manipulation_domain_many_objects(self):
        domain_name = "manipulation-domain-many-objects"
        domain_file = f"tests/benchmarks/{domain_name}/domain.pddl"
        problem_file = f"tests/benchmarks/{domain_name}/problem.pddl"
        worldmodel_file = f"tests/benchmarks/{domain_name}/worldmodel.json"
        self.run_benchmark(domain_file, problem_file, worldmodel_file)
    
    def test_manipulation_domain_geometric_threats(self):
        domain_name = "manipulation-domain-geometric-threats"
        domain_file = f"tests/benchmarks/{domain_name}/domain.pddl"
        problem_file = f"tests/benchmarks/{domain_name}/problem.pddl"
        worldmodel_file = f"tests/benchmarks/{domain_name}/worldmodel.json"
        self.run_benchmark(domain_file, problem_file, worldmodel_file)

    def run_benchmark(self, domain_file, problem_file, worldmodel_file):
        # load domain and problem
        domain, problem = load_domain_and_problem(domain_file, problem_file, worldmodel_file)

        planner = POCLPlanner(domain, problem)
        plans, _ = planner.solve(k=1, cutoff=10)
        self.assertGreater(len(plans), 0, "No plans found")
        for plan in plans:
            self.assertTrue(check_plan(plan), "Plan is not valid")

if __name__ == '__main__':
    unittest.main()