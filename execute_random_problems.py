import os
from collections import namedtuple
from PyPOCL.PyDPOCL import POCLPlanner
from PyPOCL.worldmodel import load_domain_and_problem
from PyPOCL.plan_utility import check_plan, plan_to_json

Problem = namedtuple("Problem", ["problem", "worldmodel"])

if __name__ == '__main__':
    batch_dir = "domains/manipulation-domain-batch/"

    problem_tuples = {}
    for file in os.scandir(batch_dir):
        if file.is_file():
            testname = file.name[0:6] # should be like "test_0"
            if testname in problem_tuples: # problem is already found
                continue
            problem_file = os.path.join(batch_dir, testname+"_problem.pddl")
            worldmodel_file = os.path.join(batch_dir, testname+"_worldmodel.json")
            if not os.path.isfile(problem_file):
                raise ValueError(f"file {problem_file} does not exist")
            if not os.path.isfile(worldmodel_file):
                raise ValueError(f"file {worldmodel_file} does not exist")
            problem_tuples[testname] = Problem(problem_file, worldmodel_file)

    domain_file = 'domains/manipulation-domain/manipulation-domain.pddl'
   
    iteration = 0
    success_count = 0
    faulty_plan_count = 0
    plan_not_found_count = 0

    for testname, problem in problem_tuples.items():
        print(f"iteration: {iteration}. running test: {testname}")

        domain, problem = load_domain_and_problem(domain_file, problem.problem, problem.worldmodel)

        planner = POCLPlanner(domain, problem)
        try:
            plans = planner.solve(k=1, cutoff=60)
        except ValueError:
            # valueerror raised. So planner ran out of plans to try
            plans = []

        if len(plans) == 0:
            print("no plans could be found")
            plan_not_found_count += 1
        for i in range(len(plans)):
            plan = plans[i]
            if not check_plan(plan):
                print("Error: plan is not valid")
                faulty_plan_count += 1
            else:
                print("Plan is valid")
                success_count += 1
            plan_to_json(plan, f"plans/{domain.name}/{problem.name}-plan_{i}.json")
        iteration += 1
    print(f"Done running. ran {iteration} iterations")
    print(f"{success_count} successfull.")
    print(f"{faulty_plan_count} faulty plans found.")
    print(f"{plan_not_found_count} plan not found.")