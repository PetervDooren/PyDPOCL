import sys
from PyPOCL.PyDPOCL import POCLPlanner
from PyPOCL.worldmodel import load_domain_and_problem
from PyPOCL.plan_utility import check_plan, plan_to_json, plan_to_yaml

if __name__ == '__main__':
    num_args = len(sys.argv)
    if num_args >1:
        domain_file = sys.argv[1]
        if num_args > 2:
            problem_file = sys.argv[2]
    else:
        #domain_file = 'domains/ark-domain/ark-domain.pddl'
        #problem_file = 'domains/ark-domain/ark-problem.pddl'
        #worldmodel_file = None # ark problem has no geometric variables
        
        #domain_file = 'tests/domains/test-domain.pddl'
        #problem_file = 'tests/domains/test-problem.pddl'
        #worldmodel_file = 'tests/domains/test-worldmodel.json'

        #domain_file = 'domains/manipulation-domain/manipulation-domain.pddl'
        #problem_file = 'domains/manipulation-domain/manipulation-problem2.pddl'
        #worldmodel_file = 'domains/manipulation-domain/manipulation-worldmodel2.json'

        test_i = 0
        domain_file = 'domains/manipulation-domain/manipulation-domain.pddl'
        problem_file = f'domains/manipulation-domain-batch/test_{test_i}_problem.pddl'
        worldmodel_file = f'domains/manipulation-domain-batch/test_{test_i}_worldmodel.json'

    domain, problem = load_domain_and_problem(domain_file, problem_file, worldmodel_file)

    planner = POCLPlanner(domain, problem)
    plans, _ = planner.solve(k=1)
    for i in range(len(plans)):
        plan = plans[i]
        if not check_plan(plan):
            print("Error: plan is not valid")
        else:
            print("Plan is valid")
        plan_to_json(plan, f"plans/{domain.name}/{problem.name}-plan_{i}.json")
        #if domain.name == "manipulation-domain":
        #    plan_to_yaml(plan, f"plans/{domain.name}/{problem.name}-executable_plan_{i}.yaml")
