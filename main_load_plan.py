import sys
from PyPOCL.worldmodel import load_domain_and_problem
from PyPOCL.plan_utility import check_plan, plan_from_json

if __name__ == '__main__':
    num_args = len(sys.argv)
    if num_args == 2:
        test_i = sys.argv[1]
        domain_file = 'domains/manipulation-domain/manipulation-domain.pddl'
        problem_file = f'domains/manipulation-domain-batch/test_{test_i}_problem.pddl'
        worldmodel_file = f'domains/manipulation-domain-batch/test_{test_i}_worldmodel.json'
    elif num_args > 2:
        domain_file = sys.argv[1]
        problem_file = sys.argv[2]
        if num_args > 3:
            worldmodel_file = sys.argv[2]
    else:
        #domain_file = 'domains/ark-domain/ark-domain.pddl'
        #problem_file = 'domains/ark-domain/ark-problem.pddl'
        #worldmodel_file = None # ark problem has no geometric variables
        
        #domain_file = 'tests/domains/test-domain.pddl'
        #problem_file = 'tests/domains/test-problem.pddl'
        #worldmodel_file = 'tests/domains/test-worldmodel.json'

        #domain_file = 'domains/manipulation-domain/manipulation-domain.pddl'
        #problem_file = 'domains/manipulation-domain/manipulation-problem3.pddl'
        #worldmodel_file = 'domains/manipulation-domain/manipulation-worldmodel3.json'

        test_i = 1
        domain_file = 'domains/manipulation-domain/manipulation-domain.pddl'
        problem_file = f'domains/manipulation-domain-batch/test_{test_i}_problem.pddl'
        worldmodel_file = f'domains/manipulation-domain-batch/test_{test_i}_worldmodel.json'

        # benchmarks
        #domain_file = 'tests/benchmarks/manipulation-domain-geometric-threats/domain.pddl'
        #problem_file = 'tests/benchmarks/manipulation-domain-geometric-threats/problem.pddl'
        #worldmodel_file = 'tests/benchmarks/manipulation-domain-geometric-threats/worldmodel.json'

    domain, problem = load_domain_and_problem(domain_file, problem_file, worldmodel_file)
    plan_file = f"plans/{domain.name}/{problem.name}-plan_0.json"

    # Load the plan from the JSON file
    plan = plan_from_json(domain, problem, plan_file)
    plan_valid = check_plan(plan)
    print(f"Plan {plan.name} loaded. Valid: {plan_valid}.")
