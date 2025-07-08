import sys
from PyPOCL.worldmodel import load_domain_and_problem
from PyPOCL.plan_utility import check_plan, plan_from_json

if __name__ == '__main__':
    num_args = len(sys.argv)
    if num_args >1:
        domain_file = sys.argv[1]
        if num_args > 2:
            problem_file = sys.argv[2]
            if num_args > 3:
                worldmodel_file = sys.argv[3]
                if num_args > 4:
                    plan_file = sys.argv[4]
    else:
        domain_file = 'domains/manipulation-domain/manipulation-domain.pddl'
        problem_file = 'domains/manipulation-domain/manipulation-problem.pddl'
        worldmodel_file = 'domains/manipulation-domain/manipulation-worldmodel.json'
        plan_file = 'plans/manipulation-domain/manipulation-problem-plan_0.json'

    domain, problem = load_domain_and_problem(domain_file, problem_file, worldmodel_file)
    # Load the plan from the JSON file
    plan = plan_from_json(domain, problem, plan_file)
    plan_valid = check_plan(plan)
    print(f"Plan {plan.name} loaded. Valid: {plan_valid}.")
