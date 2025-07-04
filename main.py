import sys
from PyPOCL.PyDPOCL import POCLPlanner
from PyPOCL.worldmodel import load_domain_and_problem

if __name__ == '__main__':
    num_args = len(sys.argv)
    if num_args >1:
        domain_file = sys.argv[1]
        if num_args > 2:
            problem_file = sys.argv[2]
    else:
        #domain_file = 'Ground_Compiler_Library//domains/ark-domain.pddl'
        #problem_file = 'Ground_Compiler_Library//domains/ark-problem.pddl'
        #domain_file = 'Ground_Compiler_Library//domains/manipulation-domain-symbolic.pddl'
        #problem_file = 'Ground_Compiler_Library//domains/manipulation-problem-symbolic.pddl'
        domain_file = 'domains/manipulation-domain/manipulation-domain.pddl'
        problem_file = 'domains/manipulation-domain/manipulation-problem.pddl'
        worldmodel_file = 'domains/manipulation-domain/manipulation-worldmodel.json'

    domain, problem = load_domain_and_problem(domain_file, problem_file, worldmodel_file)

    planner = POCLPlanner(domain, problem)
    plans = planner.solve(k=1)
    for i in range(len(plans)):
        plan = plans[i]
        if not plan.check_plan():
            print("Error: plan is not valid")
        else:
            print("Plan is valid")
        plan.to_json(f"plans/manipulation-domain/manipulation-problem-plan_{i}.json")
