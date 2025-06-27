import sys
from PyDPOCL import POCLPlanner
from worldmodel import load_worldmodel, update_init_state, just_compile

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
        domain_file = 'Ground_Compiler_Library//domains/manipulation-domain.pddl'
        problem_file = 'Ground_Compiler_Library//domains/manipulation-problem.pddl'
        worldmodel_file = 'Ground_Compiler_Library//domains/manipulation-worldmodel.json'

    ground_steps, objects, object_types = just_compile(domain_file, problem_file)

    # load worldmodel
    objects, area_mapping, object_mapping, object_area_mapping, robot_reach = load_worldmodel(worldmodel_file, objects)

    init_state = ground_steps[-2]
    init_state = update_init_state(init_state, area_mapping, object_area_mapping)

    POCLPlanner.pre_process_operators(ground_steps)
    PLAN = 1
    if PLAN:
        planner = POCLPlanner(ground_steps, ground_steps[-2], ground_steps[-1], objects, object_types, area_mapping, robot_reach)
        planner.solve(k=1)
