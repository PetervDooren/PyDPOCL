import unittest

from PyPOCL.worldmodel import load_domain_and_problem

class TestLoadDomainProblem(unittest.TestCase):
    def setUp(self):
        domain_file = 'tests/domains/test-domain.pddl'
        problem_file = 'tests/domains/test-problem.pddl'
        worldmodel_file = 'tests/domains/test-worldmodel.json'

        # load domain and problem
        domain, problem = load_domain_and_problem(domain_file, problem_file, worldmodel_file)

        self.domain = domain
        self.problem = problem

    def test_all_within(self):
        for o in self.problem.objects:
            if o.typ == 'item' or 'item' in self.domain.object_types[o.typ]:
                for a in self.problem.areas.keys():
                    for e in self.problem.init.effects:
                        if e.name == 'within' and e.Args == [o, a]:
                            break
                    else:
                        self.assertTrue(False, f"There is no 'within({o}, {a})' predicate in the initial state")
    
    def test_init_pose(self):
        # assert objects start in their initial poses
        self.assertTrue(all([e.truth for e in self.problem.init.effects if e.name=='within' and e.Args[0].name=='boxa' and e.Args[1].name=='boxa_init_pos']), "Box A is not within its initial pose")
        self.assertTrue(all([e.truth for e in self.problem.init.effects if e.name=='within' and e.Args[0].name=='boxb' and e.Args[1].name=='boxb_init_pos']), "Box A is not within its initial pose")
        self.assertTrue(all([e.truth for e in self.problem.init.effects if e.name=='within' and e.Args[0].name=='boxc' and e.Args[1].name=='boxc_init_pos']), "Box A is not within its initial pose")
    
    def test_within_truth(self):
        # check that the correct conditions are True
        self.assertTrue(all([e.truth for e in self.problem.init.effects if e.name=='within' and e.Args[0].name=='boxa' and e.Args[1].name=='table']), "Box A is not within the table")
        self.assertTrue(all([e.truth for e in self.problem.init.effects if e.name=='within' and e.Args[0].name=='boxa' and e.Args[1].name=='goal_left']), "Box A is not within the left goal")
        self.assertTrue(all([e.truth for e in self.problem.init.effects if e.name=='within' and e.Args[0].name=='boxb' and e.Args[1].name=='reach_robot_left']), "Box B is not within reach of the left robot")
        self.assertTrue(all([not e.truth for e in self.problem.init.effects if e.name=='within' and e.Args[0].name=='boxb' and e.Args[1].name=='reach_robot_right']), "Box B is within reach of the right robot")
        self.assertTrue(all([not e.truth for e in self.problem.init.effects if e.name=='within' and e.Args[0].name=='boxb' and e.Args[1].name=='goal_left']), "Box B is within goal_left")
        self.assertTrue(all([e.truth for e in self.problem.init.effects if e.name=='within' and e.Args[0].name=='boxc' and e.Args[1].name=='goal_right']), "Box C is not within goal_right")
        
        
if __name__ == '__main__':
    unittest.main()