import unittest

from PyPOCL.GPlan import GPlan
from PyPOCL.worldmodel import load_domain_and_problem

class TestPlanManipulation(unittest.TestCase):
    def setUp(self):
        # creates a plan with six actions moving three boxes. 
        #              init
        #       /        |      \
        #      v         v       v
        #    action1  action3   action5
        #      |         |        |
        #      v         v        v
        #    action2  action4   action6
        #       \         |      /
        #        v        v     v
        #               goal
        #
        domain_file = 'tests/domains/test-domain.pddl'
        problem_file = 'tests/domains/test-problem-3obj.pddl'
        worldmodel_file = 'tests/domains/test-worldmodel-3obj.json'

        # load domain and problem
        domain, problem = load_domain_and_problem(domain_file, problem_file, worldmodel_file)

        root_plan = GPlan.make_root_plan(domain, problem)
        operators = domain.operators

        # list goal conditions
        goal_list = [None, None, None]
        for goal in problem.goal.preconds:
            # check which box
            i_box = int(goal.Args[0].name[-1])
            goal_list[i_box] = goal

        init_list = [None, None, None, None, None]
        for obj, arg in problem.initial_positions.items():
            i_box = int(obj.name[-1])
            for eff in problem.init.effects:
                if eff.name == "within" and eff.Args[0] == obj and eff.Args[1] == arg and eff.truth == True:
                    init_list[i_box] = eff
                    break

        # setup a plan with six monomove actions
        self.plan = root_plan.instantiate('[construct]')
        monomove_ops = [o for o in operators if o.schema=='movemono']
        action1 = monomove_ops[0].instantiate()
        action2 = monomove_ops[0].instantiate()
        action3 = monomove_ops[0].instantiate()
        action4 = monomove_ops[0].instantiate()
        action5 = monomove_ops[0].instantiate()
        action6 = monomove_ops[0].instantiate()
        self.plan.insert(action1)
        self.plan.insert(action2)
        self.plan.insert(action3)
        self.plan.insert(action4)
        self.plan.insert(action5)
        self.plan.insert(action6)

        # add causal links and orderings
        for i in range(len(action1.preconds)):
            if action1.preconds[i].name == "within" and action1.preconds[i].truth == True:
                i_precondition = i
                break
        for i in range(len(action1.effects)):
            if action1.effects[i].name == "within" and action1.effects[i].truth == True:
                i_effect = i
                break
        
        # actions 1 and 2 move box 0
        boxnum = 0
        self.plan.resolve(self.plan.dummy.init, action1, init_list[boxnum], action1.preconds[i_precondition])
        self.plan.resolve(action1, action2, action1.effects[i_effect], action2.preconds[i_precondition])
        self.plan.resolve(action2, self.plan.dummy.goal, action2.effects[i_effect], goal_list[boxnum])

        # actions 3 and 4 move box 1
        boxnum = 1
        self.plan.resolve(self.plan.dummy.init, action3, init_list[boxnum], action3.preconds[i_precondition])
        self.plan.resolve(action3, action4, action3.effects[i_effect], action4.preconds[i_precondition])
        self.plan.resolve(action4, self.plan.dummy.goal, action4.effects[i_effect], goal_list[boxnum])

        # actions 5 and 6 move box 2
        boxnum = 2
        self.plan.resolve(self.plan.dummy.init, action5, init_list[boxnum], action5.preconds[i_precondition])
        self.plan.resolve(action5, action6, action5.effects[i_effect], action6.preconds[i_precondition])
        self.plan.resolve(action6, self.plan.dummy.goal, action6.effects[i_effect], goal_list[boxnum])

        # save start and goal areas for convenience
        self.startareas = [action1.Args[2], action2.Args[2], action3.Args[2], action4.Args[2], action5.Args[2], action6.Args[2]]
        self.goalareas = [action1.Args[3], action2.Args[3], action3.Args[3], action4.Args[3], action5.Args[3], action6.Args[3]]
        self.init_areas = [condition.Args[1] for condition in init_list]

        #self.plan.print()
        #visualize_plan(self.plan)
        #plan_to_dot(self.plan)
    
    def test_set_disjunctions1(self):
        # set disjunctions for the startarea of action1
        testarea = self.startareas[0]  # startarea of action1

        new_plan = self.plan.instantiate('[copy]')
        new_plan.set_disjunctions(testarea)

        # verify the correct disjunctions have been set
        expected_areas = [self.startareas[2],
                          self.startareas[4],
                          self.goalareas[2],
                          self.goalareas[3],
                          self.goalareas[4],
                          self.goalareas[5],
                          self.init_areas[3], # box 3 static pose
                          self.init_areas[4] # box 4 static pose
                          ]
        self.assertEqual(len(new_plan.variableBindings.geometric_vb.disjunctions[testarea]), len(expected_areas), "More or less disjunctions than expected")
        for area in expected_areas:
            self.assertIn(area, new_plan.variableBindings.geometric_vb.disjunctions[testarea], f"area {area} not found in disjunctions {new_plan.variableBindings.geometric_vb.disjunctions[testarea]}")
        for area in new_plan.variableBindings.geometric_vb.disjunctions[testarea]:
            self.assertIn(area, expected_areas, f"area {area} found in disjunctions but not expected in {expected_areas}")
    
    def test_set_disjunctions2(self):
        # set disjunctions for the goalarea of action1
        testarea = self.goalareas[0]  # goalarea of action1

        new_plan = self.plan.instantiate('[copy]')
        new_plan.set_disjunctions(testarea)

        # verify the correct disjunctions have been set
        expected_areas = [self.startareas[2],
                          self.startareas[4],
                          self.goalareas[2],
                          self.goalareas[3],
                          self.goalareas[4],
                          self.goalareas[5],
                          self.init_areas[3], # box 3 static pose
                          self.init_areas[4] # box 4 static pose
                          ]
        self.assertEqual(len(new_plan.variableBindings.geometric_vb.disjunctions[testarea]), len(expected_areas), "More or less disjunctions than expected")
        for area in expected_areas:
            self.assertIn(area, new_plan.variableBindings.geometric_vb.disjunctions[testarea], f"area {area} not found in disjunctions {new_plan.variableBindings.geometric_vb.disjunctions[testarea]}")
        for area in new_plan.variableBindings.geometric_vb.disjunctions[testarea]:
            self.assertIn(area, expected_areas, f"area {area} found in disjunctions but not expected in {expected_areas}")
    
    def test_set_disjunctions3(self):
        # set disjunctions for the startarea of action2
        testarea = self.startareas[1]  # startarea of action2

        new_plan = self.plan.instantiate('[copy]')
        new_plan.set_disjunctions(testarea)

        # verify the correct disjunctions have been set
        expected_areas = [self.startareas[2],
                          self.startareas[4],
                          self.goalareas[2],
                          self.goalareas[3],
                          self.goalareas[4],
                          self.goalareas[5],
                          self.init_areas[3], # box 3 static pose
                          self.init_areas[4] # box 4 static pose
                          ]
        self.assertEqual(len(new_plan.variableBindings.geometric_vb.disjunctions[testarea]), len(expected_areas), "More or less disjunctions than expected")
        for area in expected_areas:
            self.assertIn(area, new_plan.variableBindings.geometric_vb.disjunctions[testarea], f"area {area} not found in disjunctions {new_plan.variableBindings.geometric_vb.disjunctions[testarea]}")
        for area in new_plan.variableBindings.geometric_vb.disjunctions[testarea]:
            self.assertIn(area, expected_areas, f"area {area} found in disjunctions but not expected in {expected_areas}")
    
    def test_set_disjunctions4(self):
        # set disjunctions for the goalarea of action2
        testarea = self.goalareas[1]  # goalarea of action2

        new_plan = self.plan.instantiate('[copy]')
        new_plan.set_disjunctions(testarea)

        # verify the correct disjunctions have been set
        expected_areas = [self.startareas[2],
                          self.startareas[4],
                          self.goalareas[2],
                          self.goalareas[3],
                          self.goalareas[4],
                          self.goalareas[5],
                          self.init_areas[3], # box 3 static pose
                          self.init_areas[4] # box 4 static pose
                          ]
        self.assertEqual(len(new_plan.variableBindings.geometric_vb.disjunctions[testarea]), len(expected_areas), "More or less disjunctions than expected")
        for area in expected_areas:
            self.assertIn(area, new_plan.variableBindings.geometric_vb.disjunctions[testarea], f"area {area} not found in disjunctions {new_plan.variableBindings.geometric_vb.disjunctions[testarea]}")
        for area in new_plan.variableBindings.geometric_vb.disjunctions[testarea]:
            self.assertIn(area, expected_areas, f"area {area} found in disjunctions but not expected in {expected_areas}")
    
    def test_set_disjunctions_ordering1(self):
        # add more orderings to the plan to create a situation where some disjunctions are not needed
        testarea = self.startareas[1]  # startarea of action2

        new_plan = self.plan.instantiate('[copy]')
        new_plan.OrderingGraph.addOrdering(new_plan.steps[4], new_plan.steps[2])  # action3 before action1
        new_plan.set_disjunctions(testarea)

        # verify the correct disjunctions have been set
        expected_areas = [#self.startareas[2],
                          self.startareas[4],
                          self.goalareas[2],
                          self.goalareas[3],
                          self.goalareas[4],
                          self.goalareas[5],
                          self.init_areas[3], # box 3 static pose
                          self.init_areas[4] # box 4 static pose
                          ]
        #self.assertEqual(len(new_plan.variableBindings.geometric_vb.disjunctions[testarea]), len(expected_areas), "More or less disjunctions than expected")
        for area in expected_areas:
            self.assertIn(area, new_plan.variableBindings.geometric_vb.disjunctions[testarea], f"area {area} not found in disjunctions {new_plan.variableBindings.geometric_vb.disjunctions[testarea]}")
        for area in new_plan.variableBindings.geometric_vb.disjunctions[testarea]:
            self.assertIn(area, expected_areas, f"area {area} found in disjunctions but not expected in {expected_areas}")

    def test_set_disjunctions_ordering2(self):
        # add more orderings to the plan to create a situation where some disjunctions are not needed
        testarea = self.startareas[1]  # startarea of action2

        new_plan = self.plan.instantiate('[copy]')
        new_plan.OrderingGraph.addOrdering(new_plan.steps[5], new_plan.steps[2])  # action4 before action1
        new_plan.set_disjunctions(testarea)

        # verify the correct disjunctions have been set
        expected_areas = [#self.startareas[2],
                          self.startareas[4],
                          #self.goalareas[2],
                          self.goalareas[3],
                          self.goalareas[4],
                          self.goalareas[5],
                          self.init_areas[3], # box 3 static pose
                          self.init_areas[4] # box 4 static pose
                          ]
        #self.assertEqual(len(new_plan.variableBindings.geometric_vb.disjunctions[testarea]), len(expected_areas), "More or less disjunctions than expected")
        for area in expected_areas:
            self.assertIn(area, new_plan.variableBindings.geometric_vb.disjunctions[testarea], f"area {area} not found in disjunctions {new_plan.variableBindings.geometric_vb.disjunctions[testarea]}")
        for area in new_plan.variableBindings.geometric_vb.disjunctions[testarea]:
            self.assertIn(area, expected_areas, f"area {area} found in disjunctions but not expected in {expected_areas}")
    
    def test_set_disjunctions_ordering3(self):
        # add more orderings to the plan to create a situation where some disjunctions are not needed
        testarea = self.startareas[1]  # startarea of action2

        new_plan = self.plan.instantiate('[copy]')
        new_plan.OrderingGraph.addOrdering(new_plan.steps[3], new_plan.steps[4])  # action2 before action3
        new_plan.set_disjunctions(testarea)

        # verify the correct disjunctions have been set
        expected_areas = [self.startareas[2],
                          self.startareas[4],
                          #self.goalareas[2],
                          #self.goalareas[3],
                          self.goalareas[4],
                          self.goalareas[5],
                          self.init_areas[3], # box 3 static pose
                          self.init_areas[4] # box 4 static pose
                          ]
        #self.assertEqual(len(new_plan.variableBindings.geometric_vb.disjunctions[testarea]), len(expected_areas), "More or less disjunctions than expected")
        for area in expected_areas:
            self.assertIn(area, new_plan.variableBindings.geometric_vb.disjunctions[testarea], f"area {area} not found in disjunctions {new_plan.variableBindings.geometric_vb.disjunctions[testarea]}")
        for area in new_plan.variableBindings.geometric_vb.disjunctions[testarea]:
            self.assertIn(area, expected_areas, f"area {area} found in disjunctions but not expected in {expected_areas}")

    def test_set_disjunctions_ordering4(self):
        # add more orderings to the plan to create a situation where some disjunctions are not needed
        testarea = self.startareas[1]  # startarea of action2

        new_plan = self.plan.instantiate('[copy]')
        new_plan.OrderingGraph.addOrdering(new_plan.steps[3], new_plan.steps[5])  # action2 before action4
        new_plan.set_disjunctions(testarea)

        # verify the correct disjunctions have been set
        expected_areas = [self.startareas[2],
                          self.startareas[4],
                          self.goalareas[2],
                          #self.goalareas[3],
                          self.goalareas[4],
                          self.goalareas[5],
                          self.init_areas[3], # box 3 static pose
                          self.init_areas[4] # box 4 static pose
                          ]
        #self.assertEqual(len(new_plan.variableBindings.geometric_vb.disjunctions[testarea]), len(expected_areas), "More or less disjunctions than expected")
        for area in expected_areas:
            self.assertIn(area, new_plan.variableBindings.geometric_vb.disjunctions[testarea], f"area {area} not found in disjunctions {new_plan.variableBindings.geometric_vb.disjunctions[testarea]}")
        for area in new_plan.variableBindings.geometric_vb.disjunctions[testarea]:
            self.assertIn(area, expected_areas, f"area {area} found in disjunctions but not expected in {expected_areas}")
if __name__ == '__main__':
    unittest.main()