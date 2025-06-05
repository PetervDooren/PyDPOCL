import unittest

from GPlan import GPlan
from worldmodel import load_worldmodel, update_init_state, just_compile

class TestVariableBindings(unittest.TestCase):
    def setUp(self):
        domain_file = 'test/domains/test-domain.pddl'
        problem_file = 'test/domains/test-problem.pddl'
        worldmodel_file = 'test/domains/test-worldmodel.json'

        ground_steps, objects, object_types = just_compile(domain_file, problem_file)

        # load worldmodel
        objects, area_mapping, object_mapping, object_area_mapping, robot_reach = load_worldmodel(worldmodel_file, objects)

        init_state = ground_steps[-2]
        init_state = update_init_state(init_state, area_mapping, object_area_mapping)

        goal_state = ground_steps[-1]
        self.root_plan = GPlan.make_root_plan(init_state, goal_state, objects, object_types)
        self.operators = ground_steps

    def test_add_step(self):
        # find movemono step
        ops = [o for o in self.operators if o.schema=='movemono']
        action = ops[0].instantiate()

        new_plan = self.root_plan.instantiate('1[a]')
        new_plan.insert(action)

        # test action is added
        self.assertIn(action, new_plan)

        # test action cannot be before start or after end
        self.assertTrue(new_plan.OrderingGraph.isPath(new_plan.dummy.init, action))
        self.assertFalse(new_plan.OrderingGraph.isPath(new_plan.dummy.goal, action))
        self.assertFalse(new_plan.OrderingGraph.isPath(action, new_plan.dummy.init))
        self.assertTrue(new_plan.OrderingGraph.isPath(action, new_plan.dummy.goal))       

if __name__ == '__main__':
    unittest.main()