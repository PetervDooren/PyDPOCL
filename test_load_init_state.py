import unittest

from worldmodel import load_worldmodel, update_init_state, just_compile

class TestVariableBindings(unittest.TestCase):

    def test_load_init_state(self):
        domain_file = 'test/domains/test-domain.pddl'
        problem_file = 'test/domains/test-problem.pddl'
        worldmodel_file = 'test/domains/test-worldmodel.json'

        ground_steps, objects, object_types = just_compile(domain_file, problem_file)

        # load worldmodel
        objects, area_mapping, object_mapping, object_area_mapping, robot_reach = load_worldmodel(worldmodel_file, objects)

        init_state = ground_steps[-2]
        init_state = update_init_state(init_state, area_mapping, object_area_mapping)
        for o in objects:
            if o.typ == 'item' or 'item' in object_types[o.typ]:
                for a in area_mapping.keys():
                    for e in init_state.effects:
                        if e.name == 'within' and e.Args == [o, a]:
                            break
                    else:
                        self.assertTrue(False, f"There is no 'within({o}, {a})' predicate in the initial state")
        
        # check that the correct conditions are True
        self.assertTrue(all([e.truth for e in init_state.effects if e.name=='within' and e.Args[0].name=='boxa' and e.Args[1].name=='table']), "Box A is not within the table")
        self.assertTrue(all([e.truth for e in init_state.effects if e.name=='within' and e.Args[0].name=='boxa' and e.Args[1].name=='goal_left']), "Box A is not within the left goal")
        self.assertTrue(all([e.truth for e in init_state.effects if e.name=='within' and e.Args[0].name=='boxb' and e.Args[1].name=='reach_robot_left']), "Box B is not within reach of the left robot")
        self.assertTrue(all([not e.truth for e in init_state.effects if e.name=='within' and e.Args[0].name=='boxb' and e.Args[1].name=='goal_left']), "Box B is within goal_left")
        self.assertTrue(all([not e.truth for e in init_state.effects if e.name=='within' and e.Args[0].name=='boxc' and e.Args[1].name=='goal_right']), "Box C is within goal_right")
        # assert objects start in their initial poses
        self.assertTrue(all([e.truth for e in init_state.effects if e.name=='within' and e.Args[0].name=='boxa' and e.Args[1].name=='boxa_init_pos']), "Box A is not within its initial pose")
        self.assertTrue(all([e.truth for e in init_state.effects if e.name=='within' and e.Args[0].name=='boxb' and e.Args[1].name=='boxb_init_pos']), "Box A is not within its initial pose")
        self.assertTrue(all([e.truth for e in init_state.effects if e.name=='within' and e.Args[0].name=='boxc' and e.Args[1].name=='boxc_init_pos']), "Box A is not within its initial pose")
        
if __name__ == '__main__':
    unittest.main()