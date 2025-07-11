import unittest

from uuid import uuid4
from collections import defaultdict
from PyPOCL.Ground_Compiler_Library.VariableBindingsGeometric import VariableBindingsGeometric
from PyPOCL.Ground_Compiler_Library.Element import Argument
from shapely import Polygon, overlaps, within

class TestVariableBindingsGeometric(unittest.TestCase):

    def test_unify_area_area(self):
        """
        test unification of two predefined areas
        """
        vb = VariableBindingsGeometric()
        
        area_args = {"base": Argument(name="area_base"),
                     "A": Argument(name="area_A"),
                     "B": Argument(name="area_B"),
                     "C": Argument(name="area_C")}
        areas = {}
        areas[area_args["base"]] = Polygon([(0, 0), (0, 1), (1, 1), (1, 0)])
        areas[area_args["A"]] = Polygon([(0, 0), (0, 0.5), (0.5, 0.5), (0.5, 0)])
        areas[area_args["B"]] = Polygon([(0, 0), (0, 0.2), (0.2, 0.2), (0.2, 0)]) # fits in A
        areas[area_args["C"]] = Polygon([(0, 0.3), (0, 0.7), (0.4, 0.7), (0.4, 0.3)]) # does not fit in A
        vb.set_areas(areas)
        vb.set_base_area(area_args["base"])

        objects = {"A": Argument()}

        self.assertTrue(vb.unify(area_args["B"], area_args["A"]), "within(A,B) could not be added")
        self.assertFalse(vb.unify(area_args["C"], area_args["A"]), "within(A,C) could be added")
        self.assertTrue(vb.resolve(), "could not resolve variablebindings")

    def test_unify_area_variable(self):
        """
        test unification of a predefined area with a variable
        """
        vb = VariableBindingsGeometric()

        area_args = {"base": Argument(name="area_base"),
                     "A": Argument(name="area_A"),
                     "B": Argument(name="area_B"),
                     "C": Argument(name="area_C")}
        areas = {}
        areas[area_args["base"]] = Polygon([(0, 0), (0, 1), (1, 1), (1, 0)])
        areas[area_args["A"]] = Polygon([(0, 0), (0, 0.5), (0.5, 0.5), (0.5, 0)])
        areas[area_args["B"]] = Polygon([(0, 0), (0, 0.2), (0.2, 0.2), (0.2, 0)]) # fits in A
        areas[area_args["C"]] = Polygon([(0, 0.3), (0, 0.7), (0.4, 0.7), (0.4, 0.3)]) # does not fit in A
        vb.set_areas(areas)
        vb.set_base_area(area_args["base"])

        objects = {"A": Argument(name="obj_A")}
        variables = {"A": Argument(name="var_A"),
                     "B": Argument(name="var_B"),
                     "C": Argument(name="var_C")}

        vb.register_variable(variables["A"], objects["A"], 0.2, 0.2)

        self.assertTrue(vb.unify(area_args["A"], variables["A"]), "within(area,var) could not be added")
        self.assertTrue(vb.resolve(), "could not resolve variablebindings")

        area_A = vb.placelocs[variables["A"]].area_assigned
        self.assertIsNotNone(area_A)

    def test_unify_variable_area(self):
        """
        test unification of a variable with a predefined areas
        """
        vb = VariableBindingsGeometric()

        area_args = {"base": Argument(name="area_base"),
                     "A": Argument(name="area_A"),
                     "B": Argument(name="area_B"),
                     "C": Argument(name="area_C")}
        areas = {}
        areas[area_args["base"]] = Polygon([(0, 0), (0, 1), (1, 1), (1, 0)])
        areas[area_args["A"]] = Polygon([(0, 0), (0, 0.5), (0.5, 0.5), (0.5, 0)])
        areas[area_args["B"]] = Polygon([(0, 0), (0, 0.2), (0.2, 0.2), (0.2, 0)]) # fits in A
        areas[area_args["C"]] = Polygon([(0, 0.3), (0, 0.7), (0.4, 0.7), (0.4, 0.3)]) # does not fit in A
        vb.set_areas(areas)
        vb.set_base_area(area_args["base"])

        objects = {"A": Argument(name="obj_A")}
        variables = {"A": Argument(name="var_A"),
                     "B": Argument(name="var_B"),
                     "C": Argument(name="var_C")}

        vb.register_variable(variables["A"], objects["A"], 0.2, 0.2)

        self.assertTrue(vb.unify(variables["A"], area_args["A"]), "within(var,area) could not be added")
        self.assertTrue(vb.resolve(), "could not resolve variablebindings")

        area_A = vb.placelocs[variables["A"]].area_assigned
        self.assertIsNotNone(area_A)

    def test_unify_variable_variable(self):
        """
        test unification of two predefined areas
        """
        vb = VariableBindingsGeometric()

        area_args = {"base": Argument(name="area_base"),
                     "A": Argument(name="area_A"),
                     "B": Argument(name="area_B"),
                     "C": Argument(name="area_C")}
        areas = {}
        areas[area_args["base"]] = Polygon([(0, 0), (0, 1), (1, 1), (1, 0)])
        areas[area_args["A"]] = Polygon([(0, 0), (0, 0.5), (0.5, 0.5), (0.5, 0)])
        areas[area_args["B"]] = Polygon([(0, 0), (0, 0.2), (0.2, 0.2), (0.2, 0)]) # fits in A
        areas[area_args["C"]] = Polygon([(0, 0.3), (0, 0.7), (0.4, 0.7), (0.4, 0.3)]) # does not fit in A
        vb.set_areas(areas)
        vb.set_base_area(area_args["base"])

        objects = {"A": Argument(name="obj_A")}
        variables = {"A": Argument(name="var_A"),
                     "B": Argument(name="var_B"),
                     "C": Argument(name="var_C")}

        vb.register_variable(variables["A"], objects["A"], 0.2, 0.2)
        vb.register_variable(variables["B"], objects["A"], 0.2, 0.2)

        self.assertTrue(vb.unify(variables["A"], variables["B"]), "within(A,B) could not be added")
        self.assertTrue(vb.resolve(), "could not resolve variablebindings")

        area_A = vb.placelocs[variables["A"]].area_assigned
        area_B = vb.placelocs[variables["B"]].area_assigned
        self.assertIsNotNone(area_A)
        self.assertIsNotNone(area_B)
    
    def test_unify_chaining(self):
        """
        test if unification constraints remain correct during multiple calls of unify
        """
        vb = VariableBindingsGeometric()

        area_args = {"base": Argument(name="area_base"),
                     "A": Argument(name="area_A"),
                     "B": Argument(name="area_B"),
                     "C": Argument(name="area_C")}
        areas = {}
        areas[area_args["base"]] = Polygon([(0, 0), (0, 1), (1, 1), (1, 0)])
        areas[area_args["A"]] = Polygon([(0, 0), (0, 0.5), (0.5, 0.5), (0.5, 0)])
        areas[area_args["B"]] = Polygon([(0, 0), (0, 0.2), (0.2, 0.2), (0.2, 0)]) # fits in A
        areas[area_args["C"]] = Polygon([(0, 0.3), (0, 0.7), (0.4, 0.7), (0.4, 0.3)]) # does not fit in A
        vb.set_areas(areas)
        vb.set_base_area(area_args["base"])

        objects = {"A": Argument(name="obj_A")}
        variables = {"A": Argument(name="var_A"),
                     "B": Argument(name="var_B"),
                     "C": Argument(name="var_C")}

        vb.register_variable(variables["A"], objects["A"], 0.2, 0.2)
        vb.register_variable(variables["B"], objects["A"], 0.2, 0.2)

        self.assertTrue(vb.unify(variables["A"], area_args["A"]), "within(varA,areaA) could not be added")
        self.assertTrue(vb.unify(area_args["B"], variables["A"]), "within(areaB,varA) could not be added")
        self.assertTrue(vb.resolve(), "could not resolve variablebindings")

        area_A = vb.placelocs[variables["A"]].area_assigned
        self.assertIsNotNone(area_A)
        self.assertTrue(within(area_A, areas[area_args["A"]]))
        self.assertTrue(within(areas[area_args["B"]], area_A))

    def test_unify_chaining_infeasible(self):
        """
        test if an unfeasible set of constraints is detected correctly
        """
        vb = VariableBindingsGeometric()

        area_args = {"base": Argument(name="area_base"),
                     "A": Argument(name="area_A"),
                     "B": Argument(name="area_B"),
                     "C": Argument(name="area_C")}
        areas = {}
        areas[area_args["base"]] = Polygon([(0, 0), (0, 1), (1, 1), (1, 0)])
        areas[area_args["A"]] = Polygon([(0, 0), (0, 0.5), (0.5, 0.5), (0.5, 0)])
        areas[area_args["B"]] = Polygon([(0, 0), (0, 0.2), (0.2, 0.2), (0.2, 0)]) # fits in A
        areas[area_args["C"]] = Polygon([(0, 0.3), (0, 0.7), (0.4, 0.7), (0.4, 0.3)]) # does not fit in A
        vb.set_areas(areas)
        vb.set_base_area(area_args["base"])

        objects = {"A": Argument(name="obj_A")}
        variables = {"A": Argument(name="var_A"),
                     "B": Argument(name="var_B"),
                     "C": Argument(name="var_C")}

        vb.register_variable(variables["A"], objects["A"], 0.2, 0.2)
        vb.register_variable(variables["B"], objects["A"], 0.2, 0.2)

        self.assertTrue(vb.unify(variables["A"], area_args["A"]), "within(varA,areaA) could not be added")
        self.assertFalse(vb.unify(area_args["C"], variables["A"]), "within(areaC,varA) could be added, it should not be possible")

    def test_unify_chaining_infeasible2(self):
        """
        test if an unfeasible set of constraints is detected correctly. different order from the above test
        """
        vb = VariableBindingsGeometric()

        area_args = {"base": Argument(name="area_base"),
                     "A": Argument(name="area_A"),
                     "B": Argument(name="area_B"),
                     "C": Argument(name="area_C")}
        areas = {}
        areas[area_args["base"]] = Polygon([(0, 0), (0, 1), (1, 1), (1, 0)])
        areas[area_args["A"]] = Polygon([(0, 0), (0, 0.5), (0.5, 0.5), (0.5, 0)])
        areas[area_args["B"]] = Polygon([(0, 0), (0, 0.2), (0.2, 0.2), (0.2, 0)]) # fits in A
        areas[area_args["C"]] = Polygon([(0, 0.3), (0, 0.7), (0.4, 0.7), (0.4, 0.3)]) # does not fit in A
        vb.set_areas(areas)
        vb.set_base_area(area_args["base"])

        objects = {"A": Argument(name="obj_A")}
        variables = {"A": Argument(name="var_A"),
                     "B": Argument(name="var_B"),
                     "C": Argument(name="var_C")}

        vb.register_variable(variables["A"], objects["A"], 0.2, 0.2)
        vb.register_variable(variables["B"], objects["A"], 0.2, 0.2)

        self.assertTrue(vb.unify(area_args["C"], variables["A"]), "within(areaC,varA) could not be added")
        self.assertFalse(vb.unify(variables["A"], area_args["A"]), "within(varA,areaA) could be added, it should not be possible")
    
    def test_unify_chaining_infeasible3(self):
        """
        test if an unfeasible set of constraints is detected correctly. chains 5 unifies together
        """
        vb = VariableBindingsGeometric()

        area_args = {"base": Argument(name="area_base"),
                     "A": Argument(name="area_A"),
                     "B": Argument(name="area_B"),
                     "C": Argument(name="area_C")}
        areas = {}
        areas[area_args["base"]] = Polygon([(0, 0), (0, 1), (1, 1), (1, 0)])
        areas[area_args["A"]] = Polygon([(0, 0), (0, 0.5), (0.5, 0.5), (0.5, 0)])
        areas[area_args["B"]] = Polygon([(0, 0), (0, 0.2), (0.2, 0.2), (0.2, 0)]) # fits in A
        areas[area_args["C"]] = Polygon([(0, 0.3), (0, 0.7), (0.4, 0.7), (0.4, 0.3)]) # does not fit in A
        vb.set_areas(areas)
        vb.set_base_area(area_args["base"])

        objects = {"A": Argument(name="obj_A")}
        variables = {"A": Argument(name="var_A"),
                     "B": Argument(name="var_B"),
                     "C": Argument(name="var_C")}

        vb.register_variable(variables["A"], objects["A"], 0.2, 0.2)
        vb.register_variable(variables["B"], objects["A"], 0.2, 0.2)
        vb.register_variable(variables["C"], objects["A"], 0.2, 0.2)

        self.assertTrue(vb.unify(area_args["C"], variables["A"]), "within(areaC,varA) could not be added")
        self.assertTrue(vb.unify(variables["B"], variables["C"]), "within(areaC,varA) could not be added")
        self.assertTrue(vb.unify(variables["A"], variables["B"]), "within(areaC,varA) could not be added")
        self.assertFalse(vb.unify(variables["C"], area_args["A"]), "within(varA,areaA) could be added, it should not be possible")

    def test_disjunct(self):
        vb = VariableBindingsGeometric()

        area_args = {"base": Argument(name="area_base"),
                     "A": Argument(name="area_A"),
                     "B": Argument(name="area_B"),
                     "C": Argument(name="area_C")}
        areas = {}
        areas[area_args["base"]] = Polygon([(0, 0), (0, 1), (1, 1), (1, 0)])
        areas[area_args["A"]] = Polygon([(0, 0), (0, 0.5), (0.5, 0.5), (0.5, 0)])
        areas[area_args["B"]] = Polygon([(0, 0), (0, 0.2), (0.2, 0.2), (0.2, 0)]) # fits in A
        areas[area_args["C"]] = Polygon([(0, 0.3), (0, 0.7), (0.4, 0.7), (0.4, 0.3)]) # does not fit in A
        vb.set_areas(areas)
        vb.set_base_area(area_args["base"])

        objects = {"A": Argument(name="obj_A"),
                   "B": Argument(name="obj_B")}
        variables = {"A": Argument(name="var_A"),
                     "B": Argument(name="var_B"),
                     "C": Argument(name="var_C")}

        vb.register_variable(variables["A"], objects["A"], 0.2, 0.2)
        vb.register_variable(variables["B"], objects["B"], 0.3, 0.3)

        self.assertTrue(vb.add_disjunction(variables["A"], variables["B"]), "disjunct(A,B) could not be added")
        self.assertTrue(vb.resolve(), "could not resolve variablebindings")
        
        area_A = vb.placelocs[variables["A"]].area_assigned
        area_B = vb.placelocs[variables["B"]].area_assigned
        self.assertIsNotNone(area_A)
        self.assertIsNotNone(area_B)
        self.assertFalse(overlaps(area_A, area_B))

    def test_resolve(self):
        """
        test if resolve can correctly assign areas given the constraints. Uses example from the manipulation domain.
        """
        vb = VariableBindingsGeometric()

        area_args = {"base": Argument(name="area_base"),
                     "goal": Argument(name="area_goal"),
                     "start_pos": Argument(name="area_start_pos"),
                     "reach_left": Argument(name="area_reach_left"),
                     "reach_right": Argument(name="area_reach_right")}
        areas = {}
        areas[area_args["base"]] = Polygon([(0, 0), (0, 3), (1, 3), (1, 0)])
        areas[area_args["goal"]] = Polygon([(0, 0), (0, 1), (1, 1), (1, 0)])
        areas[area_args["start_pos"]] = Polygon([(0.3, 2.3), (0.3, 2.7), (0.7, 2.7), (0.7, 2.3)])
        areas[area_args["reach_left"]] = Polygon([(0, 0), (0, 2), (1, 2), (1, 0)])
        areas[area_args["reach_right"]] = Polygon([(0, 1), (0, 3), (1, 3), (1, 1)])
        vb.set_areas(areas)
        vb.set_base_area(area_args["base"])

        objects = {"A": Argument(name="obj_A")}
        variables = {"start1": Argument(name="start1"),
                     "goal1": Argument(name="goal1"),
                     "start2": Argument(name="start2"),
                     "goal2": Argument(name="goal2")}
        
        vb.register_variable(variables["start1"], objects["A"], 0.2, 0.2)
        vb.register_variable(variables["goal1"], objects["A"], 0.2, 0.2)
        vb.register_variable(variables["start2"], objects["A"], 0.2, 0.2)
        vb.register_variable(variables["goal2"], objects["A"], 0.2, 0.2)

        self.assertTrue(vb.unify(variables["start1"], area_args["reach_right"]), "within(start1, reach_right) could not be added")
        self.assertTrue(vb.unify(variables["goal1"], area_args["reach_right"]), "within(goal1, reach_right) could not be added")
        self.assertTrue(vb.unify(variables["start2"], area_args["reach_left"]), "within(start2, reach_left) could not be added")
        self.assertTrue(vb.unify(variables["goal2"], area_args["reach_left"]), "within(goal2, reach_left) could not be added")

        self.assertTrue(vb.unify(variables["start1"], area_args["start_pos"]), "within(start1, start_pos) could not be added")
        self.assertTrue(vb.unify(variables["goal1"], variables["start2"]), "within(goal1, start2) could not be added")
        self.assertTrue(vb.unify(variables["goal2"], area_args["goal"]), "within(goal1, goal) could not be added")

        # resolve variables
        self.assertTrue(vb.resolve(), "could not resolve variablebindings")

        # check if areas are assigned correctly
        self.assertTrue(within(vb.placelocs[variables["start1"]].area_assigned, areas[area_args["reach_right"]]),
                        "start1 area is not assigned correctly")
        self.assertTrue(within(vb.placelocs[variables["goal1"]].area_assigned, areas[area_args["reach_right"]]),
                        "goal1 area is not assigned correctly")
        self.assertTrue(within(vb.placelocs[variables["start2"]].area_assigned, areas[area_args["reach_left"]]),
                        "start2 area is not assigned correctly")
        self.assertTrue(within(vb.placelocs[variables["goal2"]].area_assigned, areas[area_args["reach_left"]]),
                        "goal2 area is not assigned correctly")
        self.assertTrue(within(vb.placelocs[variables["start1"]].area_assigned, areas[area_args["start_pos"]]),
                        "start1 area is not assigned correctly")
        self.assertTrue(within(vb.placelocs[variables["goal1"]].area_assigned, vb.placelocs[variables["start2"]].area_assigned),
                        "goal1 area is not assigned correctly")
        self.assertTrue(within(vb.placelocs[variables["goal2"]].area_assigned, areas[area_args["goal"]]),
                        "goal2 area is not assigned correctly")


if __name__ == '__main__':
    unittest.main()