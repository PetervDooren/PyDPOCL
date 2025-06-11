import unittest

from uuid import uuid4
from collections import defaultdict
from Ground_Compiler_Library.VariableBindingsGeometric import VariableBindingsGeometric
from Ground_Compiler_Library.Element import Argument
from shapely import Polygon, overlaps

class TestVariableBindingsGeometric(unittest.TestCase):

    def test_unify_area_area(self):
        """
        test unification of two predefined areas
        """
        vb = VariableBindingsGeometric()
        
        area_args = {"base": Argument(),
                     "A": Argument(),
                     "B": Argument(),
                     "C": Argument()}
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

        area_args = {"base": Argument(),
                     "A": Argument(),
                     "B": Argument(),
                     "C": Argument()}
        areas = {}
        areas[area_args["base"]] = Polygon([(0, 0), (0, 1), (1, 1), (1, 0)])
        areas[area_args["A"]] = Polygon([(0, 0), (0, 0.5), (0.5, 0.5), (0.5, 0)])
        areas[area_args["B"]] = Polygon([(0, 0), (0, 0.2), (0.2, 0.2), (0.2, 0)]) # fits in A
        areas[area_args["C"]] = Polygon([(0, 0.3), (0, 0.7), (0.4, 0.7), (0.4, 0.3)]) # does not fit in A
        vb.set_areas(areas)
        vb.set_base_area(area_args["base"])

        objects = {"A": Argument()}
        variables = {"A": Argument(),
                     "B": Argument(),
                     "C": Argument()}

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

        area_args = {"base": Argument(),
                     "A": Argument(),
                     "B": Argument(),
                     "C": Argument()}
        areas = {}
        areas[area_args["base"]] = Polygon([(0, 0), (0, 1), (1, 1), (1, 0)])
        areas[area_args["A"]] = Polygon([(0, 0), (0, 0.5), (0.5, 0.5), (0.5, 0)])
        areas[area_args["B"]] = Polygon([(0, 0), (0, 0.2), (0.2, 0.2), (0.2, 0)]) # fits in A
        areas[area_args["C"]] = Polygon([(0, 0.3), (0, 0.7), (0.4, 0.7), (0.4, 0.3)]) # does not fit in A
        vb.set_areas(areas)
        vb.set_base_area(area_args["base"])

        objects = {"A": Argument()}
        variables = {"A": Argument(),
                     "B": Argument(),
                     "C": Argument()}

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

        area_args = {"base": Argument(),
                     "A": Argument(),
                     "B": Argument(),
                     "C": Argument()}
        areas = {}
        areas[area_args["base"]] = Polygon([(0, 0), (0, 1), (1, 1), (1, 0)])
        areas[area_args["A"]] = Polygon([(0, 0), (0, 0.5), (0.5, 0.5), (0.5, 0)])
        areas[area_args["B"]] = Polygon([(0, 0), (0, 0.2), (0.2, 0.2), (0.2, 0)]) # fits in A
        areas[area_args["C"]] = Polygon([(0, 0.3), (0, 0.7), (0.4, 0.7), (0.4, 0.3)]) # does not fit in A
        vb.set_areas(areas)
        vb.set_base_area(area_args["base"])

        objects = {"A": Argument()}
        variables = {"A": Argument(),
                     "B": Argument(),
                     "C": Argument()}

        vb.register_variable(variables["A"], objects["A"], 0.2, 0.2)
        vb.register_variable(variables["B"], objects["A"], 0.2, 0.2)

        self.assertTrue(vb.unify(variables["A"], variables["B"]), "within(A,B) could not be added")
        self.assertTrue(vb.resolve(), "could not resolve variablebindings")

        area_A = vb.placelocs[variables["A"]].area_assigned
        area_B = vb.placelocs[variables["B"]].area_assigned
        self.assertIsNotNone(area_A)
        self.assertIsNotNone(area_B)

    def test_disjunct(self):
        vb = VariableBindingsGeometric()

        area_args = {"base": Argument(),
                     "A": Argument(),
                     "B": Argument(),
                     "C": Argument()}
        areas = {}
        areas[area_args["base"]] = Polygon([(0, 0), (0, 1), (1, 1), (1, 0)])
        areas[area_args["A"]] = Polygon([(0, 0), (0, 0.5), (0.5, 0.5), (0.5, 0)])
        areas[area_args["B"]] = Polygon([(0, 0), (0, 0.2), (0.2, 0.2), (0.2, 0)]) # fits in A
        areas[area_args["C"]] = Polygon([(0, 0.3), (0, 0.7), (0.4, 0.7), (0.4, 0.3)]) # does not fit in A
        vb.set_areas(areas)
        vb.set_base_area(area_args["base"])

        objects = {"A": Argument(),
                   "B": Argument()}
        variables = {"A": Argument(),
                     "B": Argument(),
                     "C": Argument()}

        vb.register_variable(variables["A"], objects["A"], 0.2, 0.2)
        vb.register_variable(variables["B"], objects["B"], 0.3, 0.3)

        self.assertTrue(vb.add_disjunction(variables["A"], variables["B"]), "disjunct(A,B) could not be added")
        self.assertTrue(vb.resolve(), "could not resolve variablebindings")
        
        area_A = vb.placelocs[variables["A"]].area_assigned
        area_B = vb.placelocs[variables["B"]].area_assigned
        self.assertIsNotNone(area_A)
        self.assertIsNotNone(area_B)
        self.assertFalse(overlaps(area_A, area_B))

if __name__ == '__main__':
    unittest.main()