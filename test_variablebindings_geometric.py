import unittest

from uuid import uuid4
from collections import defaultdict
from Ground_Compiler_Library.VariableBindingsGeometric import VariableBindingsGeometric
from Ground_Compiler_Library.Element import Argument
from shapely import Polygon, overlaps

class TestVariableBindingsGeometric(unittest.TestCase):

    def test_unify(self):
        vb = VariableBindingsGeometric()

        areas = {}
        areas['base'] = Polygon([(0, 0), (0, 1), (1, 1), (1, 0)])
        vb.set_areas(areas)
        vb.set_base_area('base')

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

        areas = {}
        areas['base'] = Polygon([(0, 0), (0, 1), (1, 1), (1, 0)])
        vb.set_areas(areas)
        vb.set_base_area('base')

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