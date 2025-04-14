import unittest

from uuid import uuid4
from collections import defaultdict
from Ground_Compiler_Library.VariableBindingsGeometric import VariableBindingsGeometric
from Ground_Compiler_Library.Element import Argument
from shapely import Polygon

class TestVariableBindingsGeometric(unittest.TestCase):

    def test_unify(self):
        vb = VariableBindingsGeometric()

        base_area = Polygon([(0, 0), (0, 1), (1, 1), (1, 0)])
        vb.set_base_area(base_area)

        objects = {"A": Argument()}
        variables = {"A": Argument(),
                     "B": Argument(),
                     "C": Argument()}

        vb.register_variable(variables["A"], objects["A"])
        vb.register_variable(variables["B"], objects["A"])

        self.assertTrue(vb.unify(variables["A"], variables["B"]), "within(A,B) could not be added")
        self.assertTrue(vb.resolve(), "could not resolve variablebindings")

if __name__ == '__main__':
    unittest.main()