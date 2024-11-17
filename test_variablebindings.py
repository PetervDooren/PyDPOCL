import unittest

from uuid import uuid4
from Ground_Compiler_Library.VariableBindings import VariableBindings

class TestVariableBindings(unittest.TestCase):

    def test_codesignation(self):
        vb = VariableBindings()
        variables = {"A": uuid4(),
                     "B": uuid4(),
                     "C": uuid4()}
        
        vb.register_variable(variables["A"])
        vb.register_variable(variables["B"])
        vb.register_variable(variables["C"])
        
        self.assertFalse(vb.is_codesignated(variables["A"], variables["B"]), "A and B are codesignated from instantiation")
        self.assertTrue(vb.add_codesignation(variables["A"], variables["B"]), "codesignation between A and B could not be instantiated")
        self.assertTrue(vb.is_codesignated(variables["A"], variables["B"]), "A and B are not codesignated after codesignation")

    def test_can_codesignate(self):
        vb = VariableBindings()
        variables = {"A": uuid4(),
                     "B": uuid4(),
                     "C": uuid4()}
        
        vb.register_variable(variables["A"])
        vb.register_variable(variables["B"])
        vb.register_variable(variables["C"])
        
        self.assertTrue(vb.can_codesignate(variables["A"], variables["B"]), "A and B cannot codesignated from instantiation")
        self.assertTrue(vb.add_non_codesignation(variables["A"], variables["B"]), "non codesignation between A and B could not be instantiated")
        self.assertFalse(vb.can_codesignate(variables["A"], variables["B"]), "A and B can be codesignated after non codesignation")

    def test_codesignation_multi(self):
        vb = VariableBindings()
        variables = {"A": uuid4(),
                     "B": uuid4(),
                     "C": uuid4()}
        
        vb.register_variable(variables["A"])
        vb.register_variable(variables["B"])
        vb.register_variable(variables["C"])

        self.assertTrue(vb.add_codesignation(variables["A"], variables["B"]), "initial codesignation between A and B could not be instantiated")
        self.assertTrue(vb.add_codesignation(variables["B"], variables["C"]), "second codesignation between B and C could not be instantiated")
        self.assertTrue(vb.add_codesignation(variables["A"], variables["C"]), "third codesignation between A and C could not be instantiated")

    def test_non_codesignation(self):
        vb = VariableBindings()
        variables = {"A": uuid4(),
                     "B": uuid4(),
                     "C": uuid4()}

        vb.register_variable(variables["A"])
        vb.register_variable(variables["B"])
        vb.register_variable(variables["C"])

        self.assertTrue(vb.add_non_codesignation(variables["A"], variables["B"]), "initial non codesignation between A and B could not be instantiated")
        self.assertTrue(vb.add_non_codesignation(variables["B"], variables["C"]), "second non codesignation between B and C could not be instantiated")
        self.assertTrue(vb.add_non_codesignation(variables["A"], variables["C"]), "third non codesignation between A and C could not be instantiated")

    def test_direct_conflict(self):
        vb = VariableBindings()
        variables = {"A": uuid4(),
                     "B": uuid4(),
                     "C": uuid4()}
        
        vb.register_variable(variables["A"])
        vb.register_variable(variables["B"])
        vb.register_variable(variables["C"])

        self.assertTrue(vb.add_codesignation(variables["A"], variables["B"]), "initial codesignation between A and B could not be instantiated")
        self.assertFalse(vb.add_non_codesignation(variables["A"], variables["B"]), "non codesignation between A and B can be instantiated, which it should not")

    def test_direct_conflict2(self):
        vb = VariableBindings()
        variables = {"A": uuid4(),
                     "B": uuid4(),
                     "C": uuid4()}

        vb.register_variable(variables["A"])
        vb.register_variable(variables["B"])
        vb.register_variable(variables["C"])

        self.assertTrue(vb.add_non_codesignation(variables["A"], variables["B"]), "initial codesignation between A and B could not be instantiated")
        self.assertFalse(vb.add_codesignation(variables["A"], variables["B"]), "non codesignation between A and B can be instantiated, which it should not")

    
    def test_indirect_conflict(self):
        vb = VariableBindings()
        variables = {"A": uuid4(),
                     "B": uuid4(),
                     "C": uuid4()}

        vb.register_variable(variables["A"])
        vb.register_variable(variables["B"])
        vb.register_variable(variables["C"])
        
        self.assertTrue(vb.add_codesignation(variables["A"], variables["B"]), "initial codesignation between A and B could not be instantiated")
        self.assertTrue(vb.add_codesignation(variables["B"], variables["C"]), "codesignation between B and C could not be instantiated")
        self.assertFalse(vb.add_non_codesignation(variables["A"], variables["C"]), "non codesignation between A and C can be instantiated, which it should not")

    def test_non_codesignation_chaining1(self):
        vb = VariableBindings()
        variables = {"A": uuid4(),
                     "B": uuid4(),
                     "C": uuid4()}

        vb.register_variable(variables["A"])
        vb.register_variable(variables["B"])
        vb.register_variable(variables["C"])

        self.assertTrue(vb.add_non_codesignation(variables["A"], variables["B"]), "initial non codesignation between A and B could not be instantiated")
        self.assertTrue(vb.add_non_codesignation(variables["B"], variables["C"]), "non codesignation between B and C could not be instantiated")
        self.assertTrue(vb.add_codesignation(variables["A"], variables["C"]), "codesignation between A and C could not be instantiated")

    def test_non_codesignation_chaining2(self):
        vb = VariableBindings()
        variables = {"A": uuid4(),
                     "B": uuid4(),
                     "C": uuid4()}

        vb.register_variable(variables["A"])
        vb.register_variable(variables["B"])
        vb.register_variable(variables["C"])

        self.assertTrue(vb.add_codesignation(variables["A"], variables["B"]))
        self.assertTrue(vb.add_non_codesignation(variables["B"], variables["C"]))
        self.assertFalse(vb.add_codesignation(variables["A"], variables["C"]))

    def test_const_assigning(self):
        vb = VariableBindings()
        variables = {"A": uuid4(),
                     "B": uuid4(),
                     "C": uuid4()}
        
        objects = {"A": uuid4(),
                   "B": uuid4(),
                   "C": uuid4()}
        vb.set_objects(objects.values(), None)

        vb.register_variable(objects["A"])
        vb.register_variable(variables["B"])
        vb.register_variable(objects["C"])

        self.assertIsNotNone(vb.const[objects["A"]], "A is not recognised as a constant")
        self.assertTrue(vb.can_codesignate(objects["A"], variables["B"]))
        self.assertTrue(vb.add_codesignation(objects["A"], variables["B"]))
        self.assertTrue(vb.can_codesignate(objects["A"], variables["B"]))
        self.assertIsNotNone(vb.const[variables["B"]], "B is not recognised as grounded")

        self.assertFalse(vb.can_codesignate(objects["A"], objects["C"]), "two unidentical constants can codesignate")
        self.assertFalse(vb.can_codesignate(objects["C"], variables["B"]), "two unidentical constants can codesignate")


if __name__ == '__main__':
    unittest.main()