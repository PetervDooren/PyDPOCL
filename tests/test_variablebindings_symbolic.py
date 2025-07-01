import unittest

from uuid import uuid4
from collections import defaultdict
from PyPOCL.Ground_Compiler_Library.VariableBindingsSymbolic import VariableBindingsSymbolic
from PyPOCL.Ground_Compiler_Library.Element import Argument

class TestVariableBindings(unittest.TestCase):

    def test_codesignation(self):
        vb = VariableBindingsSymbolic()
        variables = {"A": Argument(),
                     "B": Argument(),
                     "C": Argument()}
        
        vb.register_variable(variables["A"])
        vb.register_variable(variables["B"])
        vb.register_variable(variables["C"])
        
        self.assertFalse(vb.is_codesignated(variables["A"], variables["B"]), "A and B are codesignated from instantiation")
        self.assertTrue(vb.add_codesignation(variables["A"], variables["B"]), "codesignation between A and B could not be instantiated")
        self.assertTrue(vb.is_codesignated(variables["A"], variables["B"]), "A and B are not codesignated after codesignation")

    def test_can_codesignate(self):
        vb = VariableBindingsSymbolic()
        variables = {"A": Argument(),
                     "B": Argument(),
                     "C": Argument()}
        
        vb.register_variable(variables["A"])
        vb.register_variable(variables["B"])
        vb.register_variable(variables["C"])
        
        self.assertTrue(vb.can_codesignate(variables["A"], variables["B"]), "A and B cannot codesignated from instantiation")
        self.assertTrue(vb.add_non_codesignation(variables["A"], variables["B"]), "non codesignation between A and B could not be instantiated")
        self.assertFalse(vb.can_codesignate(variables["A"], variables["B"]), "A and B can be codesignated after non codesignation")

    def test_codesignation_multi(self):
        vb = VariableBindingsSymbolic()
        variables = {"A": Argument(),
                     "B": Argument(),
                     "C": Argument()}
        
        vb.register_variable(variables["A"])
        vb.register_variable(variables["B"])
        vb.register_variable(variables["C"])

        self.assertTrue(vb.add_codesignation(variables["A"], variables["B"]), "initial codesignation between A and B could not be instantiated")
        self.assertTrue(vb.add_codesignation(variables["B"], variables["C"]), "second codesignation between B and C could not be instantiated")
        self.assertTrue(vb.add_codesignation(variables["A"], variables["C"]), "third codesignation between A and C could not be instantiated")

    def test_non_codesignation(self):
        vb = VariableBindingsSymbolic()
        variables = {"A": Argument(),
                     "B": Argument(),
                     "C": Argument()}

        vb.register_variable(variables["A"])
        vb.register_variable(variables["B"])
        vb.register_variable(variables["C"])

        self.assertTrue(vb.add_non_codesignation(variables["A"], variables["B"]), "initial non codesignation between A and B could not be instantiated")
        self.assertTrue(vb.add_non_codesignation(variables["B"], variables["C"]), "second non codesignation between B and C could not be instantiated")
        self.assertTrue(vb.add_non_codesignation(variables["A"], variables["C"]), "third non codesignation between A and C could not be instantiated")

    def test_direct_conflict(self):
        vb = VariableBindingsSymbolic()
        variables = {"A": Argument(),
                     "B": Argument(),
                     "C": Argument()}
        
        vb.register_variable(variables["A"])
        vb.register_variable(variables["B"])
        vb.register_variable(variables["C"])

        self.assertTrue(vb.add_codesignation(variables["A"], variables["B"]), "initial codesignation between A and B could not be instantiated")
        self.assertFalse(vb.add_non_codesignation(variables["A"], variables["B"]), "non codesignation between A and B can be instantiated, which it should not")

    def test_direct_conflict2(self):
        vb = VariableBindingsSymbolic()
        variables = {"A": Argument(),
                     "B": Argument(),
                     "C": Argument()}

        vb.register_variable(variables["A"])
        vb.register_variable(variables["B"])
        vb.register_variable(variables["C"])

        self.assertTrue(vb.add_non_codesignation(variables["A"], variables["B"]), "initial codesignation between A and B could not be instantiated")
        self.assertFalse(vb.add_codesignation(variables["A"], variables["B"]), "non codesignation between A and B can be instantiated, which it should not")

    
    def test_indirect_conflict(self):
        vb = VariableBindingsSymbolic()
        variables = {"A": Argument(),
                     "B": Argument(),
                     "C": Argument()}

        vb.register_variable(variables["A"])
        vb.register_variable(variables["B"])
        vb.register_variable(variables["C"])
        
        self.assertTrue(vb.add_codesignation(variables["A"], variables["B"]), "initial codesignation between A and B could not be instantiated")
        self.assertTrue(vb.add_codesignation(variables["B"], variables["C"]), "codesignation between B and C could not be instantiated")
        self.assertFalse(vb.add_non_codesignation(variables["A"], variables["C"]), "non codesignation between A and C can be instantiated, which it should not")

    def test_non_codesignation_chaining1(self):
        vb = VariableBindingsSymbolic()
        variables = {"A": Argument(),
                     "B": Argument(),
                     "C": Argument()}

        vb.register_variable(variables["A"])
        vb.register_variable(variables["B"])
        vb.register_variable(variables["C"])

        self.assertTrue(vb.add_non_codesignation(variables["A"], variables["B"]), "initial non codesignation between A and B could not be instantiated")
        self.assertTrue(vb.add_non_codesignation(variables["B"], variables["C"]), "non codesignation between B and C could not be instantiated")
        self.assertTrue(vb.add_codesignation(variables["A"], variables["C"]), "codesignation between A and C could not be instantiated")

    def test_non_codesignation_chaining2(self):
        vb = VariableBindingsSymbolic()
        variables = {"A": Argument(),
                     "B": Argument(),
                     "C": Argument()}

        vb.register_variable(variables["A"])
        vb.register_variable(variables["B"])
        vb.register_variable(variables["C"])

        self.assertTrue(vb.add_codesignation(variables["A"], variables["B"]))
        self.assertTrue(vb.add_non_codesignation(variables["B"], variables["C"]))
        self.assertFalse(vb.add_codesignation(variables["A"], variables["C"]))

    def test_const_assigning(self):
        vb = VariableBindingsSymbolic()
        variables = {"A": Argument(),
                     "B": Argument(),
                     "C": Argument()}
        
        objects = {"A": Argument(),
                    "B": Argument(),
                    "C": Argument()}
        vb.set_objects(objects.values(), None)

        vb.register_variable(variables["B"])

        self.assertIsNotNone(vb.is_ground(objects["A"]), "A is not recognised as a constant")
        self.assertTrue(vb.can_codesignate(objects["A"], variables["B"]))
        self.assertTrue(vb.add_codesignation(objects["A"], variables["B"]))
        self.assertTrue(vb.can_codesignate(objects["A"], variables["B"]))
        self.assertIsNotNone(vb.is_ground(variables["B"]), "B is not recognised as grounded")

        self.assertFalse(vb.can_codesignate(objects["A"], objects["C"]), "two unidentical constants can codesignate")
        self.assertFalse(vb.can_codesignate(objects["C"], variables["B"]), "two unidentical constants can codesignate")
    
    def test_type_matching(self):
        vb = VariableBindingsSymbolic()
        variables = {"A": Argument(uuid4(), "typA"),
                     "B": Argument(uuid4(), "typA"),
                     "C": Argument(uuid4(), "typB"),
                     "D": Argument(uuid4(), "typC"),
                     "E": Argument(uuid4(), "typD")}
        
        objects = {"A": Argument(uuid4(), "typA"),
                    "B": Argument(uuid4(), "typB"),
                    "C": Argument(uuid4(), "typC")}
        
        obj_types = defaultdict(set)
        obj_types["typB"] = ["typC", "typD"]
        
        vb.set_objects(objects.values(), obj_types)

        for var in variables.values():
            vb.register_variable(var)

        self.assertTrue(vb.can_codesignate(variables["A"], variables["B"]), "matching types cannot codesignate")
        self.assertFalse(vb.can_codesignate(variables["A"], variables["C"]), "non matching types can codesignate")
        self.assertTrue(vb.can_codesignate(variables["A"], objects["A"]), "matching types cannot codesignate")
        self.assertFalse(vb.can_codesignate(variables["A"], objects["B"]), "non matching types can codesignate")

        # check object types logic
        self.assertTrue(vb.can_codesignate(variables["C"], variables["D"]), "subtype cannot codesignate with supertypes")
        self.assertTrue(vb.can_codesignate(variables["C"], objects["C"]), "subtype cannot codesignate with supertypes")
        self.assertFalse(vb.can_codesignate(variables["D"], variables["E"]), "subtypes can codesignate with each other")

    def test_object_non_codesignation(self):
        """check that two variables with a non-codesignation cannot codesignate to the same object
        """
        vb = VariableBindingsSymbolic()
        variables = {"A": Argument(),
                     "B": Argument()}
        
        objects = {"A": Argument()}
        
        obj_types = defaultdict(set)
        obj_types["typB"] = ["typC", "typD"]
        
        vb.set_objects(objects.values(), obj_types)

        for var in variables.values():
            vb.register_variable(var)

        vb.add_non_codesignation(variables["A"], variables["B"])
        self.assertTrue(vb.can_codesignate(variables["A"], objects["A"]))
        self.assertTrue(vb.add_codesignation(variables["A"], objects["A"]))
        self.assertFalse(vb.can_codesignate(variables["B"], objects["A"]), "variable can codesignate with a constant which is already codesignated with one of its non-codesignations")

if __name__ == '__main__':
    unittest.main()