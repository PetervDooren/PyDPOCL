import copy
from uuid import uuid4
from typing import List
from collections import defaultdict
from Ground_Compiler_Library.Element import Argument
from Ground_Compiler_Library.VariableBindingsSymbolic import VariableBindingsSymbolic
from Ground_Compiler_Library.VariableBindingsGeometric import VariableBindingsGeometric

class VariableBindings:
    """class to manage variablebindings

    Attributes
    ----------
    
    """
    def __init__(self):
        self.variables = []

        self.symbolic_vb = VariableBindingsSymbolic()
        self.geometric_vb = VariableBindingsGeometric()

    def __contains__(self, id):
        if id in self.symbolic_vb.variables:
            return True
        if id in self.geometric_vb.variables:
            return True
        else:
            return False
    
    def isInternallyConsistent():
        return True

    def set_objects(self, objects, object_types):
        """configure the objects present in the worldmodel

        Args:
            objects (List(arguments)): list of objects present in the world.
            object_types (defaultdict(str: set(str))): mapping between objects and their (sub) types
        """
        
        # check that no object is of both type symbol and area
        for type, subtypes in object_types.items():
            if (type == 'symbol' or 'symbol' in subtypes) and (type == 'area' or 'area' in subtypes):
                print(f"Warning: type {type} is both a symbol and an area! this should not happen. type: {type}, subtypes {subtypes}")
                raise
        self.objects =objects
        self.object_types = object_types
        self.symbolic_vb.set_objects(objects, object_types)

    def set_areas(self, areas):
        """Configure the areas used in the geometric part of variable bindings

        Args:
            areas (dict(str:Polygon)): named areas
        """
        self.geometric_vb.set_areas(areas)

    def register_variable(self, var):
        if var in self.variables:
            print(f"Warning variable {var} is already registered")
            return
        self.variables.append(var)
        if var.typ == 'symbol' or 'symbol' in self.object_types[var.typ]:
            self.symbolic_vb.register_variable(var)
        elif var.typ == 'area' or 'area' in self.object_types[var.typ]:
            self.geometric_vb.register_variable(var)
        else:
            print(f"Warning: variable {var} of type {var.typ} is neither a symbol nor an area! This should not happen")
            raise

    def link_area_to_object(self, objvar, areavar) -> None:
        if objvar not in self.symbolic_vb.variables:
            print(f"Warning: variable {objvar} is not in the symbolic variables set {self.symbolic_vb.variables}")
            raise
        self.geometric_vb.link_area_to_object(objvar, areavar)

    def is_ground(self, var) -> bool:
        return self.symbolic_vb.is_ground(var)

    def is_fully_ground(self) -> bool:
        return self.symbolic_vb.is_fully_ground()

    def get_var_par_group(self) -> List[Argument]:
        return self.symbolic_vb.get_var_per_group()

    def unify(self, provider, consumer)-> bool:
        """Unify the condition in the provider and the consumer conditions

        Args:
            provider_args (_type_): _description_
            consumer_args (_type_): _description_

        Returns:
            bool: _description_
        """
        if provider.name == 'within':
            if not self.symbolic_vb.add_codesignation(provider.Args[0], consumer.Args[0]):
                return False
            return self.geometric_vb.unify(provider.Args[1], consumer.Args[1])
        else:
            # check that all arguments are symbolic
            if any([var.typ == 'symbol' or 'symbol' in self.object_types[var.typ] for var in provider.Args]):
                print(f"Dont know how to handle predicate: {provider}")
                return False
            provider_args = provider.Args
            consumer_args = consumer.Args
            if not len(provider_args) == len(consumer_args):
                print(f"Warning: provider and consumer have a different amount of arguments: provider: {provider_args}, consumer: {consumer_args}")
                return False
            for i in range(len(provider_args)):
                if not self.can_codesignate(provider_args[i], consumer_args[i]):
                    return False
                self.add_codesignation(provider_args[i], consumer_args[i])
            return True

    def is_codesignated(self, varA, varB) -> bool:
        """check if A and B are codesignated

        Args:
            varA (uuid): _description_
            varB (uuid): _description_

        Returns:
            bool: _description_
        """
        return self.symbolic_vb.is_codesignated(varA, varB)

    def can_codesignate(self, varA, varB) -> bool:
        """check if A and B could be codesignated

        Args:
            varA (uuid): _description_
            varB (uuid): _description_

        Returns:
            bool: _description_
        """
        if varA in self.symbolic_vb.variables:
            return self.symbolic_vb.can_codesignate(varA, varB)
        else:
            return True

    def add_codesignation(self, varA, varB) -> bool:
        """add a variable binding stating that variable A must equal variable B

        Args:
            varA (uuid): _description_
            varB (uuid): _description_

        Returns:
            bool: False if the codesignation is inconsistent with the existing bindings
        """
        if varA in self.symbolic_vb.variables:
            return self.symbolic_vb.add_codesignation(varA, varB)
        else:
            return True

    def add_non_codesignation(self, varA, varB) -> bool:
        """add a variable binding stating that variable A must not equal variable B

        Args:
            varA (uuid): _description_
            varB (uuid): _description_

        Returns:
            bool: False if the non codesignation is inconsistent with the existing bindings
        """
        if varA in self.symbolic_vb.variables:
            return self.symbolic_vb.add_non_codesignation(varA, varB)
        else:
            return True

    def print_var(self, var):
        if var in self.symbolic_vb.variables:
            return self.symbolic_vb.print_var(var)
        else:
            return var

    def repr_arg(self, var):
        if var in self.symbolic_vb.variables:
            return self.symbolic_vb.repr_arg(var)
        else:
            return var

    def __repr__(self):
        return f"variablebinding set with {len(self.variables)} variables. contains {self.symbolic_vb}"
