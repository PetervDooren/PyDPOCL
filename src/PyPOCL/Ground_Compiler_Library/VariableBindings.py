from typing import List
from PyPOCL.Ground_Compiler_Library.Element import Argument
from PyPOCL.Ground_Compiler_Library.VariableBindingsSymbolic import VariableBindingsSymbolic
from PyPOCL.Ground_Compiler_Library.VariableBindingsGeometric import VariableBindingsGeometric

class VariableBindings:
    """class to manage variablebindings

    Attributes
    ----------
    
    """
    def __init__(self):
        self.symbolic_vb = VariableBindingsSymbolic()
        self.geometric_vb = VariableBindingsGeometric()

        self.reach_constraints = []
        self.initial_positions = {}

    def __contains__(self, id):
        if id in self.symbolic_vb.variables:
            return True
        if id in self.geometric_vb.variables:
            return True
        else:
            return False
    
    def isInternallyConsistent(self):
        return True

    def set_objects(self, objects, object_types, object_dimensions, initial_positions):
        """configure the objects present in the worldmodel

        Args:
            objects (List(arguments)): list of objects present in the world.
            object_types (defaultdict(str: set(str))): mapping between objects and their (sub) types
            object_dimensions (dict(Argument:tuple(float, float))): mapping between objects and their dimensions (width, length)
        """
        
        # check that no object is of both type symbol and area
        for type, subtypes in object_types.items():
            if (type == 'symbol' or 'symbol' in subtypes) and (type == 'area' or 'area' in subtypes):
                print(f"Warning: type {type} is both a symbol and an area! this should not happen. type: {type}, subtypes {subtypes}")
                raise
        self.objects =objects
        self.object_types = object_types
        self.symbolic_vb.set_objects(objects, object_types)
        self.geometric_vb.set_object_dimensions(object_dimensions)
        self.initial_positions = initial_positions

    def set_areas(self, areas):
        """Configure the areas used in the geometric part of variable bindings

        Args:
            areas (dict(str:Polygon)): named areas
        """
        self.geometric_vb.set_areas(areas)
    
    def set_reach(self, reach_areas):
        """Configure the areas used in the geometric part of variable bindings

        Args:
            reach_areas (dict(Argument:Argument)): mapping between robots and their reach. each represented by an argument 
        """
        self.reach_areas = reach_areas
    
    def is_type(self, var, type):
        return var.typ == type or type in self.object_types[var.typ]

    def register_variable(self, var):
        if var.typ == 'symbol' or 'symbol' in self.object_types[var.typ]:
            self.symbolic_vb.register_variable(var)
        elif var.typ == 'area' or 'area' in self.object_types[var.typ]:
            self.geometric_vb.register_variable(var)
        elif var.typ == 'path' or 'path' in self.object_types[var.typ]:
            self.geometric_vb.register_path_variable(var)
        else:
            raise ValueError(f"Variable {var} of type {var.typ} is neither a symbol nor an area!")

    def link_area_to_object(self, objvar, areavar) -> None:
        if objvar not in self.symbolic_vb.variables:
            raise ValueError(f"Variable {objvar} is not in the symbolic variables set {self.symbolic_vb.variables}")
        self.geometric_vb.link_area_to_object(objvar, areavar)

    def is_ground(self, var) -> bool:
        return self.symbolic_vb.is_ground(var)

    def is_fully_ground(self) -> bool:
        if not self.symbolic_vb.is_fully_ground():
            return False
        if not self.geometric_vb.is_fully_ground():
            return False
        return True

    def get_var_per_group(self) -> List[Argument]:
        return self.symbolic_vb.get_var_per_group()

    def unify(self, provider, consumer)-> bool:
        """Unify the condition in the provider and the consumer conditions

        Args:
            provider_args (_type_): _description_
            consumer_args (_type_): _description_

        Returns:
            bool: indicating wether unification is possible
        """
        if provider.name == 'within':
            if not self.add_codesignation(provider.Args[0], consumer.Args[0]):
                return False
            return self.geometric_vb.unify(provider.Args[1], consumer.Args[1])
        else:
            # check that all arguments are symbolic
            if any([var.typ != 'symbol' and 'symbol' not in self.object_types[var.typ] for var in provider.Args]):
                raise ValueError(f"Predicate {provider} is unknown and has geometric arguments, but this is not supported in the current implementation.")
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
        
    def is_unified(self, provider, consumer) -> bool:
        """check if the provider and consumer are unified. i.e. the provider implies the consumer.

        Args:
            provider (Argument): _description_
            consumer (Argument): _description_

        Returns:
            bool: True if the provider and consumer are unified
        """
        if provider.name == 'within':
            return self.symbolic_vb.is_codesignated(provider.Args[0], consumer.Args[0]) and self.geometric_vb.is_within(provider.Args[1], consumer.Args[1])
        else:
            if not len(provider.Args) == len(consumer.Args):
                return False
            for i in range(len(provider.Args)):
                if not self.symbolic_vb.is_codesignated(provider.Args[i], consumer.Args[i]):
                    return False
            return True

    def is_codesignated(self, varA, varB) -> bool:
        """check if A and B are codesignated

        Args:
            varA (uuid): _description_
            varB (uuid): _description_

        Returns:
            bool: _description_
        """
        if varA.typ == 'area' or 'area' in self.object_types[varA.typ]:
            return True
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
            varA (Argument): sybmolic argument
            varB (Argument): symbolic argument

        Returns:
            bool: False if the codesignation is inconsistent with the existing bindings
        """
        if varA in self.symbolic_vb.variables:
            if not self.symbolic_vb.add_codesignation(varA, varB):
                return False
            if self.is_ground(varA) and self.symbolic_vb.get_const(varA) in self.reach_areas:
                return self.apply_reach(varA)
            if self.is_ground(varA):
                obj = self.symbolic_vb.get_const(varA)
                for var in self.symbolic_vb.group_members[self.symbolic_vb.group_mapping[varA]]:
                    self.geometric_vb.set_object(var, obj)
            return True
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
    
    def add_reach_constraint(self, varA: Argument, varB: Argument) -> None:
        """add a constraint in_reach(varA, varB) indicating that area B should be in reach of robot A

        Args:
            varA (Argument): area variable
            varB (Argument): robot variable
        Raises:
            Exception: if varA is not a geometric variable or varB is not a symbolic variable
        """
        
        if varA not in self.geometric_vb.variables:
            print(f"Warning: variable {varA} of type {varA.typ} is not in the geometric parameter list")
            raise
        if varB not in self.symbolic_vb.variables:
            print(f"Warning: variable {varB} of type {varB.typ} is not in the symbolic parameter list")
            raise
        self.reach_constraints.append((varA, varB))
    
    def apply_reach(self, robotvar):
        """_summary_

        Args:
            robotvar (Argument): argument of a robot whose reach can now be applied.
        
        Returns:
            bool: False if applying the reach makes the CSP insolvable.
        """
        for rc in self.reach_constraints:
            if rc[1] != robotvar:
                continue
            robot = self.symbolic_vb.get_const(robotvar)
            if robot is None:
                print(f"tried to apply reach constraint of var {robotvar} but it is not ground yet. This should not happen!")
                return False
            if not self.geometric_vb.unify(rc[0], self.reach_areas[robot]):
                return False
        return True

    def print_var(self, var):
        if var in self.symbolic_vb.variables:
            return self.symbolic_vb.print_var(var)
        else:
            return var

    def repr_arg(self, var):
        if var in self.symbolic_vb.variables:
            return self.symbolic_vb.repr_arg(var)
        elif var in self.geometric_vb.variables:
            return self.geometric_vb.repr_arg(var)
        else:
            return var
        
    def to_dict(self):
        """convert the variable bindings to a dictionary representation

        Returns:
            dict: dictionary representation of the variable bindings
        """
        return {
            'symbolic': self.symbolic_vb.to_dict(),
            'geometric': self.geometric_vb.to_dict(),
        }

    def __repr__(self):
        return f"variablebinding set with {len(self.variables)} variables. contains {self.symbolic_vb}"
