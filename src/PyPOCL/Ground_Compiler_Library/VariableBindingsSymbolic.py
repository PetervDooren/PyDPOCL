import copy
from PyPOCL.deterministic_uuid import duuid4
from typing import List
from collections import defaultdict
from PyPOCL.Ground_Compiler_Library.Element import Argument

class VariableBindingsSymbolic:
    """class to manage symbolic variablebindings

    Attributes
    ----------
    objects : list(Argument)
        instances of objects present in the world
    object_types : defaultdict(str: set(str))
        mapping types to their subtypes
    const : dict(Argument:list(Argument))
        mapping between groups and a constant
    variables : list(Argument)
        list of variables in the plan
    groups : list(Argument)
        list of arguments where the properties of a group are collected
    group_mapping : dict(argument:argument)
        mapping between variables and groups
    group_members : dict(Argument:list(Argument))
        mapping between groups and the variables that codesignate with it. inverse of group_mapping
    non_codesignations : dict(Argument:set(Argument))
        mapping between groups that cannot share the same value
    """
    def __init__(self):
        self.objects = set()
        self.object_types = defaultdict(set)
        self.const = {}
        self.variables = []
        self.groups = []
        self.group_mapping = {}
        self.group_members = {}
        self.non_codesignations = {}
    
    def get_const(self, var):
        return self.const[self.group_mapping[var]]

    def isInternallyConsistent():
        return True

    def set_objects(self, objects, object_types):
        self.objects =objects
        self.object_types = object_types
        for o in objects:
            self.register_variable(o)
        # no objects may codesignate
        for i in objects:
            for j in objects:
                if i != j:
                    self.add_non_codesignation(i,j)

    def register_variable(self, var):
        if var in self.variables:
            print(f"Warning variable {var} is already registered")
            return
        self.variables.append(var)
        # add unique parameter to track properties
        param = copy.deepcopy(var)
        param.arg_name = "?param"
        param.ID == duuid4()
        self.groups.append(param)
        self.group_mapping[var] = param
        self.group_members[param] = {var}
        self.non_codesignations[param] = set()
        # if var is an object immediately map the group to the object
        if var in self.objects:
            self.const[param] = var
        else:
            self.const[param] = None
    
    def is_ground(self, var) -> bool:
        return self.const[self.group_mapping[var]] is not None

    def is_fully_ground(self) -> bool:
        return not any([v is None for v in self.const.values()])
    
    def get_var_per_group(self) -> List[Argument]:
        varlist = []
        for group in self.groups:
            varlist.append(next(iter(self.group_members[group])))
        return varlist

    def is_codesignated(self, varA, varB) -> bool:
        """check if A and B are codesignated

        Args:
            varA (uuid): _description_
            varB (uuid): _description_

        Returns:
            bool: _description_
        """
        return self.group_mapping[varA] == self.group_mapping[varB]

    def can_codesignate(self, varA, varB) -> bool:
        """check if A and B could be codesignated

        Args:
            varA (uuid): _description_
            varB (uuid): _description_

        Returns:
            bool: _description_
        """
        # check if either or both are constants
        groupA = self.group_mapping[varA]
        groupB = self.group_mapping[varB]
        if groupA == groupB:
            return True
        if groupA in self.non_codesignations[groupB]:
            return False
        return groupA.typ == groupB.typ or groupA.typ in self.object_types[groupB.typ] or groupB.typ in self.object_types[groupA.typ] 

    def add_codesignation(self, varA, varB) -> bool:
        """add a variable binding stating that variable A must equal variable B

        Args:
            varA (uuid): _description_
            varB (uuid): _description_

        Returns:
            bool: False if the codesignation is inconsistent with the existing bindings
        """

        if not self.can_codesignate(varA, varB):
            return False
        
        groupA = self.group_mapping[varA]
        groupB = self.group_mapping[varB]

        if groupA == groupB: # A and B are already codesignated
            return True

        # check if B is ground. if so merge into A
        if self.const[groupB] is not None:
            self.const[groupA] = self.const[groupB]

        # merge properties of this group
        self.group_mapping[varA].merge(self.group_mapping[varB])
        self.group_members[groupA].update(self.group_members[groupB])
        self.non_codesignations[groupA].update(self.non_codesignations[groupB])
        for group in self.non_codesignations[groupB]:
            self.non_codesignations[group].add(groupA)
            self.non_codesignations[group].remove(groupB)

        # reasign members of groupB
        for v in self.group_members[groupB]:
            self.group_mapping[v] = groupA

        del self.non_codesignations[groupB]
        del self.const[groupB]
        self.groups.remove(groupB)

        return True

    def add_non_codesignation(self, varA, varB) -> bool:
        """add a variable binding stating that variable A must not equal variable B

        Args:
            varA (uuid): _description_
            varB (uuid): _description_

        Returns:
            bool: False if the non codesignation is inconsistent with the existing bindings
        """

        groupA = self.group_mapping[varA]
        groupB = self.group_mapping[varB]

        if groupA == groupB: # variables already codesignate
            return False
        
        self.non_codesignations[groupA].add(groupB)
        self.non_codesignations[groupB].add(groupA)
        return True
    
    def print_var(self, var):
        print(f"variable: {var}")
        if self.const[self.group_mapping[var]] is not None:
            print(f"ground as {self.const[var]}")
        else:
            print(f"codesignations: {self.group_members[self.group_mapping[var]]}")
            print(f"non_codesignations: {self.non_codesignations[self.group_mapping[var]]}")
    
    def repr_arg(self, var):
        if self.const[self.group_mapping[var]] is not None:
            return self.const[self.group_mapping[var]].name
        return self.group_mapping[var]
    
    def to_dict(self):
        """convert the variable bindings to a dictionary representation

        Returns:
            dict: dictionary representation of the variable bindings
        """
        return {
            "variables": [str(v) for v in self.variables],
            #"groups": [g.to_dict() for g in self.groups],
            "const": {str(v): str(self.const[self.group_mapping[v]]) for v in self.variables},
            #"group_mapping": {str(k): str(v) for k, v in self.group_mapping.items()},
            #"group_members": {str(k): [str(m) for m in v] for k, v in self.group_members.items()},
            #"non_codesignations": {str(k): [str(n) for n in v] for k, v in self.non_codesignations.items()}
        }

    def __repr__(self):
        return f"variablebinding set with {len(self.variables)} variables, {len(self.groups)} groups, of which {len([g for g in self.groups if self.const[g] is not None])} groups are ground"
