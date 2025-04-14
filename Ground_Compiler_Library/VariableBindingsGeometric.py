import copy
from dataclasses import dataclass
from uuid import uuid4
from typing import List
from collections import defaultdict
from operator import attrgetter
from Ground_Compiler_Library.Element import Argument
from shapely import Polygon, box, difference, within

@dataclass
class placeloc:
    object: Argument
    object_width: float
    object_length: float
    area_max: Polygon
    area_assigned: Polygon

class VariableBindingsGeometric:
    """class to manage variablebindings for areas

    Attributes
    ----------
    defined areas : list(?)
        concrete areas
    variables : list(Argument)
        list of area variables
    placelocs : dict(Arugment:placeloc)
        mapping between arguments and their place location info.
    groups : list(Argument)
        list of arguments where the properties of a group are collected
    within_mapping : dict(argument:list(argument))
        maps a variable A to all other variables B where within(A, B) holds
    disjunctions : dict(Argument:set(Argument))
        maps a variable A to all other variable areas that must be disjunct from A
    """
    def __init__(self):
        self.base_area = None
        self.defined_areas = []
        self.variables = []
        self.placelocs = {}
        self.within_mapping = {}
        self.within_areas = {}
        self.disjunctions = {}
    
    def isInternallyConsistent():
        return True

    def set_areas(self, areas):
        self.defined_areas = areas

    def set_base_area(self, area):
        self.base_area = area

    def register_variable(self, areavar, objvar, width, length):
        if areavar in self.variables:
            print(f"Warning variable {areavar} is already registered")
            return
        self.variables.append(areavar)
        self.placelocs[areavar] = placeloc(objvar, width, length, self.base_area, None)
        self.within_mapping[areavar] = []
        self.within_areas[areavar] = [self.base_area]
        self.disjunctions[areavar] = []
    
    def is_ground(self, var) -> bool:
        return False

    def is_fully_ground(self) -> bool:
        return False
    
    def get_var_par_group(self) -> List[Argument]:
        return self.variables

    def is_codesignated(self, varA, varB) -> bool:
        """check if A and B are codesignated

        Args:
            varA (uuid): _description_
            varB (uuid): _description_

        Returns:
            bool: _description_
        """
        return False

    def can_codesignate(self, varA, varB) -> bool:
        """check if A and B could be codesignated

        Args:
            varA (uuid): _description_
            varB (uuid): _description_

        Returns:
            bool: _description_
        """
        return True

    def unify(self, varA, varB) -> bool:
        """add a constraint that area A must lie within area B

        Args:
            varA (uuid): child area
            varB (Argument | Polygon): parent area

        Returns:
            bool: False if the constraint is inconsistent with the existing bindings
        """

        if not self.can_codesignate(varA, varB):
            return False
        
        if isinstance(varB, Argument):
            if varB in self.within_mapping[varA]:
                print(f"{varA} is already constrained to be within {varB}. This should not happen!")
                return False
            self.within_mapping[varA].append(varB)
            if not self.placelocs[varA].area_max.intersects(self.placelocs[varB].area_max):
                return False
            new_poly = self.placelocs[varA].area_max.intersection(self.placelocs[varB].area_max)
            self.placelocs[varA].area_max = new_poly
            #TODO check if new_poly is still large enough to house the object
        else: # must be Polygon type
            self.within_areas[varA].append(varB)
        return True

    def add_disjunction(self, varA, varB) -> bool:
        """add a constraint that area A must be disjunct from area B

        Args:
            varA (uuid): _description_
            varB (uuid): _description_

        Returns:
            bool: False if the non codesignation is inconsistent with the existing bindings
        """
        self.disjunctions[varA].append(varB)
        self.disjunctions[varB].append(varA)
        return True
    
    def resolve(self):
        """ground all variables into a concrete description of an area which fits the constrainst specified.

        Returns:
            _type_: _description_
        """
        # sort the list by size of area_max
        sorted_list = self.variables.copy()
        sorted_list.sort(key=lambda v: self.placelocs[v].area_max.area)
        for var in sorted_list:
            ploc = self.placelocs[var]
            disjunct_area_max = ploc.area_max
            for d_area in self.disjunctions[var]:
                if self.placelocs[d_area].area_assigned is not None:
                    disjunct_area_max = difference(disjunct_area_max, self.placelocs[d_area].area_assigned)
            minx, miny, maxx, maxy = disjunct_area_max.bounds  # returns (minx, miny, maxx, maxy)
            x_pos = minx # lower left coordinate
            y_pos = miny # lower left coordinate
            while y_pos + ploc.object_length < maxy:
                # sample acceptable pose in the area_max
                a_min = box(x_pos, y_pos, x_pos+ploc.object_width, y_pos+ploc.object_length)
                if within(a_min, disjunct_area_max):
                    ploc.area_assigned = a_min
                    break
                # iterate to next position
                x_pos += 0.1
                if x_pos+ploc.object_width > maxx:
                    x_pos = minx
                    y_pos += 0.1
            if ploc.area_assigned is None: # No solution could be found
                return False
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

    def __repr__(self):
        return f"geometric variablebinding set with {len(self.variables)} variables"
