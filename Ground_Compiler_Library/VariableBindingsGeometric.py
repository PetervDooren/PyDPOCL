import copy
from dataclasses import dataclass
from uuid import uuid4
from typing import List
from collections import defaultdict
from operator import attrgetter
from Ground_Compiler_Library.Element import Argument
from shapely import Polygon, box, difference, within

# visualization
import matplotlib.pyplot as plt

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
    placelocs : dict(Argument:placeloc)
        mapping between arguments and their place location info.
    groups : list(Argument)
        list of arguments where the properties of a group are collected
    within_mapping : dict(Argument:list(Argument))
        maps a variable A to all other variables B where within(A, B) holds
    inverse_within_mapping : dict(Argument:list(Argument))
        maps a variable A to all other variables B where within(B, A) holds
    disjunctions : dict(Argument:set(Argument))
        maps a variable A to all other variable areas that must be disjunct from A
    """
    def __init__(self):
        self.base_area = None
        self.defined_areas = {}
        self.variables = []
        self.placelocs = {}
        self.within_mapping = {}
        self.inverse_within_mapping = {}
        self.disjunctions = {}
    
    def isInternallyConsistent():
        return True

    def set_areas(self, areas: dict):
        self.defined_areas = areas
        for a in areas.keys():
            self.within_mapping[a] = []
            self.inverse_within_mapping[a] = []
            self.disjunctions[a] = []

    def set_base_area(self, area: Argument):
        self.base_area = area

    def register_variable(self, areavar: Argument, objvar: Argument=None, width=0, length=0):
        if areavar in self.variables:
            print(f"Warning variable {areavar} is already registered")
            return
        self.variables.append(areavar)
        area_max = self.defined_areas[self.base_area]
        self.placelocs[areavar] = placeloc(objvar, width, length, area_max, None)
        self.within_mapping[areavar] = [self.base_area]
        self.inverse_within_mapping[areavar] = []
        self.inverse_within_mapping[self.base_area].append(areavar)
        self.disjunctions[areavar] = []

    def link_area_to_object(self, objvar: Argument, areavar: Argument):
        if areavar not in self.variables:
            print(f"Warning! variable {areavar} not in geometric variables set {self.variables}")
            raise
        self.placelocs[areavar].object = objvar
    
    def is_ground(self, var) -> bool:
        """check if a variable is ground"""
        if var not in self.placelocs:
            print(f"Variable {var} is not registered in the geometric variable bindings")
            return False
        if self.placelocs[var].area_assigned is not None:
            return True
        else:
            print(f"Variable {var} is not ground. area_assigned is None")
            return False
        
    def is_fully_ground(self) -> bool:
        for var in self.variables:
            if not self.is_ground(var):
                return False
        return True
    
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

    def can_unify(self, varA, varB) -> bool:
        """check if area A can lie within area B

        Args:
            varA (uuid): _description_
            varB (uuid): _description_

        Returns:
            bool: _description_
        """
        raise DeprecationWarning("can unify is deprecated. Instead attempt to unify")
        # check if either A or B is a defined area
        Aisarea = varA in self.defined_areas.keys()
        Bisarea = varB in self.defined_areas.keys()

        area_A = self.defined_areas[varA] if Aisarea else self.placelocs[varA].area_max
        area_B = self.defined_areas[varB] if Bisarea else self.placelocs[varB].area_max
        if Aisarea:
            return within(area_A, area_B)
        else: # A is variable and can thus shrink
            return area_A.intersects(area_B)

    def unify(self, varA, varB) -> bool:
        """add a constraint that area A must lie within area B

        Args:
            varA (uuid): child area
            varB (Argument): parent area

        Returns:
            bool: False if the constraint is inconsistent with the existing bindings
        """

        if varB in self.within_mapping[varA]:
            print(f"{varA} is already constrained to be within {varB}. This should not happen!")
            return False

        self.within_mapping[varA].append(varB)
        self.inverse_within_mapping[varB].append(varA)
        return self._apply_unify(varA, varB)

    def _apply_unify(self, varA, varB) -> bool:
        """recursively apply the constraint that area A must lie within area B

        Args:
            varA (uuid): child area
            varB (Argument): parent area

        Returns:
            bool: False if the constraint is inconsistent with the existing bindings
        """
        # check if either A or B is a defined area
        Aisarea = varA in self.defined_areas.keys()
        Bisarea = varB in self.defined_areas.keys()

        area_A = self.defined_areas[varA] if Aisarea else self.placelocs[varA].area_max
        area_B = self.defined_areas[varB] if Bisarea else self.placelocs[varB].area_max
        if Aisarea:
            return within(area_A, area_B)
        else: # A is variable and can thus shrink
            area_A = self.placelocs[varA].area_max 
            area_B = self.defined_areas[varB] if Bisarea else self.placelocs[varB].area_max
            new_poly = area_A.intersection(area_B)
            if type(new_poly) != Polygon: # intersection is a linesegment (or a multipolygon)
                return False
            self.placelocs[varA].area_max = new_poly
            #TODO check if new_poly is still large enough to house the object

            # chain the new area_max A to everything with a relation to it. Starting with the variables
            for areaC in self.inverse_within_mapping[varA]:
                if areaC in self.defined_areas.keys(): # do the areas last. The order matters
                    continue
                if not self._apply_unify(areaC, varA):
                    return False
            for areaC in self.inverse_within_mapping[varA]:
                if not areaC in self.defined_areas.keys():
                    continue
                if not self._apply_unify(areaC, varA):
                    return False
            return True
    
    def is_unified(self, varA, varB) -> bool:
        """check if area A is within area B

        Args:
            varA (uuid): child area
            varB (Argument): parent area

        Returns:
            bool: True if A is within B, False otherwise
        """
        if varB not in self.within_mapping[varA]:
            return False
        if varA not in self.inverse_within_mapping[varB]:
            return False
        if not within(self.placelocs[varA].area_max, self.placelocs[varB].area_max):
            print(f"Max area of {varA} is not within max area of {varB}. This should not happen!")
            return False
        if self.placelocs[varA].area_assigned is not None:
            if not within(self.placelocs[varA].area_assigned, self.placelocs[varB].area_assigned):
                print(f"Assigned area of {varA} is not within assigned area of {varB}. This should not happen!")
                return False
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
    
    def plot(self):
        plt.figure()
        plt.cla()
        # plot base area
        base_polygon = self.defined_areas[self.base_area]
        plt.fill(*base_polygon.exterior.xy, color='grey')

        for areakey, area in self.defined_areas.items():
            if areakey == self.base_area:
                continue            
            plt.fill(*area.exterior.xy, color='red', alpha=0.1)
        
        for ploc in self.placelocs.values():
            plt.fill(*ploc.area_max.exterior.xy, color='blue', alpha=0.5)
        
        plt.axis('equal')
        plt.show()

    def to_dict(self):
        """convert the variable bindings to a dictionary representation suitable for serialization

        Returns:
            dict: dictionary representation of the variable bindings
        """
        def defined_areas_to_dict(areas):
            return {str(k): list(v.exterior.coords) for k, v in areas.items()}
        
        def placelocs_to_dict(placelocs):
            return {
                str(k): {
                    'object': str(v.object),
                    'object_width': v.object_width,
                    'object_length': v.object_length,
                    'area_max': list(v.area_max.exterior.coords),
                    'area_assigned': list(v.area_assigned.exterior.coords) if v.area_assigned else None
                } for k, v in placelocs.items()
            }
        
        return {
            'defined_areas': defined_areas_to_dict(self.defined_areas),
            'variables': [str(v) for v in self.variables],
            'placelocs': placelocs_to_dict(self.placelocs),
            'base_area': str(self.base_area) if self.base_area else None,
            'within_mapping': {str(k): [str(v) for v in vs] for k, vs in self.within_mapping.items()},
            #'inverse_within_mapping': self.inverse_within_mapping,
            'disjunctions': {str(k): [str(v) for v in vs] for k, vs in self.disjunctions.items()}
        }

    def __repr__(self):
        return f"geometric variablebinding set with {len(self.variables)} variables"
