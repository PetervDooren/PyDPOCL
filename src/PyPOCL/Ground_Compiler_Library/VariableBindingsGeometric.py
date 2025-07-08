import copy
from dataclasses import dataclass
from uuid import uuid4
from typing import List
from collections import defaultdict
from operator import attrgetter
from PyPOCL.Ground_Compiler_Library.Element import Argument
from shapely import Polygon, box, difference, within, union

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
    base_area : Argument
        the base area in which all other areas are defined
    defined areas : dict(Argument:Polygon)
        mapping between area arguments and their polygons
    object_dimensions: dict(Argument:tuple(float, float))
        mapping between object arguments and their dimensions (width, length)
    variables : list(Argument)
        list of area variables
    placelocs : dict(Argument:placeloc)
        mapping between arguments and their place location info.
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
        self.object_dimensions = {}
        self.variables = []
        self.placelocs = {}
        self.within_mapping = {}
        self.inverse_within_mapping = {}
        self.disjunctions = {}

        self.buffer = 0.05 # buffer to use when placing objects in the area.
    
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
    
    def set_object_dimensions(self, objects):
        self.object_dimensions = objects
    
    def get_max_area(self, var: Argument) -> Polygon:
        """get the maximum area for a variable

        Args:
            var (Argument): variable to get the maximum area for

        Returns:
            Polygon: maximum area for the variable
        """
        if var not in self.placelocs:
            print(f"Variable {var} is not registered in the geometric variable bindings")
            return None
        return self.placelocs[var].area_max
    
    def set_object(self, objvar: Argument, obj_instance: Argument):
        """set the object for a variable. This is used to set the size of the area assigned to the variable.

        Args:
            objvar (Argument): variable to describe the object
            obj_instance (Argument): object instance to assign to the variable
        """
        #TODO check if the obj instance is a physical object
        #if obj_instance not in self.object_dimensions:
            #raise ValueError(f"Object {obj_instance} is not registered in the object dimensions {self.object_dimensions}")
        for var in self.variables:
            if self.placelocs[var].object == objvar:
                self.placelocs[var].object = obj_instance
                self.placelocs[var].object_width = self.object_dimensions[obj_instance][0]
                self.placelocs[var].object_length = self.object_dimensions[obj_instance][1]
    
    def set_assigned_area(self, var: Argument, area: Polygon):
        """set the area assigned to a variable. Should only be used when loading a plan from a file. Otherwise use resolve() to set the area.

        Args:
            var (Argument): variable to set the area for
            area (Polygon): area to assign to the variable
        """
        if var not in self.placelocs:
            print(f"Variable {var} is not registered in the geometric variable bindings")
            return
        self.placelocs[var].area_assigned = area
        # check if the area is within the maximum area
        if not within(area, self.placelocs[var].area_max):
            print(f"Assigned area {area} is not within maximum area {self.placelocs[var].area_max} for variable {var}")

    def get_assigned_area(self, var: Argument) -> Polygon:
        """get the area assigned to a variable

        Args:
            var (Argument): variable to get the area for

        Returns:
            Polygon: area assigned to the variable
        """
        if var not in self.placelocs:
            print(f"Variable {var} is not registered in the geometric variable bindings")
            return None
        return self.placelocs[var].area_assigned

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
    
    def is_within(self, varA, varB) -> bool:
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

        # check if either A or B is a defined area
        Aisarea = varA in self.defined_areas.keys()
        Bisarea = varB in self.defined_areas.keys()

        # check if the maximum area of A is within the maximum area of B
        A_area = self.defined_areas[varA] if Aisarea else self.placelocs[varA].area_max
        B_area = self.defined_areas[varB] if Bisarea else self.placelocs[varB].area_max
        if not within(A_area, B_area):
            print(f"Max area of {varA} is not within max area of {varB}. This should not happen!")
            return False
        # check if the assigned area of A is within the assigned area of B
        A_area = self.defined_areas[varA] if Aisarea else self.placelocs[varA].area_assigned
        B_area = self.defined_areas[varB] if Bisarea else self.placelocs[varB].area_assigned
        if A_area is not None or B_area is not None:
            if A_area is None or B_area is None:
                print(f"Area of {varA} or {varB} is None. This should not happen!")
                return False
            if not within(A_area, B_area):
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
            disjunct_area_max = self.defined_areas[self.base_area]
            # calculate the max area using the most recent information
            for within_var in self.within_mapping[var]:
                if within_var in self.defined_areas.keys():
                    disjunct_area_max = disjunct_area_max.intersection(self.defined_areas[within_var])
                else: # within_var is a variable
                    if self.placelocs[within_var].area_assigned is not None:
                        disjunct_area_max = disjunct_area_max.intersection(self.placelocs[within_var].area_assigned)
                    else:
                        disjunct_area_max = disjunct_area_max.intersection(self.placelocs[within_var].area_max)
            # remove all areas that are disjunct from the area_max
            for d_area in self.disjunctions[var]:
                if self.placelocs[d_area].area_assigned is not None:
                    disjunct_area_max = difference(disjunct_area_max, self.placelocs[d_area].area_assigned)
            if any(self.inverse_within_mapping[var]):
            # compile a minimum area based on areas that must lie within this area
                a_min = None
                for inv_within_var in self.inverse_within_mapping[var]:
                    if inv_within_var in self.defined_areas.keys():
                        if a_min is None:
                            a_min = self.defined_areas[inv_within_var]
                        else:
                            a_min = union(a_min, a_min = self.defined_areas[inv_within_var])
                    elif self.placelocs[inv_within_var].area_assigned is not None:
                        if a_min is None:
                            a_min = self.placelocs[inv_within_var].area_assigned
                        else:
                            a_min = union(a_min, self.placelocs[inv_within_var].area_assigned)
                if a_min is not None:
                    if type(a_min) != Polygon: # union is a multipolygon)
                        return False
                    ploc.area_assigned = a_min
                    continue # next iteration

            # area is not constrained by areas that should lie within it.
            minx, miny, maxx, maxy = disjunct_area_max.bounds  # returns (minx, miny, maxx, maxy)
            x_pos = minx # lower left coordinate
            y_pos = miny # lower left coordinate
            while y_pos + ploc.object_length < maxy:
                # sample acceptable pose in the area_max
                a_min = box(x_pos, y_pos, x_pos+ploc.object_width + self.buffer, y_pos+ploc.object_length + self.buffer)
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
            plt.fill(*ploc.area_max.exterior.xy, color='blue', alpha=0.3)
            if ploc.area_assigned is not None:
                plt.fill(*ploc.area_assigned.exterior.xy, color='green', alpha=0.5)
        
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
            'disjunctions': {str(k): [str(v) for v in vs] for k, vs in self.disjunctions.items()}
        }

    def __repr__(self):
        return f"geometric variablebinding set with {len(self.variables)} variables"
