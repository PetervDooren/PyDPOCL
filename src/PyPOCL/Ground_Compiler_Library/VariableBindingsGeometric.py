import copy
from dataclasses import dataclass
from typing import List
from collections import defaultdict
from operator import attrgetter
from PyPOCL.Ground_Compiler_Library.Element import Argument
from shapely import Polygon, MultiPolygon, LineString, box, difference, within, union, intersects, buffer
from PyPOCL.Ground_Compiler_Library.pathPlanner import find_path

# visualization
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPolygon

MARGIN_OF_ERROR = 1e-7 # buffer for nummerical problems

@dataclass
class placeloc:
    object: Argument
    object_width: float
    object_length: float
    area_max: Polygon
    area_assigned: Polygon

@dataclass
class path:
    object: Argument
    object_width: float
    object_length: float
    start_area: Argument
    goal_area: Argument
    path_assigned: LineString
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

        # path info
        self.path_variables = []
        self.paths = {}

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
        for var in self.path_variables:
            if self.paths[var].object == objvar:
                self.paths[var].object = obj_instance
                self.paths[var].object_width = self.object_dimensions[obj_instance][0]
                self.paths[var].object_length = self.object_dimensions[obj_instance][1]
    
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
    
    def get_area(self, var: Argument) -> Polygon:
        if var in self.defined_areas:
            return self.defined_areas[var]
        if var in self.placelocs:
            if self.placelocs[var].area_assigned is None:
                print("Warning, variable {var} is not yet ground")
            return self.placelocs[var].area_assigned
        if var in self.paths:
            if self.paths[var].area_assigned is None:
                print("Warning, path variable {var} is not yet ground")
            return self.paths[var].area_assigned
        print(f"Error [VariableBindingsGeometric]: Variable {var} is not registered in defined areas or in geometric variables")
        raise

    def get_path(self, var:Argument) -> LineString:
        if var in self.paths:
            if self.paths[var].path_assigned is None:
                print("Warning, path variable {var} is not yet ground")
            return self.paths[var].path_assigned
        print(f"Error [VariableBindingsGeometric]: Variable {var} is not registered in defined areas or in geometric variables")
        raise

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

    def register_path_variable(self, pathvar: Argument, startloc: Argument=None, goalloc: Argument=None, objvar: Argument=None, width=0, length=0):
        if pathvar in self.path_variables:
            print(f"Warning variable {pathvar} is already registered")
            return
        self.path_variables.append(pathvar)
        self.within_mapping[pathvar] = [self.base_area]
        self.inverse_within_mapping[pathvar] = []
        self.paths[pathvar] = path(objvar, width, length, startloc, goalloc, None, None)
        self.disjunctions[pathvar] = []

    def link_path_to_areas(self, path_variable, start_area, goal_area):
        if path_variable not in self.path_variables:
            print(f"Warning: variable {path_variable} of type {path_variable.typ} is not in the path parameter list")
            raise
        if start_area not in self.variables:
            print(f"Warning: variable {start_area} of type {start_area.typ} is not in the geometric parameter list")
            raise
        if goal_area not in self.variables:
            print(f"Warning: variable {goal_area} of type {goal_area.typ} is not in the geometric parameter list")
            raise
        self.paths[path_variable].start_area = start_area
        self.paths[path_variable].goal_area = goal_area
        if self.placelocs[start_area].object is None or self.placelocs[goal_area].object is None:
            print("object is not yet defined")
            raise
        if not self.placelocs[start_area].object == self.placelocs[goal_area].object:
            print(f"object of start location {self.placelocs[start_area].object} does not match object of goal location {self.placelocs[goal_area].object}")
        objvar = self.placelocs[start_area].object
        self.paths[path_variable].object = objvar

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
        if varA in self.path_variables or varB in self.path_variables:
            return True
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
            if new_poly.area < 0.0001:
                return False
            if self.placelocs[varA].object_width > 0: # object is defined
                minx, miny, maxx, maxy = new_poly.bounds
                if maxx - minx < self.placelocs[varA].object_width + self.buffer:
                    return False
                if maxy - miny < self.placelocs[varA].object_length + self.buffer:
                    return False

            self.placelocs[varA].area_max = new_poly

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
        #if varB not in self.within_mapping[varA]:
        #    return False
        #if varA not in self.inverse_within_mapping[varB]:
        #    return False

        # check if either A or B is a defined area
        Aisarea = varA in self.defined_areas.keys()
        Bisarea = varB in self.defined_areas.keys()

        # check if the maximum area of A is within the maximum area of B
        #A_area = self.defined_areas[varA] if Aisarea else self.placelocs[varA].area_max
        #B_area = self.defined_areas[varB] if Bisarea else self.placelocs[varB].area_max
        #if not within(A_area, B_area):
        #    print(f"Max area of {varA} is not within max area of {varB}. This should not happen!")
        #    return False
        # check if the assigned area of A is within the assigned area of B
        A_area = self.defined_areas[varA] if Aisarea else self.placelocs[varA].area_assigned
        B_area = self.defined_areas[varB] if Bisarea else self.placelocs[varB].area_assigned
        if A_area is None or B_area is None:
            print(f"Area of {varA} or {varB} is None. This should not happen!")
            return False

        buffered_B_area = buffer(B_area, MARGIN_OF_ERROR)

        if not within(A_area, buffered_B_area):
            print(f"Assigned area of {varA} is not within assigned area of {varB}. This should not happen!")
            return False
        return True
    
    def can_intersect(self, varA, varB) -> bool:
        # check if the areas can overlap
        # check if either A or B is a defined area
        Aisarea = varA in self.defined_areas.keys()
        Bisarea = varB in self.defined_areas.keys()
        Aisassigned = not Aisarea and self.placelocs[varA].area_assigned is not None
        Bisassigned = not Bisarea and self.placelocs[varB].area_assigned is not None
        
        if Aisarea:
            A_area = self.defined_areas[varA]
        elif Aisassigned:
            A_area = self.placelocs[varA].area_assigned
        else:
            A_area = self.placelocs[varA].area_max
        
        if Bisarea:
            B_area = self.defined_areas[varB]
        elif Bisassigned:
            B_area = self.placelocs[varB].area_assigned
        else:
            B_area = self.placelocs[varB].area_max
        
        if A_area is None or B_area is None:
            raise KeyError(f"Area of {varA} or {varB} is None. This should not happen!")

        return intersects(A_area, B_area)

    def add_disjunction(self, varA, varB) -> bool:
        """add a constraint that area A must be disjunct from area B

        Args:
            varA (uuid): _description_
            varB (uuid): _description_

        Returns:
            bool: False if the non codesignation is inconsistent with the existing bindings
        """
        if varB in self.disjunctions[varA]:
            return True
        
        self.disjunctions[varA].append(varB)
        self.disjunctions[varB].append(varA)
        return True
    
    def remove_disjunction(self, varA, varB) -> bool:
        """remove a constraint that area A must be disjunct from area B if it exists

        Args:
            varA (uuid): _description_
            varB (uuid): _description_

        Returns:
            bool: False if the non codesignation is inconsistent with the existing bindings
        """        
        self.disjunctions[varA].remove(varB)
        self.disjunctions[varB].remove(varA)
        return True

    def resolve(self, var):
        """ground the variable into a concrete description of an area which fits the constrainst specified.

        Returns:
            bool: True if resolving succeeded. False otherwise.
        """
        HELPER_VIZ = False # only use when debugging
        ploc = self.placelocs[var]

        # calculate the max area using the most recent information
        disjunct_area_max = self.defined_areas[self.base_area]
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
            if d_area in self.defined_areas:
                disjunct_area_max = difference(disjunct_area_max, self.defined_areas[d_area])
            elif d_area in self.variables:
                if self.placelocs[d_area].area_assigned is not None:
                    disjunct_area_max = difference(disjunct_area_max, self.placelocs[d_area].area_assigned)
            elif d_area in self.path_variables:
                if self.paths[d_area].area_assigned is not None:
                    disjunct_area_max = difference(disjunct_area_max, self.paths[d_area].area_assigned)
            else:
                print(f"Unkown variable {d_area} in disjunctions")

        buffered_disjunct_area_max = buffer(disjunct_area_max, MARGIN_OF_ERROR)
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
            minx, miny, maxx, maxy = a_min.bounds
            a_min = box(minx, miny, maxx, maxy)
            if maxx - minx - (ploc.object_width) >= -MARGIN_OF_ERROR and maxy - miny - (ploc.object_length) >= -MARGIN_OF_ERROR:
                if within(a_min, buffered_disjunct_area_max):
                    ploc.area_assigned = a_min
                    return True

        if a_min is None:
            # area is not constrained by areas that should lie within it.
            minx, miny, maxx, maxy = disjunct_area_max.bounds  # returns (minx, miny, maxx, maxy)
            candidate_width = ploc.object_width + self.buffer
            candidate_length = ploc.object_length + self.buffer
        else:
            # choose bounds and dimensions such that the assigned area is guaranteed to include a_min
            a_min_x1, a_min_y1, a_min_x2, a_min_y2 = a_min.bounds
            candidate_width = max(ploc.object_width + self.buffer, a_min_x2 - a_min_x1)
            candidate_length = max(ploc.object_length + self.buffer, a_min_y2 - a_min_y1)
            minx = a_min_x2 - candidate_width
            maxx = a_min_x1 + candidate_width
            miny = a_min_y2 - candidate_length
            maxy = a_min_y1 + candidate_length
        x_pos = minx # lower left coordinate
        y_pos = miny # lower left coordinate
        while y_pos + candidate_length <= maxy:
            # sample acceptable pose in the area_max
            a_candidate = box(x_pos, y_pos, x_pos+candidate_width, y_pos+candidate_length)
            if HELPER_VIZ:
                self.helper_show_resolve_step(disjunct_area_max, a_min, a_candidate)
            if within(a_candidate, buffered_disjunct_area_max):
                ploc.area_assigned = a_candidate
                return True
            # iterate to next position
            x_pos += 0.01
            if x_pos+candidate_width > maxx:
                x_pos = minx
                y_pos += 0.01
        # No solution could be found
        return False

    def resolve_all(self):
        """ground all variables into a concrete description of an area which fits the constrainst specified.

        Returns:
            _type_: _description_
        """
        # sort the list by size of area_max
        sorted_list = self.variables.copy()
        sorted_list.sort(key=lambda v: self.placelocs[v].area_max.area)
        for var in sorted_list:
            if not self.resolve(var):
                return False
        return True
    
    def helper_show_resolve_step(self, disjunct_area_max=None, a_min=None, a_candidate=None):
        plt.figure(1)
        plt.cla()
        # plot base area
        base_polygon = self.defined_areas[self.base_area]
        plt.fill(*base_polygon.exterior.xy, color='grey')

        if disjunct_area_max:
            plt.fill(*disjunct_area_max.exterior.xy, color='blue')
            for hole in disjunct_area_max.interiors:
                plt.fill(*hole.xy, color='gray')
        if a_candidate:
            if within(a_candidate, disjunct_area_max) and (a_min is None or within(a_min, a_candidate)):
                    plt.fill(*a_candidate.exterior.xy, color='green')
            else:
                plt.fill(*a_candidate.exterior.xy, color='red')
        if a_min:
            if isinstance(a_min, Polygon):
                plt.fill(*a_min.exterior.xy, color='cyan')                     

    def resolve_path(self, var):
        start_arg = self.paths[var].start_area
        start_area = self.placelocs[start_arg].area_assigned
        goal_arg = self.paths[var].goal_area
        goal_area = self.placelocs[goal_arg].area_assigned
        object_width = self.paths[var].object_width
        object_length = self.paths[var].object_length

        available_space = self.defined_areas[self.base_area]
        for within_var in self.within_mapping[var]:
            if within_var in self.defined_areas.keys():
                available_space = available_space.intersection(self.defined_areas[within_var])
            else: # within_var is a variable
                print(f"path {var} should not be within placement location {within_var}!")
        # check if the start and goal areas are connected
        # remove all areas that are disjunct from the area_max
        disjunct_areas = {}
        for d_arg in self.disjunctions[var]:
            if d_arg in self.defined_areas:
                d_area = self.defined_areas[d_arg]
            elif self.placelocs[d_arg].area_assigned is not None:
                d_area = self.placelocs[d_arg].area_assigned
            else:
                print(f"problem: disjunct area {d_arg} is not defined")
                continue
            available_space = difference(available_space, d_area)
            disjunct_areas[d_arg] = d_area
        # erode available space with the size of the object
        erosion_dist = 0.5*min(object_width, object_length)
        eroded = available_space.buffer(-erosion_dist)

        #self.helper_visualize_resolve_path(start_area, goal_area, disjunct_areas, eroded)

        # check if start and goal are connected in the eroded polygon
        if type(eroded) == Polygon: # eroded space is not separated. therefore there is a path from start to goal
            free_space = eroded
        elif type(eroded) == MultiPolygon:
            start_centroid = start_area.centroid # middle of the start area
            goal_centroid = goal_area.centroid # middle of the goal area
            for poly in eroded.geoms:
                if within(start_centroid, poly) and within(goal_centroid, poly):
                    free_space = poly
                    break
            else:
                # no polygon contains both start and goal. Therefore they are separated in the reachable space
                return False
        else: # eroded has an unexpected type
            print(f"eroded has an unexpected type: {type(eroded)}")
            raise
        
        # find a path through free space
        start_centroid = start_area.centroid # middle of the start area
        goal_centroid = goal_area.centroid # middle of the goal area
        path = find_path(start_centroid, goal_centroid, free_space)
        if path is None:
            print("free space is connected but no path could be found. This should not happen!")
            return False
        path_area = path.buffer(erosion_dist)
        self.paths[var].path_assigned = path
        self.paths[var].area_assigned = path_area
        return True

    def helper_visualize_resolve_path(self, start = None, goal = None, obsts = [], eroded: Polygon = None) -> None:
        """ 
        Create an image of showing the process of checking wether a connection exists.
        """

        fig, ax = plt.subplots(figsize=(8, 6))

        def plot_area(ax, area: Polygon, color='lightgray', edgecolor='black', alpha=0.5, fill = True, label=None):
            coords = list(area.exterior.coords)
            poly = MplPolygon(coords, closed=True, facecolor=color, edgecolor=edgecolor, alpha=alpha, fill=fill, label=label)
            ax.add_patch(poly)
            for hole in area.interiors:
                hole_coords = list(hole.coords)
                hole_poly = MplPolygon(hole_coords, closed=True, facecolor='white', edgecolor=edgecolor, alpha=1, fill=fill, label=label)
                ax.add_patch(hole_poly)
            if label:
                # Place label at centroid
                xs, ys = zip(*coords)
                centroid = (sum(xs)/len(xs), sum(ys)/len(ys))
                ax.text(centroid[0], centroid[1], label, ha='center', va='center', fontsize=8)

        if eroded is not None:
            if type(eroded) == Polygon: # eroded space is not separated. therefore there is a path from start to goal
                plot_area(ax, eroded, color = 'green', label='eroded')
            if type(eroded) == MultiPolygon:
                for poly in eroded.geoms:
                    plot_area(ax, poly, color = 'green', label='eroded')
            else: # eroded has an unexpected type
                print(f"eroded has an unexpected type: {type(eroded)}")

        # plot start area:
        if start is not None:
            plot_area(ax, start, color = 'magenta', label='start')
        if goal is not None:
            plot_area(ax, goal, color = 'magenta', label='goal')

        # plot all obstacles:
        for obst_arg, obst_area in obsts.items():
            if obst_arg in self.placelocs:
                label = self.placelocs[obst_area].object.name
            else:
                label = 'obst'
            plot_area(ax, obst_area, color='black', label=label)

        ax.set_aspect('equal')
        ax.autoscale()
        ax.set_title(f'Path resolve Visualization')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.show(block=False)

    def repr_arg(self, var):
        shrt_id = str(var.ID)[19:23]
        if var.arg_name is None:
            arg_name = ''
        else:
            arg_name = '-' + var.arg_name
        if var.name is None:
            name = ''
        else:
            name = '-' + self.name
        if self.placelocs[var].area_assigned is not None:
            return f"ground{name}{arg_name}-{shrt_id}"
        else:
            return f"area{name}{arg_name}-{shrt_id}"
    
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
