from collections import defaultdict

class VariableBindings:
    """class to manage variablebindings

    Attributes
    ----------
    objects : list(Argument)
        instances of objects
    object_types : defaultdict
        ???
    const : dict(Argument:list(Argument))
        mapping between variables and a constant
    variables : list(Argument)
        list of variables in the plan
    codesignations : dict(Argument:list(Argument))
        mapping between variables and other variables that must share the same value
    non_codesignations : dict(Argument:list(Argument))
        mapping between variables and other variables that cannot share the same value
    """
    def __init__(self):
        self.objects = set()
        self.object_types = defaultdict(set)
        self.const = {}
        self.variables = []
        self.codesignations = {}
        self.non_codesignations = {}
    
    def isInternallyConsistent():
        return True        

    def set_objects(self, objects, object_types):
        self.objects =objects
        self.obj_types = object_types

    def register_variable(self, var):
        if var in self.variables:
            print(f"Warning variable {var} is already registered")
            return
        self.variables.append(var)
        self.const[var] = None
        self.codesignations[var] = []
        self.non_codesignations[var] = []
        if var in self.objects:
            self.const[var] = var

    def set_const(self, var, const) -> bool:
        self.const[var] = const
    
    def is_codesignated(self, varA, varB) -> bool:
        """check if A and B are codesignated

        Args:
            varA (uuid): _description_
            varB (uuid): _description_

        Returns:
            bool: _description_
        """
        if varB not in self.codesignations:
            return False
        return varA in self.codesignations and varB in self.codesignations[varA]
    
    def can_codesignate(self, varA, varB) -> bool:
        """check if A and B could be codesignated

        Args:
            varA (uuid): _description_
            varB (uuid): _description_

        Returns:
            bool: _description_
        """
        # check if either or both are constants
        if self.const[varA] is not None and self.const[varB] is not None:
            return self.const[varA] == self.const[varB]
        elif self.const[varA] is not None:
            return not varA in self.non_codesignations[varB]
        elif self.const[varB] is not None:
            return not varA in self.non_codesignations[varB]
        #TODO check if types of A and B match
        return not varA in self.non_codesignations[varB]

    def add_codesignation(self, varA, varB) -> bool:
        """add a variable binding stating that variable A must equal variable B

        Args:
            varA (uuid): _description_
            varB (uuid): _description_

        Returns:
            bool: False if the codesignation is inconsistent with the existing bindings
        """

        #TODO check if either A or B are constants
        if self.const[varA] is not None:
            self.const[varB] = self.const[varA]
            for v in self.codesignations[varB]:
                self.const[v] = self.const[varA]
            return True
        elif self.const[varB] is not None:
            self.const[varA] = self.const[varB]
            for v in self.codesignations[varA]:
                self.const[v] = self.const[varB]
            return True

        # check if they cannot codesignate. Only need to check one list since they are symmetrical.
        if varA in self.non_codesignations[varB]:
            return False

        # add codesignation
        self.codesignations[varA].append(varB)
        self.codesignations[varA].extend(self.codesignations[varB])

        self.codesignations[varB].append(varA)
        self.codesignations[varB].extend(self.codesignations[varA])

        return True

    def add_non_codesignation(self, varA, varB) -> bool:
        """add a variable binding stating that variable A must not equal variable B

        Args:
            varA (uuid): _description_
            varB (uuid): _description_

        Returns:
            bool: False if the non codesignation is inconsistent with the existing bindings
        """

        #TODO check if either A or B are constants
        # check if they cannot codesignate. Only need to check one list since they are symmetrical.
        if varA in self.codesignations[varB]:
            return False

        # add codesignation
        self.non_codesignations[varA].append(varB)
        self.non_codesignations[varA].extend(self.codesignations[varB])

        self.non_codesignations[varB].append(varA)
        self.non_codesignations[varB].extend(self.codesignations[varA])

        return True
    
    def print_var(self, var):
        print(f"variable: {var}")
        if self.const[var] is not None:
            print(f"ground as {self.const[var]}")
        else:
            print(f"codesignations: {self.codesignations[var]}")
            print(f"non_codesignations: {self.non_codesignations[var]}")