class VariableBindings:
    def __init__(self):
        self.const = {}
        self.variables = []
        self.codesignations = {}
        self.non_codesignations = {}

    def register_variable(self, var):
        if var in self.codesignations or var in self.non_codesignations:
            print(f"Warning variable {var} is already registered")
            return
        self.variables.append(var)
        self.const[var] = None
        self.codesignations[var] = []
        self.non_codesignations[var] = []

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
        if varB not in self.non_codesignations:
            return True
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