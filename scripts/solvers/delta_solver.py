from ambf_solver import Solver


class Identity(Solver):
    """This solver is just the identity function: it returns the argument
    """

    def __init__(self):
        # self.description = 'Identity function'
        super().__init__('DELTA')

    def perform_operation(self, argument):
        """The actual implementation of the identity solver is to just return the
        argument
        """
        return argument

# from ambf_controller import ambf_solver


# class delta_solver(ambf_solver):

#     def __init__(self):
#         print("Hi...2")
