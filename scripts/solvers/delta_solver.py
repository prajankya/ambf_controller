from ambf_solver import Solver


class Delta(Solver):  # this name would be used as identifier for type of robot "DELTA"
    """Kinematics solver for Delta Parallel Robot
    """

    __name__ = 'DELTA'

    def __init__(self):
        # self.description = 'Identity function'
        super().__init__()

    def perform_operation(self, argument):
        """The actual implementation of the identity solver is to just return the
        argument
        """
        return argument

# from ambf_controller import ambf_solver


# class delta_solver(ambf_solver):

#     def __init__(self):
#         print("Hi...2")
