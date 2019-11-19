from Solver import BaseSolver
from Solver.Logger import Logger as log
from Solver import InvalidChainError


class FourBar(BaseSolver):  # this name would be used as identifier for type of robot "FOURBAR"
    """Kinematics solver for Delta Parallel Robot
    """

    def init(self, chain):
        # can be used if any class specific setup is needed.
        # self.chain will have the Kinematics chain

        # raise the error when you kinematic chain doesn't suit the solver
        #raise InvalidChainError()
        pass

    def solve_for_fk(self, joint_states):
        print("TODO: Implement FK solver")
        return joint_states

    def solve_for_ik(self, tip_6DOF):
        print("TODO: Implement IK solver")
        return tip_6DOF
