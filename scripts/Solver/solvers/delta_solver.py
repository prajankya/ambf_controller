from Solver import BaseSolver
from Solver.Logger import Logger as log
from Solver import InvalidChainError


class Delta(BaseSolver):  # this name would be used as identifier for type of robot "DELTA"
    """Kinematics solver for Delta Parallel Robot
    """

    def init(self, chain):
        # can be used if any class specific setup is needed.
        # self.chain will have the Kinematics chain

        # raise the error when you kinematic chain doesn't suit the solver
        #raise InvalidChainError()
        pass

    def solve_for_fk_pos(self, joint_states):
        print("TODO: Implement FK solver")
        pass

    def solve_for_ik_pos(self, tip_Pose):
        print("TODO: Implement IK solver")
        pass
