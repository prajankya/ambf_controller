from Solver import BaseSolver
from Solver.Logger import Logger as log
from Solver import InvalidChainError

import math


class twoD(BaseSolver):  # this name would be used as identifier for type of robot "twoD"
    """Kinematics solver for 2D mechanism
    """

    def init(self, chain):
        # can be used if any class specific setup is needed.
        # self.chain will have the Kinematics chain
        base = chain.getBaseBody()
        # raise the error when you kinematic chain doesn't suit the solver
        if not len(base.children) == 1:
            raise InvalidChainError()

    def solve_for_fk(self, joint_states):
        print("TODO: Implement FK solver")
        return joint_states

    def solve_for_ik(self, tip_6DOF):
        print("TODO: Implement IK solver")
        # q1 = math.atan2(tip_6DOF.y, tip_6DOF.x) - math.atan2()
        return tip_6DOF
