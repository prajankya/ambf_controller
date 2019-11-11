from Solver import BaseSolver
from Solver import logger as log


class Delta(BaseSolver):  # this name would be used as identifier for type of robot "DELTA"
    """Kinematics solver for Delta Parallel Robot
    """

    def init(self, chain):
        # can be used if any class specific setup is needed.
        # self.chain will have the Kinematics chain
        pass

    def solve_for_fk(self, joint_states):
        print("TODO: Implement FK solver")
        return joint_states

    def solve_for_ik(self, tip_6DOF):
        print("TODO: Implement IK solver")
        return tip_6DOF
