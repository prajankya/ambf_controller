from Solver import BaseSolver
from Solver import logger as log


class Tracik(BaseSolver):  # this name would be used as identifier for type of robot "TRACIK"
    """Kinematics solver using TRAC-IK for Serial Robots
    """
    # implementing using PyKDL
    # https://github.com/orocos/orocos_kinematics_dynamics/blob/a7a8d282a5a94948176f2c5af58fbd047d849030/python_orocos_kdl/tests/kinfamtest.py

    def init(self, chain):
        # can be used if any class specific setup is needed.
        # self.chain will have the Kinematics chain
        pass

    def solve_for_fk(self, joint_states):
        pass

    def solve_for_ik(self, tip_6DOF):
        pass
