from solver import Solver


class Delta(Solver):  # this name would be used as identifier for type of robot "DELTA"
    """Kinematics solver for Delta Parallel Robot
    """

    def FK(self, chain, joint_params):  # states
        pass

    def IK(self, chain, tip_6DOF):  # need to know which is tip in the chain first
        pass
