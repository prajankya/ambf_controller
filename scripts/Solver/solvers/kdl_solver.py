import math
import PyKDL as kdl

from Solver import BaseSolver, Pose
from Solver.Logger import Logger as log
from Solver import InvalidChainError


class KDL(BaseSolver):  # this name would be used as identifier for type of solver, i.e. "KDL"
    """Kinematics solver with Orocos KDL library
    """

    def init(self, _chain):
        kdlChain = kdl.Chain()

        body = _chain.getBaseBody()

        # Joint Limits
        self.q_min = kdl.JntArray(len(_chain.getJoints()))
        self.q_max = kdl.JntArray(len(_chain.getJoints()))

        old_pos = kdl.Vector(0, 0, 0)

        index = 0
        while(len(body.child_joints) != 0):  # Last link
            # Error if this robot is not a serial robot.
            if len(body.child_joints) > 1:
                raise InvalidChainError()

            # Get Joint Object of type Solver.Joint
            joint = _chain.getJoint(body.child_joints[0])

            # From Joint (Location of joint on parent body)
            trans = kdl.Vector(float(joint.parent_pivot['x']),
                               float(joint.parent_pivot['y']),
                               float(joint.parent_pivot['z']))

            # From Joint (Location of joint on child body)
            trans2 = kdl.Vector(float(joint.child_pivot['x']),
                                float(joint.child_pivot['y']),
                                float(joint.child_pivot['z']))

            frame = kdl.Frame(trans + old_pos)
            old_pos = trans2

            kdlJoint = kdl.Joint()  # Default Joint Axis

            if joint.parent_axis['x']:
                kdlJoint = kdl.Joint(
                    kdl.Joint.RotX, scale=joint.parent_axis['x'])
            elif joint.parent_axis['y']:
                kdlJoint = kdl.Joint(
                    kdl.Joint.RotY, scale=joint.parent_axis['y'])
            elif joint.parent_axis['z']:
                kdlJoint = kdl.Joint(
                    kdl.Joint.RotZ, scale=joint.parent_axis['z'])

            # Add to KDL
            kdlChain.addSegment(kdl.Segment(kdlJoint, frame))

            # Joint limits
            self.q_min[index] = joint.joint_limits['low']
            self.q_max[index] = joint.joint_limits['high']

            body = _chain.getBody(body.children[0])
            index += 1

        self.kdlChain = kdlChain

        # Initialize Solvers
        self.FKSolverPos = kdl.ChainFkSolverPos_recursive(self.kdlChain)
        # self.IKVelSolver = kdl.ChainIkSolverVel_pinv_givens(self.kdlChain)
        # self.IKPosSolver = kdl.ChainIkSolverPos_NR_JL(self.kdlChain, self.q_min, self.q_max,
        #   self.FKSolverPos, self.IKVelSolver)

        self.IKPosSolver = kdl.ChainIkSolverPos_LMA(self.kdlChain)

    def solve_for_fk_pos(self, joint_states):
        if len(joint_states) > self.kdlChain.getNrOfJoints():
            raise TypeError(
                "Invalid number of joints in Joint states, should be " + str(self.kdlChain.getNrOfJoints()) + ",  " + str(len(joint_states)) + " given")
        jointParameters = kdl.JntArray(self.kdlChain.getNrOfJoints())

        for i in range(len(joint_states)):
            jointParameters[i] = joint_states[i]

        output_frame = kdl.Frame()
        self.FKSolverPos.JntToCart(jointParameters, output_frame)
        quat = output_frame.M.GetQuaternion()

        pose = Pose()
        pose.x = output_frame.p[0]
        pose.y = output_frame.p[1]
        pose.z = output_frame.p[2]
        pose.qx = quat[0]
        pose.qy = quat[1]
        pose.qz = quat[2]
        pose.qw = quat[3]
        return pose

    def solve_for_ik_pos(self, tip_Pose):
        # The recursive algorithm need an initial guess, a good one can be
        # in the middle of limits, or use joint states from AMBF
        q_init = kdl.JntArray(self.kdlChain.getNrOfJoints())

        current_states = self.get_current_jointstates()

        for i in range(len(current_states)):
            q_init[i] = current_states[i]

        desired_q = kdl.JntArray(self.kdlChain.getNrOfJoints())

        rot = kdl.Rotation()

        rot.Quaternion(tip_Pose.qx, tip_Pose.qy, tip_Pose.qz, tip_Pose.qw)

        tip_frame = kdl.Frame(rot,
                              kdl.Vector(tip_Pose.x, tip_Pose.y, tip_Pose.z))

        self.IKPosSolver.CartToJnt(q_init, tip_frame, desired_q)

        return desired_q
