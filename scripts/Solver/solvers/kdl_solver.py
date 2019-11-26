import math
import PyKDL as kdl

from Solver import BaseSolver
from Solver.Logger import Logger as log
from Solver import InvalidChainError


class KDL(BaseSolver):  # this name would be used as identifier for type of solver, i.e. "KDL"
    """Kinematics solver with Orocos KDL library
    """

    def init(self, _chain):
        kdlChain = kdl.Chain()

        body = _chain.getBaseBody()

        while(True):
            # Error if this robot is not a serial robot.
            if len(body.child_joints) > 1:
                raise InvalidChainError()

            # Last link
            if len(body.child_joints) == 0:
                break
            joint = _chain.getJoint(body.child_joints[0])

            # From Body
            rot = kdl.Rotation.EulerZYX(float(body.location['orientation']['y']),
                                        float(
                                            body.location['orientation']['p']),
                                        float(body.location['orientation']['r']))

            # From Body
            trans = kdl.Vector(float(body.location['position']['x']),
                               float(body.location['position']['y']),
                               float(body.location['position']['z']))

            # From Joint (Location of joint on parent body)
            trans2 = kdl.Vector(float(joint.parent_pivot['x']),
                                float(joint.parent_pivot['y']),
                                float(joint.parent_pivot['z']))

            # TODO: If Joint['client pivot'] is set, need to add it to the Frame below
            frame = kdl.Frame(rot, trans + trans2)

            kdlJoint = kdl.Joint.None  # Default Joint Axis

            if joint.parent_axis['x']:
                kdlJoint = kdl.Joint(
                    kdl.Joint.RotX, scale=joint.parent_axis['x'])
            elif joint.parent_axis['y']:
                kdlJoint = kdl.Joint(
                    kdl.Joint.RotX, scale=joint.parent_axis['y'])
            elif joint.parent_axis['z']:
                kdlJoint = kdl.Joint(
                    kdl.Joint.RotX, scale=joint.parent_axis['z'])

            kdlChain.addSegment(kdl.Segment(kdlJoint, frame))

            body = _chain.getBody(body.children[0])

        self.kdlChain = kdlChain

        # Initialize Solvers
        self.FKSolverPos = kdl.ChainFkSolverPos_recursive(self.kdlChain)
        self.IKSolverPos = kdl.ChainFkSolverPos_recursive(self.kdlChain)

    def solve_for_fk_pos(self, joint_states):
        if len(joint_states) != self.kdlChain.getNrOfJoints():
            raise TypeError(
                "Invalid number of joints in Joint states, should be " + self.kdlChain.getNrOfJoints())
        jointParameters = kdl.JntArray(self.kdlChain.getNrOfJoints())

        for i in range(len(joint_states)):
            jointParameters[i] = joint_states[i]

        output_frame = kdl.Frame()
        self.FKSolverPos.JntToCart(jointParameters, output_frame)
        print(output_frame)

    def solve_for_ik_pos(self, tip_Pose):
        print("TODO: Implement IK solver")
        return []
