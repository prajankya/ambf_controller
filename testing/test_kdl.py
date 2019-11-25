#!/usr/bin/env python
import PyKDL as kdl

M_PI = 3.142


chain = kdl.Chain()

joint1 = kdl.Joint(kdl.Joint.None)

frame1 = kdl.Frame(kdl.Vector(0, 1, 0))
chain.addSegment(kdl.Segment(joint1, frame1))

joint2 = kdl.Joint(kdl.Joint.RotZ)
frame2 = kdl.Frame(kdl.Vector(0.0, 2.0, 0.0))
chain.addSegment(kdl.Segment(joint2, frame2))

joint3 = kdl.Joint(kdl.Joint.RotZ)
frame3 = kdl.Frame(kdl.Rotation.EulerZYX(0.0, 0.0, -M_PI / 2)
                   ) * kdl.Frame(kdl.Vector(0.0, 0.0, 2.0))
chain.addSegment(kdl.Segment(joint3, frame3))


joint4 = kdl.Joint(kdl.Joint.RotZ)
frame4 = kdl.Frame(kdl.Rotation.EulerZYX(0.0, 0.0, M_PI / 2)
                   ) * kdl.Frame(kdl.Vector(1.0, 1.0, 0.0))
chain.addSegment(kdl.Segment(joint4, frame4))

# print(chain)


jointAngles = kdl.JntArray(3)

jointAngles[0] = -M_PI / 4.       # Joint 1
jointAngles[1] = M_PI / 2.        # Joint 2
jointAngles[2] = M_PI             # Joint 3

FKSolver = kdl.ChainFkSolverPos_recursive(chain)

frame = kdl.Frame()
FKSolver.JntToCart(jointAngles, frame)
print(frame)
