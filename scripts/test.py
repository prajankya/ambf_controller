#!/usr/bin/env python

from trac_ik_python.trac_ik import IK

ik_solver = IK("torso_lift_link",
               "r_wrist_roll_link")

seed_state = [0.0] * ik_solver.number_of_joints

ik_solver.get_ik(seed_state,
                0.45, 0.1, 0.3,  # X, Y, Z
                0.0, 0.0, 0.0, 1.0)  # QX, QY, QZ, QW