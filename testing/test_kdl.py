#!/usr/bin/env python
import PyKDL as kdl
import yaml

M_PI = 3.142

yaml_file_data = None

yaml_file = '/home/prajankya/ros_ws/src/ambf_controller/example_robots/blender-kuka.yaml'


chain = kdl.Chain()

with open(yaml_file, 'rb') as f:
    yaml_file_data = yaml.load(f)

for joint_name in yaml_file_data['joints']:
    # Get all variables
    joint_data = yaml_file_data[joint_name]

    parent_body = yaml_file_data[joint_data['parent']]
    child_body = yaml_file_data[joint_data['child']]

    rot = kdl.Rotation.EulerZYX(float(parent_body['location']['orientation']['y']),
                                float(parent_body['location']
                                      ['orientation']['p']),
                                float(parent_body['location']['orientation']['r']))
    trans = kdl.Vector(float(parent_body['location']['position']['x']),
                       float(parent_body['location']
                             ['position']['y']),
                       float(parent_body['location']['position']['z']))
    trans = trans + kdl.Vector(float(joint_data['parent pivot']['x']),
                               float(joint_data['parent pivot']['y']),
                               float(joint_data['parent pivot']['z']))

    frame = kdl.Frame(rot, trans)

    joint = kdl.Joint.None

    if joint_data['parent axis']['x']:
        joint = kdl.Joint(
            kdl.Joint.RotX, scale=joint_data['parent axis']['x'])
    elif joint_data['parent axis']['y']:
        joint = kdl.Joint(
            kdl.Joint.RotX, scale=joint_data['parent axis']['y'])
    elif joint_data['parent axis']['z']:
        joint = kdl.Joint(
            kdl.Joint.RotX, scale=joint_data['parent axis']['z'])

    chain.addSegment(kdl.Segment(joint, frame))


jointAngles = kdl.JntArray(chain.getNrOfJoints())

print("Joints:", chain.getNrOfJoints())
# jointAngles[0] = -M_PI / 4.       # Joint 1
# jointAngles[1] = M_PI / 2.        # Joint 2
# jointAngles[2] = M_PI             # Joint 3

FKSolver = kdl.ChainFkSolverPos_recursive(chain)

frame = kdl.Frame()
FKSolver.JntToCart(jointAngles, frame)
print(frame)


# # joint1 = kdl.Joint(kdl.Joint.None)

# # frame1 = kdl.Frame(kdl.Vector(0, 1, 0))
# # chain.addSegment(kdl.Segment(joint1, frame1))

# # joint2 = kdl.Joint(kdl.Joint.RotZ)
# # frame2 = kdl.Frame(kdl.Vector(0.0, 2.0, 0.0))
# # chain.addSegment(kdl.Segment(joint2, frame2))

# # joint3 = kdl.Joint(kdl.Joint.RotZ)
# # frame3 = kdl.Frame(kdl.Rotation.EulerZYX(0.0, 0.0, -M_PI / 2)
# #                    ) * kdl.Frame(kdl.Vector(0.0, 0.0, 2.0))
# # chain.addSegment(kdl.Segment(joint3, frame3))


# # joint4 = kdl.Joint(kdl.Joint.RotZ)
# # frame4 = kdl.Frame(kdl.Rotation.EulerZYX(0.0, 0.0, M_PI / 2)
# #                    ) * kdl.Frame(kdl.Vector(1.0, 1.0, 0.0))
# # chain.addSegment(kdl.Segment(joint4, frame4))

# # # print(chain)


# jointAngles = kdl.JntArray(3)

# jointAngles[0] = -M_PI / 4.       # Joint 1
# jointAngles[1] = M_PI / 2.        # Joint 2
# jointAngles[2] = M_PI             # Joint 3

# FKSolver = kdl.ChainFkSolverPos_recursive(chain)

# frame = kdl.Frame()
# FKSolver.JntToCart(jointAngles, frame)
# print(frame)
