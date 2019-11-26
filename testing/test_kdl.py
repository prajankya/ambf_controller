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

    # From Parent Body
    rot = kdl.Rotation.EulerZYX(float(parent_body['location']['orientation']['y']),
                                float(parent_body['location']
                                      ['orientation']['p']),
                                float(parent_body['location']['orientation']['r']))

    # From Parent Body
    trans = kdl.Vector(float(parent_body['location']['position']['x']),
                       float(parent_body['location']
                             ['position']['y']),
                       float(parent_body['location']['position']['z']))

    # From Joint location on parent
    trans2 = kdl.Vector(float(joint_data['parent pivot']['x']),
                        float(joint_data['parent pivot']['y']),
                        float(joint_data['parent pivot']['z']))

    # TODO: If Joint['client pivot'] is set, need to add it to the Frame below
    frame = kdl.Frame(rot, trans + trans2)

    print(frame)
    print("..............")

    joint = kdl.Joint.None  # Default Joint Axis

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


print("Joints:", chain.getNrOfJoints())

FKSolver = kdl.ChainFkSolverPos_recursive(chain)

output_frame = kdl.Frame()

jointParameters = kdl.JntArray(chain.getNrOfJoints())
FKSolver.JntToCart(jointParameters, output_frame)

print(output_frame)
