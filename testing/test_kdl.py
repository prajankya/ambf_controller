#!/usr/bin/env python
import PyKDL as kdl
import yaml


yaml_file_data = None

yaml_file = '/home/prajankya/ros_ws/src/ambf_controller/example_robots/blender-kuka.yaml'


chain = kdl.Chain()

with open(yaml_file, 'rb') as f:
    yaml_file_data = yaml.load(f)

old_pos = kdl.Vector(0, 0, 0)

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
    frame = kdl.Frame(rot, trans + trans2 - old_pos)
    old_pos = trans - trans2

    # print(frame)
    # print("..............")

    kdlJoint = kdl.Joint()  # Default Joint Axis

    if joint_data['parent axis']['x']:
        joint = kdl.Joint(
            kdl.Joint.RotX, scale=joint_data['parent axis']['x'])
    elif joint_data['parent axis']['y']:
        joint = kdl.Joint(
            kdl.Joint.RotY, scale=joint_data['parent axis']['y'])
    elif joint_data['parent axis']['z']:
        joint = kdl.Joint(
            kdl.Joint.RotZ, scale=joint_data['parent axis']['z'])

    chain.addSegment(kdl.Segment(joint, frame))


print("Joints:", chain.getNrOfJoints())

FKSolver = kdl.ChainFkSolverPos_recursive(chain)

output_frame = kdl.Frame()

jointParameters = kdl.JntArray(chain.getNrOfJoints())
jointParameters[0] = 0
jointParameters[1] = 0
jointParameters[2] = 0
jointParameters[3] = 0
jointParameters[4] = 0
jointParameters[5] = 0
jointParameters[6] = 1.57

FKSolver.JntToCart(jointParameters, output_frame)

print("Final frame :---")
print(output_frame)
