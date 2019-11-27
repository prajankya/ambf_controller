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
    print(joint_name)
    joint_data = yaml_file_data[joint_name]

    parent_body = yaml_file_data[joint_data['parent']]
    child_body = yaml_file_data[joint_data['child']]

    # From Joint location on parent
    trans = kdl.Vector(float(joint_data['parent pivot']['x']),
                       float(joint_data['parent pivot']['y']),
                       float(joint_data['parent pivot']['z']))

    # kdl.
    trans2 = kdl.Vector(float(joint_data['child pivot']['x']),
                        float(joint_data['child pivot']['y']),
                        float(joint_data['child pivot']['z']))

    frame = kdl.Frame(trans + old_pos)
    old_pos = trans2

    print(frame)
    print("..............")

    kdlJoint = kdl.Joint()  # Default Joint Axis

    if joint_data['parent axis']['x']:
        kdlJoint = kdl.Joint(
            kdl.Joint.RotX, scale=joint_data['parent axis']['x'])
    elif joint_data['parent axis']['y']:
        kdlJoint = kdl.Joint(
            kdl.Joint.RotY, scale=joint_data['parent axis']['y'])
    elif joint_data['parent axis']['z']:
        kdlJoint = kdl.Joint(
            kdl.Joint.RotZ, scale=joint_data['parent axis']['z'])

    chain.addSegment(kdl.Segment(kdlJoint, frame))


print("Joints:", chain.getNrOfJoints())


output_frame = kdl.Frame()

jointParameters = kdl.JntArray(chain.getNrOfJoints())
jointParameters[0] = -0.57
jointParameters[1] = 0.12
jointParameters[2] = 0.75
jointParameters[3] = 0.71
# jointParameters[4] = 0
# jointParameters[5] = 0.785
# jointParameters[6] = -0.06

FKSolver = kdl.ChainFkSolverPos_recursive(chain)
FKSolver.JntToCart(jointParameters, output_frame)

print("Final frame :---")
print(output_frame)

IKPosSolver = kdl.ChainIkSolverPos_LMA(chain)

desired_q = kdl.JntArray(chain.getNrOfJoints())
zero_q = kdl.JntArray(chain.getNrOfJoints())

tip_frame = output_frame#kdl.Frame()

IKPosSolver.CartToJnt(zero_q, tip_frame, desired_q)
print("desired_q:--")
print(desired_q)

print("-------------------------")
print("Diff ...")
for i in range(chain.getNrOfJoints()):
    print("%3.3f"%(jointParameters[i] - desired_q[i]))