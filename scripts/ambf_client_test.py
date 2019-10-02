#! /usr/bin/env python

import time

# Import the Client from ambf_client package
from ambf_client import Client

# Create a instance of the client
_client = Client()

# Connect the client which in turn creates callable objects from ROS topics
# and initiates a shared pool of threads for bi-directional communication
_client.connect()

print('\n\n----')
print("List of Objects")
print(_client.get_obj_names())

# object names, 'ecm/baselink' and 'psm/baselink' should exist
kuka_handle = _client.get_obj_handle('base')

# Let's sleep for a very brief moment to give the internal callbacks
# to sync up new data from the running simulator
time.sleep(0.2)
print('\n\n----')

print('Kuka Base Pos:')
print(kuka_handle.get_pos())

# # print(' ')
# print('Kuka Base Rotation as Quaternion:')
# print(kuka_handle.get_rot())

print('\n\n----')

print('Number of Joints in kuka:')
print(kuka_handle.get_num_joints())

print(' ')
print('Name of kuka\'s children:')
print(kuka_handle.get_joint_names())


print(kuka_handle.get_name())
print(kuka_handle.get_pos_command())
print(kuka_handle.get_pose())

# kuka_handle

kuka_handle.set_joint_pos(0, 1.57)
# kuka_handle.set_joint_effort(0, 10)
# kuka_handle.set_joint_effort(1, 10)

print('\n\n----')
raw_input("Let's clean up. Press Enter to continue...")
# Lastly to cleanup
_client.clean_up()
