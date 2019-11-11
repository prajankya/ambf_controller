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

_handle = _client.get_obj_handle('l1')

time.sleep(0.2)
print('\n\n----')

# print(' Base Pos:')
# print(_handle.get_pos())

# print(' Base Rotation as Quaternion:')
# print(_handle.get_rot())

print('\n\n----')

print('Number of Joints in l1:')
print(_handle.get_num_joints())

print('Name of l1\'s children:')
print(_handle.get_joint_names())

# _handle.set_joint_effort(0, 20)
_handle.set_joint_pos(0, 0.0)
time.sleep(1)
_handle.set_joint_pos(0, 0.05)
time.sleep(1)
_handle.set_joint_pos(0, 0.1)
time.sleep(1)
_handle.set_joint_pos(0, 0.2)

time.sleep(10)
