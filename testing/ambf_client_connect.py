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

_handle = _client.get_obj_handle('base')

time.sleep(0.2)
print('\n\n----')

# print(' Base Pos:')
# print(_handle.get_pos())

# print(' Base Rotation as Quaternion:')
# print(_handle.get_rot())

print('\n\n----')

print('Number of Joints in base:')
print(_handle.get_num_joints())

print('Name of base\'s children:')
print(_handle.get_joint_names())

_handle.set_joint_pos(0, 0.0)
_handle.set_joint_pos(1, 0.0)
_handle.set_joint_pos(2, 0.0)
_handle.set_joint_pos(3, 0.0)
_handle.set_joint_pos(4, 0.1)
_handle.set_joint_pos(5, 0.0)
# _handle.set_joint_pos(6, 0.0)

time.sleep(10)
